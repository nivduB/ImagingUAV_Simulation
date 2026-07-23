/*
 * epc901_capture.c
 * Full EPC901 capture + SAADC pipeline — nRF54L15
 *
 * READ clock is software-driven in a tight loop
 * SAADC samples VIDEO_P triggered by TIMER22 via DPPI
 *
 * Wiring:
 *   DATA_RDY  → P1.10  J1 pin 5
 *   VIDEO_P   → P1.11  J1 pin 4  (AIN4)
 *   READ      → P1.12  J1 pin 8
 *   CLR_PIX   → P2.06  J1 pin 7
 *   SHUTTER   → P2.08  J1 pin 9
 *   CLR_DATA  → P2.09  J2 pin 3
 *   PWR_DOWN  → P2.10  J2 pin 1
 *   GND       → GND    J1 pin 10
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <hal/nrf_gpio.h>
#include <nrfx_saadc.h>
#include <nrfx_timer.h>
#include <helpers/nrfx_gppi.h>
#if defined(DPPI_PRESENT)
#include <nrfx_dppi.h>
#else
#include <nrfx_ppi.h>
#endif

LOG_MODULE_REGISTER(epc901_capture, LOG_LEVEL_INF);

/* ------------------------------------------------------------------ */
/* Pin definitions                                                      */
/* ------------------------------------------------------------------ */

#define NRF_SAADC_INPUT_AIN4   NRF_PIN_PORT_TO_PIN_NUMBER(11U, 1)

#define PIN_DATA_RDY   NRF_GPIO_PIN_MAP(1, 10)  /* P1.10 — J1 pin 5  */
#define PIN_READ       NRF_GPIO_PIN_MAP(1, 12)  /* P1.12 — J1 pin 8  */
#define PIN_CLR_PIX    NRF_GPIO_PIN_MAP(2, 6)   /* P2.06 — J1 pin 7  */
#define PIN_SHUTTER    NRF_GPIO_PIN_MAP(2, 8)   /* P2.08 — J1 pin 9  */
#define PIN_CLR_DATA   NRF_GPIO_PIN_MAP(2, 9)   /* P2.09 — J2 pin 3  */
#define PIN_PWR_DOWN   NRF_GPIO_PIN_MAP(2, 10)  /* P2.10 — J2 pin 1  */

#define BUTTON0_PIN    NRF_GPIO_PIN_MAP(1, 13)

/* ------------------------------------------------------------------ */
/* Config                                                               */
/* ------------------------------------------------------------------ */

#define SAADC_BUFFER_SIZE     1024
#define PRELOAD_CLOCKS        3
#define TIMER_INSTANCE_NUMBER 22

/* ------------------------------------------------------------------ */
/* Globals                                                              */
/* ------------------------------------------------------------------ */

static nrfx_saadc_channel_t   channel;
static const nrfx_timer_t     timer_instance = NRFX_TIMER_INSTANCE(TIMER_INSTANCE_NUMBER);

static int16_t  saadc_buf[2][SAADC_BUFFER_SIZE];
static uint32_t saadc_current_buffer = 0;

static volatile bool     capture_ready = false;
static volatile int16_t *capture_buf   = NULL;

static uint8_t ppi_timer_to_saadc;
static uint8_t ppi_saadc_end_to_start;

/* ------------------------------------------------------------------ */
/* GPIO helpers                                                         */
/* ------------------------------------------------------------------ */

static void configure_output(uint32_t pin)
{
    nrf_gpio_cfg(
        pin,
        NRF_GPIO_PIN_DIR_OUTPUT,
        NRF_GPIO_PIN_INPUT_DISCONNECT,
        NRF_GPIO_PIN_NOPULL,
        NRF_GPIO_PIN_H0H1,
        NRF_GPIO_PIN_NOSENSE
    );
    nrf_gpio_pin_clear(pin);
}

static void configure_digital_pins(void)
{
    configure_output(PIN_CLR_PIX);
    configure_output(PIN_READ);
    configure_output(PIN_SHUTTER);
    configure_output(PIN_CLR_DATA);
    configure_output(PIN_PWR_DOWN);

    nrf_gpio_cfg_input(PIN_DATA_RDY, NRF_GPIO_PIN_PULLDOWN);
    nrf_gpio_cfg_input(BUTTON0_PIN,  NRF_GPIO_PIN_PULLUP);

    LOG_INF("Digital pins configured");
}

/* ------------------------------------------------------------------ */
/* Timer init                                                           */
/* ------------------------------------------------------------------ */

static void configure_timer(void)
{
    nrfx_err_t err;

    nrfx_timer_config_t timer_config = NRFX_TIMER_DEFAULT_CONFIG(16000000);
    err = nrfx_timer_init(&timer_instance, &timer_config, NULL);
    if (err != NRFX_SUCCESS) {
        LOG_ERR("nrfx_timer_init error: 0x%08x", err);
        return;
    }

    /* CC[0] — triggers SAADC sample every 1us */
    uint32_t ticks = nrfx_timer_us_to_ticks(&timer_instance, 1);
    nrfx_timer_extended_compare(
        &timer_instance,
        NRF_TIMER_CC_CHANNEL0,
        ticks,
        NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK,
        false
    );

    LOG_INF("TIMER22 configured: %u ticks per sample", ticks);
}

/* ------------------------------------------------------------------ */
/* SAADC event handler                                                  */
/* ------------------------------------------------------------------ */

static void saadc_event_handler(nrfx_saadc_evt_t const *p_event)
{
    nrfx_err_t err;

    switch (p_event->type) {

    case NRFX_SAADC_EVT_READY:
        LOG_INF("SAADC ready");
        break;

    case NRFX_SAADC_EVT_BUF_REQ:
        err = nrfx_saadc_buffer_set(
            saadc_buf[(saadc_current_buffer++) % 2],
            SAADC_BUFFER_SIZE
        );
        if (err != NRFX_SUCCESS) {
            LOG_ERR("saadc_buffer_set error: 0x%08x", err);
        }
        break;

    case NRFX_SAADC_EVT_DONE:
        nrfx_timer_disable(&timer_instance);
        capture_buf   = p_event->data.done.p_buffer;
        capture_ready = true;
        break;

    default:
        break;
    }
}

/* ------------------------------------------------------------------ */
/* SAADC init                                                           */
/* ------------------------------------------------------------------ */

static void configure_saadc(void)
{
    nrfx_err_t err;

    IRQ_CONNECT(DT_IRQN(DT_NODELABEL(adc)),
                DT_IRQ(DT_NODELABEL(adc), priority),
                nrfx_isr, nrfx_saadc_irq_handler, 0);

    err = nrfx_saadc_init(DT_IRQ(DT_NODELABEL(adc), priority));
    if (err != NRFX_SUCCESS) {
        LOG_ERR("nrfx_saadc_init error: 0x%08x", err);
        return;
    }

    channel = (nrfx_saadc_channel_t)NRFX_SAADC_DEFAULT_CHANNEL_SE(
        NRF_SAADC_INPUT_AIN4, 0);
    channel.channel_config.gain     = NRF_SAADC_GAIN1_4;
    channel.channel_config.acq_time = 1;

    err = nrfx_saadc_channels_config(&channel, 1);
    if (err != NRFX_SUCCESS) {
        LOG_ERR("nrfx_saadc_channels_config error: 0x%08x", err);
        return;
    }

    nrfx_saadc_adv_config_t adv_config = NRFX_SAADC_DEFAULT_ADV_CONFIG;
    err = nrfx_saadc_advanced_mode_set(
        BIT(0),
        NRF_SAADC_RESOLUTION_10BIT,
        &adv_config,
        saadc_event_handler
    );
    if (err != NRFX_SUCCESS) {
        LOG_ERR("nrfx_saadc_advanced_mode_set error: 0x%08x", err);
        return;
    }

    LOG_INF("SAADC configured");
}

/* ------------------------------------------------------------------ */
/* DPPI init                                                            */
/* ------------------------------------------------------------------ */

static void configure_ppi(void)
{
    nrfx_err_t err;

    err = nrfx_gppi_channel_alloc(&ppi_timer_to_saadc);
    if (err != NRFX_SUCCESS) {
        LOG_ERR("gppi alloc (timer->saadc) error: 0x%08x", err);
        return;
    }

    err = nrfx_gppi_channel_alloc(&ppi_saadc_end_to_start);
    if (err != NRFX_SUCCESS) {
        LOG_ERR("gppi alloc (saadc end->start) error: 0x%08x", err);
        return;
    }

    /* TIMER CC[0] → SAADC SAMPLE */
    nrfx_gppi_channel_endpoints_setup(
        ppi_timer_to_saadc,
        nrfx_timer_compare_event_address_get(&timer_instance,
                                              NRF_TIMER_CC_CHANNEL0),
        nrf_saadc_task_address_get(NRF_SAADC, NRF_SAADC_TASK_SAMPLE)
    );

    /* SAADC END → SAADC START (double buffered) */
    nrfx_gppi_channel_endpoints_setup(
        ppi_saadc_end_to_start,
        nrf_saadc_event_address_get(NRF_SAADC, NRF_SAADC_EVENT_END),
        nrf_saadc_task_address_get(NRF_SAADC, NRF_SAADC_TASK_START)
    );

    nrfx_gppi_channels_enable(BIT(ppi_timer_to_saadc));
    nrfx_gppi_channels_enable(BIT(ppi_saadc_end_to_start));

    LOG_INF("DPPI configured");
}

/* ------------------------------------------------------------------ */
/* Arm SAADC for one frame                                              */
/* ------------------------------------------------------------------ */

static bool arm_saadc(void)
{
    nrfx_err_t err;

    nrfx_saadc_abort();
    k_sleep(K_MSEC(1));

    capture_ready        = false;
    saadc_current_buffer = 0;

    err = nrfx_saadc_buffer_set(saadc_buf[0], SAADC_BUFFER_SIZE);
    if (err != NRFX_SUCCESS) {
        LOG_ERR("buffer_set[0] error: 0x%08x", err);
        return false;
    }

    err = nrfx_saadc_buffer_set(saadc_buf[1], SAADC_BUFFER_SIZE);
    if (err != NRFX_SUCCESS) {
        LOG_ERR("buffer_set[1] error: 0x%08x", err);
        return false;
    }

    err = nrfx_saadc_mode_trigger();
    if (err != NRFX_SUCCESS) {
        LOG_ERR("mode_trigger error: 0x%08x", err);
        return false;
    }

    return true;
}

/* ------------------------------------------------------------------ */
/* EPC901 capture sequence                                              */
/* ------------------------------------------------------------------ */

static bool epc901_capture(void)
{
    /* Step 1: Reset */
    nrf_gpio_pin_set(PIN_CLR_PIX);
    k_sleep(K_MSEC(1));
    nrf_gpio_pin_clear(PIN_CLR_PIX);
    k_sleep(K_MSEC(1));

    nrf_gpio_pin_set(PIN_CLR_DATA);
    k_sleep(K_MSEC(1));
    nrf_gpio_pin_clear(PIN_CLR_DATA);
    k_sleep(K_MSEC(5));

    /* Step 2: Expose */
    LOG_INF("SHUTTER HIGH — exposing");
    nrf_gpio_pin_set(PIN_SHUTTER);
    k_sleep(K_MSEC(10));

    /* Step 3: End exposure */
    LOG_INF("SHUTTER LOW — exposure complete");
    nrf_gpio_pin_clear(PIN_SHUTTER);

    /* Step 4: Wait for DATA_RDY */
    LOG_INF("Waiting for DATA_RDY...");
    uint32_t timeout = 500;
    while (nrf_gpio_pin_read(PIN_DATA_RDY) == 0 && timeout > 0) {
        k_sleep(K_MSEC(1));
        timeout--;
    }
    if (timeout == 0) {
        LOG_ERR("DATA_RDY timeout");
        return false;
    }
    LOG_INF("DATA_RDY HIGH — frame ready");

    /* Step 5: Initial READ pulse — initiates CDS */
    nrf_gpio_pin_set(PIN_READ);
    k_usleep(1);
    nrf_gpio_pin_clear(PIN_READ);

    /* Step 6: Wait T_CDS */
    k_usleep(500);

    /* Step 7: Send 3 preload clocks — discard */
    for (int i = 0; i < PRELOAD_CLOCKS; i++) {
        nrf_gpio_pin_set(PIN_READ);
        k_usleep(1);
        nrf_gpio_pin_clear(PIN_READ);
        k_usleep(1);
    }

    /* Step 8: Start SAADC timer then send 1024 READ clocks */
    LOG_INF("Starting READ clock + SAADC sampling");
    nrfx_timer_enable(&timer_instance);
    for (int i = 0; i < SAADC_BUFFER_SIZE; i++) {
        nrf_gpio_pin_set(PIN_READ);
        k_usleep(1);
        nrf_gpio_pin_clear(PIN_READ);
        k_usleep(1);
    }

    return true;
}

/* ------------------------------------------------------------------ */
/* Button helper                                                        */
/* ------------------------------------------------------------------ */

static void wait_for_button(void)
{
    while (nrf_gpio_pin_read(BUTTON0_PIN) == 0) k_sleep(K_MSEC(10));
    while (nrf_gpio_pin_read(BUTTON0_PIN) == 1) k_sleep(K_MSEC(10));
    k_sleep(K_MSEC(50));
    while (nrf_gpio_pin_read(BUTTON0_PIN) == 0) k_sleep(K_MSEC(10));
}

/* ------------------------------------------------------------------ */
/* main                                                                 */
/* ------------------------------------------------------------------ */

int main(void)
{
    LOG_INF("========================================");
    LOG_INF("  EPC901 Capture + SAADC Pipeline      ");
    LOG_INF("  Press Button 0 to capture a frame    ");
    LOG_INF("========================================");

    configure_digital_pins();
    configure_timer();
    configure_saadc();
    configure_ppi();

    LOG_INF("Ready — press Button 0 to capture");

    while (1) {
        wait_for_button();
        LOG_INF("--- Capture triggered ---");

        if (!arm_saadc()) {
            LOG_ERR("Failed to arm SAADC");
            continue;
        }

        if (!epc901_capture()) {
            LOG_ERR("Capture sequence failed");
            nrfx_saadc_abort();
            continue;
        }

        /* Wait for SAADC to fill one frame */
        uint32_t timeout = 500;
        while (!capture_ready && timeout > 0) {
            k_sleep(K_MSEC(1));
            timeout--;
        }

        if (!capture_ready) {
            LOG_ERR("SAADC capture timeout");
            nrfx_saadc_abort();
            nrfx_timer_disable(&timer_instance);
            continue;
        }

        /* Log first 16 pixel values */
        const int16_t *buf = (const int16_t *)capture_buf;
        LOG_INF("Frame captured — first 16 pixels:");
        for (int i = 0; i < 16; i++) {
            LOG_INF("  pixel[%02d] = %d", i, buf[i]);
        }
        LOG_INF("Press Button 0 to capture again");
    }

    return 0;
}