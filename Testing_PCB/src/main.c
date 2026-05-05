/*
 * epc901_adc_test.c
 * ADC signal test for EPC901 PCB — nRF54L15
 *
 * Samples VIDEO_P (AIN4, P1.11) only
 * using TIMER22 + DPPI hardware-timed SAADC at 1 Msps.
 * Logs raw 10-bit values over RTT.
 *
 * Pin assignments:
 *   VIDEO_P   AIN4  P1.11   J1 pin 2  (analog input)
 *   VIDEO_N   AIN5  P1.12   J1 pin 4  (analog input, reserved for future)
 *   DATA_RDY        P1.10   J1 pin 5  (digital input)
 *   CLR_PIX         P2.05   J1 pin 7  (digital output)
 *   READ            P2.06   J1 pin 8  (digital output)
 *   SHUTTER         P2.08   J1 pin 9  (digital output)
 *   CLR_DATA        P2.09   J2 pin 3  (digital output)
 *   PWR_DOWN        P2.10   J2 pin 1  (digital output)
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

LOG_MODULE_REGISTER(epc901_adc_test, LOG_LEVEL_INF);

/* ------------------------------------------------------------------ */
/* Pin definitions                                                      */
/* ------------------------------------------------------------------ */

/* Analog inputs — must be AIN-capable P1 pins */
#define NRF_SAADC_INPUT_AIN4  NRF_PIN_PORT_TO_PIN_NUMBER(11U, 1)  /* P1.11 VIDEO_P - J1 pin 2 */
#define NRF_SAADC_INPUT_AIN5  NRF_PIN_PORT_TO_PIN_NUMBER(12U, 1)  /* P1.12 VIDEO_N - J1 pin 4 (reserved) */

#define SAADC_INPUT_VIDEO_P   NRF_SAADC_INPUT_AIN4

/* Digital input */
#define PIN_DATA_RDY   NRF_GPIO_PIN_MAP(1, 10)  /* P1.10 — J1 pin 5 */

/* Digital outputs — moved to P2 (no interrupt needed) */
#define PIN_CLR_PIX    NRF_GPIO_PIN_MAP(2, 5)   /* P2.05 — J1 pin 7 */
#define PIN_READ       NRF_GPIO_PIN_MAP(2, 6)   /* P2.06 — J1 pin 8 */
#define PIN_SHUTTER    NRF_GPIO_PIN_MAP(2, 8)   /* P2.08 — J1 pin 9 */
#define PIN_CLR_DATA   NRF_GPIO_PIN_MAP(2, 9)   /* P2.09 — J2 pin 3 */
#define PIN_PWR_DOWN   NRF_GPIO_PIN_MAP(2, 10)  /* P2.10 — J2 pin 1 */

/* ------------------------------------------------------------------ */
/* SAADC / Timer config                                                 */
/* ------------------------------------------------------------------ */

#define SAADC_SAMPLE_INTERVAL_US  1             /* 1 us → 1 Msps      */
#define SAADC_BUFFER_SIZE         1024          /* samples per buffer  */
#define TIMER_INSTANCE_NUMBER     22            /* TIMER22 on nRF54L15 */

/* One SAADC channel: ch0 = VIDEO_P */
static nrfx_saadc_channel_t channels[1];

static const nrfx_timer_t timer_instance = NRFX_TIMER_INSTANCE(TIMER_INSTANCE_NUMBER);

/* Double buffers — single channel, one sample per tick */
static int16_t saadc_buf[2][SAADC_BUFFER_SIZE];
static uint32_t saadc_current_buffer = 0;

/* ------------------------------------------------------------------ */
/* Digital output helper                                                */
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

/* ------------------------------------------------------------------ */
/* Digital I/O init                                                     */
/* ------------------------------------------------------------------ */

static void configure_digital_pins(void)
{
    /* Outputs — all default LOW */
    configure_output(PIN_CLR_PIX);
    configure_output(PIN_READ);
    configure_output(PIN_SHUTTER);
    configure_output(PIN_CLR_DATA);
    configure_output(PIN_PWR_DOWN);

    /* DATA_RDY input with pull-down — low until EPC901 is ready */
    nrf_gpio_cfg_input(PIN_DATA_RDY, NRF_GPIO_PIN_PULLDOWN);

    LOG_INF("Digital pins configured");
}

/* ------------------------------------------------------------------ */
/* Timer                                                                */
/* ------------------------------------------------------------------ */

static void configure_timer(void)
{
    nrfx_err_t err;

    nrfx_timer_config_t timer_config = NRFX_TIMER_DEFAULT_CONFIG(16000000);
    /* remove: timer_config.frequency = NRF_TIMER_FREQ_16MHz; */
    err = nrfx_timer_init(&timer_instance, &timer_config, NULL);
    if (err != NRFX_SUCCESS) {
        LOG_ERR("nrfx_timer_init error: 0x%08x", err);
        return;
    }

    uint32_t ticks = nrfx_timer_us_to_ticks(&timer_instance, SAADC_SAMPLE_INTERVAL_US);
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
        /* Buffers ready — start the timer to begin hardware-timed sampling */
        nrfx_timer_enable(&timer_instance);
        LOG_INF("SAADC ready — sampling started");
        break;

    case NRFX_SAADC_EVT_BUF_REQ:
        /* Queue next buffer while current one is being filled */
        err = nrfx_saadc_buffer_set(
            saadc_buf[(saadc_current_buffer++) % 2],
            SAADC_BUFFER_SIZE
        );
        if (err != NRFX_SUCCESS) {
            LOG_ERR("nrfx_saadc_buffer_set error: 0x%08x", err);
        }
        break;

    case NRFX_SAADC_EVT_DONE:
    {
        static uint32_t buf_count = 0;
        static uint32_t last_time = 0;
        int16_t *buf = p_event->data.done.p_buffer;
        size_t   n   = p_event->data.done.size;

        /* Runs every buffer — verifies ~1ms completion time = 1 Msps */
        uint32_t now = k_uptime_get_32();
        uint32_t elapsed = now - last_time;
        last_time = now;

        /* Buffers complete at ~1000/sec — log only once per second */
        if (buf_count++ % 1000 == 0) {
            LOG_INF("Buffer %u done: %u samples, interval=%u ms",
                buf_count, n, elapsed);
            for (size_t i = 0; i < 4; i++) {
            LOG_INF("  [%u] VIDEO_P=%d", i, buf[i]);
        }
    }
    break;
    }

    default:
        LOG_WRN("Unhandled SAADC event: %d", p_event->type);
        break;
    }
}

/* ------------------------------------------------------------------ */
/* SAADC                                                                */
/* ------------------------------------------------------------------ */

static void configure_saadc(void)
{
    nrfx_err_t err;

    /* Connect ADC interrupt to nrfx handler */
    IRQ_CONNECT(DT_IRQN(DT_NODELABEL(adc)),
                DT_IRQ(DT_NODELABEL(adc), priority),
                nrfx_isr, nrfx_saadc_irq_handler, 0);

    err = nrfx_saadc_init(DT_IRQ(DT_NODELABEL(adc), priority));
    if (err != NRFX_SUCCESS) {
        LOG_ERR("nrfx_saadc_init error: 0x%08x", err);
        return;
    }

    /* Initialize channel 0 — VIDEO_P on AIN4 (P1.11) */
    channels[0] = (nrfx_saadc_channel_t)NRFX_SAADC_DEFAULT_CHANNEL_SE(SAADC_INPUT_VIDEO_P, 0);

    /* Set gain for nRF54L15 (1/4 gain, Vref=900mV → full range=3.6V) */
    channels[0].channel_config.gain = NRF_SAADC_GAIN1_4;
    /* Set gain for nRF54L15 (1/4 gain, Vref=900mV → full range=3.6V) */
    channels[0].channel_config.gain = NRF_SAADC_GAIN1_4;
    channels[0].channel_config.acq_time = 1; /* minimum: 2 x 125ns = 250ns */    err = nrfx_saadc_channels_config(channels, 1);
    if (err != NRFX_SUCCESS) {
        LOG_ERR("nrfx_saadc_channels_config error: 0x%08x", err);
        return;
    }

    nrfx_saadc_adv_config_t adv_config = NRFX_SAADC_DEFAULT_ADV_CONFIG;
    err = nrfx_saadc_advanced_mode_set(
        BIT(0),                        /* ch0 only — VIDEO_P */
        NRF_SAADC_RESOLUTION_10BIT,
        &adv_config,
        saadc_event_handler
    );
    if (err != NRFX_SUCCESS) {
        LOG_ERR("nrfx_saadc_advanced_mode_set error: 0x%08x", err);
        return;
    }

    /* Prime both double-buffers */
    err = nrfx_saadc_buffer_set(saadc_buf[0], SAADC_BUFFER_SIZE);
    if (err != NRFX_SUCCESS) {
        LOG_ERR("nrfx_saadc_buffer_set[0] error: 0x%08x", err);
        return;
    }
    err = nrfx_saadc_buffer_set(saadc_buf[1], SAADC_BUFFER_SIZE);
    if (err != NRFX_SUCCESS) {
        LOG_ERR("nrfx_saadc_buffer_set[1] error: 0x%08x", err);
        return;
    }

    err = nrfx_saadc_mode_trigger();
    if (err != NRFX_SUCCESS) {
        LOG_ERR("nrfx_saadc_mode_trigger error: 0x%08x", err);
        return;
    }

    LOG_INF("SAADC configured: 1 channel (VIDEO_P), 10-bit, 1 Msps");
}

/* ------------------------------------------------------------------ */
/* DPPI                                                                 */
/* ------------------------------------------------------------------ */

static void configure_ppi(void)
{
    nrfx_err_t err;
    uint8_t ppi_sample_ch;
    uint8_t ppi_start_ch;

    err = nrfx_gppi_channel_alloc(&ppi_sample_ch);
    if (err != NRFX_SUCCESS) {
        LOG_ERR("gppi_channel_alloc (sample) error: 0x%08x", err);
        return;
    }

    err = nrfx_gppi_channel_alloc(&ppi_start_ch);
    if (err != NRFX_SUCCESS) {
        LOG_ERR("gppi_channel_alloc (start) error: 0x%08x", err);
        return;
    }

    /* TIMER22 COMPARE[0] → SAADC SAMPLE */
    nrfx_gppi_channel_endpoints_setup(
        ppi_sample_ch,
        nrfx_timer_compare_event_address_get(&timer_instance, NRF_TIMER_CC_CHANNEL0),
        nrf_saadc_task_address_get(NRF_SAADC, NRF_SAADC_TASK_SAMPLE)
    );

    /* SAADC END → SAADC START (continuous double-buffered sampling) */
    nrfx_gppi_channel_endpoints_setup(
        ppi_start_ch,
        nrf_saadc_event_address_get(NRF_SAADC, NRF_SAADC_EVENT_END),
        nrf_saadc_task_address_get(NRF_SAADC, NRF_SAADC_TASK_START)
    );

    nrfx_gppi_channels_enable(BIT(ppi_sample_ch));
    nrfx_gppi_channels_enable(BIT(ppi_start_ch));

    LOG_INF("DPPI configured");
}

/* ------------------------------------------------------------------ */
/* main                                                                 */
/* ------------------------------------------------------------------ */

int main(void)
{
    LOG_INF("========================================");
    LOG_INF("  EPC901 ADC Test — nRF54L15           ");
    LOG_INF("  VIDEO_P: AIN4 (P1.11) — J1 pin 2    ");
    LOG_INF("  Rate:    1 Msps, 10-bit               ");
    LOG_INF("========================================");

    configure_digital_pins();
    configure_timer();
    configure_saadc();
    configure_ppi();

    /* Sampling starts automatically once SAADC EVT_READY fires */
    k_sleep(K_FOREVER);

    return 0;
}