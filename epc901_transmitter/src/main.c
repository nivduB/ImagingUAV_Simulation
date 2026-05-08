/*
 * EPC901 BLE Transmitter — nRF54L15
 * nRF Connect SDK v3.1.1
 *
 * Flow:
 *   CMD 0x01 → start continuous capture to RAM (no BLE during capture)
 *   CMD 0x02 → stop capture, dump all frames over BLE
 *
 * Capture pipeline per frame:
 *   CLR_PIX → CLR_DATA → SHUTTER (26µs) → wait DATA_RDY →
 *   READ pulse (CDS) → 3 preload clocks → TIMER22 enable →
 *   GPIOTE hardware-toggles READ at 1MHz → SAADC samples VIDEO_P →
 *   EVT_DONE → store 1024 × 8-bit pixels in RAM frame buffer
 *
* RAM buffer:
 *   108 frames × 1024 pixels × 1 byte = 110,592 bytes (~108 KB)
 *   At 60Hz rotation and ~1.2ms/frame:
 *     ~7.7 rotations to fill buffer
 *     ~14 unique angular positions → ~25.9° angular resolution
 *   Python maps frames evenly 0°→360° for polar reconstruction
 * 
 * 
 * BLE dump after rotation:
 *   110,592 bytes ÷ 244 bytes/packet = 453 packets → ~0.9 seconds
 *
 * Wiring — nRF54L15 DK to EPC901 PCB:
 *
 *   nRF54L15 Pin   Connector     Signal       Direction
 *   ─────────────────────────────────────────────────────
 *   P1.10          J1 pin 5      DATA_RDY     INPUT
 *   P1.11 (AIN4)   J1 pin 4      VIDEO_P      INPUT (analog)
 *   P1.12          J1 pin 8      READ         OUTPUT (GPIOTE hardware)
 *   P2.06          J1 pin 7      CLR_PIX      OUTPUT (software)
 *   P2.08          J1 pin 9      SHUTTER      OUTPUT (software)
 *   P2.09          J2 pin 3      CLR_DATA     OUTPUT (software)
 *   P2.10          J2 pin 1      PWR_DOWN     OUTPUT (software)
 *   GND            J1 pin 10     GND          POWER
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <hal/nrf_gpio.h>
#include <nrfx_saadc.h>
#include <nrfx_timer.h>
#include <nrfx_gpiote.h>
#include <helpers/nrfx_gppi.h>
#if defined(DPPI_PRESENT)
#include <nrfx_dppi.h>
#else
#include <nrfx_ppi.h>
#endif

LOG_MODULE_REGISTER(EPC901_TX, LOG_LEVEL_INF);

/* ==========================================================================
 * UUIDs — must match receiver exactly
 * ========================================================================== */
#define BT_UUID_EPC901_SERVICE_VAL \
    BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef0)
#define BT_UUID_EPC901_DATA_VAL \
    BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef1)
#define BT_UUID_EPC901_CMD_VAL \
    BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef2)

#define BT_UUID_EPC901_SERVICE  BT_UUID_DECLARE_128(BT_UUID_EPC901_SERVICE_VAL)
#define BT_UUID_EPC901_DATA     BT_UUID_DECLARE_128(BT_UUID_EPC901_DATA_VAL)
#define BT_UUID_EPC901_CMD      BT_UUID_DECLARE_128(BT_UUID_EPC901_CMD_VAL)

/* ==========================================================================
 * Pin definitions
 * ========================================================================== */
#define NRF_SAADC_INPUT_AIN4   NRF_PIN_PORT_TO_PIN_NUMBER(11U, 1)  /* P1.11 — analog input for VIDEO_P (EPC901 pixel output) */

#define PIN_DATA_RDY   NRF_GPIO_PIN_MAP(1, 10)  /* P1.10 — input:  EPC901 signals frame is ready to read out */
#define PIN_READ       NRF_GPIO_PIN_MAP(1, 12)  /* P1.12 — output: GPIOTE hardware clock, pulses each pixel onto VIDEO_P */
#define PIN_CLR_PIX    NRF_GPIO_PIN_MAP(2, 6)   /* P2.06 — output: resets the pixel array before each exposure */
#define PIN_SHUTTER    NRF_GPIO_PIN_MAP(2, 8)   /* P2.08 — output: HIGH=expose, LOW=end exposure and freeze frame */
#define PIN_CLR_DATA   NRF_GPIO_PIN_MAP(2, 9)   /* P2.09 — output: clears the data register before each capture */
#define PIN_PWR_DOWN   NRF_GPIO_PIN_MAP(2, 10)  /* P2.10 — output: LOW=sensor active, HIGH=sensor powered down */

/* ==========================================================================
 * Config
 * ========================================================================== */
#define PIXELS_PER_FRAME      1024
#define BYTES_PER_FRAME       1024   /* 8-bit pixels — 1 byte per pixel */
#define MAX_FRAMES             108   /* one full rotation at 60Hz */
#define BLE_PACKET_SIZE        244   /* MTU 247 − 3 bytes ATT overhead */
#define PRELOAD_CLOCKS           3
#define TIMER_INSTANCE_NUMBER   22
#define EXPOSURE_US             26   /* minimum EPC901 exposure time */
/* At 60Hz, one frame every 154µs.
 * Since capture takes ~3ms, no extra delay needed —
 * just remove the 1ms delay in arm_saadc() instead. */
#define FRAME_INTERVAL_US  0
/* ==========================================================================
 * SAADC / Timer globals
 * ========================================================================== */
static nrfx_saadc_channel_t   saadc_channel;
static const nrfx_timer_t     timer_instance = NRFX_TIMER_INSTANCE(TIMER_INSTANCE_NUMBER);

static int16_t  saadc_buf[2][PIXELS_PER_FRAME];
static uint32_t saadc_current_buffer = 0;

static volatile bool     capture_ready = false;
static volatile int16_t *capture_buf   = NULL;

static uint8_t ppi_timer_to_saadc;
static uint8_t ppi_saadc_end_to_start;

/* ==========================================================================
 * GPIOTE globals — hardware READ clock on P1.12
 * gpiote20 is the GPIOTE instance on the P1 power domain.
 * P2 pins do NOT support GPIOTE on nRF54L15 — only P1 pins do.
 * ========================================================================== */
static nrfx_gpiote_t gpiote_inst_storage = NRFX_GPIOTE_INSTANCE(20);
static nrfx_gpiote_t *gpiote_inst        = &gpiote_inst_storage;

static uint8_t gpiote_read_channel;    /* GPIOTE output channel for READ */
static uint8_t ppi_timer_to_read_set;  /* DPPI: CC[0] → READ high */
static uint8_t ppi_timer_to_read_clr;  /* DPPI: CC[1] → READ low  */

/* ==========================================================================
 * RAM frame buffer — 108 frames × 1024 bytes = 110,592 bytes
 * ========================================================================== */
static uint8_t  frame_buffer[MAX_FRAMES][BYTES_PER_FRAME];
static uint32_t frames_captured = 0;

/* ==========================================================================
 * BLE globals
 * ========================================================================== */
static struct bt_conn *current_conn      = NULL;
static volatile bool   ble_ready         = false;  /* CCCD subscribed */
static volatile bool   capture_running   = false;  /* 0x01 received */
static volatile bool   dump_requested    = false;  /* 0x02 received */

/* ==========================================================================
 * GPIO helpers
 * Note: PIN_READ is NOT configured here — GPIOTE owns it.
 * ========================================================================== */
static void configure_output(uint32_t pin)
{
    nrf_gpio_cfg(pin,
                 NRF_GPIO_PIN_DIR_OUTPUT,
                 NRF_GPIO_PIN_INPUT_DISCONNECT,
                 NRF_GPIO_PIN_NOPULL,
                 NRF_GPIO_PIN_H0H1,
                 NRF_GPIO_PIN_NOSENSE);
    nrf_gpio_pin_clear(pin);
}

static void configure_digital_pins(void)
{
    /* P2 pins — software GPIO only (P2 has no GPIOTE on nRF54L15) */
    configure_output(PIN_CLR_PIX);
    configure_output(PIN_SHUTTER);
    configure_output(PIN_CLR_DATA);
    configure_output(PIN_PWR_DOWN);

    /* PIN_READ (P1.12) intentionally omitted — configured by GPIOTE */

    nrf_gpio_cfg_input(PIN_DATA_RDY, NRF_GPIO_PIN_PULLDOWN);

    LOG_INF("Digital pins configured (READ owned by GPIOTE).");
}

/* ==========================================================================
 * Timer
 *
 * CC[0] = 1 µs  → fires SAADC SAMPLE + READ SET (rising edge)
 * CC[1] = 0.5µs → fires READ CLR (falling edge, half-period)
 * Timer auto-clears on CC[0] → 1 MHz square wave on READ
 * ========================================================================== */
static void configure_timer(void)
{
    nrfx_timer_config_t cfg = NRFX_TIMER_DEFAULT_CONFIG(16000000);
    nrfx_err_t err = nrfx_timer_init(&timer_instance, &cfg, NULL);
    if (err != NRFX_SUCCESS) {
        LOG_ERR("timer_init error: 0x%08x", err);
        return;
    }

    /* CC[0] = 16 ticks = 1 µs — auto-clear → 1 MHz period */
    uint32_t ticks = nrfx_timer_us_to_ticks(&timer_instance, 1);
    nrfx_timer_extended_compare(&timer_instance,
                                 NRF_TIMER_CC_CHANNEL0,
                                 ticks,
                                 NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK,
                                 false);

    /* CC[1] = 8 ticks = 0.5 µs — READ falls at mid-period */
    nrfx_timer_compare(&timer_instance,
                        NRF_TIMER_CC_CHANNEL1,
                        ticks / 2,
                        false);

    LOG_INF("TIMER22 configured: CC[0]=%u ticks (1MHz), CC[1]=%u ticks (0.5MHz)",
            ticks, ticks / 2);
}

/* ==========================================================================
 * SAADC event handler
 * ========================================================================== */
static void saadc_event_handler(nrfx_saadc_evt_t const *p_event)
{
    switch (p_event->type) {

    case NRFX_SAADC_EVT_READY:
        LOG_DBG("SAADC READY");
        break;

    case NRFX_SAADC_EVT_BUF_REQ:
        nrfx_saadc_buffer_set(
            saadc_buf[(saadc_current_buffer++) % 2],
            PIXELS_PER_FRAME);
        break;

    case NRFX_SAADC_EVT_DONE:
        nrfx_timer_disable(&timer_instance);
        nrfx_gppi_channels_disable(BIT(ppi_saadc_end_to_start));

        capture_buf   = p_event->data.done.p_buffer;
        capture_ready = true;
        break;

    default:
        break;
    }
}

/* ==========================================================================
 * SAADC init
 * ========================================================================== */
static void configure_saadc(void)
{
    IRQ_CONNECT(DT_IRQN(DT_NODELABEL(adc)),
                DT_IRQ(DT_NODELABEL(adc), priority),
                nrfx_isr, nrfx_saadc_irq_handler, 0);

    nrfx_err_t err = nrfx_saadc_init(DT_IRQ(DT_NODELABEL(adc), priority));
    if (err != NRFX_SUCCESS) {
        LOG_ERR("saadc_init error: 0x%08x", err);
        return;
    }

    saadc_channel = (nrfx_saadc_channel_t)NRFX_SAADC_DEFAULT_CHANNEL_SE(
        NRF_SAADC_INPUT_AIN4, 0);
    saadc_channel.channel_config.gain     = NRF_SAADC_GAIN1_4;
    saadc_channel.channel_config.acq_time = 1;

    err = nrfx_saadc_channels_config(&saadc_channel, 1);
    if (err != NRFX_SUCCESS) {
        LOG_ERR("saadc_channels_config error: 0x%08x", err);
        return;
    }

    nrfx_saadc_adv_config_t adv = NRFX_SAADC_DEFAULT_ADV_CONFIG;
    err = nrfx_saadc_advanced_mode_set(BIT(0),
                                        NRF_SAADC_RESOLUTION_10BIT,
                                        &adv,
                                        saadc_event_handler);
    if (err != NRFX_SUCCESS) {
        LOG_ERR("saadc_advanced_mode_set error: 0x%08x", err);
        return;
    }

    LOG_INF("SAADC configured (AIN4, 10-bit, 1 Msps).");
}

/* ==========================================================================
 * DPPI
 * ========================================================================== */
static void configure_ppi(void)
{
    nrfx_err_t err;

    err = nrfx_gppi_channel_alloc(&ppi_timer_to_saadc);
    if (err != NRFX_SUCCESS) {
        LOG_ERR("gppi alloc (timer->saadc): 0x%08x", err);
        return;
    }

    err = nrfx_gppi_channel_alloc(&ppi_saadc_end_to_start);
    if (err != NRFX_SUCCESS) {
        LOG_ERR("gppi alloc (end->start): 0x%08x", err);
        return;
    }

    nrfx_gppi_channel_endpoints_setup(
        ppi_timer_to_saadc,
        nrfx_timer_compare_event_address_get(&timer_instance,
                                              NRF_TIMER_CC_CHANNEL0),
        nrf_saadc_task_address_get(NRF_SAADC, NRF_SAADC_TASK_SAMPLE));

    nrfx_gppi_channel_endpoints_setup(
        ppi_saadc_end_to_start,
        nrf_saadc_event_address_get(NRF_SAADC, NRF_SAADC_EVENT_END),
        nrf_saadc_task_address_get(NRF_SAADC, NRF_SAADC_TASK_START));

    nrfx_gppi_channels_enable(BIT(ppi_timer_to_saadc));

    LOG_INF("DPPI configured.");
}

/* ==========================================================================
 * GPIOTE — hardware READ clock on P1.12
 * ========================================================================== */
static void configure_gpiote_read(void)
{
    nrfx_err_t err;

    err = nrfx_gpiote_init(gpiote_inst,
                            DT_IRQ(DT_NODELABEL(gpiote20), priority));
    if (err != NRFX_SUCCESS) {
        LOG_ERR("GPIOTE init failed: 0x%08x", err);
        return;
    }

    err = nrfx_gpiote_channel_alloc(gpiote_inst, &gpiote_read_channel);
    if (err != NRFX_SUCCESS) {
        LOG_ERR("GPIOTE channel alloc failed: 0x%08x", err);
        return;
    }

    static const nrfx_gpiote_output_config_t out_cfg = {
        .drive         = NRF_GPIO_PIN_H0H1,
        .input_connect = NRF_GPIO_PIN_INPUT_DISCONNECT,
        .pull          = NRF_GPIO_PIN_NOPULL,
    };
    const nrfx_gpiote_task_config_t task_cfg = {
        .task_ch  = gpiote_read_channel,
        .polarity = NRF_GPIOTE_POLARITY_TOGGLE,
        .init_val = NRF_GPIOTE_INITIAL_VALUE_LOW,
    };
    err = nrfx_gpiote_output_configure(gpiote_inst, PIN_READ,
                                        &out_cfg, &task_cfg);
    if (err != NRFX_SUCCESS) {
        LOG_ERR("GPIOTE output configure failed: 0x%08x", err);
        return;
    }
    nrfx_gpiote_out_task_enable(gpiote_inst, PIN_READ);

    err = nrfx_gppi_channel_alloc(&ppi_timer_to_read_set);
    if (err != NRFX_SUCCESS) {
        LOG_ERR("gppi alloc (timer->read_set): 0x%08x", err);
        return;
    }
    nrfx_gppi_channel_endpoints_setup(
        ppi_timer_to_read_set,
        nrfx_timer_compare_event_address_get(&timer_instance,
                                              NRF_TIMER_CC_CHANNEL0),
        nrfx_gpiote_set_task_address_get(gpiote_inst, PIN_READ));
    nrfx_gppi_channels_enable(BIT(ppi_timer_to_read_set));

    err = nrfx_gppi_channel_alloc(&ppi_timer_to_read_clr);
    if (err != NRFX_SUCCESS) {
        LOG_ERR("gppi alloc (timer->read_clr): 0x%08x", err);
        return;
    }
    nrfx_gppi_channel_endpoints_setup(
        ppi_timer_to_read_clr,
        nrfx_timer_compare_event_address_get(&timer_instance,
                                              NRF_TIMER_CC_CHANNEL1),
        nrfx_gpiote_clr_task_address_get(gpiote_inst, PIN_READ));
    nrfx_gppi_channels_enable(BIT(ppi_timer_to_read_clr));

    LOG_INF("GPIOTE READ clock configured (hardware 1MHz toggle on P1.12).");
}

/* ==========================================================================
 * Arm SAADC for one frame capture
 * ========================================================================== */
static bool arm_saadc(void)
{
    nrfx_saadc_abort();
    k_usleep(10);


    capture_ready        = false;
    capture_buf          = NULL;
    saadc_current_buffer = 0;

    nrfx_gppi_channels_enable(BIT(ppi_saadc_end_to_start));

    nrfx_err_t err = nrfx_saadc_buffer_set(saadc_buf[0], PIXELS_PER_FRAME);
    if (err != NRFX_SUCCESS) {
        LOG_ERR("buffer_set[0] error: 0x%08x", err);
        return false;
    }

    err = nrfx_saadc_buffer_set(saadc_buf[1], PIXELS_PER_FRAME);
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

/* ==========================================================================
 * EPC901 capture sequence — one frame
 * Returns true when TIMER22 is running (hardware takes over).
 * Caller must wait for capture_ready flag before reading capture_buf.
 * ========================================================================== */
static bool epc901_capture(void)
{
    /* CLR_DATA and SHUTTER simultaneously — clears pixel field + frame
     * store and starts exposure in one step. No separate CLR_PIX needed
     * during continuous capture. 26µs >> 150ns CLR_DATA minimum. */
    nrf_gpio_pin_set(PIN_CLR_DATA);
    nrf_gpio_pin_set(PIN_SHUTTER);
    k_usleep(EXPOSURE_US);          /* 26µs */
    nrf_gpio_pin_clear(PIN_SHUTTER);
    nrf_gpio_pin_clear(PIN_CLR_DATA);

    /* Wait for DATA_RDY */
    uint32_t timeout = 500;
    while (nrf_gpio_pin_read(PIN_DATA_RDY) == 0 && timeout > 0) {
        k_usleep(10);
        timeout--;
    }
    if (timeout == 0) {
        LOG_ERR("DATA_RDY timeout");
        return false;
    }

    /* CDS pulse */
    nrf_gpio_pin_set(PIN_READ);
    k_usleep(1);
    nrf_gpio_pin_clear(PIN_READ);
    k_usleep(500);

    /* 3 preload clocks */
    for (int i = 0; i < PRELOAD_CLOCKS; i++) {
        nrf_gpio_pin_set(PIN_READ);
        k_usleep(1);
        nrf_gpio_pin_clear(PIN_READ);
        k_usleep(1);
    }

    /* Hand off to hardware */
    nrfx_timer_enable(&timer_instance);
    return true;
}

/* ==========================================================================
 * Convert 10-bit SAADC samples to 8-bit pixels and store in frame buffer.
 * Shifts right by 2 to drop the 2 LSBs: 0-1023 → 0-255.
 * ========================================================================== */
static void store_frame_8bit(const int16_t *samples, uint32_t frame_idx)
{
    uint8_t *dst = frame_buffer[frame_idx];
    for (int i = 0; i < PIXELS_PER_FRAME; i++) {
        int16_t val = samples[i];
        if (val < 0)   val = 0;
        if (val > 1023) val = 1023;
        dst[i] = (uint8_t)(val >> 2);   /* 10-bit → 8-bit */
    }
}

extern const struct bt_gatt_service_static epc901_svc;

/* ==========================================================================
 * Dump all captured frames over BLE.
 * Each frame is sent as raw 8-bit bytes, 1024 bytes per frame.
 * Receiver forwards to Python which reassembles into polar image.
 * ========================================================================== */
static void dump_frames_ble(void)
{
    if (frames_captured == 0) {
        LOG_WRN("No frames to dump.");
        return;
    }

    LOG_INF("Dumping %u frames over BLE...", frames_captured);

    uint32_t total_bytes = 0;

    for (uint32_t f = 0; f < frames_captured; f++) {
        uint16_t offset   = 0;
        bool     frame_ok = true;

        while (offset < BYTES_PER_FRAME) {
            uint16_t chunk = MIN(BLE_PACKET_SIZE, BYTES_PER_FRAME - offset);

            int err = bt_gatt_notify(current_conn,
                                     &epc901_svc.attrs[2],
                                     &frame_buffer[f][offset],
                                     chunk);
            if (err == -ENOMEM) {
                k_sleep(K_MSEC(10));
                continue;
            }
            if (err) {
                LOG_ERR("Notify error %d at frame %u offset %u", err, f, offset);
                frame_ok = false;
                break;
            }

            offset      += chunk;
            total_bytes += chunk;
            k_sleep(K_MSEC(2));
        }

        if (!frame_ok) {
            LOG_ERR("Frame %u failed — aborting dump.", f);
            break;
        }
    }

    LOG_INF("Dump complete: %u frames, %u bytes.", frames_captured, total_bytes);
}

/* ==========================================================================
 * BLE — GATT service
 * Attribute layout:
 *   [0] Primary service
 *   [1] DATA characteristic declaration
 *   [2] DATA characteristic value   ← bt_gatt_notify() target
 *   [3] DATA CCCD
 *   [4] CMD characteristic declaration
 *   [5] CMD characteristic value    ← cmd_write() target
 * ========================================================================== */
static void epc901_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    ble_ready = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("Notifications %s.", ble_ready ? "ENABLED" : "DISABLED");
}

static ssize_t cmd_write(struct bt_conn *conn,
                         const struct bt_gatt_attr *attr,
                         const void *buf, uint16_t len,
                         uint16_t offset, uint8_t flags)
{
    if (len < 1) return len;

    uint8_t cmd = ((const uint8_t *)buf)[0];

    if (cmd == 0x01) {
        /* Start continuous capture to RAM */
        frames_captured = 0;
        capture_running = true;
        dump_requested  = false;
        LOG_INF("CMD 0x01 — starting capture (%u frames max).", MAX_FRAMES);
    } else if (cmd == 0x02) {
        /* Stop capture and dump all frames over BLE */
        capture_running = false;
        dump_requested  = true;
        LOG_INF("CMD 0x02 — stopping capture, dumping %u frames.", frames_captured);
    }

    return len;
}

BT_GATT_SERVICE_DEFINE(epc901_svc,
    BT_GATT_PRIMARY_SERVICE(BT_UUID_EPC901_SERVICE),
    BT_GATT_CHARACTERISTIC(BT_UUID_EPC901_DATA,
                           BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_NONE,
                           NULL, NULL, NULL),
    BT_GATT_CCC(epc901_ccc_cfg_changed,
                BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    BT_GATT_CHARACTERISTIC(BT_UUID_EPC901_CMD,
                           BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
                           BT_GATT_PERM_WRITE,
                           NULL, cmd_write, NULL),
);

/* ==========================================================================
 * Burst Thread
 *
 * State machine:
 *   IDLE        → wait for capture_running or dump_requested
 *   CAPTURING   → capture frames to RAM as fast as possible
 *   DUMPING     → send all RAM frames over BLE
 * ========================================================================== */
void ble_burst_thread(void)
{
    while (1) {

        /* ---- IDLE: wait for a command ---------------------------------- */
        if (!capture_running && !dump_requested) {
            k_sleep(K_MSEC(10));
            continue;
        }

        /* ---- CAPTURING: fill RAM buffer -------------------------------- */
        if (capture_running) {
            if (!current_conn || !ble_ready) {
                LOG_WRN("Not connected — cannot capture.");
                capture_running = false;
                continue;
            }

            if (frames_captured >= MAX_FRAMES) {
                /* Buffer full — auto-stop */
                LOG_INF("Frame buffer full (%u frames). Stopping capture.", MAX_FRAMES);
                capture_running = false;
                dump_requested  = true;
                continue;
            }

            /* Arm SAADC */
            if (!arm_saadc()) {
                LOG_ERR("SAADC arm failed.");
                continue;
            }

            /* Run EPC901 capture sequence */
            if (!epc901_capture()) {
                LOG_ERR("Capture sequence failed.");
                nrfx_saadc_abort();
                nrfx_timer_disable(&timer_instance);
                continue;
            }

            /* Wait for SAADC EVT_DONE */
            uint32_t timeout = 10000;
            while (!capture_ready && timeout > 0) {
                k_usleep(10);
                timeout--;
            }

            if (!capture_ready) {
                LOG_ERR("SAADC timeout at frame %u.", frames_captured);
                nrfx_saadc_abort();
                nrfx_timer_disable(&timer_instance);
                continue;
            }

            /* Store frame as 8-bit in RAM */
            store_frame_8bit((const int16_t *)capture_buf, frames_captured);
f           rames_captured++;

        }

        /* ---- DUMPING: send all frames over BLE ------------------------- */
        if (dump_requested && !capture_running) {
            dump_requested = false;

            if (!current_conn || !ble_ready) {
                LOG_WRN("Not connected — cannot dump.");
                continue;
            }

            k_sleep(K_MSEC(200));   /* ATT channel settling */
            dump_frames_ble();
        }
    }
}

K_THREAD_DEFINE(burst_tid, 4096, ble_burst_thread, NULL, NULL, NULL, 7, 0, 0);

/* ==========================================================================
 * Advertising
 * ========================================================================== */
static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_EPC901_SERVICE_VAL),
};

static const struct bt_data sd[] = {
    BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME,
            sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

/* ==========================================================================
 * Connection callbacks
 * ========================================================================== */
static void mtu_exchange_cb(struct bt_conn *conn, uint8_t err,
                             struct bt_gatt_exchange_params *params)
{
    if (err) {
        LOG_WRN("MTU exchange failed (err %d)", err);
    } else {
        LOG_INF("MTU exchanged: %u", bt_gatt_get_mtu(conn));
    }
}

static struct bt_gatt_exchange_params mtu_exchange_params = {
    .func = mtu_exchange_cb,
};

static void connected(struct bt_conn *conn, uint8_t err)
{
    if (err) {
        LOG_ERR("Connection failed (err %u)", err);
        return;
    }
    current_conn = bt_conn_ref(conn);
    LOG_INF("Receiver connected. Waiting for CCCD subscription...");

    int mtu_err = bt_gatt_exchange_mtu(conn, &mtu_exchange_params);
    if (mtu_err) {
        LOG_WRN("MTU exchange request failed (err %d)", mtu_err);
    }
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    LOG_INF("Disconnected (0x%02x). Restarting advertising.", reason);
    if (current_conn) {
        bt_conn_unref(current_conn);
        current_conn = NULL;
    }
    ble_ready       = false;
    capture_running = false;
    dump_requested  = false;

    int err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad),
                               sd, ARRAY_SIZE(sd));
    if (err) {
        LOG_ERR("Advertising restart failed (err %d)", err);
    }
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected    = connected,
    .disconnected = disconnected,
};

/* ==========================================================================
 * Main
 * ========================================================================== */
int main(void)
{
    LOG_INF("========================================");
    LOG_INF("  EPC901 BLE Transmitter — nRF54L15    ");
    LOG_INF("  RAM buffer: %u frames × %u bytes     ", MAX_FRAMES, BYTES_PER_FRAME);
    LOG_INF("  Exposure:   %u µs                    ", EXPOSURE_US);
    LOG_INF("========================================");

    configure_digital_pins();
    configure_timer();
    configure_saadc();
    configure_ppi();
    configure_gpiote_read();

    int err = bt_enable(NULL);
    if (err) {
        LOG_ERR("bt_enable failed (err %d)", err);
        return -1;
    }

    err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad),
                           sd, ARRAY_SIZE(sd));
    if (err) {
        LOG_ERR("Advertising start failed (err %d)", err);
        return -1;
    }

    LOG_INF("Advertising as EPC901_TX. Waiting for receiver...");

    while (1) {
        k_sleep(K_SECONDS(10));
        LOG_INF("Heartbeat — conn=%s ready=%s capturing=%s frames=%u",
                current_conn    ? "yes" : "no",
                ble_ready       ? "yes" : "no",
                capture_running ? "yes" : "no",
                frames_captured);
    }

    return 0;
}
