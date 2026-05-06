/*
 * EPC901 BLE Transmitter — nRF54L15
 * nRF Connect SDK v3.1.1
 *
 * Flow per frame:
 *   1. Receiver writes 0x01 to CMD characteristic  → capture_requested = true
 *   2. Burst thread arms SAADC (two 1024-sample buffers, TIMER22+DPPI)
 *   3. Burst thread runs EPC901 capture sequence:
 *        CLR_PIX → CLR_DATA → SHUTTER (expose) → wait DATA_RDY →
 *        READ pulse (CDS) → 3 preload clocks → enable TIMER22 →
 *        1024 READ clocks (pixel clock)
 *   4. SAADC EVT_DONE fires → capture_ready = true, timer disabled
 *   5. Burst thread packs 1024 × 10-bit → 1280 bytes
 *   6. Sends 1280 bytes as BLE notifications (244 bytes per packet)
 *
 * Wiring:
 *   DATA_RDY  → P1.10  (J1 pin 5)
 *   VIDEO_P   → P1.11  (J1 pin 4)   AIN4
 *   READ      → P1.12  (J1 pin 8)
 *   CLR_PIX   → P2.06  (J1 pin 7)
 *   SHUTTER   → P2.08  (J1 pin 9)
 *   CLR_DATA  → P2.09  (J2 pin 3)
 *   PWR_DOWN  → P2.10  (J2 pin 1)
 *   GND       → GND    (J1 pin 10)
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
#define PIN_READ       NRF_GPIO_PIN_MAP(1, 12)  /* P1.12 — output: pulse to clock out each pixel onto VIDEO_P */
#define PIN_CLR_PIX    NRF_GPIO_PIN_MAP(2, 6)   /* P2.06 — output: resets the pixel array before each exposure */
#define PIN_SHUTTER    NRF_GPIO_PIN_MAP(2, 8)   /* P2.08 — output: HIGH=expose, LOW=end exposure and freeze frame */
#define PIN_CLR_DATA   NRF_GPIO_PIN_MAP(2, 9)   /* P2.09 — output: clears the data register before each capture */
#define PIN_PWR_DOWN   NRF_GPIO_PIN_MAP(2, 10)  /* P2.10 — output: LOW=sensor active, HIGH=sensor powered down */

/* ==========================================================================
 * Config
 * ========================================================================== */
#define PIXELS_PER_FRAME      1024
#define PACKED_FRAME_BYTES    1280        /* 1024 × 10-bit packed */
#define BLE_PACKET_SIZE        244        /* MTU 247 − 3 bytes ATT overhead */
#define PRELOAD_CLOCKS           3
#define TIMER_INSTANCE_NUMBER   22

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
 * BLE globals
 * ========================================================================== */
static struct bt_conn *current_conn     = NULL;
static volatile bool   ble_ready        = false;   /* CCCD subscribed */
static volatile bool   capture_requested = false;

/* Packed output buffer */
static uint8_t packed_buf[PACKED_FRAME_BYTES];

/* ==========================================================================
 * GPIO helpers
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
    configure_output(PIN_CLR_PIX);
    configure_output(PIN_READ);
    configure_output(PIN_SHUTTER);
    configure_output(PIN_CLR_DATA);
    configure_output(PIN_PWR_DOWN);

    nrf_gpio_cfg_input(PIN_DATA_RDY, NRF_GPIO_PIN_PULLDOWN);

    LOG_INF("Digital pins configured.");
}

/* ==========================================================================
 * Timer
 * ========================================================================== */
static void configure_timer(void)
{
    nrfx_timer_config_t cfg = NRFX_TIMER_DEFAULT_CONFIG(16000000);
    nrfx_err_t err = nrfx_timer_init(&timer_instance, &cfg, NULL);
    if (err != NRFX_SUCCESS) {
        LOG_ERR("timer_init error: 0x%08x", err);
        return;
    }

    /* 1 µs period → 1 Msps */
    uint32_t ticks = nrfx_timer_us_to_ticks(&timer_instance, 1);
    nrfx_timer_extended_compare(&timer_instance,
                                 NRF_TIMER_CC_CHANNEL0,
                                 ticks,
                                 NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK,
                                 false);

    LOG_INF("TIMER22 configured: %u ticks/sample (1 Msps)", ticks);
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
        /* Double-buffer: queue the next buffer when the current one starts */
        nrfx_saadc_buffer_set(
            saadc_buf[(saadc_current_buffer++) % 2],
            PIXELS_PER_FRAME);
        break;

    case NRFX_SAADC_EVT_DONE:
        /*
         * First buffer is full — stop everything.
         * Disable timer so no more SAMPLE triggers fire.
         * Disable auto-restart PPI so the second buffer doesn't start.
         */
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

    /* TIMER22 CC[0] → SAADC SAMPLE task */
    nrfx_gppi_channel_endpoints_setup(
        ppi_timer_to_saadc,
        nrfx_timer_compare_event_address_get(&timer_instance,
                                              NRF_TIMER_CC_CHANNEL0),
        nrf_saadc_task_address_get(NRF_SAADC, NRF_SAADC_TASK_SAMPLE));

    /* SAADC END event → SAADC START task (enables double buffering) */
    nrfx_gppi_channel_endpoints_setup(
        ppi_saadc_end_to_start,
        nrf_saadc_event_address_get(NRF_SAADC, NRF_SAADC_EVENT_END),
        nrf_saadc_task_address_get(NRF_SAADC, NRF_SAADC_TASK_START));

    /* Only the timer→saadc channel is always active.
     * end→start is re-enabled per capture and disabled in EVT_DONE. */
    nrfx_gppi_channels_enable(BIT(ppi_timer_to_saadc));

    LOG_INF("DPPI configured.");
}

/* ==========================================================================
 * Arm SAADC for one frame capture
 * ========================================================================== */
static bool arm_saadc(void)
{
    nrfx_saadc_abort();
    k_sleep(K_MSEC(1));

    capture_ready        = false;
    capture_buf          = NULL;
    saadc_current_buffer = 0;

    /* Re-enable auto-restart PPI for double buffering */
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
 * EPC901 capture sequence
 * ========================================================================== */
static bool epc901_capture(void)
{
    /* Step 1: Clear pixel array and data register */
    nrf_gpio_pin_set(PIN_CLR_PIX);
    k_sleep(K_MSEC(1));
    nrf_gpio_pin_clear(PIN_CLR_PIX);
    k_sleep(K_MSEC(1));

    nrf_gpio_pin_set(PIN_CLR_DATA);
    k_sleep(K_MSEC(1));
    nrf_gpio_pin_clear(PIN_CLR_DATA);
    k_sleep(K_MSEC(5));

    /* Step 2: Expose */
    nrf_gpio_pin_set(PIN_SHUTTER);
    k_sleep(K_MSEC(10));

    /* Step 3: End exposure */
    nrf_gpio_pin_clear(PIN_SHUTTER);

    /* Step 4: Wait for DATA_RDY */
    uint32_t timeout = 500;
    while (nrf_gpio_pin_read(PIN_DATA_RDY) == 0 && timeout > 0) {
        k_sleep(K_MSEC(1));
        timeout--;
    }
    if (timeout == 0) {
        LOG_ERR("DATA_RDY timeout — aborting capture");
        return false;
    }

    /* Step 5: Initial READ pulse — initiates CDS */
    nrf_gpio_pin_set(PIN_READ);
    k_usleep(1);
    nrf_gpio_pin_clear(PIN_READ);

    /* Step 6: Wait for CDS settling */
    k_usleep(500);

    /* Step 7: 3 preload clocks — discarded by SAADC (not yet running) */
    for (int i = 0; i < PRELOAD_CLOCKS; i++) {
        nrf_gpio_pin_set(PIN_READ);
        k_usleep(1);
        nrf_gpio_pin_clear(PIN_READ);
        k_usleep(1);
    }

    /* Step 8: Enable TIMER22, then clock out 1024 pixels.
     * Each rising edge of READ shifts out one pixel onto VIDEO_P.
     * TIMER22 fires SAADC SAMPLE every 1 µs via DPPI. */
    nrfx_timer_enable(&timer_instance);

    for (int i = 0; i < PIXELS_PER_FRAME; i++) {
        nrf_gpio_pin_set(PIN_READ);
        k_usleep(1);
        nrf_gpio_pin_clear(PIN_READ);
        k_usleep(1);
    }

    return true;
}

/* ==========================================================================
 * Pack 1024 × 10-bit samples into 1280 bytes (little-endian bit packing)
 *
 * 4 pixels → 5 bytes:
 *   byte 0 = px0[7:0]
 *   byte 1 = px1[5:0] << 2 | px0[9:8]
 *   byte 2 = px2[3:0] << 4 | px1[9:6]
 *   byte 3 = px3[1:0] << 6 | px2[9:4]
 *   byte 4 = px3[9:2]
 *
 * save_frames.py uses matching unpack_10bit().
 * ========================================================================== */
static void pack_10bit(const int16_t *samples, uint8_t *out, int num_pixels)
{
    int out_idx = 0;
    for (int i = 0; i < num_pixels; i += 4) {
        uint16_t p0 = (uint16_t)samples[i + 0] & 0x3FF;
        uint16_t p1 = (uint16_t)samples[i + 1] & 0x3FF;
        uint16_t p2 = (uint16_t)samples[i + 2] & 0x3FF;
        uint16_t p3 = (uint16_t)samples[i + 3] & 0x3FF;

        out[out_idx++] =  p0        & 0xFF;
        out[out_idx++] = (p0 >> 8)  | ((p1 & 0x3F) << 2);
        out[out_idx++] = (p1 >> 6)  | ((p2 & 0x0F) << 4);
        out[out_idx++] = (p2 >> 4)  | ((p3 & 0x03) << 6);
        out[out_idx++] =  p3 >> 2;
    }
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
    if (len >= 1 && ((const uint8_t *)buf)[0] == 0x01) {
        capture_requested = true;
        LOG_INF("Capture triggered via BLE CMD.");
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
 * Waits for capture_requested, runs EPC901 capture, packs pixels,
 * sends 1280 bytes as BLE notifications (244 bytes per packet = 6 packets).
 * ========================================================================== */
void ble_burst_thread(void)
{
    while (1) {
        /* Wait for trigger from receiver */
        while (!capture_requested) {
            k_sleep(K_MSEC(10));
        }
        capture_requested = false;

        if (!current_conn || !ble_ready) {
            LOG_WRN("Trigger received but BLE not ready — skipping.");
            continue;
        }

        /* --- Arm SAADC --------------------------------------------------- */
        if (!arm_saadc()) {
            LOG_ERR("SAADC arm failed — skipping frame.");
            continue;
        }

        /* --- EPC901 capture sequence ------------------------------------- */
        LOG_INF("Starting EPC901 capture...");
        if (!epc901_capture()) {
            LOG_ERR("EPC901 capture failed — skipping frame.");
            nrfx_saadc_abort();
            nrfx_timer_disable(&timer_instance);
            continue;
        }

        /* --- Wait for SAADC to finish ------------------------------------ */
        uint32_t timeout = 100;
        while (!capture_ready && timeout > 0) {
            k_sleep(K_MSEC(1));
            timeout--;
        }

        if (!capture_ready) {
            LOG_ERR("SAADC capture timeout — skipping frame.");
            nrfx_saadc_abort();
            nrfx_timer_disable(&timer_instance);
            continue;
        }

        /* --- Pack 1024 × 10-bit → 1280 bytes ---------------------------- */
        pack_10bit((const int16_t *)capture_buf, packed_buf, PIXELS_PER_FRAME);

        /* --- Transmit as BLE notifications ------------------------------ */
    k_sleep(K_MSEC(200));

    uint16_t offset   = 0;
    int      packet   = 0;
    bool     tx_ok    = true;
    uint16_t pkt_size = BLE_PACKET_SIZE;   /* start with 244, fall back to 20 */

    while (offset < PACKED_FRAME_BYTES) {
        uint16_t chunk = MIN(pkt_size, PACKED_FRAME_BYTES - offset);

        int err = bt_gatt_notify(current_conn,
                                &epc901_svc.attrs[2],
                                &packed_buf[offset],
                                chunk);
        if (err == -ENOMEM) {
            if (pkt_size > 20) {
                pkt_size = 20;
                LOG_WRN("MTU fallback to 20-byte packets.");
            }
            k_sleep(K_MSEC(10));
            continue;
        }
        if (err) {
            LOG_ERR("Notify error %d at offset %u", err, offset);
            tx_ok = false;
            break;
        }

        offset += chunk;
        packet++;
        k_sleep(K_MSEC(2));
    }

        if (tx_ok) {
            LOG_INF("Frame transmitted: %d packets, %u bytes.",
                    packet, offset);
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
    ble_ready         = false;
    capture_requested = false;

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
    LOG_INF("========================================");

    configure_digital_pins();
    configure_timer();
    configure_saadc();
    configure_ppi();

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

    /* Burst thread handles everything — main just keeps the system alive */
    while (1) {
        k_sleep(K_SECONDS(10));
        LOG_INF("Heartbeat — conn=%s ble_ready=%s",
                current_conn ? "yes" : "no",
                ble_ready    ? "yes" : "no");
    }

    return 0;
}