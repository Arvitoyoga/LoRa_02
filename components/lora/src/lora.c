/*
 * lora.c - Driver SX1276/77/78/79 (mode LoRa) untuk ESP-IDF.
 */
#include "lora.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_attr.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

static const char *TAG = "lora";

/* ------------------------------------------------------------------------ */
/*  Register SX127x (mode LoRa)                                             */
/* ------------------------------------------------------------------------ */
#define REG_FIFO              0x00
#define REG_OP_MODE           0x01
#define REG_FRF_MSB           0x06
#define REG_FRF_MID           0x07
#define REG_FRF_LSB           0x08
#define REG_PA_CONFIG         0x09
#define REG_PA_RAMP           0x0A
#define REG_OCP               0x0B
#define REG_LNA               0x0C
#define REG_FIFO_ADDR_PTR     0x0D
#define REG_FIFO_TX_BASE      0x0E
#define REG_FIFO_RX_BASE      0x0F
#define REG_FIFO_RX_CURR      0x10
#define REG_IRQ_MASK          0x11
#define REG_IRQ_FLAGS         0x12
#define REG_RX_NB_BYTES       0x13
#define REG_PKT_SNR           0x19
#define REG_PKT_RSSI          0x1A
#define REG_RSSI              0x1B
#define REG_MODEM_CFG1        0x1D
#define REG_MODEM_CFG2        0x1E
#define REG_SYMB_TIMEOUT_LSB  0x1F
#define REG_PREAMBLE_MSB      0x20
#define REG_PREAMBLE_LSB      0x21
#define REG_PAYLOAD_LEN       0x22
#define REG_MAX_PAYLOAD_LEN   0x23
#define REG_MODEM_CFG3        0x26
#define REG_FEI_MSB           0x28
#define REG_FEI_MID           0x29
#define REG_FEI_LSB           0x2A
#define REG_DETECT_OPTIMIZE   0x31
#define REG_INVERT_IQ         0x33
#define REG_ERRATA_36         0x36
#define REG_DETECT_THRESHOLD  0x37
#define REG_SYNC_WORD         0x39
#define REG_ERRATA_3A         0x3A
#define REG_INVERT_IQ2        0x3B
#define REG_DIO_MAP1          0x40
#define REG_VERSION           0x42
#define REG_PA_DAC            0x4D

/* RegOpMode */
#define OPMODE_LORA           0x80
#define OPMODE_LOW_FREQ       0x08
#define MODE_SLEEP            0x00
#define MODE_STANDBY          0x01
#define MODE_TX               0x03
#define MODE_RX_CONT          0x05
#define MODE_RX_SINGLE        0x06
#define MODE_CAD              0x07

/* RegIrqFlags */
#define IRQ_RX_TIMEOUT        0x80
#define IRQ_RX_DONE           0x40
#define IRQ_CRC_ERR           0x20
#define IRQ_VALID_HDR         0x10
#define IRQ_TX_DONE           0x08
#define IRQ_CAD_DONE          0x04
#define IRQ_CAD_DETECTED      0x01

/* RegDioMapping1 (hanya DIO0 yang dipakai) */
#define DIO0_RX_DONE          0x00
#define DIO0_TX_DONE          0x40
#define DIO0_CAD_DONE         0x80

#define LF_HF_THRESHOLD_HZ    525000000UL
#define FXOSC_HZ              32000000ULL

/* ------------------------------------------------------------------------ */
/*  Struktur internal                                                       */
/* ------------------------------------------------------------------------ */
struct lora_t {
    lora_config_t       cfg;
    spi_device_handle_t spi;
    SemaphoreHandle_t   lock;
    SemaphoreHandle_t   irq_sem;
    SemaphoreHandle_t   rx_exit_sem;
    TaskHandle_t        rx_task;
    lora_rx_cb_t        rx_cb;
    void               *rx_ctx;
    volatile bool       rx_active;
    volatile bool       rx_stop;
    bool                spi_bus_owned;
    bool                isr_added;
    uint8_t             reg3a_default;
    uint8_t             spi_tx[LORA_MAX_PAYLOAD + 1] __attribute__((aligned(4)));
    uint8_t             spi_rx[LORA_MAX_PAYLOAD + 1] __attribute__((aligned(4)));
};

static const uint32_t BW_HZ[10] = {
    7800, 10400, 15600, 20800, 31250, 41700, 62500, 125000, 250000, 500000
};

/* ------------------------------------------------------------------------ */
/*  SPI                                                                     */
/* ------------------------------------------------------------------------ */
static esp_err_t spi_xfer(lora_t *r, size_t nbytes)
{
    spi_transaction_t t = {
        .length    = nbytes * 8,
        .tx_buffer = r->spi_tx,
        .rx_buffer = r->spi_rx,
    };
    return spi_device_polling_transmit(r->spi, &t);
}

static uint8_t rd(lora_t *r, uint8_t reg)
{
    r->spi_tx[0] = reg & 0x7F;
    r->spi_tx[1] = 0;
    spi_xfer(r, 2);
    return r->spi_rx[1];
}

static void wr(lora_t *r, uint8_t reg, uint8_t val)
{
    r->spi_tx[0] = reg | 0x80;
    r->spi_tx[1] = val;
    spi_xfer(r, 2);
}

static void rmw(lora_t *r, uint8_t reg, uint8_t mask, uint8_t val)
{
    wr(r, reg, (uint8_t)((rd(r, reg) & ~mask) | (val & mask)));
}

/* ------------------------------------------------------------------------ */
/*  Helper umum                                                             */
/* ------------------------------------------------------------------------ */
static bool is_low_freq(const lora_config_t *c)
{
    return c->frequency_hz < LF_HF_THRESHOLD_HZ;
}

static void set_mode(lora_t *r, uint8_t mode)
{
    uint8_t lf = is_low_freq(&r->cfg) ? OPMODE_LOW_FREQ : 0;
    wr(r, REG_OP_MODE, (uint8_t)(OPMODE_LORA | lf | mode));
}

static bool ldro_active(const lora_config_t *c)
{
    if (c->ldro == LORA_LDRO_ON)  return true;
    if (c->ldro == LORA_LDRO_OFF) return false;
    double ts_ms = (double)(1u << c->sf) * 1000.0 / (double)BW_HZ[c->bw];
    return ts_ms > 16.0;
}

static TickType_t ticks_5ms(void)
{
    TickType_t t = pdMS_TO_TICKS(5);
    return t ? t : 1;
}

/* Tunggu salah satu flag IRQ. Mengembalikan flag, atau 0 bila timeout. */
static uint8_t wait_irq(lora_t *r, uint8_t mask, uint32_t timeout_ms)
{
    int64_t deadline = (timeout_ms == LORA_WAIT_FOREVER)
                           ? INT64_MAX
                           : esp_timer_get_time() + (int64_t)timeout_ms * 1000;
    for (;;) {
        uint8_t f = rd(r, REG_IRQ_FLAGS);
        if (f & mask) return f;
        if (esp_timer_get_time() >= deadline) return 0;
        if (r->cfg.pin_dio0 >= 0) {
            xSemaphoreTake(r->irq_sem, ticks_5ms());
        } else {
            vTaskDelay(1);
        }
    }
}

static void IRAM_ATTR dio0_isr(void *arg)
{
    lora_t *r = (lora_t *)arg;
    BaseType_t woke = pdFALSE;
    xSemaphoreGiveFromISR(r->irq_sem, &woke);
    portYIELD_FROM_ISR(woke);
}

/* ------------------------------------------------------------------------ */
/*  Menerapkan konfigurasi ke register                                      */
/* ------------------------------------------------------------------------ */
static void apply_frequency(lora_t *r)
{
    uint64_t frf = ((uint64_t)r->cfg.frequency_hz << 19) / FXOSC_HZ;
    wr(r, REG_FRF_MSB, (uint8_t)(frf >> 16));
    wr(r, REG_FRF_MID, (uint8_t)(frf >> 8));
    wr(r, REG_FRF_LSB, (uint8_t)(frf));
    rmw(r, REG_OP_MODE, OPMODE_LOW_FREQ, is_low_freq(&r->cfg) ? OPMODE_LOW_FREQ : 0);
}

static void apply_modem(lora_t *r)
{
    lora_config_t *c = &r->cfg;

    if (c->sf == LORA_SF6 && c->header_mode != LORA_HEADER_IMPLICIT) {
        ESP_LOGW(TAG, "SF6 mewajibkan header implicit, diubah otomatis");
        c->header_mode = LORA_HEADER_IMPLICIT;
    }

    uint16_t symb = c->rx_timeout_symbols;
    if (symb < 4)    symb = 4;
    if (symb > 1023) symb = 1023;

    uint8_t cfg1 = (uint8_t)((c->bw << 4) | (c->cr << 1) |
                             (c->header_mode == LORA_HEADER_IMPLICIT ? 1 : 0));
    uint8_t cfg2 = (uint8_t)((c->sf << 4) | (c->crc ? 0x04 : 0x00) | ((symb >> 8) & 0x03));
    uint8_t cfg3 = (uint8_t)((ldro_active(c) ? 0x08 : 0x00) |
                             (c->lna_gain == LORA_LNA_AUTO ? 0x04 : 0x00));

    wr(r, REG_MODEM_CFG1, cfg1);
    wr(r, REG_MODEM_CFG2, cfg2);
    wr(r, REG_SYMB_TIMEOUT_LSB, (uint8_t)(symb & 0xFF));
    wr(r, REG_MODEM_CFG3, cfg3);

    if (c->sf == LORA_SF6) {
        rmw(r, REG_DETECT_OPTIMIZE, 0x07, 0x05);
        wr(r, REG_DETECT_THRESHOLD, 0x0C);
    } else {
        rmw(r, REG_DETECT_OPTIMIZE, 0x07, 0x03);
        wr(r, REG_DETECT_THRESHOLD, 0x0A);
    }

    /* Errata sensitivitas untuk BW 500 kHz */
    if (c->bw == LORA_BW_500K) {
        wr(r, REG_ERRATA_36, 0x02);
        if (c->frequency_hz <= LF_HF_THRESHOLD_HZ)      wr(r, REG_ERRATA_3A, 0x7F);
        else if (c->frequency_hz >= 862000000UL)        wr(r, REG_ERRATA_3A, 0x64);
    } else {
        wr(r, REG_ERRATA_36, 0x03);
        wr(r, REG_ERRATA_3A, r->reg3a_default);
    }

    if (c->header_mode == LORA_HEADER_IMPLICIT) {
        wr(r, REG_PAYLOAD_LEN, c->implicit_payload_len);
    }
    wr(r, REG_MAX_PAYLOAD_LEN, 0xFF);
}

static void apply_preamble(lora_t *r)
{
    wr(r, REG_PREAMBLE_MSB, (uint8_t)(r->cfg.preamble_len >> 8));
    wr(r, REG_PREAMBLE_LSB, (uint8_t)(r->cfg.preamble_len & 0xFF));
}

static void apply_sync_word(lora_t *r)
{
    wr(r, REG_SYNC_WORD, r->cfg.sync_word);
}

static void apply_iq(lora_t *r)
{
    if (r->cfg.invert_iq) {
        wr(r, REG_INVERT_IQ, 0x66);
        wr(r, REG_INVERT_IQ2, 0x19);
    } else {
        wr(r, REG_INVERT_IQ, 0x27);
        wr(r, REG_INVERT_IQ2, 0x1D);
    }
}

static void apply_pa(lora_t *r)
{
    lora_config_t *c = &r->cfg;

    if (c->tx_power_mw > 0.0f) {            /* daya diberikan dalam mW: ubah ke dBm */
        c->tx_power_dbm = lora_mw_to_dbm(c->tx_power_mw);
        c->tx_power_mw  = 0.0f;             /* sudah dikonversi, dBm yang berlaku */
    }

    int p = c->tx_power_dbm;
    int ocp = c->ocp_ma;

    if (c->pa_pin == LORA_PA_BOOST) {
        if (p < 2)  p = 2;
        if (p > 20) p = 20;
        if (p > 17) {                       /* mode +20 dBm */
            wr(r, REG_PA_DAC, 0x87);
            wr(r, REG_PA_CONFIG, (uint8_t)(0x80 | (p - 5)));
            if (ocp != 0 && ocp < 140) ocp = 140;
        } else {
            wr(r, REG_PA_DAC, 0x84);
            wr(r, REG_PA_CONFIG, (uint8_t)(0x80 | (p - 2)));
        }
    } else {
        if (p < 0)  p = 0;
        if (p > 14) p = 14;
        wr(r, REG_PA_DAC, 0x84);
        wr(r, REG_PA_CONFIG, (uint8_t)((7 << 4) | p));   /* MaxPower=7 -> Pout = OutputPower */
    }

    c->tx_power_dbm = (int8_t)p;            /* simpan daya efektif setelah dibatasi */

    if (ocp == 0) {
        wr(r, REG_OCP, 0x0B);               /* OCP mati */
    } else {
        if (ocp < 45)  ocp = 45;
        if (ocp > 240) ocp = 240;
        uint8_t trim = (ocp <= 120) ? (uint8_t)((ocp - 45) / 5) : (uint8_t)((ocp + 30) / 10);
        wr(r, REG_OCP, (uint8_t)(0x20 | (trim & 0x1F)));
    }
}

static void apply_pa_ramp(lora_t *r)
{
    rmw(r, REG_PA_RAMP, 0x0F, (uint8_t)r->cfg.pa_ramp);
}

static void apply_lna(lora_t *r)
{
    uint8_t gain = (r->cfg.lna_gain == LORA_LNA_AUTO) ? 1 : (uint8_t)r->cfg.lna_gain;
    wr(r, REG_LNA, (uint8_t)((gain << 5) | (r->cfg.lna_boost_hf ? 0x03 : 0x00)));
}

static void apply_all(lora_t *r)
{
    wr(r, REG_FIFO_TX_BASE, 0x00);
    wr(r, REG_FIFO_RX_BASE, 0x00);
    wr(r, REG_IRQ_MASK, 0x00);
    apply_frequency(r);
    apply_modem(r);
    apply_preamble(r);
    apply_sync_word(r);
    apply_iq(r);
    apply_pa(r);
    apply_pa_ramp(r);
    apply_lna(r);
}

/* ------------------------------------------------------------------------ */
/*  Operasi radio (tanpa lock)                                              */
/* ------------------------------------------------------------------------ */
static void enter_rx(lora_t *r)
{
    wr(r, REG_FIFO_ADDR_PTR, 0x00);
    wr(r, REG_DIO_MAP1, DIO0_RX_DONE);
    wr(r, REG_IRQ_FLAGS, 0xFF);
    set_mode(r, MODE_RX_CONT);
}

static int rssi_offset(const lora_config_t *c)
{
    return is_low_freq(c) ? 164 : 157;
}

static esp_err_t read_packet(lora_t *r, uint8_t flags, lora_packet_t *pkt)
{
    bool crc_err = (flags & IRQ_CRC_ERR) != 0;
    wr(r, REG_IRQ_FLAGS, IRQ_RX_DONE | IRQ_CRC_ERR | IRQ_VALID_HDR);

    if (crc_err && !r->cfg.keep_crc_error_packets) {
        return ESP_ERR_INVALID_CRC;
    }

    uint8_t n   = rd(r, REG_RX_NB_BYTES);
    uint8_t cur = rd(r, REG_FIFO_RX_CURR);
    wr(r, REG_FIFO_ADDR_PTR, cur);

    if (n > 0) {
        r->spi_tx[0] = REG_FIFO & 0x7F;
        memset(&r->spi_tx[1], 0, n);
        spi_xfer(r, (size_t)n + 1);
        memcpy(pkt->data, &r->spi_rx[1], n);
    }
    pkt->data[n] = '\0';
    pkt->len     = n;
    pkt->crc_ok  = !crc_err;

    /* SNR & RSSI paket (rumus datasheet) */
    int8_t snr_raw = (int8_t)rd(r, REG_PKT_SNR);
    int    rssi_raw = rd(r, REG_PKT_RSSI);
    float  snr = snr_raw * 0.25f;
    float  off = (float)rssi_offset(&r->cfg);
    float  rssi = (snr < 0) ? (-off + (float)rssi_raw + snr)
                            : (-off + (16.0f / 15.0f) * (float)rssi_raw);
    pkt->snr  = snr;
    pkt->rssi = (int16_t)lroundf(rssi);

    /* Frequency error */
    int32_t fei = ((int32_t)(rd(r, REG_FEI_MSB) & 0x0F) << 16) |
                  ((int32_t)rd(r, REG_FEI_MID) << 8) |
                  (int32_t)rd(r, REG_FEI_LSB);
    if (fei & 0x80000) fei -= 0x100000;
    float bw = (float)BW_HZ[r->cfg.bw];
    pkt->freq_error_hz = (int32_t)lroundf((float)fei * 0.524288f * (bw / 500000.0f));

    pkt->timestamp_us = esp_timer_get_time();
    return ESP_OK;
}

static esp_err_t transmit(lora_t *r, const uint8_t *data, size_t len, uint32_t timeout_ms)
{
    set_mode(r, MODE_STANDBY);
    wr(r, REG_FIFO_ADDR_PTR, 0x00);

    r->spi_tx[0] = REG_FIFO | 0x80;
    memcpy(&r->spi_tx[1], data, len);
    spi_xfer(r, len + 1);

    wr(r, REG_PAYLOAD_LEN, (uint8_t)len);
    wr(r, REG_DIO_MAP1, DIO0_TX_DONE);
    wr(r, REG_IRQ_FLAGS, 0xFF);
    set_mode(r, MODE_TX);

    uint8_t f = wait_irq(r, IRQ_TX_DONE, timeout_ms);
    wr(r, REG_IRQ_FLAGS, 0xFF);
    set_mode(r, MODE_STANDBY);
    wr(r, REG_DIO_MAP1, DIO0_RX_DONE);
    return f ? ESP_OK : ESP_ERR_TIMEOUT;
}

/* Hentikan RX sementara untuk mengubah konfigurasi; mengembalikan status RX. */
static bool cfg_begin(lora_t *r)
{
    xSemaphoreTake(r->lock, portMAX_DELAY);
    bool rx = r->rx_active;
    set_mode(r, MODE_STANDBY);
    return rx;
}

static void cfg_end(lora_t *r, bool rx)
{
    if (rx) enter_rx(r);
    xSemaphoreGive(r->lock);
}

/* ------------------------------------------------------------------------ */
/*  Informasi                                                               */
/* ------------------------------------------------------------------------ */
uint32_t lora_get_airtime_ms(lora_t *r, size_t payload_len)
{
    if (!r) return 0;
    const lora_config_t *c = &r->cfg;
    double sf    = (double)c->sf;
    double tsym  = (double)(1u << c->sf) * 1000.0 / (double)BW_HZ[c->bw];
    double de    = ldro_active(c) ? 1.0 : 0.0;
    double ih    = (c->header_mode == LORA_HEADER_IMPLICIT) ? 1.0 : 0.0;
    double crc   = c->crc ? 1.0 : 0.0;
    double num   = 8.0 * (double)payload_len - 4.0 * sf + 28.0 + 16.0 * crc - 20.0 * ih;
    double den   = 4.0 * (sf - 2.0 * de);
    double nsym  = ceil(num / den) * ((double)c->cr + 4.0);
    if (nsym < 0) nsym = 0;
    double total = ((double)c->preamble_len + 4.25 + 8.0 + nsym) * tsym;
    return (uint32_t)ceil(total);
}

float lora_get_bitrate_bps(lora_t *r)
{
    if (!r) return 0;
    const lora_config_t *c = &r->cfg;
    return (float)c->sf * ((float)BW_HZ[c->bw] / (float)(1u << c->sf)) * (4.0f / (4.0f + (float)c->cr));
}

float lora_get_symbol_time_us(lora_t *r)
{
    if (!r) return 0;
    return (float)(1u << r->cfg.sf) * 1e6f / (float)BW_HZ[r->cfg.bw];
}

esp_err_t lora_get_config(lora_t *r, lora_config_t *out)
{
    if (!r || !out) return ESP_ERR_INVALID_ARG;
    xSemaphoreTake(r->lock, portMAX_DELAY);
    *out = r->cfg;
    xSemaphoreGive(r->lock);
    return ESP_OK;
}

void lora_print_config(lora_t *r)
{
    if (!r) return;
    static const char *bw_names[] = {"7.8", "10.4", "15.6", "20.8", "31.25", "41.7", "62.5", "125", "250", "500"};
    static const char *cr_names[] = {"", "4/5", "4/6", "4/7", "4/8"};
    static const char *ramp_us[]  = {"3400", "2000", "1000", "500", "250", "125", "100", "62",
                                     "50", "40", "31", "25", "20", "15", "12", "10"};
    const lora_config_t *c = &r->cfg;

    ESP_LOGI(TAG, "---- Konfigurasi LoRa ----");
    ESP_LOGI(TAG, "Frekuensi     : %.3f MHz", c->frequency_hz / 1e6);
    ESP_LOGI(TAG, "SF / BW / CR  : SF%d / %s kHz / %s", c->sf, bw_names[c->bw], cr_names[c->cr]);
    ESP_LOGI(TAG, "Preamble/Sync : %u simbol / 0x%02X", c->preamble_len, c->sync_word);
    ESP_LOGI(TAG, "CRC / Header  : %s / %s", c->crc ? "on" : "off",
             c->header_mode == LORA_HEADER_IMPLICIT ? "implicit" : "explicit");
    ESP_LOGI(TAG, "IQ terbalik   : %s", c->invert_iq ? "ya" : "tidak");
    ESP_LOGI(TAG, "TX power      : %d dBm (%.1f mW) di %s, ramp %s us, OCP %u mA", c->tx_power_dbm,
             lora_dbm_to_mw((float)c->tx_power_dbm),
             c->pa_pin == LORA_PA_BOOST ? "PA_BOOST" : "RFO", ramp_us[c->pa_ramp], c->ocp_ma);
    ESP_LOGI(TAG, "LNA           : %s, boost HF %s", c->lna_gain == LORA_LNA_AUTO ? "AGC" : "manual",
             c->lna_boost_hf ? "on" : "off");
    ESP_LOGI(TAG, "LDRO          : %s", ldro_active(c) ? "aktif" : "mati");
    ESP_LOGI(TAG, "Durasi simbol : %.2f ms", lora_get_symbol_time_us(r) / 1000.0f);
    ESP_LOGI(TAG, "Bitrate       : %.1f bps", lora_get_bitrate_bps(r));
    ESP_LOGI(TAG, "Airtime 20 B  : %u ms", (unsigned)lora_get_airtime_ms(r, 20));
}

uint8_t lora_read_reg(lora_t *r, uint8_t reg)
{
    if (!r) return 0;
    xSemaphoreTake(r->lock, portMAX_DELAY);
    uint8_t v = rd(r, reg);
    xSemaphoreGive(r->lock);
    return v;
}

void lora_write_reg(lora_t *r, uint8_t reg, uint8_t value)
{
    if (!r) return;
    xSemaphoreTake(r->lock, portMAX_DELAY);
    wr(r, reg, value);
    xSemaphoreGive(r->lock);
}

/* ------------------------------------------------------------------------ */
/*  Setter                                                                  */
/* ------------------------------------------------------------------------ */
esp_err_t lora_set_frequency(lora_t *r, uint32_t hz)
{
    if (!r || hz < 137000000UL || hz > 1020000000UL) return ESP_ERR_INVALID_ARG;
    bool rx = cfg_begin(r);
    r->cfg.frequency_hz = hz;
    apply_frequency(r);
    apply_modem(r);                 /* errata & offset RSSI bergantung pada frekuensi */
    cfg_end(r, rx);
    return ESP_OK;
}

esp_err_t lora_set_sf(lora_t *r, lora_sf_t sf)
{
    if (!r || sf < LORA_SF6 || sf > LORA_SF12) return ESP_ERR_INVALID_ARG;
    bool rx = cfg_begin(r);
    r->cfg.sf = sf;
    apply_modem(r);
    cfg_end(r, rx);
    return ESP_OK;
}

esp_err_t lora_set_bw(lora_t *r, lora_bw_t bw)
{
    if (!r || bw < LORA_BW_7K8 || bw > LORA_BW_500K) return ESP_ERR_INVALID_ARG;
    bool rx = cfg_begin(r);
    r->cfg.bw = bw;
    apply_modem(r);
    cfg_end(r, rx);
    return ESP_OK;
}

esp_err_t lora_set_cr(lora_t *r, lora_cr_t cr)
{
    if (!r || cr < LORA_CR_4_5 || cr > LORA_CR_4_8) return ESP_ERR_INVALID_ARG;
    bool rx = cfg_begin(r);
    r->cfg.cr = cr;
    apply_modem(r);
    cfg_end(r, rx);
    return ESP_OK;
}

esp_err_t lora_set_preamble_len(lora_t *r, uint16_t symbols)
{
    if (!r || symbols < 6) return ESP_ERR_INVALID_ARG;
    bool rx = cfg_begin(r);
    r->cfg.preamble_len = symbols;
    apply_preamble(r);
    cfg_end(r, rx);
    return ESP_OK;
}

esp_err_t lora_set_sync_word(lora_t *r, uint8_t sync_word)
{
    if (!r) return ESP_ERR_INVALID_ARG;
    bool rx = cfg_begin(r);
    r->cfg.sync_word = sync_word;
    apply_sync_word(r);
    cfg_end(r, rx);
    return ESP_OK;
}

esp_err_t lora_set_crc(lora_t *r, bool enable)
{
    if (!r) return ESP_ERR_INVALID_ARG;
    bool rx = cfg_begin(r);
    r->cfg.crc = enable;
    apply_modem(r);
    cfg_end(r, rx);
    return ESP_OK;
}

esp_err_t lora_set_header_mode(lora_t *r, lora_header_t mode, uint8_t implicit_payload_len)
{
    if (!r) return ESP_ERR_INVALID_ARG;
    if (mode == LORA_HEADER_IMPLICIT && implicit_payload_len == 0) return ESP_ERR_INVALID_ARG;
    if (mode == LORA_HEADER_EXPLICIT && r->cfg.sf == LORA_SF6) return ESP_ERR_INVALID_STATE;
    bool rx = cfg_begin(r);
    r->cfg.header_mode = mode;
    if (mode == LORA_HEADER_IMPLICIT) r->cfg.implicit_payload_len = implicit_payload_len;
    apply_modem(r);
    cfg_end(r, rx);
    return ESP_OK;
}

esp_err_t lora_set_invert_iq(lora_t *r, bool invert)
{
    if (!r) return ESP_ERR_INVALID_ARG;
    bool rx = cfg_begin(r);
    r->cfg.invert_iq = invert;
    apply_iq(r);
    cfg_end(r, rx);
    return ESP_OK;
}

esp_err_t lora_set_tx_power(lora_t *r, int8_t dbm)
{
    if (!r) return ESP_ERR_INVALID_ARG;
    bool rx = cfg_begin(r);
    r->cfg.tx_power_dbm = dbm;
    apply_pa(r);
    cfg_end(r, rx);
    return ESP_OK;
}

float lora_dbm_to_mw(float dbm)
{
    return powf(10.0f, dbm / 10.0f);
}

int8_t lora_mw_to_dbm(float mw)
{
    if (mw <= 0.0f) return -100;
    long v = lroundf(10.0f * log10f(mw));
    if (v < -100) v = -100;
    if (v > 100)  v = 100;
    return (int8_t)v;
}

esp_err_t lora_set_tx_power_mw(lora_t *r, float mw)
{
    if (!r || !(mw > 0.0f)) return ESP_ERR_INVALID_ARG;
    return lora_set_tx_power(r, lora_mw_to_dbm(mw));
}

esp_err_t lora_set_pa_pin(lora_t *r, lora_pa_pin_t pin)
{
    if (!r || (pin != LORA_PA_BOOST && pin != LORA_PA_RFO)) return ESP_ERR_INVALID_ARG;
    bool rx = cfg_begin(r);
    r->cfg.pa_pin = pin;
    apply_pa(r);
    cfg_end(r, rx);
    return ESP_OK;
}

esp_err_t lora_set_pa_ramp(lora_t *r, lora_pa_ramp_t ramp)
{
    if (!r || ramp > LORA_PA_RAMP_10US) return ESP_ERR_INVALID_ARG;
    bool rx = cfg_begin(r);
    r->cfg.pa_ramp = ramp;
    apply_pa_ramp(r);
    cfg_end(r, rx);
    return ESP_OK;
}

esp_err_t lora_set_ocp(lora_t *r, uint16_t ma)
{
    if (!r) return ESP_ERR_INVALID_ARG;
    bool rx = cfg_begin(r);
    r->cfg.ocp_ma = ma;
    apply_pa(r);
    cfg_end(r, rx);
    return ESP_OK;
}

esp_err_t lora_set_lna_gain(lora_t *r, lora_lna_gain_t gain)
{
    if (!r || gain > LORA_LNA_G6) return ESP_ERR_INVALID_ARG;
    bool rx = cfg_begin(r);
    r->cfg.lna_gain = gain;
    apply_lna(r);
    apply_modem(r);                 /* bit AGC ada di MODEM_CFG3 */
    cfg_end(r, rx);
    return ESP_OK;
}

esp_err_t lora_set_lna_boost_hf(lora_t *r, bool enable)
{
    if (!r) return ESP_ERR_INVALID_ARG;
    bool rx = cfg_begin(r);
    r->cfg.lna_boost_hf = enable;
    apply_lna(r);
    cfg_end(r, rx);
    return ESP_OK;
}

esp_err_t lora_set_ldro(lora_t *r, lora_ldro_t mode)
{
    if (!r || mode > LORA_LDRO_ON) return ESP_ERR_INVALID_ARG;
    bool rx = cfg_begin(r);
    r->cfg.ldro = mode;
    apply_modem(r);
    cfg_end(r, rx);
    return ESP_OK;
}

esp_err_t lora_set_rx_timeout_symbols(lora_t *r, uint16_t symbols)
{
    if (!r || symbols < 4 || symbols > 1023) return ESP_ERR_INVALID_ARG;
    bool rx = cfg_begin(r);
    r->cfg.rx_timeout_symbols = symbols;
    apply_modem(r);
    cfg_end(r, rx);
    return ESP_OK;
}

esp_err_t lora_apply_config(lora_t *r, const lora_config_t *cfg)
{
    if (!r || !cfg) return ESP_ERR_INVALID_ARG;
    bool rx = cfg_begin(r);
    /* salin hanya bagian radio; pin dan SPI tidak diubah */
    lora_config_t keep = r->cfg;
    r->cfg = *cfg;
    r->cfg.spi_host     = keep.spi_host;
    r->cfg.pin_mosi     = keep.pin_mosi;
    r->cfg.pin_miso     = keep.pin_miso;
    r->cfg.pin_sck      = keep.pin_sck;
    r->cfg.pin_nss      = keep.pin_nss;
    r->cfg.pin_rst      = keep.pin_rst;
    r->cfg.pin_dio0     = keep.pin_dio0;
    r->cfg.spi_clock_hz = keep.spi_clock_hz;
    r->cfg.init_spi_bus = keep.init_spi_bus;
    apply_all(r);
    cfg_end(r, rx);
    return ESP_OK;
}

/* ------------------------------------------------------------------------ */
/*  Kirim                                                                   */
/* ------------------------------------------------------------------------ */
esp_err_t lora_send(lora_t *r, const void *data, size_t len, uint32_t timeout_ms)
{
    if (!r || (!data && len > 0)) return ESP_ERR_INVALID_ARG;
    if (len == 0 || len > LORA_MAX_PAYLOAD) return ESP_ERR_INVALID_SIZE;

    xSemaphoreTake(r->lock, portMAX_DELAY);
    if (r->cfg.header_mode == LORA_HEADER_IMPLICIT && len != r->cfg.implicit_payload_len) {
        xSemaphoreGive(r->lock);
        return ESP_ERR_INVALID_SIZE;    /* mode implicit: panjang harus tetap */
    }
    if (timeout_ms == 0) timeout_ms = lora_get_airtime_ms(r, len) + 1500;

    esp_err_t err = transmit(r, (const uint8_t *)data, len, timeout_ms);
    if (r->rx_active) enter_rx(r);
    xSemaphoreGive(r->lock);
    return err;
}

esp_err_t lora_send_string(lora_t *r, const char *str)
{
    if (!str) return ESP_ERR_INVALID_ARG;
    return lora_send(r, str, strlen(str), 0);
}

esp_err_t lora_printf(lora_t *r, const char *fmt, ...)
{
    char buf[LORA_MAX_PAYLOAD + 1];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n <= 0) return ESP_ERR_INVALID_SIZE;
    if (n > LORA_MAX_PAYLOAD) n = LORA_MAX_PAYLOAD;
    return lora_send(r, buf, (size_t)n, 0);
}

/* ------------------------------------------------------------------------ */
/*  Terima                                                                  */
/* ------------------------------------------------------------------------ */
esp_err_t lora_receive(lora_t *r, lora_packet_t *pkt, uint32_t timeout_ms)
{
    if (!r || !pkt) return ESP_ERR_INVALID_ARG;
    xSemaphoreTake(r->lock, portMAX_DELAY);
    if (r->rx_active) {
        xSemaphoreGive(r->lock);
        return ESP_ERR_INVALID_STATE;
    }
    enter_rx(r);
    uint8_t f = wait_irq(r, IRQ_RX_DONE, timeout_ms);
    esp_err_t err;
    if (!f) {
        err = ESP_ERR_TIMEOUT;
    } else {
        err = read_packet(r, f, pkt);
    }
    set_mode(r, MODE_STANDBY);
    xSemaphoreGive(r->lock);
    return err;
}

esp_err_t lora_receive_single(lora_t *r, lora_packet_t *pkt)
{
    if (!r || !pkt) return ESP_ERR_INVALID_ARG;
    xSemaphoreTake(r->lock, portMAX_DELAY);
    if (r->rx_active) {
        xSemaphoreGive(r->lock);
        return ESP_ERR_INVALID_STATE;
    }
    wr(r, REG_FIFO_ADDR_PTR, 0x00);
    wr(r, REG_DIO_MAP1, DIO0_RX_DONE);
    wr(r, REG_IRQ_FLAGS, 0xFF);
    set_mode(r, MODE_RX_SINGLE);

    uint32_t wait_ms = (uint32_t)((float)r->cfg.rx_timeout_symbols * lora_get_symbol_time_us(r) / 1000.0f) + 500;
    uint8_t f = wait_irq(r, IRQ_RX_DONE | IRQ_RX_TIMEOUT, wait_ms);
    esp_err_t err;
    if (f & IRQ_RX_DONE) {
        err = read_packet(r, f, pkt);
    } else {
        wr(r, REG_IRQ_FLAGS, 0xFF);
        err = ESP_ERR_TIMEOUT;
    }
    set_mode(r, MODE_STANDBY);
    xSemaphoreGive(r->lock);
    return err;
}

static void rx_task(void *arg)
{
    lora_t *r = (lora_t *)arg;
    lora_packet_t pkt;

    while (!r->rx_stop) {
        if (r->cfg.pin_dio0 >= 0) xSemaphoreTake(r->irq_sem, ticks_5ms() * 4);
        else                      vTaskDelay(1);

        bool got = false;
        xSemaphoreTake(r->lock, portMAX_DELAY);
        if (!r->rx_stop) {
            uint8_t f = rd(r, REG_IRQ_FLAGS);
            if (f & IRQ_RX_DONE) {
                got = (read_packet(r, f, &pkt) == ESP_OK);
            }
        }
        xSemaphoreGive(r->lock);

        if (got && r->rx_cb) r->rx_cb(r, &pkt, r->rx_ctx);
    }

    xSemaphoreTake(r->lock, portMAX_DELAY);
    set_mode(r, MODE_STANDBY);
    r->rx_active = false;
    xSemaphoreGive(r->lock);
    xSemaphoreGive(r->rx_exit_sem);
    vTaskDelete(NULL);
}

esp_err_t lora_start_receive(lora_t *r, lora_rx_cb_t cb, void *user_ctx)
{
    if (!r || !cb) return ESP_ERR_INVALID_ARG;
    xSemaphoreTake(r->lock, portMAX_DELAY);
    if (r->rx_active) {
        xSemaphoreGive(r->lock);
        return ESP_ERR_INVALID_STATE;
    }
    xSemaphoreTake(r->rx_exit_sem, 0);          /* kosongkan sisa sinyal lama */
    r->rx_cb   = cb;
    r->rx_ctx  = user_ctx;
    r->rx_stop = false;
    enter_rx(r);
    r->rx_active = true;
    BaseType_t ok = xTaskCreate(rx_task, "lora_rx", 4096, r, 5, &r->rx_task);
    if (ok != pdPASS) {
        r->rx_active = false;
        set_mode(r, MODE_STANDBY);
        xSemaphoreGive(r->lock);
        return ESP_ERR_NO_MEM;
    }
    xSemaphoreGive(r->lock);
    return ESP_OK;
}

esp_err_t lora_stop_receive(lora_t *r)
{
    if (!r) return ESP_ERR_INVALID_ARG;
    if (!r->rx_active) return ESP_OK;

    r->rx_stop = true;
    xSemaphoreGive(r->irq_sem);
    if (xTaskGetCurrentTaskHandle() == r->rx_task) {
        return ESP_OK;                           /* dipanggil dari callback: berhenti setelah callback selesai */
    }
    xSemaphoreTake(r->rx_exit_sem, portMAX_DELAY);
    return ESP_OK;
}

/* ------------------------------------------------------------------------ */
/*  Deteksi kanal, daya, status                                             */
/* ------------------------------------------------------------------------ */
esp_err_t lora_channel_busy(lora_t *r, bool *detected)
{
    if (!r || !detected) return ESP_ERR_INVALID_ARG;
    xSemaphoreTake(r->lock, portMAX_DELAY);
    if (r->rx_active) {
        xSemaphoreGive(r->lock);
        return ESP_ERR_INVALID_STATE;
    }
    wr(r, REG_DIO_MAP1, DIO0_CAD_DONE);
    wr(r, REG_IRQ_FLAGS, 0xFF);
    set_mode(r, MODE_CAD);
    uint32_t wait_ms = (uint32_t)(lora_get_symbol_time_us(r) * 4.0f / 1000.0f) + 100;
    uint8_t f = wait_irq(r, IRQ_CAD_DONE, wait_ms);
    wr(r, REG_IRQ_FLAGS, 0xFF);
    set_mode(r, MODE_STANDBY);
    wr(r, REG_DIO_MAP1, DIO0_RX_DONE);
    xSemaphoreGive(r->lock);
    if (!f) return ESP_ERR_TIMEOUT;
    *detected = (f & IRQ_CAD_DETECTED) != 0;
    return ESP_OK;
}

esp_err_t lora_get_channel_rssi(lora_t *r, int *rssi_dbm)
{
    if (!r || !rssi_dbm) return ESP_ERR_INVALID_ARG;
    xSemaphoreTake(r->lock, portMAX_DELAY);
    bool was_rx = r->rx_active;
    if (!was_rx) enter_rx(r);
    vTaskDelay(ticks_5ms());
    *rssi_dbm = (int)rd(r, REG_RSSI) - rssi_offset(&r->cfg);
    if (!was_rx) set_mode(r, MODE_STANDBY);
    xSemaphoreGive(r->lock);
    return ESP_OK;
}

esp_err_t lora_scan(lora_t *r, uint32_t start_hz, uint32_t stop_hz, uint32_t step_hz,
                    int samples, lora_scan_result_t *results, size_t max_results,
                    size_t *out_count)
{
    if (!r || !results || !out_count || step_hz == 0 || stop_hz < start_hz ||
        start_hz < 137000000UL || stop_hz > 1020000000UL || samples < 1) {
        return ESP_ERR_INVALID_ARG;
    }
    if (samples > 200) samples = 200;

    uint64_t needed = ((uint64_t)(stop_hz - start_hz) / step_hz) + 1;
    if (needed > max_results) return ESP_ERR_INVALID_SIZE;

    xSemaphoreTake(r->lock, portMAX_DELAY);
    bool     was_rx = r->rx_active;
    uint32_t orig   = r->cfg.frequency_hz;
    size_t   n      = 0;

    set_mode(r, MODE_STANDBY);
    for (uint64_t f = start_hz; f <= stop_hz; f += step_hz) {
        r->cfg.frequency_hz = (uint32_t)f;
        apply_frequency(r);
        apply_modem(r);
        enter_rx(r);
        vTaskDelay(ticks_5ms());                 /* beri waktu penerima stabil */

        int peak = -200;
        long sum = 0;
        for (int i = 0; i < samples; i++) {
            int v = (int)rd(r, REG_RSSI) - rssi_offset(&r->cfg);
            if (v > peak) peak = v;
            sum += v;
            vTaskDelay(1);
        }
        set_mode(r, MODE_STANDBY);

        results[n].frequency_hz  = (uint32_t)f;
        results[n].rssi_peak_dbm = peak;
        results[n].rssi_avg_dbm  = (int)(sum / samples);
        n++;
    }

    r->cfg.frequency_hz = orig;                  /* kembalikan frekuensi semula */
    apply_frequency(r);
    apply_modem(r);
    if (was_rx) enter_rx(r);
    xSemaphoreGive(r->lock);

    *out_count = n;
    return ESP_OK;
}

esp_err_t lora_scan_print(lora_t *r, uint32_t start_hz, uint32_t stop_hz,
                          uint32_t step_hz, uint32_t *best_hz)
{
    if (!r || step_hz == 0 || stop_hz < start_hz) return ESP_ERR_INVALID_ARG;
    size_t cap = ((uint64_t)(stop_hz - start_hz) / step_hz) + 1;
    lora_scan_result_t *res = malloc(cap * sizeof(*res));
    if (!res) return ESP_ERR_NO_MEM;

    size_t n = 0;
    esp_err_t err = lora_scan(r, start_hz, stop_hz, step_hz, 20, res, cap, &n);
    if (err != ESP_OK) {
        free(res);
        return err;
    }

    size_t best = 0;
    ESP_LOGI(TAG, "---- Hasil scan kanal ----");
    for (size_t i = 0; i < n; i++) {
        const char *state = (res[i].rssi_peak_dbm <= -105) ? "sepi"
                          : (res[i].rssi_peak_dbm <= -90)  ? "agak ramai"
                                                           : "RAMAI";
        ESP_LOGI(TAG, "%8.3f MHz  puncak %4d dBm  rata-rata %4d dBm  %s",
                 res[i].frequency_hz / 1e6, res[i].rssi_peak_dbm, res[i].rssi_avg_dbm, state);
        if (res[i].rssi_peak_dbm < res[best].rssi_peak_dbm) best = i;
    }
    if (n > 0) {
        ESP_LOGI(TAG, "Kanal paling sepi: %.3f MHz (puncak %d dBm)",
                 res[best].frequency_hz / 1e6, res[best].rssi_peak_dbm);
        if (best_hz) *best_hz = res[best].frequency_hz;
    }
    free(res);
    return ESP_OK;
}

esp_err_t lora_sleep(lora_t *r)
{
    if (!r) return ESP_ERR_INVALID_ARG;
    xSemaphoreTake(r->lock, portMAX_DELAY);
    if (r->rx_active) {
        xSemaphoreGive(r->lock);
        return ESP_ERR_INVALID_STATE;
    }
    set_mode(r, MODE_SLEEP);
    xSemaphoreGive(r->lock);
    return ESP_OK;
}

esp_err_t lora_standby(lora_t *r)
{
    if (!r) return ESP_ERR_INVALID_ARG;
    xSemaphoreTake(r->lock, portMAX_DELAY);
    if (r->rx_active) {
        xSemaphoreGive(r->lock);
        return ESP_ERR_INVALID_STATE;
    }
    set_mode(r, MODE_STANDBY);
    xSemaphoreGive(r->lock);
    return ESP_OK;
}

/* ------------------------------------------------------------------------ */
/*  Mulai / selesai                                                         */
/* ------------------------------------------------------------------------ */
static void destroy(lora_t *r)
{
    if (!r) return;
    if (r->isr_added) gpio_isr_handler_remove((gpio_num_t)r->cfg.pin_dio0);
    if (r->spi) spi_bus_remove_device(r->spi);
    if (r->spi_bus_owned) spi_bus_free(r->cfg.spi_host);
    if (r->lock) vSemaphoreDelete(r->lock);
    if (r->irq_sem) vSemaphoreDelete(r->irq_sem);
    if (r->rx_exit_sem) vSemaphoreDelete(r->rx_exit_sem);
    heap_caps_free(r);
}

esp_err_t lora_begin(const lora_config_t *cfg, lora_t **out)
{
    if (!cfg || !out) return ESP_ERR_INVALID_ARG;
    if (cfg->sf < LORA_SF6 || cfg->sf > LORA_SF12 ||
        cfg->bw > LORA_BW_500K ||
        cfg->cr < LORA_CR_4_5 || cfg->cr > LORA_CR_4_8 ||
        cfg->frequency_hz < 137000000UL || cfg->frequency_hz > 1020000000UL ||
        cfg->preamble_len < 6 || cfg->spi_clock_hz <= 0 || cfg->spi_clock_hz > 10000000) {
        return ESP_ERR_INVALID_ARG;
    }
    if (cfg->header_mode == LORA_HEADER_IMPLICIT && cfg->implicit_payload_len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    lora_t *r = heap_caps_calloc(1, sizeof(lora_t), MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
    if (!r) return ESP_ERR_NO_MEM;
    r->cfg = *cfg;

    r->lock        = xSemaphoreCreateMutex();
    r->irq_sem     = xSemaphoreCreateBinary();
    r->rx_exit_sem = xSemaphoreCreateBinary();
    if (!r->lock || !r->irq_sem || !r->rx_exit_sem) {
        destroy(r);
        return ESP_ERR_NO_MEM;
    }

    esp_err_t err;

    /* --- SPI --- */
    if (cfg->init_spi_bus) {
        spi_bus_config_t bus = {
            .mosi_io_num   = cfg->pin_mosi,
            .miso_io_num   = cfg->pin_miso,
            .sclk_io_num   = cfg->pin_sck,
            .data2_io_num  = -1,
            .data3_io_num  = -1,
            .max_transfer_sz = LORA_MAX_PAYLOAD + 1,
        };
        err = spi_bus_initialize(cfg->spi_host, &bus, SPI_DMA_CH_AUTO);
        if (err == ESP_OK) {
            r->spi_bus_owned = true;
        } else if (err != ESP_ERR_INVALID_STATE) {   /* INVALID_STATE = bus sudah ada, tidak masalah */
            ESP_LOGE(TAG, "spi_bus_initialize gagal: %s", esp_err_to_name(err));
            destroy(r);
            return err;
        }
    }
    spi_device_interface_config_t dev = {
        .clock_speed_hz = cfg->spi_clock_hz,
        .mode           = 0,
        .spics_io_num   = cfg->pin_nss,
        .queue_size     = 1,
    };
    err = spi_bus_add_device(cfg->spi_host, &dev, &r->spi);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_add_device gagal: %s", esp_err_to_name(err));
        r->spi = NULL;
        destroy(r);
        return err;
    }

    /* --- Reset --- */
    if (cfg->pin_rst >= 0) {
        gpio_config_t io = {
            .pin_bit_mask = 1ULL << cfg->pin_rst,
            .mode         = GPIO_MODE_OUTPUT,
        };
        gpio_config(&io);
        gpio_set_level((gpio_num_t)cfg->pin_rst, 0);
        vTaskDelay(pdMS_TO_TICKS(10));
        gpio_set_level((gpio_num_t)cfg->pin_rst, 1);
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    /* --- Cek chip --- */
    uint8_t ver = rd(r, REG_VERSION);
    if (ver != 0x12) {
        ESP_LOGE(TAG, "Chip SX127x tidak terdeteksi (versi 0x%02X, seharusnya 0x12). Cek wiring, NSS, dan catu 3,3 V.", ver);
        destroy(r);
        return ESP_ERR_NOT_FOUND;
    }

    /* --- DIO0 --- */
    if (cfg->pin_dio0 >= 0) {
        gpio_config_t io = {
            .pin_bit_mask = 1ULL << cfg->pin_dio0,
            .mode         = GPIO_MODE_INPUT,
            .pull_down_en = GPIO_PULLDOWN_ENABLE,
            .intr_type    = GPIO_INTR_POSEDGE,
        };
        gpio_config(&io);
        err = gpio_install_isr_service(0);
        if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
            destroy(r);
            return err;
        }
        err = gpio_isr_handler_add((gpio_num_t)cfg->pin_dio0, dio0_isr, r);
        if (err != ESP_OK) {
            destroy(r);
            return err;
        }
        r->isr_added = true;
    }

    /* --- Masuk mode LoRa (harus lewat sleep) --- */
    wr(r, REG_OP_MODE, MODE_SLEEP);
    vTaskDelay(pdMS_TO_TICKS(10));
    wr(r, REG_OP_MODE, OPMODE_LORA | MODE_SLEEP);
    vTaskDelay(pdMS_TO_TICKS(10));

    r->reg3a_default = rd(r, REG_ERRATA_3A);
    apply_all(r);
    set_mode(r, MODE_STANDBY);

    ESP_LOGI(TAG, "SX127x siap (%.3f MHz, SF%d, BW %u Hz)", cfg->frequency_hz / 1e6, r->cfg.sf,
             (unsigned)BW_HZ[r->cfg.bw]);
    *out = r;
    return ESP_OK;
}

esp_err_t lora_end(lora_t *r)
{
    if (!r) return ESP_ERR_INVALID_ARG;
    lora_stop_receive(r);
    xSemaphoreTake(r->lock, portMAX_DELAY);
    set_mode(r, MODE_SLEEP);
    xSemaphoreGive(r->lock);
    destroy(r);
    return ESP_OK;
}