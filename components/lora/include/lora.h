/*
 * lora.h - Library LoRa sederhana untuk SX1276/77/78/79 (termasuk modul RA-02)
 *          untuk ESP-IDF (v5.x dan v6.x).
 *
 * Cara pakai paling singkat:
 *
 *     lora_config_t cfg = LORA_CONFIG_DEFAULT(PIN_MOSI, PIN_MISO, PIN_SCK, PIN_NSS, PIN_RST, PIN_DIO0);
 *     lora_t *radio;
 *     ESP_ERROR_CHECK(lora_begin(&cfg, &radio));
 *
 *     lora_send_string(radio, "halo");              // kirim
 *
 *     lora_packet_t pkt;
 *     if (lora_receive(radio, &pkt, 5000) == ESP_OK) // terima (tunggu 5 detik)
 *         printf("%s (RSSI %d)\n", (char *)pkt.data, pkt.rssi);
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------------ */
/*  Konstanta                                                               */
/* ------------------------------------------------------------------------ */

#define LORA_MAX_PAYLOAD   255            /* ukuran payload maksimum (byte)   */
#define LORA_WAIT_FOREVER  0xFFFFFFFFu    /* timeout tak terbatas             */
#define LORA_PIN_UNUSED    (-1)           /* pin RST / DIO0 tidak dipakai     */

/* ------------------------------------------------------------------------ */
/*  Enum parameter                                                          */
/* ------------------------------------------------------------------------ */

/** Spreading factor. SF6 khusus: header otomatis implicit. */
typedef enum {
    LORA_SF6  = 6,
    LORA_SF7  = 7,
    LORA_SF8  = 8,
    LORA_SF9  = 9,
    LORA_SF10 = 10,
    LORA_SF11 = 11,
    LORA_SF12 = 12,
} lora_sf_t;

/** Bandwidth. Nilai enum = kode register. */
typedef enum {
    LORA_BW_7K8   = 0,
    LORA_BW_10K4  = 1,
    LORA_BW_15K6  = 2,
    LORA_BW_20K8  = 3,
    LORA_BW_31K25 = 4,
    LORA_BW_41K7  = 5,
    LORA_BW_62K5  = 6,
    LORA_BW_125K  = 7,
    LORA_BW_250K  = 8,
    LORA_BW_500K  = 9,
} lora_bw_t;

/** Coding rate. */
typedef enum {
    LORA_CR_4_5 = 1,
    LORA_CR_4_6 = 2,
    LORA_CR_4_7 = 3,
    LORA_CR_4_8 = 4,
} lora_cr_t;

/** Mode header. Implicit: panjang payload, CR, dan CRC tidak dikirim. */
typedef enum {
    LORA_HEADER_EXPLICIT = 0,
    LORA_HEADER_IMPLICIT = 1,
} lora_header_t;

/** Pin output PA. RA-02 memakai PA_BOOST. */
typedef enum {
    LORA_PA_BOOST = 0,   /* daya 2..17 dBm (18..20 dBm dengan PA DAC) */
    LORA_PA_RFO   = 1,   /* daya 0..14 dBm                            */
} lora_pa_pin_t;

/** Waktu ramp-up PA. */
typedef enum {
    LORA_PA_RAMP_3400US = 0,
    LORA_PA_RAMP_2000US = 1,
    LORA_PA_RAMP_1000US = 2,
    LORA_PA_RAMP_500US  = 3,
    LORA_PA_RAMP_250US  = 4,
    LORA_PA_RAMP_125US  = 5,
    LORA_PA_RAMP_100US  = 6,
    LORA_PA_RAMP_62US   = 7,
    LORA_PA_RAMP_50US   = 8,
    LORA_PA_RAMP_40US   = 9,   /* default chip */
    LORA_PA_RAMP_31US   = 10,
    LORA_PA_RAMP_25US   = 11,
    LORA_PA_RAMP_20US   = 12,
    LORA_PA_RAMP_15US   = 13,
    LORA_PA_RAMP_12US   = 14,
    LORA_PA_RAMP_10US   = 15,
} lora_pa_ramp_t;

/** Gain LNA. AUTO = AGC aktif (disarankan). G1 = gain maksimum, G6 = minimum. */
typedef enum {
    LORA_LNA_AUTO = 0,
    LORA_LNA_G1   = 1,
    LORA_LNA_G2   = 2,
    LORA_LNA_G3   = 3,
    LORA_LNA_G4   = 4,
    LORA_LNA_G5   = 5,
    LORA_LNA_G6   = 6,
} lora_lna_gain_t;

/** Low Data Rate Optimize. AUTO: aktif otomatis bila durasi simbol > 16 ms. */
typedef enum {
    LORA_LDRO_AUTO = 0,
    LORA_LDRO_OFF  = 1,
    LORA_LDRO_ON   = 2,
} lora_ldro_t;

/* ------------------------------------------------------------------------ */
/*  Struktur                                                                */
/* ------------------------------------------------------------------------ */

/** Handle radio (opaque). */
typedef struct lora_t lora_t;

/** Paket yang diterima. data selalu diakhiri '\0' sehingga aman dicetak. */
typedef struct {
    uint8_t  data[LORA_MAX_PAYLOAD + 1];
    uint8_t  len;              /* jumlah byte payload                      */
    int16_t  rssi;             /* RSSI paket (dBm)                         */
    float    snr;              /* SNR paket (dB)                           */
    int32_t  freq_error_hz;    /* selisih frekuensi TX vs RX (Hz)          */
    bool     crc_ok;           /* false hanya jika keep_crc_error_packets  */
    int64_t  timestamp_us;     /* esp_timer_get_time() saat diterima       */
} lora_packet_t;

/** Callback penerimaan (dijalankan di task internal; boleh memanggil lora_send). */
typedef void (*lora_rx_cb_t)(lora_t *radio, const lora_packet_t *pkt, void *user_ctx);

/** Hasil pemindaian satu kanal (lihat lora_scan). */
typedef struct {
    uint32_t frequency_hz;     /* frekuensi kanal yang diukur              */
    int      rssi_peak_dbm;    /* RSSI tertinggi selama pengukuran (dBm)   */
    int      rssi_avg_dbm;     /* RSSI rata-rata selama pengukuran (dBm)   */
} lora_scan_result_t;

/** Seluruh konfigurasi. Isi dengan LORA_CONFIG_DEFAULT lalu ubah yang perlu. */
typedef struct {
    /* --- hardware ------------------------------------------------------ */
    spi_host_device_t spi_host;      /* SPI2_HOST dst.                        */
    int      pin_mosi;
    int      pin_miso;
    int      pin_sck;
    int      pin_nss;                /* chip select                           */
    int      pin_rst;                /* LORA_PIN_UNUSED bila tidak disambung  */
    int      pin_dio0;               /* LORA_PIN_UNUSED = mode polling        */
    int      spi_clock_hz;           /* maks 10 MHz                           */
    bool     init_spi_bus;           /* false bila bus SPI sudah diinisialisasi */

    /* --- radio (harus sama di TX dan RX) ------------------------------- */
    uint32_t        frequency_hz;    /* mis. 433000000                        */
    lora_sf_t       sf;
    lora_bw_t       bw;
    lora_cr_t       cr;
    uint16_t        preamble_len;    /* jumlah simbol preamble (6..65535)     */
    uint8_t         sync_word;       /* 0x12 privat, 0x34 LoRaWAN             */
    bool            crc;             /* CRC payload                           */
    lora_header_t   header_mode;
    uint8_t         implicit_payload_len; /* dipakai bila header implicit     */
    bool            invert_iq;

    /* --- pemancar ------------------------------------------------------ */
    int8_t          tx_power_dbm;
    float           tx_power_mw;     /* alternatif tx_power_dbm: isi SALAH SATU. >0 = pakai mW, 0 = pakai dBm */
    lora_pa_pin_t   pa_pin;
    lora_pa_ramp_t  pa_ramp;
    uint16_t        ocp_ma;          /* batas arus (45..240 mA), 0 = OCP mati */

    /* --- penerima ------------------------------------------------------ */
    lora_lna_gain_t lna_gain;
    bool            lna_boost_hf;    /* hanya efektif di band HF (> 525 MHz)  */
    lora_ldro_t     ldro;
    uint16_t        rx_timeout_symbols;     /* untuk lora_receive_single (4..1023) */
    bool            keep_crc_error_packets; /* teruskan paket dengan CRC salah */
} lora_config_t;

/**
 * Konfigurasi bawaan: 433 MHz, SF9, BW125, CR4/5, preamble 8, sync 0x12,
 * CRC aktif, 14 dBm di PA_BOOST. Urutan field mengikuti struct di atas.
 */
#define LORA_CONFIG_DEFAULT(mosi, miso, sck, nss, rst, dio0) { \
    .spi_host = SPI2_HOST,                                    \
    .pin_mosi = (mosi), .pin_miso = (miso), .pin_sck = (sck), \
    .pin_nss = (nss), .pin_rst = (rst), .pin_dio0 = (dio0),   \
    .spi_clock_hz = 4000000,                                  \
    .init_spi_bus = true,                                     \
    .frequency_hz = 433000000,                                \
    .sf = LORA_SF9,                                           \
    .bw = LORA_BW_125K,                                       \
    .cr = LORA_CR_4_5,                                        \
    .preamble_len = 8,                                        \
    .sync_word = 0x12,                                        \
    .crc = true,                                              \
    .header_mode = LORA_HEADER_EXPLICIT,                      \
    .implicit_payload_len = 16,                               \
    .invert_iq = false,                                       \
    .tx_power_dbm = 14,                                       \
    .tx_power_mw = 0,                                         \
    .pa_pin = LORA_PA_BOOST,                                  \
    .pa_ramp = LORA_PA_RAMP_40US,                             \
    .ocp_ma = 100,                                            \
    .lna_gain = LORA_LNA_AUTO,                                \
    .lna_boost_hf = false,                                    \
    .ldro = LORA_LDRO_AUTO,                                   \
    .rx_timeout_symbols = 100,                                \
    .keep_crc_error_packets = false,                          \
}

/* ------------------------------------------------------------------------ */
/*  Mulai / selesai                                                         */
/* ------------------------------------------------------------------------ */

/** Inisialisasi bus SPI, reset chip, cek versi, dan terapkan semua konfigurasi.
 *  ESP_ERR_NOT_FOUND = chip tidak terdeteksi (cek wiring / catu 3,3 V). */
esp_err_t lora_begin(const lora_config_t *cfg, lora_t **out_radio);

/** Matikan radio dan bebaskan semua resource. */
esp_err_t lora_end(lora_t *radio);

/* ------------------------------------------------------------------------ */
/*  Kirim                                                                   */
/* ------------------------------------------------------------------------ */

/** Kirim data. timeout_ms = 0 berarti otomatis (airtime + 1,5 detik). */
esp_err_t lora_send(lora_t *radio, const void *data, size_t len, uint32_t timeout_ms);

/** Kirim string C (tanpa '\0' di ujung). */
esp_err_t lora_send_string(lora_t *radio, const char *str);

/** Kirim teks berformat, seperti printf. Maksimum 255 karakter. */
esp_err_t lora_printf(lora_t *radio, const char *fmt, ...) __attribute__((format(printf, 2, 3)));

/* ------------------------------------------------------------------------ */
/*  Terima                                                                  */
/* ------------------------------------------------------------------------ */

/** Terima satu paket (blocking). ESP_ERR_TIMEOUT bila tidak ada paket,
 *  ESP_ERR_INVALID_CRC bila CRC salah. timeout_ms boleh LORA_WAIT_FOREVER.
 *  Tidak boleh dipakai saat lora_start_receive aktif. */
esp_err_t lora_receive(lora_t *radio, lora_packet_t *pkt, uint32_t timeout_ms);

/** Terima satu paket dengan mode RX single (timeout = rx_timeout_symbols). */
esp_err_t lora_receive_single(lora_t *radio, lora_packet_t *pkt);

/** Terima terus-menerus di background, panggil callback tiap paket masuk. */
esp_err_t lora_start_receive(lora_t *radio, lora_rx_cb_t cb, void *user_ctx);

/** Hentikan penerimaan background. */
esp_err_t lora_stop_receive(lora_t *radio);

/* ------------------------------------------------------------------------ */
/*  Deteksi kanal, daya, dan status                                         */
/* ------------------------------------------------------------------------ */

/** Channel Activity Detection: *detected = true bila ada preamble LoRa di kanal. */
esp_err_t lora_channel_busy(lora_t *radio, bool *detected);

/** RSSI kanal saat ini (dBm), cocok untuk memeriksa kebisingan. */
esp_err_t lora_get_channel_rssi(lora_t *radio, int *rssi_dbm);

/**
 * Pindai banyak kanal dan ukur kebisingan tiap kanal (untuk memilih frekuensi sepi).
 * Frekuensi dan mode radio dikembalikan seperti semula setelah selesai.
 *
 *   start_hz, stop_hz : rentang frekuensi (Hz), stop ikut diukur
 *   step_hz           : jarak antar kanal (Hz), mis. 200000
 *   samples           : jumlah pembacaan per kanal (1..200), disarankan 20
 *   results           : array hasil milikmu, ukuran max_results
 *   out_count         : diisi jumlah kanal yang terukur
 */
esp_err_t lora_scan(lora_t *radio, uint32_t start_hz, uint32_t stop_hz, uint32_t step_hz,
                    int samples, lora_scan_result_t *results, size_t max_results,
                    size_t *out_count);

/** Pindai lalu cetak tabel hasilnya ke log. best_hz (boleh NULL) diisi kanal paling sepi. */
esp_err_t lora_scan_print(lora_t *radio, uint32_t start_hz, uint32_t stop_hz,
                          uint32_t step_hz, uint32_t *best_hz);

esp_err_t lora_sleep(lora_t *radio);     /* arus terendah, konfigurasi tetap */
esp_err_t lora_standby(lora_t *radio);

/* ------------------------------------------------------------------------ */
/*  Setter (bisa dipanggil kapan saja setelah lora_begin)                   */
/* ------------------------------------------------------------------------ */

esp_err_t lora_set_frequency(lora_t *radio, uint32_t hz);
esp_err_t lora_set_sf(lora_t *radio, lora_sf_t sf);
esp_err_t lora_set_bw(lora_t *radio, lora_bw_t bw);
esp_err_t lora_set_cr(lora_t *radio, lora_cr_t cr);
esp_err_t lora_set_preamble_len(lora_t *radio, uint16_t symbols);
esp_err_t lora_set_sync_word(lora_t *radio, uint8_t sync_word);
esp_err_t lora_set_crc(lora_t *radio, bool enable);
esp_err_t lora_set_header_mode(lora_t *radio, lora_header_t mode, uint8_t implicit_payload_len);
esp_err_t lora_set_invert_iq(lora_t *radio, bool invert);
esp_err_t lora_set_tx_power(lora_t *radio, int8_t dbm);
esp_err_t lora_set_tx_power_mw(lora_t *radio, float mw);   /* sama dengan lora_set_tx_power, satuan mW */
int8_t    lora_mw_to_dbm(float mw);                        /* konversi mW -> dBm (dibulatkan) */
float     lora_dbm_to_mw(float dbm);                       /* konversi dBm -> mW */
esp_err_t lora_set_pa_pin(lora_t *radio, lora_pa_pin_t pin);
esp_err_t lora_set_pa_ramp(lora_t *radio, lora_pa_ramp_t ramp);
esp_err_t lora_set_ocp(lora_t *radio, uint16_t ma);
esp_err_t lora_set_lna_gain(lora_t *radio, lora_lna_gain_t gain);
esp_err_t lora_set_lna_boost_hf(lora_t *radio, bool enable);
esp_err_t lora_set_ldro(lora_t *radio, lora_ldro_t mode);
esp_err_t lora_set_rx_timeout_symbols(lora_t *radio, uint16_t symbols);

/** Terapkan seluruh struct konfigurasi sekaligus (pin hardware diabaikan). */
esp_err_t lora_apply_config(lora_t *radio, const lora_config_t *cfg);

/* ------------------------------------------------------------------------ */
/*  Informasi                                                               */
/* ------------------------------------------------------------------------ */

esp_err_t lora_get_config(lora_t *radio, lora_config_t *out_cfg);
uint32_t  lora_get_airtime_ms(lora_t *radio, size_t payload_len);   /* waktu di udara */
float     lora_get_bitrate_bps(lora_t *radio);
float     lora_get_symbol_time_us(lora_t *radio);
void      lora_print_config(lora_t *radio);                         /* cetak ke log   */

/* Akses register mentah (untuk pengguna mahir). */
uint8_t   lora_read_reg(lora_t *radio, uint8_t reg);
void      lora_write_reg(lora_t *radio, uint8_t reg, uint8_t value);

#ifdef __cplusplus
}
#endif