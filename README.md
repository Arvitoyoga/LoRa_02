# Library LoRa untuk ESP-IDF (SX1276/77/78/79, modul RA-02)

Library ini membuat modul LoRa mudah dipakai. Kamu tidak perlu membaca datasheet atau
mengatur register satu per satu. Cukup isi satu struct konfigurasi, panggil `lora_begin`,
lalu kirim dan terima data dengan satu baris kode.

Mendukung ESP-IDF v5.2 ke atas dan v6.x (memakai komponen `esp_driver_spi` dan `esp_driver_gpio`).

## Daftar isi

1. [Memasang library](#1-memasang-library)
2. [Wiring RA-02 ke ESP32-C3 SuperMini](#2-wiring-ra-02-ke-esp32-c3-supermini)
3. [Konsep dasar yang perlu dipahami](#3-konsep-dasar-yang-perlu-dipahami)
4. [Program pertama: kirim dan terima](#4-program-pertama-kirim-dan-terima)
5. [Struct konfigurasi `lora_config_t`](#5-struct-konfigurasi-lora_config_t)
6. [Memulai dan mengakhiri](#6-memulai-dan-mengakhiri)
7. [Mengirim data](#7-mengirim-data)
8. [Menerima data](#8-menerima-data)
9. [Daya pancar: dBm dan mW](#9-daya-pancar-dbm-dan-mw)
10. [Mengubah pengaturan saat program berjalan](#10-mengubah-pengaturan-saat-program-berjalan)
11. [Memeriksa kanal: CAD, RSSI kanal, dan scan](#11-memeriksa-kanal-cad-rssi-kanal-dan-scan)
12. [Hemat daya: sleep dan standby](#12-hemat-daya-sleep-dan-standby)
13. [Informasi dan perhitungan](#13-informasi-dan-perhitungan)
14. [Akses register mentah](#14-akses-register-mentah)
15. [Kode error dan cara mengatasinya](#15-kode-error-dan-cara-mengatasinya)
16. [Contoh lanjutan: membuat ACK sendiri](#16-contoh-lanjutan-membuat-ack-sendiri)
17. [Tips untuk banyak modul dan venue ramai](#17-tips-untuk-banyak-modul-dan-venue-ramai)
18. [Batasan library](#18-batasan-library)

---

## 1. Memasang library

Struktur folder:

```
components/lora/        <- komponen library (salin folder ini ke proyekmu)
    include/lora.h
    src/lora.c
    CMakeLists.txt
    idf_component.yml
examples/transmitter/   <- contoh pengirim
examples/receiver/      <- contoh penerima
```

Langkah:

1. Salin folder `components/lora` ke folder `components/` di proyek ESP-IDF-mu
   (sejajar dengan folder `main`).
2. Buka `main/CMakeLists.txt` dan tambahkan `REQUIRES lora`:

   ```cmake
   idf_component_register(SRCS "main.c" INCLUDE_DIRS "." REQUIRES lora)
   ```

3. Di `main.c` tulis `#include "lora.h"`.

Untuk ESP32-C3 SuperMini, supaya `printf` muncul di monitor, buka `idf.py menuconfig`, lalu
*Component config → ESP System Settings → Channel for console output* dan pilih
**USB Serial/JTAG**.

---

## 2. Wiring RA-02 ke ESP32-C3 SuperMini

RA-02 hanya boleh diberi **3,3 V**. Jangan pernah memberi 5 V ke pin mana pun.

| RA-02 | ESP32-C3 SuperMini | Keterangan |
|-------|--------------------|------------|
| 3.3V  | 3V3                | Catu daya. Beri kapasitor 100 µF dan 100 nF dekat modul |
| GND   | GND                | Sambungkan semua pin GND modul |
| SCK   | GPIO4              | Clock SPI |
| MISO  | GPIO5              | |
| MOSI  | GPIO6              | |
| NSS   | GPIO7              | Chip select |
| RESET | GPIO3              | |
| DIO0  | GPIO1              | Sinyal selesai kirim/terima |
| ANT   | Antena 433 MHz     | **Pasang sebelum modul dinyalakan** |

Catatan:

- Jangan memakai GPIO2, GPIO8, dan GPIO9 untuk pin ini, karena itu pin strapping dan bisa
  mengganggu proses booting.
- Mengirim (TX) tanpa antena bisa merusak penguat daya di dalam modul. Menerima tanpa antena
  aman, tapi jangkauannya sangat pendek.
- Pin RA-02 berjarak 2 mm, jadi pakai adapter breakout atau solder kabel langsung.

---

## 3. Konsep dasar yang perlu dipahami

**Dua modul hanya bisa saling bicara kalau pengaturan radionya sama.** Parameter berikut
harus identik di pengirim dan penerima:

- frekuensi
- spreading factor (SF)
- bandwidth (BW)
- coding rate (CR)
- sync word
- panjang preamble
- CRC (aktif atau tidak)
- mode header (explicit atau implicit)
- invert IQ

Kalau satu saja berbeda, penerima tidak menerima apa-apa.

Parameter yang **boleh berbeda** antara dua modul: daya pancar, pengaturan LNA, batas arus
(OCP), dan waktu ramp PA.

**Hubungan SF dan BW dengan jarak dan kecepatan:**

| Pengaturan | Jarak | Kecepatan | Airtime paket |
|------------|-------|-----------|---------------|
| SF lebih tinggi (SF12) | lebih jauh | lebih lambat | lebih lama |
| SF lebih rendah (SF7) | lebih dekat | lebih cepat | lebih singkat |
| BW lebih lebar (500 kHz) | sedikit berkurang | lebih cepat | lebih singkat |
| BW lebih sempit (62,5 kHz) | sedikit bertambah | lebih lambat | lebih lama |

Nilai awal yang aman untuk pemula: SF9, BW 125 kHz, CR 4/5. Itu juga nilai bawaan library.

**LoRa tidak punya ACK.** Pengirim hanya tahu paketnya sudah dipancarkan, bukan bahwa ada
yang menerimanya. Kalau butuh kepastian, lihat [bagian 16](#16-contoh-lanjutan-membuat-ack-sendiri).

**Konstanta bawaan library:**

| Nama | Nilai | Fungsi |
|------|-------|--------|
| `LORA_MAX_PAYLOAD` | 255 | Ukuran payload maksimum (byte) |
| `LORA_WAIT_FOREVER` | 0xFFFFFFFF | Dipakai sebagai timeout "tunggu selamanya" |
| `LORA_PIN_UNUSED` | -1 | Dipakai untuk pin RST atau DIO0 yang tidak disambung |

---

## 4. Program pertama: kirim dan terima

### Pengirim

```c
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lora.h"

#define PIN_SCK   4
#define PIN_MISO  5
#define PIN_MOSI  6
#define PIN_NSS   7
#define PIN_RST   3
#define PIN_DIO0  1

void app_main(void)
{
    /* 1. Siapkan konfigurasi bawaan */
    lora_config_t cfg = LORA_CONFIG_DEFAULT(PIN_MOSI, PIN_MISO, PIN_SCK, PIN_NSS, PIN_RST, PIN_DIO0);

    /* 2. Ubah yang perlu (opsional) */
    cfg.tx_power_dbm = 10;

    /* 3. Mulai radio */
    lora_t *radio;
    ESP_ERROR_CHECK(lora_begin(&cfg, &radio));
    lora_print_config(radio);          /* cetak ringkasan pengaturan */

    /* 4. Kirim tiap detik */
    int counter = 0;
    while (1) {
        esp_err_t err = lora_printf(radio, "halo %d", counter++);
        printf("kirim: %s\n", err == ESP_OK ? "ok" : esp_err_to_name(err));
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
```

### Penerima

```c
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lora.h"

#define PIN_SCK   4
#define PIN_MISO  5
#define PIN_MOSI  6
#define PIN_NSS   7
#define PIN_RST   3
#define PIN_DIO0  1

/* Fungsi ini dipanggil otomatis setiap ada paket masuk */
static void on_receive(lora_t *radio, const lora_packet_t *pkt, void *ctx)
{
    (void)radio; (void)ctx;
    printf("terima: \"%s\" (%u byte)  RSSI %d dBm  SNR %.1f dB  FEI %ld Hz\n",
           (const char *)pkt->data, pkt->len, pkt->rssi, pkt->snr, (long)pkt->freq_error_hz);
}

void app_main(void)
{
    lora_config_t cfg = LORA_CONFIG_DEFAULT(PIN_MOSI, PIN_MISO, PIN_SCK, PIN_NSS, PIN_RST, PIN_DIO0);

    lora_t *radio;
    ESP_ERROR_CHECK(lora_begin(&cfg, &radio));

    ESP_ERROR_CHECK(lora_start_receive(radio, on_receive, NULL));   /* terima di background */
    vTaskDelay(portMAX_DELAY);                                      /* biarkan program hidup */
}
```

Arti hasil yang tercetak di penerima:

| Bagian | Arti |
|--------|------|
| `"halo 3"` | Isi paket |
| `(6 byte)` | Panjang payload |
| `RSSI -25 dBm` | Kuat sinyal paket. Makin dekat ke 0, makin kuat. Paket sehat ada di sekitar -30 sampai -120 dBm |
| `SNR 12.0 dB` | Perbandingan sinyal terhadap noise. LoRa masih bisa membaca sampai SNR negatif (sekitar -7,5 dB di SF7 sampai -20 dB di SF12) |
| `FEI 728 Hz` | Selisih frekuensi antara pemancar dan penerima. Nilai kecil (ribuan Hz ke bawah) normal |

---

## 5. Struct konfigurasi `lora_config_t`

Semua pengaturan awal ada di satu struct. Cara memakainya:

```c
lora_config_t cfg = LORA_CONFIG_DEFAULT(PIN_MOSI, PIN_MISO, PIN_SCK, PIN_NSS, PIN_RST, PIN_DIO0);
cfg.sf = LORA_SF10;           /* ubah field yang kamu mau */
cfg.frequency_hz = 433000000;
lora_begin(&cfg, &radio);     /* baru dipakai di sini */
```

Urutan argumen makro: **MOSI, MISO, SCK, NSS, RST, DIO0**.

Penting: perubahan `cfg.` harus dilakukan **sebelum** `lora_begin`. Setelah `lora_begin`,
struct `cfg` sudah disalin ke dalam library, jadi mengubah `cfg` lagi tidak berpengaruh.
Setelah `lora_begin`, pakai fungsi `lora_set_xxx` (lihat bagian 10).

Nama variabel `cfg` bebas diganti. Yang tidak boleh diganti adalah nama tipe dan nama field.

### Field hardware

| Field | Fungsi | Bawaan |
|-------|--------|--------|
| `spi_host` | Peripheral SPI yang dipakai | `SPI2_HOST` |
| `pin_mosi`, `pin_miso`, `pin_sck` | Pin SPI | dari argumen makro |
| `pin_nss` | Pin chip select | dari argumen makro |
| `pin_rst` | Pin reset. Isi `LORA_PIN_UNUSED` kalau tidak disambung | dari argumen makro |
| `pin_dio0` | Pin interrupt. Isi `LORA_PIN_UNUSED` untuk mode polling (tetap jalan, sedikit lebih boros CPU) | dari argumen makro |
| `spi_clock_hz` | Kecepatan SPI, maksimum 10 MHz | 4000000 |
| `init_spi_bus` | `true` kalau library yang menginisialisasi bus SPI. Isi `false` kalau bus SPI sudah kamu inisialisasi sendiri (misalnya dipakai bersama perangkat lain) | `true` |

### Field radio (harus sama di pengirim dan penerima)

| Field | Fungsi | Nilai | Bawaan |
|-------|--------|-------|--------|
| `frequency_hz` | Frekuensi tengah | 137 MHz sampai 1020 MHz (RA-02: sekitar 410 sampai 525 MHz) | 433000000 |
| `sf` | Spreading factor | `LORA_SF6` sampai `LORA_SF12` | `LORA_SF9` |
| `bw` | Bandwidth | `LORA_BW_7K8`, `_10K4`, `_15K6`, `_20K8`, `_31K25`, `_41K7`, `_62K5`, `_125K`, `_250K`, `_500K` | `LORA_BW_125K` |
| `cr` | Coding rate (koreksi error) | `LORA_CR_4_5`, `_4_6`, `_4_7`, `_4_8` | `LORA_CR_4_5` |
| `preamble_len` | Jumlah simbol preamble | 6 sampai 65535 | 8 |
| `sync_word` | Penanda jaringan | 0x00 sampai 0xFF. 0x12 untuk privat, 0x34 untuk LoRaWAN | 0x12 |
| `crc` | CRC payload | `true` / `false` | `true` |
| `header_mode` | Mode header | `LORA_HEADER_EXPLICIT` atau `LORA_HEADER_IMPLICIT` | explicit |
| `implicit_payload_len` | Panjang paket tetap, dipakai hanya di mode implicit | 1 sampai 255 | 16 |
| `invert_iq` | Membalik fase chirp | `true` / `false` | `false` |

Penjelasan singkat:

- **Frekuensi:** harus sesuai dengan modulmu dan aturan setempat. RA-02 adalah modul 433 MHz.
- **SF6:** mode khusus. Header wajib implicit. Library mengubahnya otomatis dan mencetak
  peringatan di log.
- **Coding rate:** 4/5 tercepat, 4/8 paling tahan error tapi paling lambat.
- **Sync word:** dua modul beda sync word akan saling mengabaikan paket. Ini bukan fitur
  keamanan, hanya cara membedakan jaringan.
- **Header explicit:** panjang paket, CR, dan CRC dikirim di dalam paket (lebih mudah).
  **Implicit:** semua itu tidak dikirim, jadi lebih singkat tapi panjang paket harus selalu
  sama di pengirim dan penerima.
- **Invert IQ:** biasanya dimatikan. Dipakai kalau kamu ingin dua kelompok perangkat (misalnya
  uplink dan downlink) tidak saling mendengar.

### Field pemancar

| Field | Fungsi | Nilai | Bawaan |
|-------|--------|-------|--------|
| `tx_power_dbm` | Daya pancar | PA_BOOST 2 sampai 20 dBm, RFO 0 sampai 14 dBm | 14 |
| `tx_power_mw` | Daya pancar dalam mW (alternatif `tx_power_dbm`, pakai salah satu) | PA_BOOST sekitar 1,6 sampai 100 mW, RFO sekitar 1 sampai 25 mW | 0 (tidak dipakai) |
| `pa_pin` | Pin output penguat | `LORA_PA_BOOST` atau `LORA_PA_RFO` | `LORA_PA_BOOST` |
| `pa_ramp` | Waktu naik daya | `LORA_PA_RAMP_3400US` sampai `LORA_PA_RAMP_10US` | `LORA_PA_RAMP_40US` |
| `ocp_ma` | Batas arus pelindung | 45 sampai 240 mA, 0 = mati | 100 |

- **RA-02 memakai PA_BOOST.** Jangan ubah ke RFO kecuali modulmu memang begitu.
- Daya di atas 17 dBm mengaktifkan mode +20 dBm. Library otomatis menaikkan OCP ke minimal
  140 mA. Untuk RA-02 sebaiknya jangan melewati 17 sampai 18 dBm.
- Nilai di luar batas otomatis dipotong ke batas terdekat, dan nilai yang benar-benar dipakai
  disimpan kembali di `tx_power_dbm`.

### Field penerima

| Field | Fungsi | Nilai | Bawaan |
|-------|--------|-------|--------|
| `lna_gain` | Penguatan penerima | `LORA_LNA_AUTO` (AGC) atau `LORA_LNA_G1` (maksimum) sampai `LORA_LNA_G6` (minimum) | `LORA_LNA_AUTO` |
| `lna_boost_hf` | Penguatan tambahan | `true` / `false`, hanya berpengaruh di band di atas 525 MHz | `false` |
| `ldro` | Low Data Rate Optimize | `LORA_LDRO_AUTO`, `LORA_LDRO_ON`, `LORA_LDRO_OFF` | `LORA_LDRO_AUTO` |
| `rx_timeout_symbols` | Batas waktu `lora_receive_single`, dalam simbol | 4 sampai 1023 | 100 |
| `keep_crc_error_packets` | Teruskan paket dengan CRC salah | `true` / `false` | `false` |

- **LNA otomatis (AGC)** adalah pilihan yang disarankan. Gain manual hanya perlu kalau kamu
  tahu alasannya, misalnya sinyal sangat kuat yang membuat penerima jenuh (pakai G4 sampai G6).
- **LDRO** harus aktif saat satu simbol lebih lama dari 16 ms (misalnya SF11 atau SF12 di BW
  125 kHz). Mode AUTO menghitungnya sendiri.
- **`keep_crc_error_packets`:** bawaannya paket rusak dibuang. Kalau `true`, paket rusak tetap
  diteruskan dan `pkt->crc_ok` bernilai `false`.

---

## 6. Memulai dan mengakhiri

### `lora_begin`

```c
esp_err_t lora_begin(const lora_config_t *cfg, lora_t **out_radio);
```

Menyiapkan bus SPI, mereset chip, memeriksa bahwa chip SX127x benar-benar terdeteksi, lalu
menulis semua pengaturan ke chip. Hasilnya adalah `radio`, yaitu "pegangan" yang dipakai oleh
semua fungsi lain.

```c
lora_t *radio;
esp_err_t err = lora_begin(&cfg, &radio);
if (err == ESP_ERR_NOT_FOUND) {
    printf("Chip LoRa tidak terdeteksi. Cek wiring, NSS, dan catu 3,3 V\n");
} else if (err != ESP_OK) {
    printf("Gagal: %s\n", esp_err_to_name(err));
}
```

Atau versi singkat yang menghentikan program kalau gagal:

```c
ESP_ERROR_CHECK(lora_begin(&cfg, &radio));
```

### `lora_end`

```c
esp_err_t lora_end(lora_t *radio);
```

Mematikan radio dan membebaskan semua resource. Dipakai kalau kamu ingin berhenti memakai
LoRa sepenuhnya. Setelah ini, `radio` tidak boleh dipakai lagi.

```c
lora_end(radio);
radio = NULL;
```

---

## 7. Mengirim data

Semua fungsi kirim menunggu sampai paket selesai dipancarkan, baru kembali.

### `lora_send`

```c
esp_err_t lora_send(lora_t *radio, const void *data, size_t len, uint32_t timeout_ms);
```

Mengirim data mentah sepanjang `len` byte (1 sampai 255). `timeout_ms = 0` berarti timeout
otomatis (lama paket di udara ditambah 1,5 detik).

```c
uint8_t data[] = {0x01, 0x02, 0x03, 0xAB};
esp_err_t err = lora_send(radio, data, sizeof(data), 0);
```

Mengirim struct:

```c
typedef struct {
    uint8_t id;
    int16_t suhu_x10;     /* 27,5 derajat dikirim sebagai 275 */
    uint16_t baterai_mv;
} __attribute__((packed)) laporan_t;

laporan_t lap = { .id = 3, .suhu_x10 = 275, .baterai_mv = 3900 };
lora_send(radio, &lap, sizeof(lap), 0);
```

### `lora_send_string`

```c
esp_err_t lora_send_string(lora_t *radio, const char *str);
```

Mengirim teks. Karakter `'\0'` di ujung tidak ikut dikirim.

```c
lora_send_string(radio, "halo dunia");
```

### `lora_printf`

```c
esp_err_t lora_printf(lora_t *radio, const char *fmt, ...);
```

Seperti `printf`, tapi hasilnya dikirim lewat LoRa. Maksimum 255 karakter.

```c
lora_printf(radio, "suhu=%.1f lembap=%d", 27.5f, 60);
lora_printf(radio, "halo %d", counter++);
```

### Hasil yang mungkin

| Hasil | Arti |
|-------|------|
| `ESP_OK` | Paket selesai dipancarkan (bukan berarti sudah diterima) |
| `ESP_ERR_TIMEOUT` | Chip tidak melaporkan selesai tepat waktu |
| `ESP_ERR_INVALID_SIZE` | Panjang 0, lebih dari 255, atau (mode implicit) tidak sama dengan `implicit_payload_len` |
| `ESP_ERR_INVALID_ARG` | Argumen kosong (NULL) |

---

## 8. Menerima data

### Struct `lora_packet_t`

Paket yang diterima disimpan di struct ini:

| Field | Tipe | Arti |
|-------|------|------|
| `data` | `uint8_t[256]` | Isi paket. **Selalu diakhiri `'\0'`**, jadi aman dicetak sebagai string |
| `len` | `uint8_t` | Jumlah byte payload |
| `rssi` | `int16_t` | Kuat sinyal paket (dBm) |
| `snr` | `float` | SNR paket (dB) |
| `freq_error_hz` | `int32_t` | Selisih frekuensi pengirim dan penerima (Hz) |
| `crc_ok` | `bool` | `false` hanya kalau `keep_crc_error_packets` aktif dan CRC paket salah |
| `timestamp_us` | `int64_t` | Waktu paket diterima (`esp_timer_get_time()`), dalam mikrodetik |

### `lora_receive` (menunggu satu paket)

```c
esp_err_t lora_receive(lora_t *radio, lora_packet_t *pkt, uint32_t timeout_ms);
```

Program berhenti menunggu sampai ada paket atau waktu habis. `timeout_ms` boleh
`LORA_WAIT_FOREVER`.

```c
lora_packet_t pkt;
esp_err_t err = lora_receive(radio, &pkt, 5000);     /* tunggu maksimal 5 detik */

if (err == ESP_OK) {
    printf("isi: %s, RSSI %d\n", (char *)pkt.data, pkt.rssi);
} else if (err == ESP_ERR_TIMEOUT) {
    printf("tidak ada paket\n");
} else if (err == ESP_ERR_INVALID_CRC) {
    printf("paket rusak (CRC salah)\n");
}
```

Dalam loop:

```c
while (1) {
    lora_packet_t pkt;
    if (lora_receive(radio, &pkt, LORA_WAIT_FOREVER) == ESP_OK) {
        printf("%s\n", (char *)pkt.data);
    }
}
```

Hal yang perlu diingat:

- Tidak boleh dipakai bersamaan dengan `lora_start_receive`. Hasilnya `ESP_ERR_INVALID_STATE`.
- Selama menunggu, fungsi radio lain dari task lain ikut menunggu.
- Setelah `lora_receive` selesai, radio kembali ke mode standby. Kalau kamu butuh terus
  mendengar, panggil lagi atau pakai `lora_start_receive`.

### `lora_receive_single`

```c
esp_err_t lora_receive_single(lora_t *radio, lora_packet_t *pkt);
```

Mendengar sebentar saja. Lama dengarnya ditentukan oleh `rx_timeout_symbols` (bawaan 100
simbol). Cocok kalau kamu hanya ingin mengecek apakah ada paket tanpa menunggu lama.

```c
lora_packet_t pkt;
if (lora_receive_single(radio, &pkt) == ESP_OK) {
    printf("ada paket: %s\n", (char *)pkt.data);
}
```

Mengubah lama dengarnya:

```c
lora_set_rx_timeout_symbols(radio, 200);
```

### `lora_start_receive` dan `lora_stop_receive` (background)

```c
esp_err_t lora_start_receive(lora_t *radio, lora_rx_cb_t cb, void *user_ctx);
esp_err_t lora_stop_receive(lora_t *radio);
```

Radio terus mendengar di background. Setiap ada paket, fungsi `cb` kamu dipanggil. Program
utamamu bebas melakukan hal lain.

```c
static void on_receive(lora_t *radio, const lora_packet_t *pkt, void *ctx)
{
    printf("masuk: %s\n", (const char *)pkt->data);
}

lora_start_receive(radio, on_receive, NULL);

/* ... program lain berjalan ... */

lora_stop_receive(radio);      /* berhenti mendengar */
```

Parameter `user_ctx` adalah pointer bebas yang diteruskan ke callback sebagai `ctx`. Contoh
memakainya untuk menghitung paket:

```c
static int jumlah = 0;

static void on_receive(lora_t *radio, const lora_packet_t *pkt, void *ctx)
{
    int *counter = (int *)ctx;
    (*counter)++;
    printf("paket ke-%d: %s\n", *counter, (const char *)pkt->data);
}

lora_start_receive(radio, on_receive, &jumlah);
```

Aturan callback:

- Callback berjalan di task internal berukuran 4 KB. Jaga agar singkat. Pekerjaan berat
  (misalnya menulis file) sebaiknya diteruskan ke task lain lewat queue.
- Callback **boleh memanggil `lora_send`** untuk membalas. Setelah mengirim, radio otomatis
  kembali mendengar.
- Callback boleh memanggil `lora_stop_receive`. Penerimaan berhenti setelah callback selesai.
- Selama penerimaan background aktif, `lora_receive`, `lora_receive_single`,
  `lora_channel_busy`, `lora_sleep`, dan `lora_standby` mengembalikan `ESP_ERR_INVALID_STATE`.
  Fungsi `lora_set_xxx` tetap boleh dipakai (radio berhenti sebentar lalu mendengar lagi).

Contoh membalas paket yang masuk:

```c
static void on_receive(lora_t *radio, const lora_packet_t *pkt, void *ctx)
{
    printf("masuk: %s\n", (const char *)pkt->data);
    lora_printf(radio, "balasan untuk: %s", (const char *)pkt->data);
}
```

---

## 9. Daya pancar: dBm dan mW

Ada dua fungsi dengan fungsi yang sama. Pakai salah satu sesuai selera.

```c
esp_err_t lora_set_tx_power(lora_t *radio, int8_t dbm);       /* satuan dBm */
esp_err_t lora_set_tx_power_mw(lora_t *radio, float mw);      /* satuan mW  */
```

Keduanya mengatur daya pancar yang sama. Contoh yang hasilnya identik:

```c
lora_set_tx_power(radio, 14);        /* 14 dBm */
lora_set_tx_power_mw(radio, 25);     /* 25 mW, hasilnya sama: 14 dBm */
```

Fungsi bantu untuk konversi:

```c
int8_t lora_mw_to_dbm(float mw);     /* mW  -> dBm (dibulatkan ke bilangan bulat) */
float  lora_dbm_to_mw(float dbm);    /* dBm -> mW */
```

```c
int8_t dbm = lora_mw_to_dbm(25);     /* 14 */
float  mw  = lora_dbm_to_mw(10);     /* 10.0 */
```

Di struct konfigurasi (sebelum `lora_begin`) juga ada dua field yang setara. Pakai salah satu:

```c
cfg.tx_power_dbm = 14;      /* dalam dBm */
```

atau

```c
cfg.tx_power_mw = 25;       /* dalam mW, hasilnya sama: 14 dBm */
```

Aturannya: kalau `tx_power_mw` diisi lebih dari 0, nilai itu yang dipakai dan `tx_power_dbm`
diabaikan. Kalau `tx_power_mw` dibiarkan 0 (bawaan), yang dipakai adalah `tx_power_dbm`. Jadi
cukup isi salah satu saja.

Hal yang perlu diketahui:

- **Chip hanya punya langkah 1 dBm.** Nilai mW dibulatkan ke dBm terdekat. Misalnya 30 mW
  (14,77 dBm) menjadi 15 dBm (31,6 mW).
- **Batas daya:** PA_BOOST 2 sampai 20 dBm (sekitar 1,6 sampai 100 mW), RFO 0 sampai 14 dBm
  (sekitar 1 sampai 25 mW). Nilai di luar batas dipotong ke batas terdekat.
- **Untuk RA-02**, sebaiknya jangan melewati 17 sampai 18 dBm (sekitar 50 sampai 63 mW).
- Fungsi mW mengembalikan `ESP_ERR_INVALID_ARG` kalau nilainya nol atau negatif.
- `lora_print_config` menampilkan daya dalam dBm dan mW sekaligus.

Tabel konversi cepat:

| dBm | mW (kira-kira) |
|-----|----------------|
| 2 | 1,6 |
| 5 | 3,2 |
| 10 | 10 |
| 13 | 20 |
| 14 | 25 |
| 17 | 50 |
| 18 | 63 |
| 20 | 100 |

Rumusnya: `mW = 10^(dBm/10)` dan `dBm = 10 × log10(mW)`. Setiap kenaikan 3 dBm berarti daya
sekitar dua kali lipat.

---

## 10. Mengubah pengaturan saat program berjalan

Setiap parameter punya fungsi `lora_set_xxx`. Semuanya bisa dipanggil kapan saja setelah
`lora_begin`, bahkan saat penerimaan background sedang aktif (radio berhenti sebentar, lalu
mendengar lagi dengan pengaturan baru). Semuanya mengembalikan `ESP_OK` kalau berhasil dan
`ESP_ERR_INVALID_ARG` kalau nilainya di luar batas.

Ingat: parameter radio harus tetap sama di pengirim dan penerima.

```c
lora_set_frequency(radio, 433200000);        /* ganti kanal ke 433,2 MHz */
lora_set_sf(radio, LORA_SF10);               /* SF6 sampai SF12 */
lora_set_bw(radio, LORA_BW_250K);            /* 7,8 sampai 500 kHz */
lora_set_cr(radio, LORA_CR_4_6);             /* 4/5 sampai 4/8 */
lora_set_preamble_len(radio, 12);            /* minimal 6 */
lora_set_sync_word(radio, 0x34);             /* penanda jaringan */
lora_set_crc(radio, true);                   /* CRC aktif atau mati */
lora_set_invert_iq(radio, false);            /* pembalikan IQ */
```

### `lora_set_header_mode`

```c
esp_err_t lora_set_header_mode(lora_t *radio, lora_header_t mode, uint8_t implicit_payload_len);
```

```c
lora_set_header_mode(radio, LORA_HEADER_IMPLICIT, 20);   /* semua paket tepat 20 byte */
lora_set_header_mode(radio, LORA_HEADER_EXPLICIT, 0);    /* kembali ke normal */
```

- Di mode implicit, panjang paket yang dikirim harus **persis** sama dengan angka itu, kalau
  tidak `lora_send` menolak dengan `ESP_ERR_INVALID_SIZE`.
- Panjang implicit harus 1 sampai 255.
- Saat SF6, mode explicit ditolak (`ESP_ERR_INVALID_STATE`) karena SF6 wajib implicit.

### Pengaturan pemancar

```c
lora_set_tx_power(radio, 14);                       /* dBm, lihat bagian 9 */
lora_set_tx_power_mw(radio, 25);                    /* mW */
lora_set_pa_pin(radio, LORA_PA_BOOST);              /* RA-02: PA_BOOST */
lora_set_pa_ramp(radio, LORA_PA_RAMP_40US);         /* waktu naik daya */
lora_set_ocp(radio, 120);                           /* batas arus 120 mA, 0 = mati */
```

Batas arus (OCP) valid di 45 sampai 240 mA. Kalau daya di atas 17 dBm, library otomatis
memakai minimal 140 mA.

### Pengaturan penerima

```c
lora_set_lna_gain(radio, LORA_LNA_AUTO);            /* AGC otomatis (disarankan) */
lora_set_lna_gain(radio, LORA_LNA_G4);              /* manual: gain lebih kecil */
lora_set_lna_boost_hf(radio, true);                 /* hanya efektif di atas 525 MHz */
lora_set_ldro(radio, LORA_LDRO_AUTO);               /* AUTO, ON, atau OFF */
lora_set_rx_timeout_symbols(radio, 200);            /* 4 sampai 1023 simbol */
```

### `lora_apply_config`

```c
esp_err_t lora_apply_config(lora_t *radio, const lora_config_t *cfg);
```

Menerapkan seluruh pengaturan radio dari satu struct sekaligus. Berguna kalau kamu punya
beberapa "profil", misalnya mode jarak dekat dan mode jarak jauh. Pin dan pengaturan SPI
diabaikan, jadi tidak berubah.

```c
lora_config_t jauh = LORA_CONFIG_DEFAULT(PIN_MOSI, PIN_MISO, PIN_SCK, PIN_NSS, PIN_RST, PIN_DIO0);
jauh.sf = LORA_SF12;
jauh.bw = LORA_BW_125K;

lora_config_t dekat = LORA_CONFIG_DEFAULT(PIN_MOSI, PIN_MISO, PIN_SCK, PIN_NSS, PIN_RST, PIN_DIO0);
dekat.sf = LORA_SF7;
dekat.bw = LORA_BW_500K;

lora_apply_config(radio, &jauh);      /* ganti ke profil jarak jauh */
lora_apply_config(radio, &dekat);     /* ganti ke profil jarak dekat */
```

---

## 11. Memeriksa kanal: CAD, RSSI kanal, dan scan

### `lora_channel_busy` (CAD)

```c
esp_err_t lora_channel_busy(lora_t *radio, bool *detected);
```

Mengecek apakah ada sinyal LoRa (preamble) di kanal saat ini. Cepat dan hemat daya. Berguna
untuk "dengar dulu sebelum bicara".

```c
bool sibuk;
if (lora_channel_busy(radio, &sibuk) == ESP_OK) {
    if (sibuk) {
        printf("kanal dipakai, tunggu dulu\n");
    } else {
        lora_send_string(radio, "kanal kosong, aku kirim");
    }
}
```

CAD hanya mendeteksi sinyal **LoRa** dengan setelan SF dan BW yang sama. Sinyal non-LoRa
(remote, telemetri lain) tidak terdeteksi. Untuk itu pakai RSSI kanal di bawah.

### `lora_get_channel_rssi`

```c
esp_err_t lora_get_channel_rssi(lora_t *radio, int *rssi_dbm);
```

Mengukur kuat sinyal (energi) di kanal saat ini, apa pun jenisnya. Dipakai untuk mengetahui
seberapa berisik kanal.

```c
int rssi;
lora_get_channel_rssi(radio, &rssi);
printf("RSSI kanal: %d dBm\n", rssi);
```

Kanal yang sepi biasanya sekitar -110 sampai -120 dBm di BW 125 kHz. Satu pembacaan bisa
melewatkan pemancar yang putus-putus, jadi untuk pengukuran yang andal pakai `lora_scan`.

### `lora_scan`

```c
esp_err_t lora_scan(lora_t *radio, uint32_t start_hz, uint32_t stop_hz, uint32_t step_hz,
                    int samples, lora_scan_result_t *results, size_t max_results,
                    size_t *out_count);
```

Memindai banyak kanal dan mengukur kebisingan tiap kanal. Frekuensi dan mode radio
dikembalikan seperti semula setelah selesai.

| Parameter | Arti |
|-----------|------|
| `start_hz`, `stop_hz` | Rentang frekuensi (Hz). Frekuensi `stop_hz` ikut diukur |
| `step_hz` | Jarak antar kanal (Hz), misalnya 200000 |
| `samples` | Jumlah pembacaan per kanal (1 sampai 200). Disarankan 20 |
| `results` | Array hasil milikmu |
| `max_results` | Ukuran array `results`. Harus cukup untuk semua kanal |
| `out_count` | Diisi jumlah kanal yang terukur |

Setiap hasil (`lora_scan_result_t`) berisi `frequency_hz`, `rssi_peak_dbm` (nilai tertinggi),
dan `rssi_avg_dbm` (rata-rata).

```c
lora_scan_result_t hasil[10];
size_t jumlah = 0;

/* pindai 433,1 sampai 434,7 MHz tiap 200 kHz (9 kanal) */
esp_err_t err = lora_scan(radio, 433100000, 434700000, 200000, 20, hasil, 10, &jumlah);

if (err == ESP_OK) {
    for (size_t i = 0; i < jumlah; i++) {
        printf("%.1f MHz  puncak %d dBm  rata-rata %d dBm\n",
               hasil[i].frequency_hz / 1e6, hasil[i].rssi_peak_dbm, hasil[i].rssi_avg_dbm);
    }
}
```

Kalau array `results` terlalu kecil untuk semua kanal, hasilnya `ESP_ERR_INVALID_SIZE`.
Jumlah kanal = `(stop - start) / step + 1`.

### `lora_scan_print`

```c
esp_err_t lora_scan_print(lora_t *radio, uint32_t start_hz, uint32_t stop_hz,
                          uint32_t step_hz, uint32_t *best_hz);
```

Versi praktis: memindai (20 pembacaan per kanal) lalu langsung mencetak tabel ke log. Parameter
`best_hz` boleh `NULL`; kalau diisi, ia berisi frekuensi paling sepi.

```c
uint32_t terbaik;
lora_scan_print(radio, 433100000, 434700000, 200000, &terbaik);

printf("kanal terpilih: %lu Hz\n", (unsigned long)terbaik);
lora_set_frequency(radio, terbaik);
```

Contoh keluaran:

```
 433.100 MHz  puncak -118 dBm  rata-rata -121 dBm  sepi
 433.300 MHz  puncak  -88 dBm  rata-rata  -97 dBm  RAMAI
 433.500 MHz  puncak -112 dBm  rata-rata -116 dBm  sepi
Kanal paling sepi: 433.100 MHz (puncak -118 dBm)
```

Penilaian di tabel adalah perkiraan untuk BW 125 kHz: puncak -105 dBm atau lebih rendah
dinilai "sepi", sampai -90 dBm "agak ramai", di atasnya "RAMAI". Ini patokan kasar, bukan
nilai pasti. Akurasi RSSI sekitar ±2 sampai 3 dB, jadi gunakan untuk membandingkan kanal.

Tips: lakukan scan beberapa kali di waktu yang berbeda, dan jalankan saat kondisi mirip
dengan saat dipakai (misalnya saat venue sedang ramai).

---

## 12. Hemat daya: sleep dan standby

```c
esp_err_t lora_sleep(lora_t *radio);
esp_err_t lora_standby(lora_t *radio);
```

- `lora_sleep`: arus paling rendah. Semua pengaturan tetap tersimpan.
- `lora_standby`: siaga, siap kerja.

```c
lora_send_string(radio, "kirim data sensor");
lora_sleep(radio);                    /* tidur sampai kirim berikutnya */
vTaskDelay(pdMS_TO_TICKS(60000));
lora_send_string(radio, "kirim lagi");   /* radio otomatis bangun saat mengirim */
```

Fungsi kirim dan terima otomatis membangunkan radio, jadi tidak perlu memanggil
`lora_standby` sebelum `lora_send` atau `lora_receive`. Kedua fungsi ini mengembalikan
`ESP_ERR_INVALID_STATE` kalau penerimaan background sedang aktif; hentikan dulu dengan
`lora_stop_receive`.

---

## 13. Informasi dan perhitungan

### `lora_get_airtime_ms`

```c
uint32_t lora_get_airtime_ms(lora_t *radio, size_t payload_len);
```

Lama sebuah paket berada di udara (milidetik) dengan pengaturan saat ini. Berguna untuk
menentukan jeda minimal antar kiriman dan timeout ACK.

```c
uint32_t ms = lora_get_airtime_ms(radio, 20);
printf("paket 20 byte butuh %u ms\n", (unsigned)ms);
```

### `lora_get_bitrate_bps` dan `lora_get_symbol_time_us`

```c
float lora_get_bitrate_bps(lora_t *radio);        /* bit per detik (efektif) */
float lora_get_symbol_time_us(lora_t *radio);     /* lama satu simbol (mikrodetik) */
```

```c
printf("bitrate: %.0f bps\n", lora_get_bitrate_bps(radio));
printf("simbol : %.0f us\n", lora_get_symbol_time_us(radio));
```

### `lora_get_config`

```c
esp_err_t lora_get_config(lora_t *radio, lora_config_t *out_cfg);
```

Membaca pengaturan yang sedang aktif, termasuk daya pancar efektif setelah dipotong ke batas.

```c
lora_config_t aktif;
lora_get_config(radio, &aktif);
printf("SF aktif: %d, daya: %d dBm\n", aktif.sf, aktif.tx_power_dbm);
```

### `lora_print_config`

```c
void lora_print_config(lora_t *radio);
```

Mencetak ringkasan semua pengaturan ke log (frekuensi, SF, BW, CR, preamble, sync word, CRC,
header, daya dalam dBm dan mW, LNA, LDRO, durasi simbol, bitrate, dan airtime). Cocok
dipanggil sekali setelah `lora_begin` untuk memastikan pengaturan sesuai, dan untuk
membandingkan pengaturan pengirim dan penerima.

```c
lora_print_config(radio);
```

---

## 14. Akses register mentah

```c
uint8_t lora_read_reg(lora_t *radio, uint8_t reg);
void    lora_write_reg(lora_t *radio, uint8_t reg, uint8_t value);
```

Untuk pengguna mahir yang ingin memakai fitur chip yang belum dibungkus library. Alamat dan
arti register ada di datasheet SX1278. Hati-hati: salah menulis register bisa membuat radio
berperilaku aneh.

```c
printf("versi chip: 0x%02X\n", lora_read_reg(radio, 0x42));   /* harus 0x12 */
printf("sync word : 0x%02X\n", lora_read_reg(radio, 0x39));
```

---

## 15. Kode error dan cara mengatasinya

| Kode | Arti | Cara mengatasi |
|------|------|----------------|
| `ESP_OK` | Berhasil | - |
| `ESP_ERR_NOT_FOUND` (dari `lora_begin`) | Chip tidak terdeteksi | Periksa wiring (terutama MISO/MOSI tertukar), pin NSS dan RST sesuai kode, catu 3,3 V, dan semua GND tersambung |
| `ESP_ERR_INVALID_ARG` | Argumen salah atau di luar batas | Periksa nilai yang kamu berikan (SF, BW, frekuensi, dan sebagainya) |
| `ESP_ERR_INVALID_SIZE` | Panjang paket salah, atau array hasil scan terlalu kecil | Panjang paket harus 1 sampai 255 (mode implicit: harus sama dengan `implicit_payload_len`) |
| `ESP_ERR_INVALID_STATE` | Operasi tidak boleh saat ini | Misalnya `lora_receive` saat penerimaan background aktif. Panggil `lora_stop_receive` dulu |
| `ESP_ERR_TIMEOUT` | Waktu habis | `lora_send`: radio tidak melapor selesai (cek DIO0 dan antena). `lora_receive`: memang tidak ada paket |
| `ESP_ERR_INVALID_CRC` | Paket diterima tapi rusak | Sinyal terlalu lemah atau ada gangguan. Dekatkan atau naikkan SF |
| `ESP_ERR_NO_MEM` | Memori tidak cukup | Kurangi pemakaian memori di bagian lain |

**Penerima tidak menerima apa-apa?** Periksa berurutan:

1. Antena terpasang di kedua modul.
2. Frekuensi, SF, BW, CR, sync word, preamble, CRC, dan header sama di kedua sisi
   (bandingkan hasil `lora_print_config` keduanya).
3. Kedua board benar-benar menjalankan firmware terbaru (flash sukses).
4. Wiring DIO0 benar, atau ganti `pin_dio0` ke `LORA_PIN_UNUSED` untuk mode polling.
5. Jangan terlalu dekat: jarak beberapa meter lebih baik untuk pengujian. Terlalu dekat dengan
   daya besar bisa membuat penerima jenuh.

**Tidak ada tampilan `printf` di monitor (ESP32-C3)?** Ubah *Channel for console output* ke
USB Serial/JTAG di `idf.py menuconfig`.

---

## 16. Contoh lanjutan: membuat ACK sendiri

LoRa tidak punya ACK bawaan. Contoh ini membuatnya di level aplikasi: pengirim menyertakan
nomor urut dan menunggu balasan; kalau tidak datang, ia mengirim ulang. Kode ini **belum diuji
di hardware**.

Format paket: `D,<nomor>,<pesan>` untuk data, `A,<nomor>` untuk balasan ACK.

### Pengirim

```c
#include <stdio.h>
#include <string.h>
#include "esp_timer.h"
#include "esp_random.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lora.h"

bool send_with_ack(lora_t *radio, uint8_t seq, const char *msg, int max_retry)
{
    char buf[100];
    snprintf(buf, sizeof buf, "D,%u,%s", seq, msg);

    uint32_t ack_wait = lora_get_airtime_ms(radio, 6) + 300;   /* waktu tunggu ACK */

    for (int i = 0; i <= max_retry; i++) {
        if (lora_send(radio, buf, strlen(buf), 0) != ESP_OK) continue;

        int64_t deadline = esp_timer_get_time() + (int64_t)ack_wait * 1000;
        while (esp_timer_get_time() < deadline) {
            uint32_t left = (uint32_t)((deadline - esp_timer_get_time()) / 1000);
            lora_packet_t p;
            esp_err_t e = lora_receive(radio, &p, left);
            if (e == ESP_OK) {
                unsigned s;
                if (sscanf((const char *)p.data, "A,%u", &s) == 1 && s == seq) return true;
            } else if (e == ESP_ERR_TIMEOUT) {
                break;                                  /* tidak ada balasan, kirim ulang */
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100 + (esp_random() % 200)));   /* jeda acak */
    }
    return false;                                       /* gagal setelah semua percobaan */
}

/* pemakaian di app_main */
uint8_t seq = 0;
while (1) {
    bool ok = send_with_ack(radio, seq++, "halo", 3);
    printf("kirim: %s\n", ok ? "diterima (ACK)" : "GAGAL, tidak ada ACK");
    vTaskDelay(pdMS_TO_TICKS(2000));
}
```

### Penerima

```c
static int last_seq = -1;

static void on_receive(lora_t *radio, const lora_packet_t *pkt, void *ctx)
{
    unsigned seq;
    char msg[100];
    if (sscanf((const char *)pkt->data, "D,%u,%99[^\n]", &seq, msg) == 2) {
        if ((int)seq != last_seq) {              /* abaikan duplikat */
            last_seq = (int)seq;
            printf("data #%u: %s (RSSI %d)\n", seq, msg, pkt->rssi);
        }
        lora_printf(radio, "A,%u", seq);         /* tetap balas ACK */
    }
}
```

Catatan: ACK sendiri bisa hilang walau datanya sampai. Itu sebabnya penerima mengecek nomor
urut agar data yang dikirim ulang tidak diproses dua kali.

---

## 17. Tips untuk banyak modul dan venue ramai

- **Hindari tabrakan antar modulmu:** pakai satu koordinator yang memanggil node satu per satu
  (polling), atau beri jeda acak sebelum mengirim dan pakai ACK. Beri tiap node ID di isi paket.
- **Pilih kanal sepi:** jalankan `lora_scan_print` di lokasi sebelum acara dan siapkan 2 sampai
  3 kanal cadangan yang bisa diganti cepat.
- **Setelan tahan noise:** SF8 sampai SF10 dengan BW 125 kHz, payload sependek mungkin, CRC
  aktif, daya TX seperlunya.
- **Jangan menaruh modul terlalu berdekatan** saat pengujian, karena penerima bisa jenuh oleh
  pemancar di sebelahnya.
- **Datang lebih awal** untuk tes jangkauan di lokasi, dan bawa antena serta modul cadangan.
- Periksa aturan frekuensi 433 MHz yang berlaku untuk acaramu (tanyakan ke panitia).

---

## 18. Batasan library

Library ini hanya mode LoRa lapisan fisik. Tidak termasuk:

- Frequency hopping
- Pemetaan pin DIO1 sampai DIO5 (hanya DIO0 yang dipakai)
- TX continuous (pemancaran terus-menerus untuk uji)
- Koreksi PPM dan input TCXO
- Mode FSK/OOK
- LoRaWAN (join, enkripsi, MAC)
- ACK dan kirim ulang otomatis (lihat bagian 16 untuk contoh)

Fitur chip yang belum dibungkus bisa dicoba lewat `lora_read_reg` dan `lora_write_reg`.