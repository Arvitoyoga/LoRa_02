#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "lora.h"

#define PIN_SCK   4
#define PIN_MISO  5
#define PIN_MOSI  6
#define PIN_NSS   7
#define PIN_RST   3
#define PIN_DIO0  1

void app_main(void)
{
    lora_config_t cfg = LORA_CONFIG_DEFAULT(PIN_MOSI, PIN_MISO, PIN_SCK, PIN_NSS, PIN_RST, PIN_DIO0);
    cfg.sync_word = 0x16;
    cfg.frequency_hz = 433000000;
    cfg.sf           = LORA_SF9;
    cfg.bw           = LORA_BW_125K;
    cfg.tx_power_dbm = 16;
    

    lora_t *radio;
    ESP_ERROR_CHECK(lora_begin(&cfg, &radio));
    lora_print_config(radio);

    int counter = 0;
    while (1) {
        esp_err_t err = lora_printf(radio, "halo %d", counter++);
        printf("kirim: %s\n", err == ESP_OK ? "ok" : esp_err_to_name(err));
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}


// #include <stdio.h>
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "esp_log.h"
// #include "lora.h"

// #define PIN_SCK   4
// #define PIN_MISO  5
// #define PIN_MOSI  6
// #define PIN_NSS   7
// #define PIN_RST   3
// #define PIN_DIO0  1

// /* Dipanggil otomatis setiap paket masuk */
// static void on_receive(lora_t *radio, const lora_packet_t *pkt, void *ctx)
// {
//     (void)radio; (void)ctx;
//     printf("terima: \"%s\" (%u byte)  RSSI %d dBm  SNR %.1f dB  FEI %ld Hz\n",
//            (const char *)pkt->data, pkt->len, pkt->rssi, pkt->snr, (long)pkt->freq_error_hz);
// }

// void app_main(void)
// {
//     lora_config_t cfg = LORA_CONFIG_DEFAULT(PIN_MOSI, PIN_MISO, PIN_SCK, PIN_NSS, PIN_RST, PIN_DIO0);
//     cfg.sync_word = 0x16;
//     lora_t *radio;
//     ESP_ERROR_CHECK(lora_begin(&cfg, &radio));
//     lora_print_config(radio);

//     /* Cara 1: background + callback */
//     ESP_ERROR_CHECK(lora_start_receive(radio, on_receive, NULL));

//     /* Cara 2 (alternatif, blocking):
//      *   lora_packet_t pkt;
//      *   if (lora_receive(radio, &pkt, 5000) == ESP_OK) printf("%s\n", (char *)pkt.data);
//      */
//     vTaskDelay(portMAX_DELAY);
// }
