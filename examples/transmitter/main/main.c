#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "lora.h"

/* Wiring RA-02 -> ESP32-C3 SuperMini */
#define PIN_SCK   4
#define PIN_MISO  5
#define PIN_MOSI  6
#define PIN_NSS   7
#define PIN_RST   3
#define PIN_DIO0  1

void app_main(void)
{
    lora_config_t cfg = LORA_CONFIG_DEFAULT(PIN_MOSI, PIN_MISO, PIN_SCK, PIN_NSS, PIN_RST, PIN_DIO0);
    //Ubah parameter bila perlu, contoh:
       cfg.frequency_hz = 433000000;
       cfg.sf           = LORA_SF9;
       cfg.bw           = LORA_BW_125K;
       cfg.tx_power_dbm = 10;
    

    lora_t *radio;
    ESP_ERROR_CHECK(lora_begin(&cfg, &radio));
    lora_print_config(radio);

    int counter = 0;
    while (1) {
        esp_err_t err = lora_printf(radio, "halo %d", counter++);
        printf("kirim: %s\n", err == ESP_OK ? "ok" : esp_err_to_name(err));
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}
