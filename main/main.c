#include <stdbool.h>
#include <stdio.h>
#include "nvs_flash.h"
#include "esp_log.h"
#include "bluetooth_manager.h"
#include "wifi_manager.h"
#include "store/config/ble_store_config.h"
#include "qrcode_display.h"
static const char *TAG = "MAIN";

void app_main(void)
{   
    nvs_flash_init();
    // esp_err_t ret = nvs_flash_init();
    // if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    // {
    //     ESP_ERROR_CHECK(nvs_flash_erase());
    //     ret = nvs_flash_init();
    // }
    // ESP_ERROR_CHECK(ret);
    
    ble_store_config_init();
    update_ble_whitelist();
    qrcode_display_init();
    wifi_manager_init();
    initBluetooth();
    button_init();  
    //wifi_manager_print_saved_credentials(); //debug return ssid/password
    ESP_LOGI(TAG, "Initialization complete");
}