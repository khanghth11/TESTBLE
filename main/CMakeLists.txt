set(srcs "main.c"
        "wifi_manager.c"
        "bluetooth_manager.c"
        )

idf_component_register(SRCS "qrcodegen.c" "qrcode_display.c" "esp_qrcode_wrapper.c" "esp_qrcode_main.c" "esp_lcd_ssd1351.c" "main.c" "${srcs}"
                       INCLUDE_DIRS ".")
