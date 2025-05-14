#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "qrcode.h"
#include "esp_lcd_ssd1351.h"
#include "bluetooth_manager.h"
#include "qrcode_display.h"

static const char *TAG = "QRCODE_DISPLAY";

// Display config
#define LCD_PIXEL_CLOCK_HZ (6000000)
#define LCD_SPI_HOST SPI2_HOST
#define LCD_CMD_BITS 8
#define LCD_PARAM_BITS 8
#define LCD_H_RES 128
#define LCD_V_RES 128
#define LCD_BITS_PER_PIXEL 16

// SSD1351 pin spi
#define LCD_SPI_CS_PIN 3
#define LCD_SPI_DC_PIN 7
#define LCD_SPI_SCLK_PIN 12
#define LCD_SPI_MOSI_PIN 11
#define LCD_SPI_RST_PIN -1
#define LCD_SPI_BL_PIN -1

static esp_lcd_panel_handle_t panel_handle = NULL;
static uint16_t *lcd_buffer = NULL;

// Initialize SSD1351 display
esp_err_t qrcode_display_init(void)
{
    esp_err_t ret = ESP_OK;
    ESP_LOGI(TAG, "Initialize SPI bus");
    spi_bus_config_t buscfg = {
        .sclk_io_num = LCD_SPI_SCLK_PIN,
        .mosi_io_num = LCD_SPI_MOSI_PIN,
        .miso_io_num = -1,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = LCD_H_RES * LCD_V_RES * sizeof(uint16_t),
    };
    ret = spi_bus_initialize(LCD_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "SPI bus initialize failed");
        return ret;
    }

    ESP_LOGI(TAG, "Install panel IO");
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = LCD_SPI_DC_PIN,
        .cs_gpio_num = LCD_SPI_CS_PIN,
        .pclk_hz = LCD_PIXEL_CLOCK_HZ,
        .lcd_cmd_bits = LCD_CMD_BITS,
        .lcd_param_bits = LCD_PARAM_BITS,
        .spi_mode = 0,
        .trans_queue_depth = 10,
    };
    ret = esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_SPI_HOST, &io_config, &io_handle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "New panel IO failed");
        return ret;
    }

    ESP_LOGI(TAG, "Install SSD1351 panel driver");
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = LCD_SPI_RST_PIN,
        .color_space = ESP_LCD_COLOR_SPACE_RGB,
        .bits_per_pixel = LCD_BITS_PER_PIXEL,
        .flags = {
            .reset_active_high = 0,
        },
    };
    ret = esp_lcd_new_panel_ssd1351(io_handle, &panel_config, &panel_handle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "New panel failed");
        return ret;
    }

    ESP_LOGI(TAG, "Initialize panel");
    ret = esp_lcd_panel_reset(panel_handle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Panel reset failed");
        return ret;
    }

    ret = esp_lcd_panel_init(panel_handle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Panel init failed");
        return ret;
    }

    ret = esp_lcd_panel_disp_on_off(panel_handle, true);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Display on failed");
        return ret;
    }

    lcd_buffer = heap_caps_malloc(LCD_H_RES * LCD_V_RES * sizeof(uint16_t), MALLOC_CAP_DMA);
    if (lcd_buffer == NULL)
    {
        ESP_LOGE(TAG, "Failed to allocate memory for LCD buffer");
        return ESP_ERR_NO_MEM;
    }

    // Clear display
    memset(lcd_buffer, 0, LCD_H_RES * LCD_V_RES * sizeof(uint16_t));
    esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, LCD_H_RES, LCD_V_RES, lcd_buffer);

    return ESP_OK;
}

// Helper function to set a pixel in the LCD buffer
static void set_pixel(int x, int y, uint16_t color)
{
    if (x >= 0 && x < LCD_H_RES && y >= 0 && y < LCD_V_RES)
    {
        lcd_buffer[y * LCD_H_RES + x] = color;
    }
}

// Draw QR code callback function
void qrcode_display_draw_qr(esp_qrcode_handle_t qrcode)
{
    for (int i = 0; i < LCD_H_RES * LCD_V_RES; i++)
    {
        lcd_buffer[i] = 0xFFFF; // White in RGB565
    }

    // Get QR code size
    int qr_size = esp_qrcode_get_size(qrcode);

    // Calculate scaling and position to center the QR code
    int scale = LCD_H_RES / qr_size;
    if (scale < 1)
        scale = 1;

    int offset_x = (LCD_H_RES - qr_size * scale) / 2;
    int offset_y = (LCD_V_RES - qr_size * scale) / 2;

    // Define colors
    uint16_t black = 0x0000; // Black color (0,0,0) in RGB565

    // Draw QR code to buffer
    for (int y = 0; y < qr_size; y++)
    {
        for (int x = 0; x < qr_size; x++)
        {
            bool is_black = esp_qrcode_get_module(qrcode, x, y);

            if (is_black)
            {
                for (int dy = 0; dy < scale; dy++)
                {
                    for (int dx = 0; dx < scale; dx++)
                    {
                        set_pixel(offset_x + x * scale + dx, offset_y + y * scale + dy, black);
                    }
                }
            }
        }
    }

    esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, LCD_H_RES, LCD_V_RES, lcd_buffer);
}

// Generate and display QR code
esp_err_t qrcode_display_generate_and_show(void)
{
    char qr_content[128];
    uint8_t bd_addr[6];
    ble_hs_id_copy_addr(own_addr_type, bd_addr, NULL);
    snprintf(qr_content, sizeof(qr_content), "%02X:%02X:%02X:%02X:%02X:%02X|%s|%04X",
             bd_addr[5], bd_addr[4], bd_addr[3], bd_addr[2], bd_addr[1], bd_addr[0],
             DEVICE_ID,
             MY_SERVICE_UUID_16);

    ESP_LOGI(TAG, "QR Code content: %s", qr_content);

    // Generate QR code
    esp_qrcode_config_t cfg = ESP_QRCODE_CONFIG_DEFAULT();
    cfg.display_func = qrcode_display_draw_qr;
    cfg.max_qrcode_version = 10; // Adjust for version qr
    cfg.qrcode_ecc_level = ESP_QRCODE_ECC_LOW;

    esp_err_t ret = esp_qrcode_generate(&cfg, qr_content);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to generate QR code: %d", ret);
    }

    return ret;
}