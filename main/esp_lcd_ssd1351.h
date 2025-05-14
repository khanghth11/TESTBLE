#ifndef __H_ESP_LCD_SSD1351__

#define __H_ESP_LCD_SSD1351__ 1

#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include <esp_lcd_panel_interface.h>
#include <esp_lcd_types.h>
#include <esp_lcd_panel_vendor.h>

#include <esp_lcd_panel_io_interface.h>

esp_err_t esp_lcd_new_panel_ssd1351(const esp_lcd_panel_io_handle_t io, const esp_lcd_panel_dev_config_t *panel_dev_config, esp_lcd_panel_handle_t *ret_panel);

// static const uint8_t initList[] = {};
typedef struct
{
    esp_lcd_panel_t base;
    esp_lcd_panel_io_handle_t io;
    int reset_gpio_num;
    bool reset_level;
    int x_gap;
    int y_gap;
    unsigned int bits_per_pixel;
    bool swap_axes;
    bool x_mirror;
    bool y_mirror;
    // madctl bits:
    // 6,7 Color depth (01 = 64K)
    // 5   Odd/even split COM (0: disable, 1: enable)
    // 4   Scan direction (0: top-down, 1: bottom-up)
    // 3   Reserved
    // 2   Color remap (0: A->B->C, 1: C->B->A)
    // 1   Column remap (0: 0-127, 1: 127-0)
    // 0   Address increment (0: horizontal, 1: vertical)
    uint8_t madctl;
} ssd1351_panel_t;

#endif