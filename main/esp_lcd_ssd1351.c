
#include "esp_err.h"
#include "esp_check.h"

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "driver/gpio.h"
#include "sdkconfig.h"
#include <stdint.h>
#include "qrcode.h"
#include "esp_lcd_ssd1351.h"

static const char *TAG = "lcd_panel.ssd1351";

static esp_err_t panel_ssd1351_del(esp_lcd_panel_t *panel);
static esp_err_t panel_ssd1351_reset(esp_lcd_panel_t *panel);
static esp_err_t panel_ssd1351_init(esp_lcd_panel_t *panel);
static esp_err_t panel_ssd1351_draw_bitmap(esp_lcd_panel_t *panel, int x_start, int y_start, int x_end, int y_end, const void *color_data);
static esp_err_t panel_ssd1351_invert_color(esp_lcd_panel_t *panel, bool invert_color_data);
static esp_err_t panel_ssd1351_mirror(esp_lcd_panel_t *panel,bool mirror_x, bool mirror_y);
static esp_err_t panel_ssd1351_swap_xy(esp_lcd_panel_t *panel, bool swap_axes);
static esp_err_t panel_ssd1351_set_gap(esp_lcd_panel_t *panel, int x_gap, int y_gap);
static esp_err_t panel_ssd1351_disp_on_off(esp_lcd_panel_t *panel, bool off);

// SSD1351 Commands
#define SSD1351_CMD_SETCOLUMN 0x15      ///< See datasheet
#define SSD1351_CMD_SETROW 0x75         ///< See datasheet
#define SSD1351_CMD_WRITERAM 0x5C       ///< See datasheet
#define SSD1351_CMD_READRAM 0x5D        ///< Not currently used
#define SSD1351_CMD_SETREMAP 0xA0       ///< See datasheet
#define SSD1351_CMD_STARTLINE 0xA1      ///< See datasheet
#define SSD1351_CMD_DISPLAYOFFSET 0xA2  ///< See datasheet
#define SSD1351_CMD_DISPLAYALLOFF 0xA4  ///< Not currently used
#define SSD1351_CMD_DISPLAYALLON 0xA5   ///< Not currently used
#define SSD1351_CMD_NORMALDISPLAY 0xA6  ///< See datasheet
#define SSD1351_CMD_INVERTDISPLAY 0xA7  ///< See datasheet
#define SSD1351_CMD_FUNCTIONSELECT 0xAB ///< See datasheet
#define SSD1351_CMD_DISPLAYOFF 0xAE     ///< See datasheet
#define SSD1351_CMD_DISPLAYON 0xAF      ///< See datasheet
#define SSD1351_CMD_PRECHARGE 0xB1      ///< See datasheet
#define SSD1351_CMD_DISPLAYENHANCE 0xB2 ///< Not currently used
#define SSD1351_CMD_CLOCKDIV 0xB3       ///< See datasheet
#define SSD1351_CMD_SETVSL 0xB4         ///< See datasheet
#define SSD1351_CMD_SETGPIO 0xB5        ///< See datasheet
#define SSD1351_CMD_PRECHARGE2 0xB6     ///< See datasheet
#define SSD1351_CMD_SETGRAY 0xB8        ///< Not currently used
#define SSD1351_CMD_USELUT 0xB9         ///< Not currently used
#define SSD1351_CMD_PRECHARGELEVEL 0xBB ///< Not currently used
#define SSD1351_CMD_VCOMH 0xBE          ///< See datasheet
#define SSD1351_CMD_CONTRASTABC 0xC1    ///< See datasheet
#define SSD1351_CMD_CONTRASTMASTER 0xC7 ///< See datasheet
#define SSD1351_CMD_MUXRATIO 0xCA       ///< See datasheet
#define SSD1351_CMD_COMMANDLOCK 0xFD    ///< See datasheet
#define SSD1351_CMD_HORIZSCROLL 0x96    ///< Not currently used
#define SSD1351_CMD_STOPSCROLL 0x9E     ///< Not currently used
#define SSD1351_CMD_STARTSCROLL 0x9F    ///< Not currently used

static esp_err_t panel_ssd1351_del(esp_lcd_panel_t *panel)
{
    ssd1351_panel_t *ssd1351 = __containerof(panel, ssd1351_panel_t, base);

    if (ssd1351->reset_gpio_num >= 0)
    {
        gpio_reset_pin(ssd1351->reset_gpio_num);
    }
    ESP_LOGD(TAG, "del ssd1351 panel @%p", ssd1351);
    free(ssd1351);
    return ESP_OK;
}

static esp_err_t panel_ssd1351_reset(esp_lcd_panel_t *panel)
{
    ssd1351_panel_t *ssd1351 = __containerof(panel, ssd1351_panel_t, base);
    esp_lcd_panel_io_handle_t io = ssd1351->io;

    // perform hardware reset
    if (ssd1351->reset_gpio_num >= 0)
    {
        gpio_set_level(ssd1351->reset_gpio_num, ssd1351->reset_level);
        vTaskDelay(pdMS_TO_TICKS(10));
        gpio_set_level(ssd1351->reset_gpio_num, !ssd1351->reset_level);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    else
    { // perform software reset
        ESP_LOGW(TAG, "Software reset not implemented, please define a reset GPIO pin for hardware reset.");
    }

    return ESP_OK;
}

static esp_err_t panel_ssd1351_init(esp_lcd_panel_t *panel)
{
    // ESP_LOGW(TAG, "Stub only : panel_ssd1351_init Not implemented yet");

    ssd1351_panel_t *ssd1351 = __containerof(panel, ssd1351_panel_t, base);
    esp_lcd_panel_io_handle_t io = ssd1351->io;

    esp_lcd_panel_io_tx_param(io, SSD1351_CMD_SETREMAP, (uint8_t[]){ssd1351->madctl}, 1);

    // Enters unlock mode
    esp_lcd_panel_io_tx_param(io, SSD1351_CMD_COMMANDLOCK, (uint8_t[]){0x12}, 1);
    // Unlock special commands
    esp_lcd_panel_io_tx_param(io, SSD1351_CMD_COMMANDLOCK, (uint8_t[]){0xB1}, 1);
    // Display off
    esp_lcd_panel_io_tx_param(io, SSD1351_CMD_DISPLAYOFF, NULL, 0);
    // Front clock divider
    // 7:4 = Oscillator Freq, 3:0 = CLK Div Ratio (A[3:0]+1 = 1..16)
    // F1 = max oscillator frequency, clock divider by 2
    esp_lcd_panel_io_tx_param(io, SSD1351_CMD_CLOCKDIV, (uint8_t[]){0xF1}, 1);
    // Reset MUX ratio
    esp_lcd_panel_io_tx_param(io, SSD1351_CMD_MUXRATIO, (uint8_t[]){127}, 1);

    // Display offset reset to 0
    esp_lcd_panel_io_tx_param(io, SSD1351_CMD_DISPLAYOFFSET, (uint8_t[]){0x0}, 1);

    // esp_lcd_panel_io_tx_param(io, , (uint8_t[]) { },1);
    esp_lcd_panel_io_tx_param(io, SSD1351_CMD_SETGPIO, (uint8_t[]){0x0}, 1);

    esp_lcd_panel_io_tx_param(io, SSD1351_CMD_FUNCTIONSELECT, (uint8_t[]){0x01}, 1);
    // internal (diode drop)
    esp_lcd_panel_io_tx_param(io, SSD1351_CMD_PRECHARGE, (uint8_t[]){0x32}, 1);

    esp_lcd_panel_io_tx_param(io, SSD1351_CMD_VCOMH, (uint8_t[]){0x05}, 1);
    esp_lcd_panel_io_tx_param(io, SSD1351_CMD_NORMALDISPLAY, NULL, 0);
    esp_lcd_panel_io_tx_param(io, SSD1351_CMD_CONTRASTABC, (uint8_t[]){0xC8, 0x80, 0xC8}, 3);
    esp_lcd_panel_io_tx_param(io, SSD1351_CMD_CONTRASTMASTER, (uint8_t[]){0x0F}, 1);
    esp_lcd_panel_io_tx_param(io, SSD1351_CMD_SETVSL, (uint8_t[]){0xA0, 0xB5, 0x55}, 3);
    esp_lcd_panel_io_tx_param(io, SSD1351_CMD_PRECHARGE2, (uint8_t[]){0x01}, 1);
    esp_lcd_panel_io_tx_param(io, SSD1351_CMD_DISPLAYON, NULL, 0);

    return ESP_OK;
}

static esp_err_t panel_ssd1351_draw_bitmap(esp_lcd_panel_t *panel, int x_start, int y_start, int x_end, int y_end, const void *color_data)
{
    // ESP_LOGW(TAG, "Stub only : panel_ssd1351_draw_bitmap Not implemented yet");
    // ESP_LOGW(TAG, "panel_ssd1351_draw_bitmap : x_start=%i x_end=%i y_start=%i y_end=%i",x_start,x_end,y_start,y_end);

    ssd1351_panel_t *ssd1351 = __containerof(panel, ssd1351_panel_t, base);
    esp_lcd_panel_io_handle_t io = ssd1351->io;

    // adding extra gap
    x_start += ssd1351->x_gap;
    x_end += ssd1351->x_gap;
    y_start += ssd1351->y_gap;
    y_end += ssd1351->y_gap;

    if (ssd1351->swap_axes)
    {
        int x = x_start;
        x_start = y_start;
        y_start = x;
        x = x_end;
        x_end = y_end;
        y_end = x;
    }

    esp_err_t ret = ESP_OK;

    // X range
    esp_lcd_panel_io_tx_param(io, SSD1351_CMD_SETCOLUMN, (uint8_t[]){x_start, x_end - 1}, 2);
    // Y range
    esp_lcd_panel_io_tx_param(io, SSD1351_CMD_SETROW, (uint8_t[]){y_start, y_end - 1}, 2);

    // transfer frame buffer
    size_t len = (y_end - y_start) * (x_end - x_start) * ssd1351->bits_per_pixel / 8;
    esp_lcd_panel_io_tx_color(io, SSD1351_CMD_WRITERAM, color_data, len);

    return ret;
}

static esp_err_t panel_ssd1351_invert_color(esp_lcd_panel_t *panel, bool invert_color_data)
{
    ESP_LOGW(TAG, "Stub only : panel_ssd1351_invert_color Not implemented yet");
    ssd1351_panel_t *ssd1351 = __containerof(panel, ssd1351_panel_t, base);
    esp_lcd_panel_io_handle_t io = ssd1351->io;

    esp_err_t ret = ESP_OK;

    return ret;
}

static esp_err_t panel_ssd1351_mirror(esp_lcd_panel_t *panel, bool mirror_x, bool mirror_y)
{
    ESP_LOGW(TAG, "Stub only : panel_ssd1351_mirror Not implemented yet");
    ssd1351_panel_t *ssd1351 = __containerof(panel, ssd1351_panel_t, base);
    esp_lcd_panel_io_handle_t io = ssd1351->io;

    esp_err_t ret = ESP_OK;

    ret = ESP_ERR_NOT_SUPPORTED;

    return ret;
}

static esp_err_t panel_ssd1351_swap_xy(esp_lcd_panel_t *panel, bool swap_axes)
{
    ESP_LOGW(TAG, "Stub only : panel_ssd1351_swap_xy Not implemented yet");
    ssd1351_panel_t *ssd1351 = __containerof(panel, ssd1351_panel_t, base);
    esp_lcd_panel_io_handle_t io = ssd1351->io;

    esp_err_t ret = ESP_OK;

    // ret=ESP_ERR_NOT_SUPPORTED;

    return ret;
}

static esp_err_t panel_ssd1351_set_gap(esp_lcd_panel_t *panel, int x_gap, int y_gap)
{
    ESP_LOGW(TAG, "Stub only : panel_ssd1351_set_gap Not implemented yet");
    ssd1351_panel_t *ssd1351 = __containerof(panel, ssd1351_panel_t, base);
    esp_lcd_panel_io_handle_t io = ssd1351->io;

    esp_err_t ret = ESP_OK;

    return ret;
}

static esp_err_t panel_ssd1351_disp_on_off(esp_lcd_panel_t *panel, bool on_off)
{
    ssd1351_panel_t *ssd1351 = __containerof(panel, ssd1351_panel_t, base);
    esp_lcd_panel_io_handle_t io = ssd1351->io;
    int command = 0;
    if (on_off)
    {
        command = SSD1351_CMD_DISPLAYON;
    }
    else
    {
        command = SSD1351_CMD_DISPLAYOFF;
    }
    esp_lcd_panel_io_tx_param(io, command, NULL, 0);
    // SEG/COM will be ON/OFF after 200ms after sending DISP_ON/OFF command
    vTaskDelay(pdMS_TO_TICKS(200));
    return ESP_OK;
}

esp_err_t esp_lcd_new_panel_ssd1351(const esp_lcd_panel_io_handle_t io, const esp_lcd_panel_dev_config_t *panel_dev_config, esp_lcd_panel_handle_t *ret_panel)
{

    esp_log_level_set(TAG, ESP_LOG_DEBUG);

    esp_err_t ret = ESP_OK;
    ssd1351_panel_t *ssd1351_panel = NULL;

    // Parameters validation
    ESP_GOTO_ON_FALSE(io && panel_dev_config && ret_panel, ESP_ERR_INVALID_ARG, err, TAG, "invalid argument");
    ssd1351_panel = calloc(1, sizeof(ssd1351_panel_t));

    ESP_GOTO_ON_FALSE(ssd1351_panel, ESP_ERR_NO_MEM, err, TAG, "no mem for ssd1351 panel");

    if (panel_dev_config->reset_gpio_num >= 0)
    {
        gpio_config_t io_conf = {
            .mode = GPIO_MODE_OUTPUT,
            .pin_bit_mask = 1ULL << panel_dev_config->reset_gpio_num,
        };
        ESP_GOTO_ON_ERROR(gpio_config(&io_conf), err, TAG, "configure GPIO for RST line failed");
    }

    ssd1351_panel->io = io;
    ssd1351_panel->bits_per_pixel = panel_dev_config->bits_per_pixel;
    ssd1351_panel->reset_gpio_num = panel_dev_config->reset_gpio_num;
    ssd1351_panel->reset_level = panel_dev_config->flags.reset_active_high;
    ssd1351_panel->base.del = panel_ssd1351_del;
    ssd1351_panel->base.reset = panel_ssd1351_reset;
    ssd1351_panel->base.init = panel_ssd1351_init;
    ssd1351_panel->base.draw_bitmap = panel_ssd1351_draw_bitmap;
    ssd1351_panel->base.invert_color = panel_ssd1351_invert_color;
    ssd1351_panel->base.set_gap = panel_ssd1351_set_gap;
    ssd1351_panel->base.mirror = panel_ssd1351_mirror;
    ssd1351_panel->base.swap_xy = panel_ssd1351_swap_xy;
    ssd1351_panel->base.disp_on_off = panel_ssd1351_disp_on_off;
    ssd1351_panel->madctl = 0b01100110; // 64K, enable split, CBA

    *ret_panel = &(ssd1351_panel->base);
    ESP_LOGD(TAG, "new ssd1351 panel @%p", ssd1351_panel);

    //    ESP_LOGW(TAG, "Stub only : esp_lcd_new_panel_ssd1351 Not implemented yet");

    return ESP_OK;

err:

    if (ssd1351_panel)
    {
        if (panel_dev_config->reset_gpio_num >= 0)
        {
            gpio_reset_pin(panel_dev_config->reset_gpio_num);
        }
        free(ssd1351_panel);
    }

    return ret;
}
