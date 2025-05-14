#ifndef QRCODE_DISPLAY_H
#define QRCODE_DISPLAY_H

#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>
#include "qrcode.h"
/**
 * @brief Initialize the SSD1351 display
 *
 * @return ESP_OK if successful, error code otherwise
 */
esp_err_t qrcode_display_init(void);

/**
 * @brief Generate and display QR code on the SSD1351 display
 *
 * @return ESP_OK if successful, error code otherwise
 */
esp_err_t qrcode_display_generate_and_show(void);

/**
 * @brief Draw QR code display function callback
 * 
 * @param qrcode QR code handle
 */
void qrcode_display_draw_qr(esp_qrcode_handle_t qrcode);

#endif
