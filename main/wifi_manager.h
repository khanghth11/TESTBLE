#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    WIFI_STATUS_UNKNOWN,
    WIFI_STATUS_OK,
    WIFI_STATUS_SCAN_OK,
    WIFI_STATUS_FAIL
} wifi_status_t;

void wifi_manager_init(void);
void wifi_manager_start_scan(void);
void wifi_manager_connect(const char *credentials);
void update_wifi_status_notify(wifi_status_t status);
void trigger_wifi_scan_notify(const char *scan_results);
char* wifi_manager_get_saved_ssid(void);
char* wifi_manager_get_saved_password(void);
void wifi_manager_connect_saved(void);
void wifi_manager_print_saved_credentials(void);
#endif