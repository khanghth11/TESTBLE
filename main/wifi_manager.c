#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_log.h"
#include "freertos/task.h"
#include "cJSON.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "wifi_manager.h"
#include "bluetooth_manager.h"

static const char *TAG = "WIFI_MANAGER";
static bool wifi_station_started = false;
static bool wifi_is_connected = false;
static bool wifi_scan_running = false;
static wifi_status_t current_wifi_status = WIFI_STATUS_UNKNOWN;
static const char *wifi_status_str[] = {"UNKNOWN", "WIFI_OK", "WIFI_SCAN_OK", "WIFI_FAIL"};
static char saved_ssid[33] = {0};
static char saved_password[64] = {0};
static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
static void wifi_scan_task(void *pvParameters);
static void send_wifi_scan_results(const char *scan_results);

// Wifi events handler
static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT)
    {
        switch (event_id)
        {
        case WIFI_EVENT_STA_START:
            wifi_station_started = true;
            break;
        case WIFI_EVENT_STA_CONNECTED:
            break;
        case WIFI_EVENT_STA_DISCONNECTED:
            wifi_is_connected = false;
            update_wifi_status_notify(WIFI_STATUS_FAIL);
            break;
        case WIFI_EVENT_SCAN_DONE:
            break;
        }
    }
    else if (event_base == IP_EVENT)
    {
        if (event_id == IP_EVENT_STA_GOT_IP)
        {
            wifi_is_connected = true;
            update_wifi_status_notify(WIFI_STATUS_OK);
            // When STA_GOT_IP, disconnect BLE like PB-5
            vTaskDelay(pdMS_TO_TICKS(200));
            // ble_manager_disconnect(); disconenct ble khi connect dc wifi
        }
    }
}

// Wifi
void wifi_manager_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    wifi_station_started = true;
    // Read info Wifi form NVS
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("wifi", NVS_READWRITE, &nvs_handle);
    if (err == ESP_OK)
    {
        size_t ssid_len = sizeof(saved_ssid);
        size_t pwd_len = sizeof(saved_password);

        nvs_get_str(nvs_handle, "ssid", saved_ssid, &ssid_len);
        nvs_get_str(nvs_handle, "password", saved_password, &pwd_len);

        nvs_close(nvs_handle);

        // autoconnect if have saved wifi
        if (strlen(saved_ssid) > 0)
        {
            wifi_manager_connect_saved();
        }
    }
    ESP_LOGI(TAG, "WiFi Manager initialized. Station started: %s", wifi_station_started ? "YES" : "NO");
    if (strlen(saved_ssid) > 0)
    {
        ESP_LOGI(TAG, "Found saved WiFi credentials - SSID: %s", saved_ssid);
        ESP_LOGI(TAG, "Attempting to connect to saved network...");
    }
    else
    {
        ESP_LOGI(TAG, "No saved WiFi credentials found");
    }
}

// scan Wifi networks
static void wifi_scan_task(void *pvParameters)
{
    wifi_scan_running = true;
    update_wifi_status_notify(WIFI_STATUS_UNKNOWN);

    wifi_scan_config_t scan_conf = {
        .scan_type = WIFI_SCAN_TYPE_ACTIVE,
        .show_hidden = true};

    // Start Wifi scan
    esp_err_t ret = esp_wifi_scan_start(&scan_conf, true);
    if (ret != ESP_OK)
    {
        update_wifi_status_notify(WIFI_STATUS_FAIL);
        goto cleanup;
    }

    uint16_t ap_count = 0;
    ret = esp_wifi_scan_get_ap_num(&ap_count);
    if (ret != ESP_OK)
    {
        update_wifi_status_notify(WIFI_STATUS_FAIL);
        goto cleanup;
    }

    if (ap_count == 0)
    {
        send_wifi_scan_results("{\"networks\":[]}");
        update_wifi_status_notify(WIFI_STATUS_SCAN_OK);
        goto cleanup;
    }

    wifi_ap_record_t *ap_list = malloc(sizeof(wifi_ap_record_t) * ap_count);
    if (!ap_list)
    {
        update_wifi_status_notify(WIFI_STATUS_FAIL);
        goto cleanup;
    }
    ret = esp_wifi_scan_get_ap_records(&ap_count, ap_list);
    if (ret != ESP_OK)
    {
        free(ap_list);
        update_wifi_status_notify(WIFI_STATUS_FAIL);
        goto cleanup;
    }

    // JSON Resutl
    cJSON *root = cJSON_CreateObject();
    cJSON *networks = cJSON_CreateArray();
    cJSON_AddItemToObject(root, "networks", networks);

    for (int i = 0; i < ap_count; i++)
    {
        cJSON *ap = cJSON_CreateObject();
        cJSON_AddStringToObject(ap, "ssid", (char *)ap_list[i].ssid);
        cJSON_AddNumberToObject(ap, "rssi", ap_list[i].rssi);
        cJSON_AddNumberToObject(ap, "channel", ap_list[i].primary);

        const char *auth_mode;
        switch (ap_list[i].authmode)
        {
        case WIFI_AUTH_OPEN:
            auth_mode = "OPEN";
            break;
        case WIFI_AUTH_WEP:
            auth_mode = "WEP";
            break;
        case WIFI_AUTH_WPA_PSK:
            auth_mode = "WPA";
            break;
        case WIFI_AUTH_WPA2_PSK:
            auth_mode = "WPA2";
            break;
        case WIFI_AUTH_WPA_WPA2_PSK:
            auth_mode = "WPA/WPA2";
            break;
        case WIFI_AUTH_WPA2_ENTERPRISE:
            auth_mode = "WPA2-Enterprise";
            break;
        case WIFI_AUTH_WPA3_PSK:
            auth_mode = "WPA3";
            break;
        case WIFI_AUTH_WPA2_WPA3_PSK:
            auth_mode = "WPA2/WPA3";
            break;
        default:
            auth_mode = "UNKNOWN";
            break;
        }
        cJSON_AddStringToObject(ap, "auth", auth_mode);

        cJSON_AddItemToArray(networks, ap);
    }

    char *json_str = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    free(ap_list);

    if (!json_str)
    {
        update_wifi_status_notify(WIFI_STATUS_FAIL);
        goto cleanup;
    }

    // Send results via BLE if notification enable
    if (ble_manager_is_connected())
    {
        send_wifi_scan_results(json_str);
    }
    else
    {
        ESP_LOGW(TAG, "BLE not connected, can't send scan results");
    }

    free(json_str);
    update_wifi_status_notify(WIFI_STATUS_SCAN_OK);

cleanup:
    wifi_scan_running = false;
    ESP_LOGI(TAG, "WiFi scan task completed");
    vTaskDelete(NULL);
}

static void send_wifi_scan_results(const char *scan_results)
{
    if (ble_manager_is_connected())
    {
        size_t len = strlen(scan_results);
        size_t chunk_size = ble_manager_get_mtu() - 3;
        size_t offset = 0;

        while (offset < len)
        {
            size_t remaining = len - offset;
            size_t send_size = remaining > chunk_size ? chunk_size : remaining;

            if (!ble_manager_send_wifi_scan_result_chunk((const uint8_t *)(scan_results + offset), send_size))
            {
                ESP_LOGE(TAG, "Failed to send scan results chunk");
                break;
            }

            offset += send_size;
            vTaskDelay(pdMS_TO_TICKS(20));
        }
    }
}

// Start Wifi scan
void wifi_manager_start_scan(void)
{
    if (wifi_scan_running)
    {
        ESP_LOGW(TAG, "Scan already running");
        return;
    }

    if (!wifi_station_started)
    {
        ESP_LOGE(TAG, "WiFi station not started");
        update_wifi_status_notify(WIFI_STATUS_FAIL);
        return;
    }

    ESP_LOGI(TAG, "Creating WiFi scan task");
    xTaskCreate(&wifi_scan_task, "wifi_scan_task", 4096, NULL, 5, NULL);
}

// Connect to Wifi
void wifi_manager_connect(const char *credentials)
{
    if (!wifi_station_started)
    {
        ESP_LOGE(TAG, "WiFi station not started");
        update_wifi_status_notify(WIFI_STATUS_FAIL);
        return;
    }

    // Parse JSON input
    cJSON *root = cJSON_Parse(credentials);
    if (root == NULL)
    {
        ESP_LOGE(TAG, "Failed to parse JSON credentials");
        update_wifi_status_notify(WIFI_STATUS_FAIL);
        return;
    }

    // Extract SSID and password
    cJSON *ssid_json = cJSON_GetObjectItem(root, "ssid");
    cJSON *password_json = cJSON_GetObjectItem(root, "password");

    if (!ssid_json || !cJSON_IsString(ssid_json) || strlen(ssid_json->valuestring) == 0)
    {
        ESP_LOGE(TAG, "Missing or invalid SSID in credentials");
        cJSON_Delete(root);
        update_wifi_status_notify(WIFI_STATUS_FAIL);
        return;
    }

    // Copy SSID to saved_ssid
    strncpy(saved_ssid, ssid_json->valuestring, sizeof(saved_ssid) - 1);
    saved_ssid[sizeof(saved_ssid) - 1] = '\0';

    // Copy password (if available) to saved_password
    if (password_json && cJSON_IsString(password_json))
    {
        strncpy(saved_password, password_json->valuestring, sizeof(saved_password) - 1);
        saved_password[sizeof(saved_password) - 1] = '\0';
    }
    else
    {
        saved_password[0] = '\0'; // Empty password for open networks
    }

    cJSON_Delete(root);

    // Save credentials to NVS
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("wifi", NVS_READWRITE, &nvs_handle);
    if (err == ESP_OK)
    {
        nvs_set_str(nvs_handle, "ssid", saved_ssid);
        nvs_set_str(nvs_handle, "password", saved_password);
        nvs_commit(nvs_handle);
        nvs_close(nvs_handle);
        ESP_LOGI(TAG, "Saved WiFi credentials to NVS");
    }
    else
    {
        ESP_LOGE(TAG, "Failed to open NVS for WiFi credentials");
    }

    // Disconnect from any current network
    esp_wifi_disconnect();

    // Configure WiFi with new credentials
    wifi_config_t wifi_config = {0};
    strncpy((char *)wifi_config.sta.ssid, saved_ssid, sizeof(wifi_config.sta.ssid) - 1);
    strncpy((char *)wifi_config.sta.password, saved_password, sizeof(wifi_config.sta.password) - 1);

    // Set appropriate authentication mode
    if (strlen(saved_password) == 0)
    {
        wifi_config.sta.threshold.authmode = WIFI_AUTH_OPEN;
    }
    else
    {
        wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    }

    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));

    wifi_is_connected = false;
    update_wifi_status_notify(WIFI_STATUS_UNKNOWN);

    ESP_LOGI(TAG, "Connecting to WiFi: %s", saved_ssid);
    esp_err_t rc = esp_wifi_connect();

    if (rc != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to connect to WiFi, error: 0x%x", rc);
        update_wifi_status_notify(WIFI_STATUS_FAIL);
    }
}

// Update Wifi status
void update_wifi_status_notify(wifi_status_t status)
{
    current_wifi_status = status;
    const char *status_str = wifi_status_str[current_wifi_status];
    ble_manager_send_status_notification(status_str);
}
char *wifi_manager_get_saved_ssid(void)
{
    return saved_ssid;
}
char *wifi_manager_get_saved_password(void)
{
    return saved_password;
}
void wifi_manager_connect_saved(void)
{
    if (strlen(saved_ssid) > 0)
    {
        ESP_LOGI(TAG, "Connecting to saved WiFi: %s", saved_ssid);

        wifi_config_t wifi_config = {0};
        strncpy((char *)wifi_config.sta.ssid, saved_ssid, sizeof(wifi_config.sta.ssid) - 1);
        strncpy((char *)wifi_config.sta.password, saved_password, sizeof(wifi_config.sta.password) - 1);

        // Đặt authmode phù hợp với loại mạng
        if (strlen(saved_password) == 0)
        {
            wifi_config.sta.threshold.authmode = WIFI_AUTH_OPEN;
        }
        else
        {
            wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
        }

        ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
        esp_err_t result = esp_wifi_connect();
        ESP_LOGI(TAG, "WiFi connect command result: %s (0x%x)",
                 result == ESP_OK ? "SUCCESS" : "FAILED", result);
    }
    else
    {
        ESP_LOGI(TAG, "Cannot connect to saved WiFi - missing SSID");
    }
}

void wifi_manager_print_saved_credentials(void)
{ // debug
    ESP_LOGI(TAG, "Saved WiFi credentials:");
    ESP_LOGI(TAG, "  SSID: %s", saved_ssid[0] ? saved_ssid : "(empty)");
    ESP_LOGI(TAG, "  Password: %s", saved_password[0] ? saved_password : "(empty)");
}