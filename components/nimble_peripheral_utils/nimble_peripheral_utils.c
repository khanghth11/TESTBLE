#include <host/ble_hs.h>
#include <host/ble_uuid.h>
#include <host/ble_gatt.h>
#include <host/ble_gap.h>
#include <host/util/util.h>
#include <services/gap/ble_svc_gap.h>
#include "nimble_peripheral_utils.h"

#include <esp_log.h>

static const char *TAG = "NimBLE_PERIPHERAL_UTILS";

void ble_store_config_init(void)
{
    /* Do nothing for now */
    ESP_LOGI(TAG, "BLE store config initialized");
}

int nimble_peripheral_init(const char *device_name)
{
    int rc;

    /* Set the default device name */
    rc = ble_svc_gap_device_name_set(device_name);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "Failed to set device name, error code: %d", rc);
        return rc;
    }

    ESP_LOGI(TAG, "BLE peripheral initialized with name: %s", device_name);
    return 0;
}

int nimble_peripheral_advertise(void)
{
    struct ble_gap_adv_params adv_params;
    struct ble_hs_adv_fields fields;
    int rc;

    /* Configure advertisement data */
    memset(&fields, 0, sizeof(fields));
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "Failed to set advertisement fields, error code: %d", rc);
        return rc;
    }

    /* Begin advertising */
    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    rc = ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER,
                           &adv_params, NULL, NULL);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "Failed to start advertising, error code: %d", rc);
        return rc;
    }

    ESP_LOGI(TAG, "BLE peripheral started advertising");
    return 0;
}