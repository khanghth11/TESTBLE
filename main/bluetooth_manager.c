#include <stdbool.h>
#include <assert.h>
#include <stdio.h>
#include <stdint.h>
#include "driver/gpio.h"
#include <string.h>
#include "nimble/ble.h"
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "nvs_flash.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/util/util.h"
#include "esp_log.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/FreeRTOS.h"
#include "host/ble_hs_pvcy.h"
#include "store/config/ble_store_config.h"
#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "qrcode_display.h"
#include "bluetooth_manager.h"
#include "wifi_manager.h"
#include "esp_system.h"
static bool allow_new_bonding_timeout_active = false;
static TickType_t allow_new_bonding_start_time = 0;
static const uint32_t ALLOW_NEW_BONDING_TIMEOUT_MS = 60000;
static const char *TAG = "TOFU";
static uint16_t connection_handle = BLE_HS_CONN_HANDLE_NONE;
static uint16_t wifi_scan_chr_handle;
static uint16_t wifi_cred_chr_handle;
static uint16_t status_chr_handle;
uint8_t own_addr_type;
static bool wifi_scan_notify_enabled = false;
static bool status_notify_enabled = false;
#define BUTTON_GPIO GPIO_NUM_0
#define BUTTON_DEBOUNCE_MS 50
static uint8_t wifi_cred_buf[WIFI_CRED_MAX_LEN];
static uint16_t wifi_cred_len;
static void button_task(void *arg);
static ble_connection_state_t ble_state = {
    .is_connected = false,
    .is_bonded = false,
    .has_irk = false,
    .mtu_size = 247,
    .allow_new_bonding = true};

static void bleprph_on_reset(int reason);
static void bleprph_on_sync(void);
static int bleprph_gap_event(struct ble_gap_event *event, void *arg);
static void start_bonded_advertising(void);
static int gatt_svc_access(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg);
static int write_flat_buffer(struct os_mbuf *om, uint16_t max_len, void *dst, uint16_t *len);
static void bleprph_host_task(void *param);
bool ble_manager_send_wifi_scan_result_chunk(const uint8_t *data, uint16_t length);

static const struct ble_gatt_svc_def gatt_svr_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(MY_SERVICE_UUID_16),
        .characteristics = (struct ble_gatt_chr_def[]){
            {// Wifi Scan Character
             .uuid = BLE_UUID16_DECLARE(WIFI_SCAN_CHR_UUID),
             .access_cb = gatt_svc_access,
             .flags = BLE_GATT_CHR_F_NOTIFY,
             .val_handle = &wifi_scan_chr_handle,
             .descriptors = (struct ble_gatt_dsc_def[]){
                 {
                     .uuid = BLE_UUID16_DECLARE(BLE_GATT_DSC_CLT_CFG_UUID16),
                     .att_flags = BLE_ATT_F_READ | BLE_ATT_F_WRITE,
                     .access_cb = gatt_svc_access,
                 },
                 {0}}},
            {// Wifi Credential Character
             .uuid = BLE_UUID16_DECLARE(WIFI_CRED_CHR_UUID),
             .access_cb = gatt_svc_access,
             .flags = BLE_GATT_CHR_F_WRITE,
             .val_handle = &wifi_cred_chr_handle},
            {// Status Character
             .uuid = BLE_UUID16_DECLARE(STATUS_CHR_UUID),
             .access_cb = gatt_svc_access,
             .flags = BLE_GATT_CHR_F_NOTIFY,
             .val_handle = &status_chr_handle},

            {0},
        },
    },
    {0},
};

// Check if address is RPA
static bool ble_addr_is_rpa(const ble_addr_t *addr)
{
    if (addr->type != BLE_ADDR_RANDOM)
    {
        return false;
    }
    uint8_t msb = addr->val[5];
    return (msb & 0xC0) == 0x40;
}

// Check if device is bonded
bool ble_gap_is_bonded_device(const ble_addr_t *addr)
{
    if (!ble_state.is_bonded)
    {
        return false;
    }
    if (ble_addr_is_rpa(addr))
    {
        ESP_LOGI(TAG, "Device is using RPA. Attempting to resolve...");
        return true;
    }
    return (addr->type == ble_state.bonded_addr_type &&
            memcmp(addr->val, ble_state.bonded_addr, 6) == 0);
}
static int write_flat_buffer(struct os_mbuf *om, uint16_t max_len, void *dst, uint16_t *len)
{
    uint16_t om_len = OS_MBUF_PKTLEN(om);
    if (om_len > max_len)
    {
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }

    int rc = ble_hs_mbuf_to_flat(om, dst, max_len, len);
    if (rc != 0)
    {
        return BLE_ATT_ERR_UNLIKELY;
    }

    return 0;
}

// GATT service access
static int gatt_svc_access(uint16_t conn_handle, uint16_t attr_handle,
                           struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    ESP_LOGI(TAG, "gatt_svc_access called: attr_handle = 0x%04X, op = %d", attr_handle, ctxt->op);

    const ble_uuid_t *uuid = NULL;
    int rc;
    uint16_t cccd_value;
    uint8_t buf[2];

    switch (ctxt->op)
    {
    case BLE_GATT_ACCESS_OP_READ_CHR:
    case BLE_GATT_ACCESS_OP_WRITE_CHR:
        if (ctxt->chr == NULL)
            return BLE_ATT_ERR_INVALID_HANDLE;
        uuid = ctxt->chr->uuid;
        break;
    case BLE_GATT_ACCESS_OP_READ_DSC:
    case BLE_GATT_ACCESS_OP_WRITE_DSC:
        if (ctxt->dsc == NULL || ctxt->chr == NULL)
            return BLE_ATT_ERR_INVALID_HANDLE;
        uuid = ctxt->dsc->uuid;
        break;
    default:
        ESP_LOGE(TAG, "Invalid GATT operation: %d", ctxt->op);
        return BLE_ATT_ERR_UNLIKELY;
    }

    if (ble_uuid_cmp(uuid, BLE_UUID16_DECLARE(BLE_GATT_DSC_CLT_CFG_UUID16)) == 0)
    {
        switch (ctxt->op)
        {
        case BLE_GATT_ACCESS_OP_WRITE_DSC:
            rc = os_mbuf_copydata(ctxt->om, 0, sizeof(cccd_value), &cccd_value);
            if (rc != 0)
            {
                ESP_LOGE(TAG, "Failed to read CCCD value");
                return BLE_ATT_ERR_UNLIKELY;
            }
            cccd_value = le16toh(cccd_value);

            if (ble_uuid_cmp(ctxt->chr->uuid, BLE_UUID16_DECLARE(WIFI_SCAN_CHR_UUID)) == 0)
            {
                wifi_scan_notify_enabled = (cccd_value & 0x0001);
                ESP_LOGI(TAG, "wifi_scan_notify_enable = %d", wifi_scan_notify_enabled);
                ESP_LOGI(TAG, "[WiFi Scan 0xA003] Notifications %s (handle: 0x%04X, value: 0x%04X)",
                         wifi_scan_notify_enabled ? "ENABLED" : "DISABLED",
                         attr_handle, cccd_value);
            }

            return 0;

        case BLE_GATT_ACCESS_OP_READ_DSC:
            if (ble_uuid_cmp(ctxt->chr->uuid, BLE_UUID16_DECLARE(WIFI_SCAN_CHR_UUID)) == 0)
            {
                buf[0] = wifi_scan_notify_enabled ? 1 : 0;
                buf[1] = 0;
                rc = os_mbuf_append(ctxt->om, buf, sizeof(buf));
                return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
            }
            break;
        default:
            break;
        }
        return 0;
    }

    if (ble_uuid_cmp(uuid, BLE_UUID16_DECLARE(WIFI_SCAN_CHR_UUID)) == 0)
    {
        if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR)
        {
            return BLE_ATT_ERR_READ_NOT_PERMITTED;
        }
        else if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR)
        {
            return BLE_ATT_ERR_WRITE_NOT_PERMITTED;
        }
        else
        {
            return BLE_ATT_ERR_REQ_NOT_SUPPORTED;
        }
    }

    if (ble_uuid_cmp(uuid, BLE_UUID16_DECLARE(WIFI_CRED_CHR_UUID)) == 0)
    {
        if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR)
        {
            rc = write_flat_buffer(ctxt->om, sizeof(wifi_cred_buf) - 1,
                                   wifi_cred_buf, &wifi_cred_len);
            if (rc == 0)
            {
                wifi_cred_buf[wifi_cred_len] = '\0';
                if (wifi_cred_len == 4 && strcmp((const char *)wifi_cred_buf, "SCAN") == 0)
                {
                    wifi_manager_start_scan();
                }
                else if (wifi_cred_len == 4 && strcmp((const char *)wifi_cred_buf, "PAIR") == 0)
                {
                    ESP_LOGI(TAG, "PAIR command received - allowing new bonding for 60 seconds!");
                    ble_state.allow_new_bonding = true;
                    ble_gap_adv_stop();
                    startAdvertising();
                    allow_new_bonding_timeout_active = true;
                    allow_new_bonding_start_time = xTaskGetTickCount();
                    ble_manager_send_status_notification("PAIRING_MODE");
                }
                else
                {
                    if (wifi_cred_len > 0 && memchr(wifi_cred_buf, '/', wifi_cred_len))
                    {
                        wifi_manager_connect((const char *)wifi_cred_buf);
                    }
                    else
                    {
                        update_wifi_status_notify(WIFI_STATUS_FAIL);
                    }
                }
                return 0;
            }
            else
            {
                return BLE_ATT_ERR_UNLIKELY;
            }
        }
        else
        {
            return BLE_ATT_ERR_REQ_NOT_SUPPORTED;
        }
    }

    if (ble_uuid_cmp(uuid, BLE_UUID16_DECLARE(STATUS_CHR_UUID)) == 0)
    {
        if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR)
        {
            return BLE_ATT_ERR_READ_NOT_PERMITTED;
        }
        else if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR)
        {
            return BLE_ATT_ERR_WRITE_NOT_PERMITTED;
        }
        else
        {
            return BLE_ATT_ERR_REQ_NOT_SUPPORTED;
        }
    }

    ESP_LOGE(TAG, "Unknown attribute accessed: handle=0x%04X", attr_handle);
    return BLE_ATT_ERR_ATTR_NOT_FOUND;
}

// GATT server register
void gatt_svr_register_cb(struct ble_gatt_register_ctxt *ctxt, void *arg)
{
    // Can be used for debug if needed
}

// GATT
int gatt_svr_init(void)
{
    int rc;
    ble_svc_gap_init();
    ble_svc_gatt_init();
    rc = ble_gatts_count_cfg(gatt_svr_svcs);
    if (rc != 0)
    {
        return rc;
    }

    rc = ble_gatts_add_svcs(gatt_svr_svcs);
    if (rc != 0)
    {
        return rc;
    }

    return 0;
}

// Start advertising
void startAdvertising(void)
{
    struct ble_gap_adv_params adv_params;
    struct ble_hs_adv_fields fields;
    struct ble_hs_adv_fields scan_rsp_fields;
    const char *name;
    int rc;

    memset(&fields, 0, sizeof fields);
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.tx_pwr_lvl_is_present = 1;
    fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;

    name = ble_svc_gap_device_name();
    fields.name = (uint8_t *)name;
    fields.name_len = strlen(name);
    fields.name_is_complete = 1;

    fields.uuids16 = (ble_uuid16_t[]){
        BLE_UUID16_INIT(MY_SERVICE_UUID_16)};
    fields.num_uuids16 = 1;
    fields.uuids16_is_complete = 1;

    // Scan response data
    memset(&scan_rsp_fields, 0, sizeof scan_rsp_fields);
    uint8_t addr_val[6];
    ble_hs_id_copy_addr(own_addr_type, addr_val, NULL);

    // DEVICE_ID|BD_ADDR
    char scan_rsp_data[32];
    snprintf(scan_rsp_data, sizeof(scan_rsp_data), "%s|%02X%02X%02X%02X%02X%02X",
             DEVICE_ID,
             addr_val[5], addr_val[4], addr_val[3],
             addr_val[2], addr_val[1], addr_val[0]);

    scan_rsp_fields.mfg_data = (uint8_t *)scan_rsp_data;
    scan_rsp_fields.mfg_data_len = strlen(scan_rsp_data);

    // Config
    rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0)
    {
        return;
    }

    rc = ble_gap_adv_rsp_set_fields(&scan_rsp_fields);
    if (rc != 0)
    {
        return;
    }

    memset(&adv_params, 0, sizeof adv_params);
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    adv_params.filter_policy = 0;
    rc = ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER, &adv_params, bleprph_gap_event, NULL);
    if (rc != 0)
    {
        return;
    }
}

// Start advertising for bonded devices
static void start_bonded_advertising(void)
{
    struct ble_gap_adv_params adv_params;
    struct ble_hs_adv_fields fields;
    struct ble_hs_adv_fields scan_rsp_fields;
    const char *name;
    int rc;

    // Stop current advertising
    ble_gap_adv_stop();

    memset(&fields, 0, sizeof fields);
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.tx_pwr_lvl_is_present = 1;
    fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;

    name = ble_svc_gap_device_name();
    fields.name = (uint8_t *)name;
    fields.name_len = strlen(name);
    fields.name_is_complete = 1;

    fields.uuids16 = (ble_uuid16_t[]){
        BLE_UUID16_INIT(MY_SERVICE_UUID_16)};
    fields.num_uuids16 = 1;
    fields.uuids16_is_complete = 1;

    // Scan response data
    memset(&scan_rsp_fields, 0, sizeof scan_rsp_fields);
    uint8_t addr_val[6];
    ble_hs_id_copy_addr(own_addr_type, addr_val, NULL);

    char scan_rsp_data[32];
    snprintf(scan_rsp_data, sizeof(scan_rsp_data), "%s|%02X%02X%02X%02X%02X%02X",
             DEVICE_ID,
             addr_val[5], addr_val[4], addr_val[3],
             addr_val[2], addr_val[1], addr_val[0]);

    scan_rsp_fields.mfg_data = (uint8_t *)scan_rsp_data;
    scan_rsp_fields.mfg_data_len = strlen(scan_rsp_data);

    // Config
    rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "Failed to set advertising fields: %d", rc);
        return;
    }

    rc = ble_gap_adv_rsp_set_fields(&scan_rsp_fields);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "Failed to set scan response fields: %d", rc);
        return;
    }

    memset(&adv_params, 0, sizeof adv_params);
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    // Use whitelist
    adv_params.filter_policy = 3; // BLE_GAP_ADV_FILTER_POLICY_WHITELIST;

    rc = ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER,
                           &adv_params, bleprph_gap_event, NULL);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "Failed to start bonded advertising: %d", rc);
    }
    else
    {
        ESP_LOGI(TAG, "Started bonded advertising with whitelist filtering");
    }
}

// Get rpa for QR code
void ble_manager_get_rpa(uint8_t *rpa_addr)
{
    ble_addr_t addr;
    int rc = ble_hs_id_copy_addr(BLE_ADDR_RANDOM, addr.val, NULL);
    if (rc == 0)
    {
        memcpy(rpa_addr, addr.val, 6);
    }
    else
    {
        memset(rpa_addr, 0, 6);
    }
}
// GAP event handler
static int bleprph_gap_event(struct ble_gap_event *event, void *arg)
{
    switch (event->type)
    {
        struct ble_gap_conn_desc desc;  
        int rc; 
    case BLE_GAP_EVENT_CONNECT:
    rc = ble_gap_conn_find(event->connect.conn_handle, &desc);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to find connection");
        break;
    }

    connection_handle = event->connect.conn_handle;
    ESP_LOGI(TAG, "Incoming connection. Peer address: %02X:%02X:%02X:%02X:%02X:%02X (type %d, RPA: %d)",
             desc.peer_id_addr.val[5], desc.peer_id_addr.val[4], desc.peer_id_addr.val[3],
             desc.peer_id_addr.val[2], desc.peer_id_addr.val[1], desc.peer_id_addr.val[0],
             desc.peer_id_addr.type, ble_addr_is_rpa(&desc.peer_id_addr));

    if (event->connect.status == 0) {
        // Kết nối thành công
        ble_state.is_connected = true;
        ble_state.active_scan_mode = false;
        ble_state.reconnect_mode = false;
        ble_gap_adv_stop();
        
        // Kiểm tra xem thiết bị đã ghép nối chưa
        bool is_already_bonded = is_bonded_device(&desc.peer_id_addr, desc.peer_id_addr.type);
        
        // Khởi tạo bảo mật ngay lập tức cho mọi kết nối mới
        if (!is_already_bonded && ble_state.allow_new_bonding) {
            ESP_LOGI(TAG, "New device connected - initiating security/bonding");
            rc = ble_gap_security_initiate(event->connect.conn_handle);
            if (rc != 0) {
                ESP_LOGE(TAG, "Failed to initiate security: %d", rc);
                ble_gap_terminate(event->connect.conn_handle, BLE_ERR_REM_USER_CONN_TERM);
                return 0;
            }
        }
        else if (is_already_bonded) {
            // Nếu đã ghép nối, cập nhật thông tin kết nối
            update_device_connection_info(&desc.peer_id_addr, desc.peer_id_addr.type);
            
            // Khởi tạo bảo mật cho thiết bị đã ghép nối
            ESP_LOGI(TAG, "Bonded device connected - initiating security");
            rc = ble_gap_security_initiate(event->connect.conn_handle);
            if (rc != 0) {
                ESP_LOGE(TAG, "Failed to initiate security: %d", rc);
            }
        }
        else if (!ble_state.allow_new_bonding) {
            // Nếu không cho phép ghép nối mới và thiết bị chưa ghép nối
            ESP_LOGI(TAG, "Rejecting non-bonded device");
            ble_gap_terminate(event->connect.conn_handle, BLE_ERR_REM_USER_CONN_TERM);
            return 0;
        }
    }
    else {
        // Xử lý kết nối thất bại...
    }
    return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "Disconnected, reason: %d", event->disconnect.reason);
        connection_handle = BLE_HS_CONN_HANDLE_NONE;
        ble_state.is_connected = false;

        // Đánh dấu tất cả thiết bị là không hoạt động
        for (int i = 0; i < ble_state.bonded_count; i++)
        {
            ble_state.bonded_devices[i].is_active = false;
        }

        // Thử kết nối lại với thiết bị ưu tiên cao nhất
        if (ble_state.is_bonded)
        {
            vTaskDelay(pdMS_TO_TICKS(100));
            reconnect_to_last_device();
        }
        else
        {
            vTaskDelay(pdMS_TO_TICKS(100));
            startAdvertising();
        }
        return 0;
    case BLE_GAP_EVENT_DISC:
        // Xử lý kết quả quét chủ động
        if (ble_state.active_scan_mode && !ble_state.is_connected)
        {
            // Kiểm tra xem thiết bị quét được có phải là thiết bị đã ghép nối không
            for (int i = 0; i < ble_state.bonded_count; i++)
            {
                if (memcmp(ble_state.bonded_devices[i].addr.val, event->disc.addr.val, 6) == 0 &&
                    ble_state.bonded_devices[i].addr_type == event->disc.addr.type)
                {

                    ESP_LOGI(TAG, "Found bonded device during scan: %02X:%02X:%02X:%02X:%02X:%02X",
                             event->disc.addr.val[5], event->disc.addr.val[4], event->disc.addr.val[3],
                             event->disc.addr.val[2], event->disc.addr.val[1], event->disc.addr.val[0]);

                    // Dừng quét và kết nối với thiết bị này
                    ble_gap_disc_cancel();
                    ble_state.active_scan_mode = false;

                    // Kết nối với thiết bị tìm thấy
                    struct ble_gap_conn_params conn_params = {
                        .scan_itvl = 16,
                        .scan_window = 16,
                        .itvl_min = 24,
                        .itvl_max = 40,
                        .latency = 0,
                        .supervision_timeout = 400,
                        .min_ce_len = 0,
                        .max_ce_len = 0,
                    };

                    rc = ble_gap_connect(own_addr_type, &event->disc.addr,
                                         5000, &conn_params, bleprph_gap_event, NULL);
                    if (rc != 0)
                    {
                        ESP_LOGE(TAG, "Failed to connect to found device: %d", rc);
                        // Tiếp tục quét
                        start_active_scan_for_bonded_devices();
                    }
                    return 0;
                }
            }
        }
        return 0;
    case BLE_GAP_EVENT_DISC_COMPLETE:
        ESP_LOGI(TAG, "Discovery complete");
        ble_state.active_scan_mode = false;

        // Nếu không tìm thấy thiết bị nào và đang trong chế độ kết nối lại
        if (ble_state.reconnect_mode && !ble_state.is_connected)
        {
            // Chờ một khoảng thời gian trước khi quét lại
            vTaskDelay(pdMS_TO_TICKS(5000));
            start_active_scan_for_bonded_devices();
        }
        else if (!ble_state.is_connected)
        {
            // Nếu không trong chế độ kết nối lại, quay lại chế độ quảng cáo
            if (ble_state.is_bonded)
            {
                start_bonded_advertising();
            }
            else
            {
                startAdvertising();
            }
        }
        return 0;
    case BLE_GAP_EVENT_ENC_CHANGE:
    {
        struct ble_gap_conn_desc desc;
        int rc = ble_gap_conn_find(event->enc_change.conn_handle, &desc);
        assert(rc == 0);
        ESP_LOGI(TAG, "ENC_CHANGE: status=%d, bonded=%d", event->enc_change.status, desc.sec_state.bonded);
        ESP_LOGI(TAG, "Encryption change event. Status: %d, Bonded: %d",
                 event->enc_change.status, desc.sec_state.bonded);

        if (desc.sec_state.bonded)
        {
            if (!is_bonded_device(&desc.peer_id_addr, desc.peer_id_addr.type))
            {
                add_bonded_device(&desc.peer_id_addr, desc.peer_id_addr.type);
                update_ble_whitelist();
            }
            ble_state.is_bonded = true;
            ble_state.allow_new_bonding = false;

            // Cập nhật thông tin kết nối
            update_device_connection_info(&desc.peer_id_addr, desc.peer_id_addr.type);
        }
        break;
    }
    case BLE_GAP_EVENT_MTU:
        ble_state.mtu_size = event->mtu.value;
        return 0;
    case BLE_GAP_EVENT_SUBSCRIBE:
        if (event->subscribe.attr_handle == wifi_scan_chr_handle)
        {
            wifi_scan_notify_enabled = event->subscribe.cur_notify;
            if (wifi_scan_notify_enabled)
            {
                wifi_manager_start_scan();
            }
        }
        else if (event->subscribe.attr_handle == status_chr_handle)
        {
            status_notify_enabled = event->subscribe.cur_notify;
            ESP_LOGI(TAG, "[Status 0xA010] Notifications %s (handle: 0x%04X)", status_notify_enabled ? "ENABLED" : "DISABLED", event->subscribe.attr_handle);
        }
        return 0;
    case BLE_GAP_EVENT_REPEAT_PAIRING:
    rc = ble_gap_conn_find(event->repeat_pairing.conn_handle, &desc);
    assert(rc == 0);
    if (is_bonded_device(&desc.peer_id_addr, desc.peer_id_addr.type)) {
        return BLE_GAP_REPEAT_PAIRING_IGNORE;
    }
    return BLE_GAP_REPEAT_PAIRING_RETRY;

    default:
        break;
    }

    return 0;
}

// BLE reset callback
static void bleprph_on_reset(int reason)
{
    ESP_LOGE(TAG, "Resetting state; reason=%d", reason);
}

// BLE sync callback
static void bleprph_on_sync(void)
{
    load_bonded_devices_from_nvs();
    int rc = ble_hs_id_infer_auto(0, &own_addr_type);
    assert(rc == 0);
    rc = ble_hs_util_ensure_addr(own_addr_type);
    assert(rc == 0);
    uint8_t addr_val[6];
    ble_hs_id_copy_addr(own_addr_type, addr_val, NULL);
    ESP_LOGI(TAG, "Our BLE addr type: %d, addr: %02x:%02x:%02x:%02x:%02x:%02x",
             own_addr_type,
             addr_val[0], addr_val[1], addr_val[2],
             addr_val[3], addr_val[4], addr_val[5]);
    ble_att_set_preferred_mtu(247); // set max mtu ve 247
    vTaskDelay(pdMS_TO_TICKS(100));
    log_all_bonded_devices();
    if (ble_state.is_bonded)
    {
        update_ble_whitelist();
        start_bonded_advertising();
    }
    else
    {
        startAdvertising();
    }
    display_qr_code_callback();
}

// BLE host task
static void bleprph_host_task(void *param)
{
    nimble_port_run();
    nimble_port_freertos_deinit();
}

// Initialize Bluetooth
void initBluetooth(void)
{
    nimble_port_init();
    ble_store_config_init();
    ble_hs_cfg.reset_cb = bleprph_on_reset;
    ble_hs_cfg.sync_cb = bleprph_on_sync;
    ble_hs_cfg.gatts_register_cb = gatt_svr_register_cb;
    ble_hs_cfg.sm_io_cap = BLE_HS_IO_NO_INPUT_OUTPUT;
    ble_hs_cfg.sm_bonding = 1;
    ble_hs_cfg.sm_mitm = 0;
    ble_hs_cfg.sm_sc = 1;
    ble_hs_cfg.sm_our_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
    ble_hs_cfg.sm_their_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
    int rc = gatt_svr_init();
    assert(rc == 0);
    rc = ble_svc_gap_device_name_set("TOFU");
    assert(rc == 0);
    nimble_port_freertos_init(bleprph_host_task);
}

// Check if BLE is connected
bool ble_manager_is_connected(void)
{
    return ble_state.is_connected;
}

// Get MTU size
uint16_t ble_manager_get_mtu(void)
{
    return ble_state.mtu_size;
}
bool ble_manager_send_wifi_scan_result_chunk(const uint8_t *data, uint16_t length)
{
    if (!ble_state.is_connected || !wifi_scan_notify_enabled)
    {
        return false;
    }

    struct os_mbuf *om = ble_hs_mbuf_from_flat(data, length);
    if (!om)
    {
        ESP_LOGE(TAG, "Failed to create mbuf");
        return false;
    }

    int rc = ble_gatts_notify_custom(connection_handle, wifi_scan_chr_handle, om);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "Failed to notify, rc=%d", rc);
        return false;
    }

    return true;
}
void ble_manager_send_status_notification(const char *status)
{
    if (!status_notify_enabled || !ble_state.is_connected)
    {
        return;
    }

    size_t len = strlen(status);
    struct os_mbuf *om = ble_hs_mbuf_from_flat(status, len);
    if (!om)
    {
        ESP_LOGE(TAG, "Failed to create mbuf for status notification");
        return;
    }

    int rc = ble_gatts_notify_custom(connection_handle, status_chr_handle, om);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "Failed to send status notification, rc=%d", rc);
    }
}
void ble_manager_disconnect(void)
{
    if (ble_state.is_connected)
    {
        // Terminate the connection
        int rc = ble_gap_terminate(connection_handle, BLE_ERR_REM_USER_CONN_TERM);
        if (rc != 0)
        {
            ESP_LOGE(TAG, "Failed to terminate connection: %d", rc);
        }
        else
        {
            ESP_LOGI(TAG, "BLE connection terminated");
        }
    }

    // Stop advertising
    ble_gap_adv_stop();
    ESP_LOGI(TAG, "BLE advertising stopped");

    // Clear connection state
    ble_state.is_connected = false;
}
void display_qr_code_callback(void)
{
    ESP_LOGI(TAG, "Displaying QR code");
    qrcode_display_init();
    qrcode_display_generate_and_show();
}
void save_bonded_devices_to_nvs(void)
{
    nvs_handle_t nvs;
    if (nvs_open("ble_bond", NVS_READWRITE, &nvs) == ESP_OK)
    {
        nvs_set_blob(nvs, "bonded_list", ble_state.bonded_devices, sizeof(ble_state.bonded_devices));
        nvs_set_u8(nvs, "bonded_count", ble_state.bonded_count);
        nvs_commit(nvs);
        nvs_close(nvs);
    }
}

void load_bonded_devices_from_nvs(void)
{
    nvs_handle_t nvs;
    size_t size = sizeof(ble_state.bonded_devices);
    if (nvs_open("ble_bond", NVS_READONLY, &nvs) == ESP_OK)
    {
        nvs_get_blob(nvs, "bonded_list", ble_state.bonded_devices, &size);
        nvs_get_u8(nvs, "bonded_count", &ble_state.bonded_count);
        nvs_close(nvs);
    }
    ble_state.is_bonded = (ble_state.bonded_count > 0);

    // Sắp xếp thiết bị theo thứ tự ưu tiên khi khởi động
    if (ble_state.bonded_count > 0)
    {
        sort_bonded_devices_by_priority();
    }
}
bool add_bonded_device(const ble_addr_t *addr, uint8_t addr_type)
{
    // Check exist
    for (int i = 0; i < ble_state.bonded_count; ++i)
    {
        if (memcmp(ble_state.bonded_devices[i].addr.val, addr->val, 6) == 0 &&
            ble_state.bonded_devices[i].addr_type == addr_type)
            return false;
    }
    if (ble_state.bonded_count < MAX_BONDED_DEVICES)
    {
        memcpy(&ble_state.bonded_devices[ble_state.bonded_count].addr, addr, sizeof(ble_addr_t));
        ble_state.bonded_devices[ble_state.bonded_count].addr_type = addr_type;
        ble_state.bonded_count++;
        save_bonded_devices_to_nvs();
        return true;
    }
    return false;
}
bool is_bonded_device(const ble_addr_t *addr, uint8_t addr_type)
{
    for (int i = 0; i < ble_state.bonded_count; ++i)
    {
        if (memcmp(ble_state.bonded_devices[i].addr.val, addr->val, 6) == 0 &&
            ble_state.bonded_devices[i].addr_type == addr_type)
            return true;
    }
    return false;
}
void update_ble_whitelist(void)
{
    ble_gap_wl_set(NULL, 0);
    ble_addr_t wl[MAX_BONDED_DEVICES];
    for (int i = 0; i < ble_state.bonded_count; ++i)
    {
        wl[i] = ble_state.bonded_devices[i].addr;
    }
    if (ble_state.bonded_count > 0)
        ble_gap_wl_set(wl, ble_state.bonded_count);
}
void on_boot_button_pressed(void)
{
    ESP_LOGI(TAG, "Allow new bonding mode!");
    ble_state.allow_new_bonding = true;
    ble_gap_adv_stop();
    startAdvertising();
}
static void button_task(void *arg)
{
    int last_state = 1;
    int stable_state = 1;
    TickType_t last_change = xTaskGetTickCount();
    TickType_t press_start_time = 0;
    bool medium_press_detected = false;
    bool long_press_detected = false;

    while (1)
    {
        int level = gpio_get_level(BUTTON_GPIO);
        // Kiểm tra timeout cho chế độ cho phép ghép nối mới
        if (allow_new_bonding_timeout_active)
        {
            TickType_t current_time = xTaskGetTickCount();
            uint32_t elapsed_ms = (current_time - allow_new_bonding_start_time) * portTICK_PERIOD_MS;

            if (elapsed_ms >= ALLOW_NEW_BONDING_TIMEOUT_MS)
            {
                ESP_LOGI(TAG, "Allow new bonding timeout expired");
                allow_new_bonding_timeout_active = false;

                // Nếu đã bonded với thiết bị, quay lại chế độ whitelist
                if (ble_state.is_bonded)
                {
                    ble_state.allow_new_bonding = false;
                    ble_gap_adv_stop();
                    update_ble_whitelist();
                    start_bonded_advertising();
                    ESP_LOGI(TAG, "Returned to bonded-only mode");
                }
            }
        }

        if (level != last_state)
        {
            last_change = xTaskGetTickCount();
            last_state = level;
        }
        // Debounce
        if ((xTaskGetTickCount() - last_change) * portTICK_PERIOD_MS > BUTTON_DEBOUNCE_MS)
        {
            if (stable_state != level)
            {
                stable_state = level;

                if (stable_state == 0)
                { // Nút được nhấn xuống
                    ESP_LOGI(TAG, "BOOT button pressed!");
                    press_start_time = xTaskGetTickCount();
                    medium_press_detected = false;
                    long_press_detected = false;
                }
                else
                { // Nút được thả ra
                  // Không cần xử lý gì khi thả nút vì đã xử lý trong quá trình nhấn giữ
                }
            }
            // Kiểm tra nhấn giữ khi nút đang được nhấn
            if (stable_state == 0)
            {
                TickType_t current_press_duration = xTaskGetTickCount() - press_start_time;
                // Kiểm tra nhấn giữ 2 giây
                if (!medium_press_detected && current_press_duration * portTICK_PERIOD_MS >= 2000 &&
                    current_press_duration * portTICK_PERIOD_MS < 5000)
                {
                    ESP_LOGI(TAG, "BOOT button held for 2 seconds - allowing new bonding for 60 seconds!");
                    // Cho phép ghép nối thiết bị mới
                    ble_state.allow_new_bonding = true;
                    ble_gap_adv_stop();
                    startAdvertising();
                    // Kích hoạt timeout
                    allow_new_bonding_timeout_active = true;
                    allow_new_bonding_start_time = xTaskGetTickCount();
                    medium_press_detected = true; // Đánh dấu đã xử lý nhấn giữ 2 giây
                    // Có thể thêm hiệu ứngLED để thông báo cho người dùng
                }

                // Kiểm tra nhấn giữ 5 giây
                if (!long_press_detected && current_press_duration * portTICK_PERIOD_MS >= 5000)
                {
                    ESP_LOGI(TAG, "BOOT button held for 5 seconds - erasing NVS!");

                    // Xóa NVS
                    esp_err_t err = nvs_flash_erase();
                    if (err == ESP_OK)
                    {
                        ESP_LOGI(TAG, "NVS erased successfully");

                        // Khởi tạo lại NVS
                        err = nvs_flash_init();
                        if (err == ESP_OK)
                        {
                            ESP_LOGI(TAG, "NVS reinitialized");
                        }
                        else
                        {
                            ESP_LOGE(TAG, "Failed to reinitialize NVS: %s", esp_err_to_name(err));
                        }

                        ble_state.is_bonded = false;
                        ble_state.has_irk = false;
                        ble_state.allow_new_bonding = true;
                        ble_state.bonded_count = 0;
                        memset(ble_state.bonded_devices, 0, sizeof(ble_state.bonded_devices));
                        ble_gap_wl_set(NULL, 0);
                        ble_gap_adv_stop();
                        startAdvertising();
                        allow_new_bonding_timeout_active = false;
                        display_qr_code_callback();
                    }
                    else
                    {
                        ESP_LOGE(TAG, "Failed to erase NVS: %s", esp_err_to_name(err));
                    }

                    long_press_detected = true; // Đánh dấu đã xử lý nhấn giữ 5 giây
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
void button_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BUTTON_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE};
    gpio_config(&io_conf);
    // create task button
    xTaskCreate(button_task, "button_task", 4096, NULL, 10, NULL);
}
void log_all_bonded_devices(void)
{
    ESP_LOGI(TAG, "===== Bonded Devices List =====");
    for (int i = 0; i < ble_state.bonded_count; ++i)
    {
        const ble_addr_t *addr = &ble_state.bonded_devices[i].addr;
        uint8_t type = ble_state.bonded_devices[i].addr_type;
        ESP_LOGI(TAG, "[%d] Addr: %02X:%02X:%02X:%02X:%02X:%02X Type: %d",
                 i + 1,
                 addr->val[5], addr->val[4], addr->val[3],
                 addr->val[2], addr->val[1], addr->val[0],
                 type);
    }
    if (ble_state.bonded_count == 0)
    {
        ESP_LOGI(TAG, "No bonded device.");
    }
    ESP_LOGI(TAG, "===============================");
}
void sort_bonded_devices_by_priority(void)
{
    ESP_LOGI(TAG, "Sorting bonded devices by priority and timestamp");
    
    // Sắp xếp theo priority trước, sau đó theo timestamp
    for (int i = 0; i < ble_state.bonded_count - 1; i++) {
        for (int j = 0; j < ble_state.bonded_count - i - 1; j++) {
            // Ưu tiên theo priority trước
            if (ble_state.bonded_devices[j].priority < ble_state.bonded_devices[j+1].priority) {
                bonded_device_t temp = ble_state.bonded_devices[j];  // Sửa thành bonded_device_t
                ble_state.bonded_devices[j] = ble_state.bonded_devices[j+1];
                ble_state.bonded_devices[j+1] = temp;
            }
            // Nếu priority bằng nhau, xét timestamp
            else if (ble_state.bonded_devices[j].priority == ble_state.bonded_devices[j+1].priority && 
                     ble_state.bonded_devices[j].last_connected_timestamp < 
                     ble_state.bonded_devices[j+1].last_connected_timestamp) {
                bonded_device_t temp = ble_state.bonded_devices[j];  // Sửa thành bonded_device_t
                ble_state.bonded_devices[j] = ble_state.bonded_devices[j+1];
                ble_state.bonded_devices[j+1] = temp;
            }
        }
    }
    
    // In danh sách thiết bị đã sắp xếp
    ESP_LOGI(TAG, "===== Sorted Bonded Devices List =====");
    for (int i = 0; i < ble_state.bonded_count; ++i) {
        const ble_addr_t *addr = &ble_state.bonded_devices[i].addr;
        ESP_LOGI(TAG, "[%d] Addr: %02X:%02X:%02X:%02X:%02X:%02X Priority: %d Last Connected: %lu", 
                 i + 1,
                 addr->val[5], addr->val[4], addr->val[3],
                 addr->val[2], addr->val[1], addr->val[0],
                 ble_state.bonded_devices[i].priority,
                 (unsigned long)ble_state.bonded_devices[i].last_connected_timestamp);  // Ép kiểu
    }
    ESP_LOGI(TAG, "=======================================");
}
void update_device_connection_info(const ble_addr_t *addr, uint8_t addr_type)
{
    bool device_found = false;
    
    // Tìm thiết bị trong danh sách
    for (int i = 0; i < ble_state.bonded_count; ++i) {
        if (memcmp(ble_state.bonded_devices[i].addr.val, addr->val, 6) == 0 &&
            ble_state.bonded_devices[i].addr_type == addr_type) {
            // Cập nhật timestamp
            ble_state.bonded_devices[i].last_connected_timestamp = (uint32_t)(esp_timer_get_time() / 1000000);
            
            // Tăng mức ưu tiên (tối đa là 10)
            if (ble_state.bonded_devices[i].priority < 10) {
                ble_state.bonded_devices[i].priority++;
            }
            
            // Đánh dấu thiết bị đang hoạt động
            ble_state.bonded_devices[i].is_active = true;
            
            device_found = true;
            ESP_LOGI(TAG, "Updated device connection info: Priority=%d, Timestamp=%lu",
                     ble_state.bonded_devices[i].priority,
                     (unsigned long)ble_state.bonded_devices[i].last_connected_timestamp);
            break;
        }
    }
    
    if (device_found) {
        save_bonded_devices_to_nvs();
        sort_bonded_devices_by_priority();
    }
}
void start_active_scan_for_bonded_devices(void)
{
    if (ble_state.active_scan_mode || ble_state.is_connected)
    {
        return;
    }

    ESP_LOGI(TAG, "Starting active scan for bonded devices");

    struct ble_gap_disc_params scan_params = {
        .filter_duplicates = 1,
        .passive = 0,
        .itvl = 16,
        .window = 16,
        .filter_policy = 0,
        .limited = 0,
    };

    ble_state.active_scan_mode = true;
    int rc = ble_gap_disc(own_addr_type, 5000, &scan_params, bleprph_gap_event, NULL);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "Failed to start active scan: %d", rc);
        ble_state.active_scan_mode = false;
    }
}
bool connect_to_preferred_device(void)
{
    if (ble_state.bonded_count == 0 || ble_state.is_connected)
    {
        return false;
    }
    sort_bonded_devices_by_priority();

    // Kết nối với thiết bị đầu tiên (ưu tiên cao nhất)
    ESP_LOGI(TAG, "Attempting to connect to preferred device: %02X:%02X:%02X:%02X:%02X:%02X",
             ble_state.bonded_devices[0].addr.val[5],
             ble_state.bonded_devices[0].addr.val[4],
             ble_state.bonded_devices[0].addr.val[3],
             ble_state.bonded_devices[0].addr.val[2],
             ble_state.bonded_devices[0].addr.val[1],
             ble_state.bonded_devices[0].addr.val[0]);
    ble_gap_adv_stop();

    struct ble_gap_conn_params conn_params = {
        .scan_itvl = 16,   // 10ms
        .scan_window = 16, // 10ms
        .itvl_min = 24,    // 30ms
        .itvl_max = 40,    // 50ms
        .latency = 0,
        .supervision_timeout = 400, // 4s
        .min_ce_len = 0,
        .max_ce_len = 0,
    };

    int rc = ble_gap_connect(own_addr_type, &ble_state.bonded_devices[0].addr,
                             10000, &conn_params, bleprph_gap_event, NULL);

    if (rc != 0)
    {
        ESP_LOGE(TAG, "Failed to initiate connection: %d", rc);
        if (ble_state.is_bonded)
        {
            start_bonded_advertising();
        }
        else
        {
            startAdvertising();
        }
        return false;
    }

    return true;
}
void reconnect_to_last_device(void)
{
    if (ble_state.is_connected || ble_state.bonded_count == 0)
    {
        return;
    }

    ESP_LOGI(TAG, "Attempting to reconnect to last connected device");

    // Đánh dấu đang trong chế độ kết nối lại
    ble_state.reconnect_mode = true;

    // Thử kết nối với thiết bị ưu tiên cao nhất
    if (!connect_to_preferred_device())
    {
        // Nếu kết nối thất bại, bắt đầu quét chủ động
        start_active_scan_for_bonded_devices();
    }
}