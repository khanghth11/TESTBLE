#ifndef BLUETOOTH_MANAGER_H
#define BLUETOOTH_MANAGER_H

#include <stdbool.h>
#include <stdint.h>
#include "nimble/ble.h"
#include "host/ble_hs.h"

#define DEVICE_ID "TOFU"
#define MY_SERVICE_UUID_16 0x18F0
#define WIFI_SCAN_CHR_UUID 0xA001
#define WIFI_CRED_CHR_UUID 0xA002
#define STATUS_CHR_UUID 0xA005
#define WIFI_CRED_MAX_LEN 128
#define MAX_BONDED_DEVICES 8
#define MAX_DEVICE_NAME_LEN 32

typedef struct {
    ble_addr_t addr;
    uint8_t addr_type;
    uint32_t last_connected_timestamp;  
    uint8_t priority;                 
    char device_name[MAX_DEVICE_NAME_LEN];
    bool is_active;                   
} bonded_device_t;

typedef struct
{
    bool is_connected;
    bool is_bonded;
    uint8_t bonded_addr[6];
    uint8_t bonded_addr_type;
    uint8_t irk[16];
    bool has_irk;
    uint16_t mtu_size;
    bool allow_new_bonding;
    ble_addr_t peer_addr;
    bonded_device_t bonded_devices[MAX_BONDED_DEVICES];
    uint8_t bonded_count;
     bool active_scan_mode;     
    bool reconnect_mode;    
} ble_connection_state_t;
extern uint8_t own_addr_type;
void initBluetooth(void);
void startAdvertising(void);
void clear_all_bonded_devices(void);
bool ble_manager_is_connected(void);
uint16_t ble_manager_get_mtu(void);
void ble_manager_send_status_notification(const char *status);
bool ble_manager_send_wifi_scan_result_chunk(const uint8_t *data, uint16_t length);
int gatt_svr_init(void);
bool ble_gap_is_bonded_device(const ble_addr_t *addr);
void ble_store_config_init(void);
void ble_manager_disconnect(void);
void ble_manager_get_rpa(uint8_t *rpa_addr);
void display_qr_code_callback(void);
void save_bonded_devices_to_nvs(void);  
void load_bonded_devices_from_nvs(void);
bool add_bonded_device(const ble_addr_t *addr, uint8_t addr_type);
void update_ble_whitelist(void);
bool is_bonded_device(const ble_addr_t *addr, uint8_t addr_type);
void on_boot_button_pressed(void);
void button_init(void);
void log_all_bonded_devices(void);
void sort_bonded_devices_by_priority(void);
void update_device_connection_info(const ble_addr_t *addr, uint8_t addr_type);
void start_active_scan_for_bonded_devices(void);
bool connect_to_preferred_device(void);
void reconnect_to_last_device(void);
#endif