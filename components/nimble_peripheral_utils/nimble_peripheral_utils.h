#ifndef NIMBLE_PERIPHERAL_UTILS_H
#define NIMBLE_PERIPHERAL_UTILS_H

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @brief Initialize BLE store configuration
     */
    void ble_store_config_init(void);

    /**
     * @brief Initialize NimBLE peripheral with device name
     *
     * @param device_name Name to advertise
     * @return 0 on success, error code on failure
     */
    int nimble_peripheral_init(const char *device_name);

    /**
     * @brief Start advertising
     *
     * @return 0 on success, error code on failure
     */
    int nimble_peripheral_advertise(void);

#ifdef __cplusplus
}
#endif

#endif /* NIMBLE_PERIPHERAL_UTILS_H */