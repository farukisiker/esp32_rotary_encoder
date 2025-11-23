#ifndef __BLE_APP_BLE_H__
#define __BLE_APP_BLE_H__

#include <stdint.h>

/**
 * @brief BLE GATT sunucusunu başlatır
 */
void app_ble_init(void);

/**
 * @brief Float değerlerini günceller ve BLE istemcilerine bildirim gönderir
 * 
 * @param value1 Birinci float değer
 * @param value2 İkinci float değer
 * @param value3 Üçüncü float değer
 */
void app_ble_update_values(float value1, float value2, float value3);

/**
 * @brief Mevcut float değerlerini okur
 * 
 * @param value1 Birinci float değer için pointer
 * @param value2 İkinci float değer için pointer
 * @param value3 Üçüncü float değer için pointer
 */
void app_ble_get_values(float *value1, float *value2, float *value3);

#endif // __BLE_APP_BLE_H__
