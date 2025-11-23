#include "app_ble.h"
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

static const char *TAG = "BLE_GATT";

// BLE cihaz adı
#define DEVICE_NAME "ESP32_PULSE"

// UUID'ler (16-bit UUID kullanarak)
// Servis UUID
#define GATT_SVR_SVC_UUID                   0x180D  // Heart Rate Service UUID'sini örnek olarak kullanıyoruz
// Karakteristik UUID'leri
#define GATT_SVR_CHR_VALUE1_UUID            0x2A37  // Float değer 1
#define GATT_SVR_CHR_VALUE2_UUID            0x2A38  // Float değer 2
#define GATT_SVR_CHR_VALUE3_UUID            0x2A39  // Float değer 3

// Float değerleri
static float g_value1 = 0.0f;
static float g_value2 = 0.0f;
static float g_value3 = 0.0f;

// Bildirim durumu
static bool notify_state_value1 = false;
static bool notify_state_value2 = false;
static bool notify_state_value3 = false;

// Bağlantı handle'ı
static uint16_t conn_handle = 0;

// Karakteristik değer handle'ları
static uint16_t value1_handle;
static uint16_t value2_handle;
static uint16_t value3_handle;

/* GATT server callback fonksiyonları */
static int gatt_svr_chr_access_value1(uint16_t conn_handle, uint16_t attr_handle,
                                      struct ble_gatt_access_ctxt *ctxt, void *arg);
static int gatt_svr_chr_access_value2(uint16_t conn_handle, uint16_t attr_handle,
                                      struct ble_gatt_access_ctxt *ctxt, void *arg);
static int gatt_svr_chr_access_value3(uint16_t conn_handle, uint16_t attr_handle,
                                      struct ble_gatt_access_ctxt *ctxt, void *arg);

/* GATT servis tanımı */
static const struct ble_gatt_svc_def gatt_svr_svcs[] = {
    {
        /* Servis: Pulse Counter Service */
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(GATT_SVR_SVC_UUID),
        .characteristics = (struct ble_gatt_chr_def[]){
            {
                /* Karakteristik: Float Değer 1 */
                .uuid = BLE_UUID16_DECLARE(GATT_SVR_CHR_VALUE1_UUID),
                .access_cb = gatt_svr_chr_access_value1,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &value1_handle,
            },
            {
                /* Karakteristik: Float Değer 2 */
                .uuid = BLE_UUID16_DECLARE(GATT_SVR_CHR_VALUE2_UUID),
                .access_cb = gatt_svr_chr_access_value2,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &value2_handle,
            },
            {
                /* Karakteristik: Float Değer 3 */
                .uuid = BLE_UUID16_DECLARE(GATT_SVR_CHR_VALUE3_UUID),
                .access_cb = gatt_svr_chr_access_value3,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &value3_handle,
            },
            {
                0, /* Karakteristiklerin sonu */
            }
        },
    },
    {
        0, /* Servislerin sonu */
    },
};

/* Karakteristik erişim callback'leri */
static int gatt_svr_chr_access_value1(uint16_t conn_handle, uint16_t attr_handle,
                                      struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    int rc;
    
    switch (ctxt->op) {
    case BLE_GATT_ACCESS_OP_READ_CHR:
        ESP_LOGI(TAG, "Value1 okunuyor: %.2f", g_value1);
        rc = os_mbuf_append(ctxt->om, &g_value1, sizeof(g_value1));
        return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;

    case BLE_GATT_ACCESS_OP_READ_DSC:
        ESP_LOGI(TAG, "Descriptor okunuyor (value1)");
        return 0;

    case BLE_GATT_ACCESS_OP_WRITE_DSC:
        // Notify descriptor yazıldı
        if (ctxt->dsc->uuid == BLE_UUID16_DECLARE(BLE_GATT_DSC_CLT_CFG_UUID16)) {
            uint16_t val;
            rc = ble_hs_mbuf_to_flat(ctxt->om, &val, sizeof(val), NULL);
            if (rc == 0) {
                notify_state_value1 = (val == 1);
                ESP_LOGI(TAG, "Value1 notify durumu: %s", notify_state_value1 ? "Açık" : "Kapalı");
            }
        }
        return 0;

    default:
        return BLE_ATT_ERR_UNLIKELY;
    }
}

static int gatt_svr_chr_access_value2(uint16_t conn_handle, uint16_t attr_handle,
                                      struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    int rc;
    
    switch (ctxt->op) {
    case BLE_GATT_ACCESS_OP_READ_CHR:
        ESP_LOGI(TAG, "Value2 okunuyor: %.2f", g_value2);
        rc = os_mbuf_append(ctxt->om, &g_value2, sizeof(g_value2));
        return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;

    case BLE_GATT_ACCESS_OP_READ_DSC:
        ESP_LOGI(TAG, "Descriptor okunuyor (value2)");
        return 0;

    case BLE_GATT_ACCESS_OP_WRITE_DSC:
        // Notify descriptor yazıldı
        if (ctxt->dsc->uuid == BLE_UUID16_DECLARE(BLE_GATT_DSC_CLT_CFG_UUID16)) {
            uint16_t val;
            rc = ble_hs_mbuf_to_flat(ctxt->om, &val, sizeof(val), NULL);
            if (rc == 0) {
                notify_state_value2 = (val == 1);
                ESP_LOGI(TAG, "Value2 notify durumu: %s", notify_state_value2 ? "Açık" : "Kapalı");
            }
        }
        return 0;

    default:
        return BLE_ATT_ERR_UNLIKELY;
    }
}

static int gatt_svr_chr_access_value3(uint16_t conn_handle, uint16_t attr_handle,
                                      struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    int rc;
    
    switch (ctxt->op) {
    case BLE_GATT_ACCESS_OP_READ_CHR:
        ESP_LOGI(TAG, "Value3 okunuyor: %.2f", g_value3);
        rc = os_mbuf_append(ctxt->om, &g_value3, sizeof(g_value3));
        return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;

    case BLE_GATT_ACCESS_OP_READ_DSC:
        ESP_LOGI(TAG, "Descriptor okunuyor (value3)");
        return 0;

    case BLE_GATT_ACCESS_OP_WRITE_DSC:
        // Notify descriptor yazıldı
        if (ctxt->dsc->uuid == BLE_UUID16_DECLARE(BLE_GATT_DSC_CLT_CFG_UUID16)) {
            uint16_t val;
            rc = ble_hs_mbuf_to_flat(ctxt->om, &val, sizeof(val), NULL);
            if (rc == 0) {
                notify_state_value3 = (val == 1);
                ESP_LOGI(TAG, "Value3 notify durumu: %s", notify_state_value3 ? "Açık" : "Kapalı");
            }
        }
        return 0;

    default:
        return BLE_ATT_ERR_UNLIKELY;
    }
}

/* GAP event handler */
static int ble_gap_event(struct ble_gap_event *event, void *arg)
{
    struct ble_gap_conn_desc desc;
    int rc;

    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        /* Bağlantı kuruldu veya başarısız oldu */
        ESP_LOGI(TAG, "Bağlantı %s; durum=%d ",
                 event->connect.status == 0 ? "kuruldu" : "başarısız",
                 event->connect.status);
        if (event->connect.status == 0) {
            rc = ble_gap_conn_find(event->connect.conn_handle, &desc);
            assert(rc == 0);
            conn_handle = event->connect.conn_handle;
            ESP_LOGI(TAG, "Bağlantı handle: %d", conn_handle);
        }
        
        if (event->connect.status != 0) {
            /* Bağlantı başarısız oldu; tekrar reklam yap */
            ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER,
                            NULL, NULL, NULL);
        }
        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "Bağlantı kesildi; sebep=%d ", event->disconnect.reason);
        conn_handle = 0;
        
        /* Bağlantı sonlandı; tekrar reklam yap */
        ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER,
                        NULL, NULL, NULL);
        return 0;

    case BLE_GAP_EVENT_CONN_UPDATE:
        /* Bağlantı güncellendi */
        ESP_LOGI(TAG, "Bağlantı güncellendi; durum=%d ",
                 event->conn_update.status);
        rc = ble_gap_conn_find(event->conn_update.conn_handle, &desc);
        assert(rc == 0);
        return 0;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        ESP_LOGI(TAG, "Reklam tamamlandı; sebep=%d",
                 event->adv_complete.reason);
        ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER,
                        NULL, NULL, NULL);
        return 0;

    case BLE_GAP_EVENT_SUBSCRIBE:
        ESP_LOGI(TAG, "Abonelik eventi; conn_handle=%d attr_handle=%d "
                      "reason=%d prevn=%d curn=%d previ=%d curi=%d",
                 event->subscribe.conn_handle,
                 event->subscribe.attr_handle,
                 event->subscribe.reason,
                 event->subscribe.prev_notify,
                 event->subscribe.cur_notify,
                 event->subscribe.prev_indicate,
                 event->subscribe.cur_indicate);
        return 0;

    case BLE_GAP_EVENT_MTU:
        ESP_LOGI(TAG, "MTU güncellendi; conn_handle=%d cid=%d mtu=%d",
                 event->mtu.conn_handle,
                 event->mtu.channel_id,
                 event->mtu.value);
        return 0;
    }

    return 0;
}

/* Reklam yapılandırması */
static void ble_app_advertise(void)
{
    struct ble_hs_adv_fields fields;
    const char *name;
    int rc;

    memset(&fields, 0, sizeof fields);

    /* Reklam bayrakları */
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;

    /* UUID listesi */
    fields.uuids16 = (ble_uuid16_t[]) {
        BLE_UUID16_INIT(GATT_SVR_SVC_UUID)
    };
    fields.num_uuids16 = 1;
    fields.uuids16_is_complete = 1;

    /* Cihaz adı */
    name = ble_svc_gap_device_name();
    fields.name = (uint8_t *)name;
    fields.name_len = strlen(name);
    fields.name_is_complete = 1;

    rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "Reklam alanları ayarlanamadı; rc=%d", rc);
        return;
    }

    /* Reklamı başlat */
    struct ble_gap_adv_params adv_params;
    memset(&adv_params, 0, sizeof adv_params);
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    rc = ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER,
                          &adv_params, ble_gap_event, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "Reklam başlatılamadı; rc=%d", rc);
        return;
    }

    ESP_LOGI(TAG, "BLE reklamı başlatıldı");
}

/* BLE host görev başlatıldığında çağrılır */
static void ble_app_on_sync(void)
{
    int rc;

    /* Adresin ayarlandığından emin ol */
    rc = ble_hs_util_ensure_addr(0);
    assert(rc == 0);

    /* Reklamı başlat */
    ble_app_advertise();
}

/* BLE host görevi */
void ble_host_task(void *param)
{
    ESP_LOGI(TAG, "BLE Host görevi başlatıldı");
    
    /* NimBLE host görevini çalıştır - bu döngü asla dönmez */
    nimble_port_run();
    
    nimble_port_freertos_deinit();
}

/* BLE'yi başlat */
void app_ble_init(void)
{
    int rc;

    ESP_LOGI(TAG, "BLE sunucusu başlatılıyor...");


    /* NimBLE host yapılandırmasını başlat */
    ESP_ERROR_CHECK(nimble_port_init());

    /* GAP ve GATT servislerini başlat */
    ble_svc_gap_init();
    ble_svc_gatt_init();

    /* GATT servisini kaydet */
    rc = ble_gatts_count_cfg(gatt_svr_svcs);
    if (rc != 0) {
        ESP_LOGE(TAG, "GATT servisi sayılamadı; rc=%d", rc);
        return;
    }

    rc = ble_gatts_add_svcs(gatt_svr_svcs);
    if (rc != 0) {
        ESP_LOGE(TAG, "GATT servisi eklenemedi; rc=%d", rc);
        return;
    }

    /* Cihaz adını ayarla */
    rc = ble_svc_gap_device_name_set(DEVICE_NAME);
    if (rc != 0) {
        ESP_LOGE(TAG, "Cihaz adı ayarlanamadı; rc=%d", rc);
        return;
    }

    /* Callback'i ayarla */
    ble_hs_cfg.sync_cb = ble_app_on_sync;
    ble_hs_cfg.gatts_register_cb = NULL;

    /* BLE host görevini başlat */
    nimble_port_freertos_init(ble_host_task);

    ESP_LOGI(TAG, "BLE sunucusu başlatıldı - Cihaz adı: %s", DEVICE_NAME);
}

/* Float değerlerini güncelle ve bildirim gönder */
void app_ble_update_values(float value1, float value2, float value3)
{
    g_value1 = value1;
    g_value2 = value2;
    g_value3 = value3;

    /* Bağlantı varsa ve notify aktifse bildirim gönder */
    if (conn_handle != 0) {
        if (notify_state_value1) {
            struct os_mbuf *om;
            om = ble_hs_mbuf_from_flat(&g_value1, sizeof(g_value1));
            if (om != NULL) {
                ble_gatts_notify_custom(conn_handle, value1_handle, om);
            }
        }

        if (notify_state_value2) {
            struct os_mbuf *om;
            om = ble_hs_mbuf_from_flat(&g_value2, sizeof(g_value2));
            if (om != NULL) {
                ble_gatts_notify_custom(conn_handle, value2_handle, om);
            }
        }

        if (notify_state_value3) {
            struct os_mbuf *om;
            om = ble_hs_mbuf_from_flat(&g_value3, sizeof(g_value3));
            if (om != NULL) {
                ble_gatts_notify_custom(conn_handle, value3_handle, om);
            }
        }
    }
}

/* Mevcut değerleri oku */
void app_ble_get_values(float *value1, float *value2, float *value3)
{
    if (value1 != NULL) {
        *value1 = g_value1;
    }
    if (value2 != NULL) {
        *value2 = g_value2;
    }
    if (value3 != NULL) {
        *value3 = g_value3;
    }
}

