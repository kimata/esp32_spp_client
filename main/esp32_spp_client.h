#include "esp_gattc_api.h"
#include <esp_gap_ble_api.h>

typedef struct gatts_spp_status {
    esp_gattc_cb_t gattc_cb;
    uint16_t gattc_if;
    uint16_t app_id;
    uint16_t connection_id;
    uint16_t service_start_handle;
    uint16_t service_end_handle;
    uint16_t char_handle;
    esp_bd_addr_t remote_bda;
    bool is_connected;
    bool is_scanning;
    bool is_registered;
    uint16_t mtu_size;
    esp_gattc_db_elem_t *db;
    uint16_t reg_cmd;
} gattc_spp_status_t;

enum{
    SPP_IDX_SVC,

    SPP_IDX_SPP_DATA_RECV_VAL,

    SPP_IDX_SPP_DATA_NTY_VAL,
    SPP_IDX_SPP_DATA_NTF_CFG,

    SPP_IDX_SPP_COMMAND_VAL,

    SPP_IDX_SPP_STATUS_VAL,
    SPP_IDX_SPP_STATUS_CFG,

#ifdef SUPPORT_HEARTBEAT
    SPP_IDX_SPP_HEARTBEAT_VAL,
    SPP_IDX_SPP_HEARTBEAT_CFG,
#endif

    SPP_IDX_NB,
};

#define UART_NUM UART_NUM_0

#define TAG_SPP  "ESP32_BLE_SPP"
#define PROFILE_APP_ID              0

void notify_event_handler(esp_ble_gattc_cb_param_t * p_data);
gattc_spp_status_t *gattc_spp_status();

void gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if,
                         esp_ble_gattc_cb_param_t *param);

void spp_client_regist_task(void* arg);

void gattc_profile_event_handler(esp_gattc_cb_event_t event,
                                 esp_gatt_if_t gattc_if,
                                 esp_ble_gattc_cb_param_t *param);
void gap_event_handler(esp_gap_ble_cb_event_t event,
                       esp_ble_gap_cb_param_t *param);
