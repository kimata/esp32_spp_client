#include <string.h>

#include "esp32_spp_client.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_gattc_api.h"
#include "esp_gap_ble_api.h"
#include "esp_log.h"

#define ESP_GATT_SPP_SERVICE_UUID   0xABF0
#define PROFILE_NUM                 1

static const char spp_device_name[] = "ESP_SPP_SERVER";

static struct gattc_profile_inst spp_status[PROFILE_NUM] = {
    [PROFILE_APP_ID] = {
        .gattc_cb = gattc_profile_event_handler,
        .gattc_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
        .is_connected = false,
        .is_scanning = false,
        .is_registered = false,
        .mtu_size = 23,
        .db = NULL,
    },
};

static esp_bt_uuid_t SPP_SERVICE_UUID = {
    .len  = ESP_UUID_LEN_16,
    .uuid = {.uuid16 = ESP_GATT_SPP_SERVICE_UUID,},
};

static esp_ble_scan_params_t ble_scan_params = {
    .scan_type              = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type          = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy     = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval          = 0x50,
    .scan_window            = 0x30
};

static xQueueHandle cmd_regist_queue;
static esp_ble_gap_cb_param_t scan_result;

gattc_spp_status_t *gattc_spp_status()
{
    return &(spp_status[PROFILE_APP_ID]);
}

static void reset_spp_status(void)
{
    spp_status[PROFILE_APP_ID].is_connected = false;
    spp_status[PROFILE_APP_ID].connection_id = 0;
    spp_status[PROFILE_APP_ID].mtu_size = 23;
    spp_status[PROFILE_APP_ID].reg_cmd = 0;
    spp_status[PROFILE_APP_ID].service_start_handle = 0;
    spp_status[PROFILE_APP_ID].service_end_handle = 0;

    /* memset(spp_status[PROFILE_APP_ID].remote_bda, 0x00, sizeof(esp_bd_addr_t)); */

    if (spp_status[PROFILE_APP_ID].db) {
        free(spp_status[PROFILE_APP_ID].db);
        spp_status[PROFILE_APP_ID].db = NULL;
    }
}

static void handle_regist_for_notify_event(esp_ble_gattc_cb_param_t *param)
{
    uint16_t notify_enable = 1;

    esp_ble_gattc_write_char_descr(gattc_spp_status()->gattc_if,
                                   gattc_spp_status()->connection_id,
                                   gattc_spp_status()->db[gattc_spp_status()->reg_cmd+1].attribute_handle,
                                   sizeof(notify_enable),
                                   (uint8_t *)&notify_enable,
                                   ESP_GATT_WRITE_TYPE_RSP,
                                   ESP_GATT_AUTH_REQ_NONE);
}

static void handle_write_descriptor_event(esp_ble_gattc_cb_param_t *param)
{
    switch (gattc_spp_status()->reg_cmd) {
    case SPP_IDX_SPP_DATA_NTY_VAL:
        gattc_spp_status()->reg_cmd = SPP_IDX_SPP_STATUS_VAL;
        xQueueSend(cmd_regist_queue, &(gattc_spp_status()->reg_cmd), 10/portTICK_PERIOD_MS);
        break;
    case SPP_IDX_SPP_STATUS_VAL:
        gattc_spp_status()->is_registered = true;
        break;
    default:
        break;
    };
}

static void handle_configure_mtu_event(esp_ble_gattc_cb_param_t *param)
{
    uint16_t count = SPP_IDX_NB;

    gattc_spp_status()->mtu_size = param->cfg_mtu.mtu;

    gattc_spp_status()->db = (esp_gattc_db_elem_t *)malloc(count*sizeof(esp_gattc_db_elem_t));
    if (gattc_spp_status()->db == NULL) {
        ESP_LOGE(TAG_SPP, "Failed to malloc at %s.", __func__);
        return;
    }

    if (esp_ble_gattc_get_db(gattc_spp_status()->gattc_if, gattc_spp_status()->connection_id,
                             gattc_spp_status()->service_start_handle,
                             gattc_spp_status()->service_end_handle,
                             gattc_spp_status()->db, &count) != ESP_GATT_OK) {
        ESP_LOGE(TAG_SPP, "Failed to get db.");
        return;
    }
    if (count != SPP_IDX_NB) {

        return;
    }
    gattc_spp_status()->reg_cmd = SPP_IDX_SPP_DATA_NTY_VAL;

    // NOTE: Add to handle reset of server device.
    if (gattc_spp_status()->is_registered) {
        esp_ble_gattc_unregister_for_notify(gattc_spp_status()->gattc_if,
                                            gattc_spp_status()->remote_bda,
                                            gattc_spp_status()->db[SPP_IDX_SPP_DATA_NTY_VAL].attribute_handle);
        esp_ble_gattc_unregister_for_notify(gattc_spp_status()->gattc_if,
                                            gattc_spp_status()->remote_bda,
                                            gattc_spp_status()->db[SPP_IDX_SPP_STATUS_VAL].attribute_handle);
    }

    xQueueSend(cmd_regist_queue, &(gattc_spp_status()->reg_cmd), 10/portTICK_PERIOD_MS);
}

void gattc_profile_event_handler(esp_gattc_cb_event_t event,
                                 esp_gatt_if_t gattc_if,
                                 esp_ble_gattc_cb_param_t *param)
{
    switch (event) {
    case ESP_GATTC_REG_EVT:
        esp_ble_gap_set_scan_params(&ble_scan_params);
        break;
    case ESP_GATTC_CONNECT_EVT:
        gattc_spp_status()->gattc_if = gattc_if;
        gattc_spp_status()->is_connected = true;
        gattc_spp_status()->connection_id = param->connect.conn_id;
        memcpy(gattc_spp_status()->remote_bda, param->connect.remote_bda,
               sizeof(esp_bd_addr_t));

        esp_ble_gattc_search_service(gattc_spp_status()->gattc_if,
                                     gattc_spp_status()->connection_id,
                                     &SPP_SERVICE_UUID);
        break;
    case ESP_GATTC_DISCONNECT_EVT:
        reset_spp_status();
        gattc_spp_status()->is_scanning = true;
        esp_ble_gap_start_scanning(0xFFFF);
        break;
    case ESP_GATTC_SEARCH_RES_EVT:
        gattc_spp_status()->service_start_handle = param->search_res.start_handle;
        gattc_spp_status()->service_end_handle = param->search_res.end_handle;
        break;
    case ESP_GATTC_SEARCH_CMPL_EVT:
        esp_ble_gattc_send_mtu_req(gattc_if, gattc_spp_status()->connection_id);
        break;
    case ESP_GATTC_REG_FOR_NOTIFY_EVT:
        if (param->reg_for_notify.status != ESP_GATT_OK) {
            ESP_LOGE(TAG_SPP, "Failed to regist for norify.");
            break;
        }
        handle_regist_for_notify_event(param);
        break;
    case ESP_GATTC_NOTIFY_EVT:
        notify_event_handler(param);
        break;
    case ESP_GATTC_READ_CHAR_EVT:
        break;
    case ESP_GATTC_WRITE_CHAR_EVT:
        if (param->write.status != ESP_GATT_OK) {
            ESP_LOGE(TAG_SPP, "Failed to write characteristic.");
            break;
        }
        break;
    case ESP_GATTC_PREP_WRITE_EVT:
        break;
    case ESP_GATTC_EXEC_EVT:
        break;
    case ESP_GATTC_WRITE_DESCR_EVT:
        if (param->write.status != ESP_GATT_OK) {
            ESP_LOGE(TAG_SPP, "Failed to write descriptor.");
            break;
        }
        handle_write_descriptor_event(param);
        break;
    case ESP_GATTC_CFG_MTU_EVT:
        if(param->cfg_mtu.status != ESP_OK){
            ESP_LOGE(TAG_SPP, "Failed to configure mtu.");
            break;
        }
        handle_configure_mtu_event(param);
        break;
    case ESP_GATTC_SRVC_CHG_EVT:
        break;
    default:
        break;
    }
}

void gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if,
                         esp_ble_gattc_cb_param_t *param)
{
    if (event == ESP_GATTC_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            spp_status[param->reg.app_id].gattc_if = gattc_if;
        } else {
            ESP_LOGE(TAG_SPP, "Failed to regist application.");
            return;
        }
    }

    for (int i = 0; i < PROFILE_NUM; i++) {
        if ((gattc_if == ESP_GATT_IF_NONE) ||
            (gattc_if == spp_status[i].gattc_if)) {
            if (spp_status[i].gattc_cb != NULL) {
                spp_status[i].gattc_cb(event, gattc_if, param);
            }
        }
    }
}

static void handle_scan_result_event(esp_ble_gap_cb_param_t *param)
{
    if (param->scan_rst.search_evt == ESP_GAP_SEARCH_INQ_RES_EVT) {
        uint8_t *adv_name;
        uint8_t adv_name_len = 0;

        adv_name = esp_ble_resolve_adv_data(param->scan_rst.ble_adv,
                                            ESP_BLE_AD_TYPE_NAME_CMPL,
                                            &adv_name_len);
        if (adv_name != NULL) {
            if (strncmp((char *)adv_name, spp_device_name, adv_name_len) == 0) {
                memcpy(&(scan_result), param, sizeof(scan_result));
                // NOTE: Add to handle reset of server device.
                // When server is reset, ESP_GAP_BLE_SCAN_RESULT_EVT event occurs twice.
                // (I don't know the reason...)
                if (gattc_spp_status()->is_scanning) {
                    esp_ble_gap_stop_scanning();
                    gattc_spp_status()->is_scanning = false;
                }
            }
        }
    }
}

void gap_event_handler(esp_gap_ble_cb_event_t event,
                       esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT:
        if (param->scan_param_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(TAG_SPP, "Failed to scan param set.");
            break;
        }
        esp_ble_gap_start_scanning(0xFFFF);
        gattc_spp_status()->is_scanning = true;
        break;
    case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
        if (param->scan_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(TAG_SPP, "Failed to scan start.");
            break;
        }
        break;
    case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
        if (param->scan_stop_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(TAG_SPP, "Failed to scan stop.");
            break;
        }
        if (gattc_spp_status()->is_connected == false) {
            esp_ble_gattc_open(gattc_spp_status()->gattc_if,
                               scan_result.scan_rst.bda, true);
        }
        break;
    case ESP_GAP_BLE_SCAN_RESULT_EVT:
        handle_scan_result_event(param);
        break;
    default:
        break;
    }
}

void spp_client_regist_task(void* arg)
{
    uint16_t reg_cmd;
    cmd_regist_queue = xQueueCreate(10, sizeof(uint32_t));

    while (1) {
        vTaskDelay(100 / portTICK_PERIOD_MS);
        if (xQueueReceive(cmd_regist_queue, &reg_cmd, portMAX_DELAY) == pdFALSE) {
            continue;
        }
        esp_ble_gattc_register_for_notify(gattc_spp_status()->gattc_if,
                                          gattc_spp_status()->remote_bda,
                                          gattc_spp_status()->db[reg_cmd].attribute_handle);
    }
}

