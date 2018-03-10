#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>

#include "esp32_spp_client.h"

#include "driver/uart.h"

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gatt_common_api.h"
#include "esp_gattc_api.h"
#include "esp_log.h"

#include "nvs_flash.h"

static const char device_name[] = "ESP_SPP_SERVER";

static xQueueHandle cmd_regist_queue = NULL;
QueueHandle_t spp_uart_queue = NULL;

#ifdef SUPPORT_HEARTBEAT
static uint8_t  heartbeat_s[9] = {'E','s','p','r','e','s','s','i','f'};
static xQueueHandle cmd_heartbeat_queue = NULL;
#endif

////////////////////////////////////////////////////////////////////////////////
// UART function
static void uart_write(uint8_t *str, uint32_t len)
{
    uart_write_bytes(UART_NUM, (char *)str, len);
}

static void uart_read(uint8_t *buf, uint32_t len)
{
    uart_read_bytes(UART_NUM, buf, len, portMAX_DELAY);
}


////////////////////////////////////////////////////////////////////////////////
// UART handler: Remote to Local
// Receive UART data via BLE and write data.
void notify_event_handler(esp_ble_gattc_cb_param_t * p_data)
{
    uint8_t handle = p_data->notify.handle;

    if (handle == gattc_spp_status()->db[SPP_IDX_SPP_DATA_NTY_VAL].attribute_handle) {
        uart_write(p_data->notify.value, p_data->notify.value_len);
    } else if (handle == gattc_spp_status()->db[SPP_IDX_SPP_STATUS_VAL].attribute_handle) {
        // TODO: server notify status characteristic
    } else {
        ESP_LOGE(TAG_SPP, "Notify event occured with unkown handle");
    }
}

////////////////////////////////////////////////////////////////////////////////
// UART handler: Local to Remote
// Read UART data and send it to remote via BLE.
void uart_task(void *arg)
{
    uart_event_t event;

    uart_driver_install(UART_NUM_0, 4096, 8192, 10, &spp_uart_queue, 0);

    while (1) {
        uint8_t *buf;
        uint32_t len;

        if (xQueueReceive(spp_uart_queue, (void * )&event, (portTickType)portMAX_DELAY) == pdFALSE) {
            continue;
        }
        if ((event.type != UART_DATA) || (event.size == 0)) {
            continue;
        }
        if ((gattc_spp_status()->is_connected != true) ||
            (gattc_spp_status()->db == NULL)) {
            continue;
        }
        if (!(gattc_spp_status()->db[SPP_IDX_SPP_DATA_RECV_VAL].properties &
             (ESP_GATT_CHAR_PROP_BIT_WRITE_NR | ESP_GATT_CHAR_PROP_BIT_WRITE))) {
            continue;
        }

        len = event.size;
        buf = (uint8_t *)malloc(sizeof(uint8_t)*len);
        if (buf == NULL) {
            ESP_LOGE(TAG_SPP, "Failed to malloc at %s.", __func__);
            break;
        }
        memset(buf, 0x00, len);
        uart_read(buf, len);

        esp_ble_gattc_write_char(gattc_spp_status()->gattc_if,
                                 gattc_spp_status()->connection_id,
                                 gattc_spp_status()->db[SPP_IDX_SPP_DATA_RECV_VAL].attribute_handle,
                                 len,
                                 buf,
                                 ESP_GATT_WRITE_TYPE_RSP,
                                 ESP_GATT_AUTH_REQ_NONE);
        free(buf);
    }
    vTaskDelete(NULL);
}

static void uart_init(void)
{
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };

    uart_param_config(UART_NUM_0, &uart_config);
    uart_set_pin(UART_NUM,
                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE,
                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

static void spp_task_init(void)
{
    xTaskCreate(uart_task, "uart_task", 2048, NULL, 8, NULL);
    xTaskCreate(spp_client_regist_task, "spp_client_regist_task", 2048, NULL, 10, NULL);
}

void app_main()
{
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();

    nvs_flash_init();

    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());

    cmd_regist_queue = xQueueCreate(10, sizeof(uint32_t));

    ESP_ERROR_CHECK(esp_ble_gap_register_callback(gap_event_handler));
    ESP_ERROR_CHECK(esp_ble_gattc_register_callback(gattc_event_handler));
    ESP_ERROR_CHECK(esp_ble_gattc_app_register(PROFILE_APP_ID));
    ESP_ERROR_CHECK(esp_ble_gatt_set_local_mtu(200));

    uart_init();
    spp_task_init();

    ESP_LOGI(TAG_SPP, "Task is started.");
}
