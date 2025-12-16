/*
 * SPDX-FileCopyrightText: 2021-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"

#if CONFIG_BT_NIMBLE_ENABLED
#include "host/ble_hs.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#else
#include "esp_bt_defs.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#endif

#if CONFIG_BT_NIMBLE_ENABLED
#include "host/ble_hs.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#define ESP_BD_ADDR_STR         "%02x:%02x:%02x:%02x:%02x:%02x"
#define ESP_BD_ADDR_HEX(addr)   addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]
#else
#include "esp_bt_defs.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#endif

#include "esp_hidh.h"
#include "esp_hid_common.h"
#include "esp_hid_gap.h"

// data transfer over uart
#include "driver/uart.h"
#include "driver/gpio.h"

#if CONFIG_BT_HID_HOST_ENABLED
const int device_list_len = 2;
static const char * remote_device_names[] = { "Perixx_BT_Mouse", "Bluetooth 5.0 Keyboard" };
#endif // CONFIG_BT_HID_HOST_ENABLED

static uart_port_t s_link_uart; 
static int s_link_tx;           
static int s_link_rx;

static const char *TAG = "ESP_BLUETOOTH_HOST_HID";

// callback types
typedef void (*hidh_mouse_input_cb_t)(esp_hidh_dev_t *dev, const uint8_t *data, size_t len, esp_hid_usage_t usage);
typedef void (*hidh_keyboard_input_cb_t)(esp_hidh_dev_t *dev, const uint8_t *data, size_t len, esp_hid_usage_t usage);

// callbacks
static hidh_mouse_input_cb_t s_hidh_mouse_cb = NULL;
static hidh_keyboard_input_cb_t s_hidh_keyboard_cb = NULL;

typedef struct {
    int16_t dx;       // relative movement x
    int16_t dy;       // relative movement y
    uint8_t buttons;  // bitmask: bit0=left button, bit1=right button
} hidh_mouse_rel_t;

// inline helper
static inline bool hidh_mouse_btn_left(uint8_t b)   { return (b & 0x01) != 0; }
static inline bool hidh_mouse_btn_right(uint8_t b)  { return (b & 0x02) != 0; }

// parser for mouse movements
bool hidh_mouse_parse_rel(const uint8_t *data, size_t len, hidh_mouse_rel_t *out)
{
    if (!data || !out) return false;
    
    // defaults
    out->dx = 0;
    out->dy = 0;
    out->buttons = 0;
    
    if (len == 7) {
    
        out->buttons = data[0];
        out->dx = (int16_t)((data[2] << 8) | data[1]);
        out->dy = (int16_t)((data[4] << 8) | data[3]);   
    
    } else if (len == 4) {
        
        out->buttons = data[0];
        out->dx = (int8_t)data[1];
        out->dy = (int8_t)data[2];      
    
    } else return false;
    
    return true;
}

// set callbacks
void hidh_set_input_callbacks(hidh_mouse_input_cb_t mouse_cb, hidh_keyboard_input_cb_t keyboard_cb)
{
    s_hidh_mouse_cb = mouse_cb;
    s_hidh_keyboard_cb = keyboard_cb;
}

// helpers to identify mouse and keyboard
static inline bool hidh_is_mouse_usage(esp_hid_usage_t u)
{
    const char *s = esp_hid_usage_str(u);
    return (s && strcmp(s, "MOUSE") == 0);
}

static inline bool hidh_is_keyboard_usage(esp_hid_usage_t u)
{
    const char *s = esp_hid_usage_str(u);
    return (s && strcmp(s, "KEYBOARD") == 0);
}

// transfer bluetooth data via uart
void hid_link_uart_init(uart_port_t uart, int tx_pin, int rx_pin, int baud)
{
    s_link_uart = uart;
    s_link_tx = tx_pin;
    s_link_rx = rx_pin;
    
    uart_config_t cfg = {
        .baud_rate = baud,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    
    uart_param_config(uart, &cfg);
    uart_set_pin(uart, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  
    uart_driver_install(uart, 1024, 0, 0, NULL, 0);
}

static inline void hid_link_send_packet(char type, const uint8_t *data, size_t len)
{
    if ((len && !data) || len > 0xFFFF) return;
    
    uint8_t hdr[3];
    hdr[0] = (uint8_t)type;                  // 'M' or 'K'
    hdr[1] = (uint8_t)(len & 0xFF);          // length low byte
    hdr[2] = (uint8_t)((len >> 8) & 0xFF);   // length high byte
    
    // XOR checksum
    uint8_t chk = hdr[0] ^ hdr[1] ^ hdr[2];
    for (size_t i = 0; i < len; ++i) {
        chk ^= data[i];
    }
    
    // send: header, payload, checksum
    uart_write_bytes(s_link_uart, (const char *)hdr, sizeof(hdr));
    if (len) {
        uart_write_bytes(s_link_uart, (const char *)data, len);
    }
    uart_write_bytes(s_link_uart, (const char *)&chk, 1);
    // Optional: uart_wait_tx_done(s_link_uart, pdMS_TO_TICKS(10));

}

#if !CONFIG_BT_NIMBLE_ENABLED
static char *bda2str(uint8_t *bda, char *str, size_t size)
{
    if (bda == NULL || str == NULL || size < 18) {
        return NULL;
    }

    uint8_t *p = bda;
    sprintf(str, "%02x:%02x:%02x:%02x:%02x:%02x",
            p[0], p[1], p[2], p[3], p[4], p[5]);
    return str;
}
#endif

bool after_hid_open = false;

void hidh_callback(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
    esp_hidh_event_t event = (esp_hidh_event_t)id;
    esp_hidh_event_data_t *param = (esp_hidh_event_data_t *)event_data;

    switch (event) {
    case ESP_HIDH_OPEN_EVENT: {
        if (param->open.status == ESP_OK) {
            const uint8_t *bda = esp_hidh_dev_bda_get(param->open.dev);
            if (bda) {
                ESP_LOGI(TAG, ESP_BD_ADDR_STR " OPEN: %s", ESP_BD_ADDR_HEX(bda), esp_hidh_dev_name_get(param->open.dev));
                esp_hidh_dev_dump(param->open.dev, stdout);
                after_hid_open = true;
            }
        } else {
            ESP_LOGE(TAG, " OPEN failed!");
        }
        break;
    }
    case ESP_HIDH_BATTERY_EVENT: {
        const uint8_t *bda = esp_hidh_dev_bda_get(param->battery.dev);
        if (bda) {
            ESP_LOGI(TAG, ESP_BD_ADDR_STR " BATTERY: %d%%", ESP_BD_ADDR_HEX(bda), param->battery.level);
        }
        break;
    }
    
    case ESP_HIDH_INPUT_EVENT: {
        vTaskDelay(1);
        const uint8_t *bda = esp_hidh_dev_bda_get(param->input.dev);
        if (bda) {
            ESP_LOGI(TAG, ESP_BD_ADDR_STR " INPUT: %8s, MAP: %2u, ID: %3u, Len: %d, Data:",
                     ESP_BD_ADDR_HEX(bda),
                     esp_hid_usage_str(param->input.usage),
                     param->input.map_index,
                     param->input.report_id,
                     param->input.length);
            ESP_LOG_BUFFER_HEX(TAG, param->input.data, param->input.length);  
     
            // call mouse or keyboard callback
            esp_hid_usage_t u = param->input.usage;
        
            if (hidh_is_mouse_usage(u)) {
                if (s_hidh_mouse_cb) {
                    s_hidh_mouse_cb(param->input.dev, param->input.data, param->input.length, u);
                }
            } else if (hidh_is_keyboard_usage(u)) {
                if (s_hidh_keyboard_cb) {
                    s_hidh_keyboard_cb(param->input.dev, param->input.data, param->input.length, u);
                }
            } else {
                ESP_LOGD(TAG, "Unhandled HID usage: %s", esp_hid_usage_str(u));
            }
        }
        break;        
    }
    
    case ESP_HIDH_FEATURE_EVENT: {
        const uint8_t *bda = esp_hidh_dev_bda_get(param->feature.dev);
        if (bda) {
            ESP_LOGI(TAG, ESP_BD_ADDR_STR " FEATURE: %8s, MAP: %2u, ID: %3u, Len: %d", ESP_BD_ADDR_HEX(bda),
                    esp_hid_usage_str(param->feature.usage), param->feature.map_index, param->feature.report_id,
                    param->feature.length);
            ESP_LOG_BUFFER_HEX(TAG, param->feature.data, param->feature.length);
        }
        break;
    }
    case ESP_HIDH_CLOSE_EVENT: {
        const uint8_t *bda = esp_hidh_dev_bda_get(param->close.dev);
        if (bda) {
            ESP_LOGI(TAG, ESP_BD_ADDR_STR " CLOSE: %s", ESP_BD_ADDR_HEX(bda), esp_hidh_dev_name_get(param->close.dev));
        }
        break;
    }
    default:
        ESP_LOGI(TAG, "EVENT: %d", event);
        break;
    }
}

#define SCAN_DURATION_SECONDS 5

void hid_demo_task(void *pvParameters)
{
    size_t results_len = 0;
    esp_hid_scan_result_t *results = NULL;
    ESP_LOGI(TAG, "SCAN...");
    //start scan for HID devices
    esp_hid_scan(SCAN_DURATION_SECONDS, &results_len, &results);
    ESP_LOGI(TAG, "SCAN: %u results", results_len);
    if (results_len) {
        esp_hid_scan_result_t *r = results;
        esp_hid_scan_result_t *cr = NULL;
        while (r) {
            printf("  %s: " ESP_BD_ADDR_STR ", ", (r->transport == ESP_HID_TRANSPORT_BLE) ? "BLE" : "BT ", ESP_BD_ADDR_HEX(r->bda));
            printf("RSSI: %d, ", r->rssi);
            printf("USAGE: %s, ", esp_hid_usage_str(r->usage));
#if CONFIG_BT_BLE_ENABLED
            if (r->transport == ESP_HID_TRANSPORT_BLE) {
                cr = r;
                printf("APPEARANCE: 0x%04x, ", r->ble.appearance);
                printf("ADDR_TYPE: '%s', ", ble_addr_type_str(r->ble.addr_type));
            }
#endif /* CONFIG_BT_BLE_ENABLED */
#if CONFIG_BT_NIMBLE_ENABLED
            if (r->transport == ESP_HID_TRANSPORT_BLE) {
                cr = r;
                printf("APPEARANCE: 0x%04x, ", r->ble.appearance);
                printf("ADDR_TYPE: '%d', ", r->ble.addr_type);
            }
#endif /* CONFIG_BT_BLE_ENABLED */
#if CONFIG_BT_HID_HOST_ENABLED
            if (r->transport == ESP_HID_TRANSPORT_BT) {
                cr = r;
                printf("COD: %s[", esp_hid_cod_major_str(r->bt.cod.major));
                esp_hid_cod_minor_print(r->bt.cod.minor, stdout);
                printf("] srv 0x%03x, ", r->bt.cod.service);
                print_uuid(&r->bt.uuid);
                printf(", ");
 /*               if (strncmp(r->name, remote_device_names, strlen(remote_device_names)) == 0) {
                    break;
                } */
            }
#endif /* CONFIG_BT_HID_HOST_ENABLED */
            printf("NAME: %s ", r->name ? r->name : "");
            printf("\n");
            
#if CONFIG_BT_HID_HOST_ENABLED
            // only open files found in device list
            if (cr) {
                for (size_t i = 0; i < device_list_len; i++) {
                    if (strncmp(cr->name, remote_device_names[i], strlen(remote_device_names[i])) == 0) {
                        esp_hidh_dev_open(cr->bda, cr->transport, cr->ble.addr_type);
 
                        ESP_LOGI(TAG, "Open");
                        
                        // wait for HID_OPEN_EVENT (hack)
                        while (!after_hid_open) { vTaskDelay(pdMS_TO_TICKS(100)); }
                        after_hid_open = false;
                        
                         ESP_LOGI(TAG, "Nach Openevent");
                        
                        break; // found device in list -> open device
                    }
                }
            }
#else
            // open each device found
            if (cr) {
                //open device
                esp_hidh_dev_open(cr->bda, cr->transport, cr->ble.addr_type);
                
                // wait for HID_OPEN_EVENT (hack)
                while (!after_hid_open) { vTaskDelay(pdMS_TO_TICKS(100)); }
                after_hid_open = false;
            }
#endif // CONFIG_BT_HID_HOST_ENABLED
            
            r = r->next;
        }

        //free the results
        esp_hid_scan_results_free(results);
    }
    vTaskDelete(NULL);
}

#if CONFIG_BT_NIMBLE_ENABLED
void ble_hid_host_task(void *param)
{
    ESP_LOGI(TAG, "BLE Host Task Started");
    /* This function will return only when nimble_port_stop() is executed */
    nimble_port_run();

    nimble_port_freertos_deinit();
}
void ble_store_config_init(void);
#endif

static uint8_t parsed_data[5];

void my_mouse_cb(esp_hidh_dev_t *dev, const uint8_t *data, size_t len, esp_hid_usage_t usage)
{
    if (!data || len == 0)
        return;
    
    hidh_mouse_rel_t r;
    if (hidh_mouse_parse_rel(data, len, &r)) {
            ESP_LOGD(TAG, "Mouse dx=%d dy=%d btn=0x%02X", r.dx, r.dy, r.buttons);
    
            parsed_data[0] = r.buttons;
            parsed_data[1] = (uint8_t)(r.dx & 0xFF);      // low byte
            parsed_data[2] = (uint8_t)((r.dx >> 8) & 0xFF); // high byte
            parsed_data[3] = (uint8_t)(r.dy & 0xFF);
            parsed_data[4] = (uint8_t)((r.dy >> 8) & 0xFF);
    
            hid_link_send_packet('M', parsed_data, 5);
                 
        } else {
            ESP_LOGW(TAG, "unknown mouse report (len=%d)", (int)len);
    }
}

void my_keyboard_cb(esp_hidh_dev_t *dev, const uint8_t *data, size_t len, esp_hid_usage_t usage)
{
    if (!data || len == 0)
        return;
  
    hid_link_send_packet('K', data, len);
}

void app_main(void)
{
    esp_err_t ret;
    
    // init uart to send bluetooth hid data
    hid_link_uart_init(UART_NUM_1, GPIO_NUM_4, GPIO_NUM_5, 921600);
    
#if HID_HOST_MODE == HIDH_IDLE_MODE
    ESP_LOGE(TAG, "Please turn on BT HID host or BLE!");
    return;
#endif
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );
    ESP_LOGI(TAG, "setting hid gap, mode:%d", HID_HOST_MODE);
    ESP_ERROR_CHECK( esp_hid_gap_init(HID_HOST_MODE) );
#if CONFIG_BT_BLE_ENABLED
    ESP_ERROR_CHECK( esp_ble_gattc_register_callback(esp_hidh_gattc_event_handler) );
#endif /* CONFIG_BT_BLE_ENABLED */
    esp_hidh_config_t config = {
        .callback = hidh_callback, 
        .event_stack_size = 4096,
        .callback_arg = NULL,
    };
    ESP_ERROR_CHECK( esp_hidh_init(&config) );

#if !CONFIG_BT_NIMBLE_ENABLED
    char bda_str[18] = {0};
    ESP_LOGI(TAG, "Own address:[%s]", bda2str((uint8_t *)esp_bt_dev_get_address(), bda_str, sizeof(bda_str)));
#endif

    // set callbacks for mouse an keyboard
    hidh_set_input_callbacks(my_mouse_cb, my_keyboard_cb);


#if CONFIG_BT_NIMBLE_ENABLED
    /* XXX Need to have template for store */
    ble_store_config_init();

    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;
	/* Starting nimble task after gatts is initialized*/
    ret = esp_nimble_enable(ble_hid_host_task);
    if (ret) {
        ESP_LOGE(TAG, "esp_nimble_enable failed: %d", ret);
    }

    vTaskDelay(200);

    uint8_t own_addr_type = 0;
    int rc;
    uint8_t addr_val[6] = {0};

    rc = ble_hs_id_copy_addr(BLE_ADDR_PUBLIC, NULL, NULL);

    rc = ble_hs_id_infer_auto(0, &own_addr_type);

    if (rc != 0) {
        ESP_LOGI(TAG, "error determining address type; rc=%d\n", rc);
        return;
    }

    rc = ble_hs_id_copy_addr(own_addr_type, addr_val, NULL);

    ESP_LOGI(TAG, "Device Address: ");
    ESP_LOGI(TAG, "%02x:%02x:%02x:%02x:%02x:%02x \n", addr_val[5], addr_val[4], addr_val[3],
		                                      addr_val[2], addr_val[1], addr_val[0]);

#endif
    xTaskCreate(&hid_demo_task, "hid_task", 6 * 1024, NULL, 2, NULL);
}
