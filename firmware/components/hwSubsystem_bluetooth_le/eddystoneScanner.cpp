#include "esp_bt.h"
#include <string.h>
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_gatt_defs.h"
#include "esp_gattc_api.h"
#include "esp_gap_ble_api.h"
#include "freertos/FreeRTOS.h"

#include "eddystoneScanner.hpp"

internalHardwareSubsystem::bluetooth::eddystoneScanner::scan_results internalHardwareSubsystem::bluetooth::eddystoneScanner::all_scans[internalHardwareSubsystem::bluetooth::eddystoneScanner::scan_history_len];
char internalHardwareSubsystem::bluetooth::eddystoneScanner::mac_string[18];
int16_t internalHardwareSubsystem::bluetooth::eddystoneScanner::running_packet_sum;
int16_t internalHardwareSubsystem::bluetooth::eddystoneScanner::running_rssi_sum;
uint8_t internalHardwareSubsystem::bluetooth::eddystoneScanner::desired_tag_addr[6];

static const char *TAG = "internalHardwareSubsystem::bluetooth";

internalHardwareSubsystem::bluetooth::eddystoneScanner::eddystoneScanner()
{
    esp_err_t status = ESP_FAIL;
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_bt_controller_init(&bt_cfg);
    esp_bt_controller_enable(ESP_BT_MODE_BLE);

    /*Zero out scan history buffer*/
    memset(all_scans, 0, scan_history_len*sizeof(scan_results));

    /*Read base MAC, we look for the tag with the same instance ID as this base MAC*/
    esp_err_t ret = esp_efuse_mac_get_default(desired_tag_addr);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to get base MAC address from EFUSE BLK0. (%s)", esp_err_to_name(ret));
        ESP_LOGE(TAG, "Aborting");
        abort();
    }
    else
    {
        ESP_LOGI(TAG, "Base MAC Address read from EFUSE BLK0");
    }

    
    snprintf(mac_string, sizeof(mac_string), "%02x:%02x:%02x:%02x:%02x:%02x",
             desired_tag_addr[0], desired_tag_addr[1], desired_tag_addr[2], desired_tag_addr[3], desired_tag_addr[4], desired_tag_addr[5]);

    esp_bluedroid_init();
    esp_bluedroid_enable();

    status = esp_ble_gap_register_callback(eddystoneScanner_event_callback);

    /*<! register the scan event callback function to the gap module */
    while(status != ESP_OK)
    {
        ESP_LOGE(TAG,"gap register error: %s", esp_err_to_name(status));
        status = esp_ble_gap_register_callback(eddystoneScanner_event_callback);
    }

    esp_ble_scan_params_t ble_scan_params = 
    {
        .scan_type              = BLE_SCAN_TYPE_ACTIVE,
        .own_addr_type          = BLE_ADDR_TYPE_PUBLIC,
        .scan_filter_policy     = BLE_SCAN_FILTER_ALLOW_ALL,
        .scan_interval          = 0x50,
        .scan_window            = 0x30,
        .scan_duplicate         = BLE_SCAN_DUPLICATE_DISABLE
    };

    esp_ble_gap_set_scan_params(&ble_scan_params);
}

void internalHardwareSubsystem::bluetooth::eddystoneScanner::eddystoneScanner_event_callback(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t* param)
{
    esp_err_t err;

    switch(event)
    {
        case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT: {
            uint32_t duration = 0;
            esp_ble_gap_start_scanning(duration);
            break;
        }
        case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT: {
            if((err = param->scan_start_cmpl.status) != ESP_BT_STATUS_SUCCESS)
            {
                ESP_LOGE(TAG,"Scan start failed: %s", esp_err_to_name(err));
            }
            else
            {
                ESP_LOGI(TAG,"Start scanning...");
            }
            break;
        }
        case ESP_GAP_BLE_SCAN_RESULT_EVT:
        {
            esp_ble_gap_cb_param_t* scan_result = (esp_ble_gap_cb_param_t*)param;
            switch(scan_result->scan_rst.search_evt)
            {
                case ESP_GAP_SEARCH_INQ_RES_EVT:
                {
                    esp_eddystone_result_t res;
                    memset(&res, 0, sizeof(res));
                    esp_err_t ret = eddystoneScanner_decode(scan_result->scan_rst.ble_adv, scan_result->scan_rst.adv_data_len, res);
                    if (ret)
                    {
                        // error:The received data is not an eddystone frame packet or a correct eddystone frame packet.
                        // just return
                        return;
                    }
                    else
                    {
                        // The received adv data is a correct eddystone frame packet.
                        // Here, we get the eddystone infomation in eddystone_res, we can use the data in res to do other things.
                        // For example, just print them:
                        esp_log_buffer_hex("Eddystone UID frame Found. TAG address:", res.inform.uid.instance_id, ESP_BD_ADDR_LEN);
                        uint8_t byte_match = 0;

                        for(int i = 0; i < ESP_BD_ADDR_LEN; ++i)
                        {
                            if(res.inform.uid.instance_id[i] == desired_tag_addr[i])
                            {
                                ++byte_match;
                            }
                        }
                        if (byte_match == ESP_BD_ADDR_LEN)
                        {
                            running_packet_sum += 1;
                            running_rssi_sum += (scan_result->scan_rst).rssi;
                            ESP_LOGI(TAG, "Successfully found desired tag. RSSI of packet:%d dbm", scan_result->scan_rst.rssi);
                            for(int idx=scan_history_len-2; idx>=0; --idx)
                            {
                                all_scans[idx+1] = all_scans[idx];
                                if(idx == 0)
                                {
                                    strlcpy(all_scans[idx].bt_address, mac_string, sizeof(all_scans[idx].bt_address));
                                    all_scans[idx].rssi = (scan_result->scan_rst).rssi;
                                }
                            }
                        }
                    }
                    break;
                }
                default:
                    break;
            }
            break;
        }
        case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
        {
            if((err = param->scan_stop_cmpl.status) != ESP_BT_STATUS_SUCCESS)
            {
                ESP_LOGE(TAG,"Scan stop failed: %s", esp_err_to_name(err));
            }
            else
            {
                ESP_LOGI(TAG,"Stop scan successfully");
            }
            break;
        }
        default:
            break;
    }
}

void internalHardwareSubsystem::bluetooth::eddystoneScanner::get_average_rssi(int16_t &packet_num, float &average)
{
    average = (float)running_rssi_sum/running_packet_sum;
    packet_num = running_packet_sum;
    running_rssi_sum = 0; 
    running_packet_sum =0;
}

esp_err_t internalHardwareSubsystem::bluetooth::eddystoneScanner::eddystoneScanner_decode(const uint8_t* buf, uint8_t len, esp_eddystone_result_t &res)
{
    if (len == 0 || buf == NULL)
    {
        return -1;
    }
    uint8_t pos=0;
    while(res.common.srv_data_type != EDDYSTONE_SERVICE_UUID)
    {
        pos++;
        if(pos >= len ) {
            return -1;
        }
        uint8_t ad_type = buf[pos++];
        switch(ad_type)
        {
            case ESP_BLE_AD_TYPE_FLAG: {
                res.common.flags = buf[pos++];
                break;
            }
            case ESP_BLE_AD_TYPE_16SRV_CMPL: {
                uint16_t uuid = little_endian_read_16(buf, pos);
                if(uuid != EDDYSTONE_SERVICE_UUID) {
                    return -1;
                }
                res.common.srv_uuid = uuid;
                pos += 2;
                break;
            }
            case ESP_BLE_AD_TYPE_SERVICE_DATA: {
                uint16_t type = little_endian_read_16(buf, pos);
                pos += 2;
                uint8_t frame_type = buf[pos++];
                if(type != EDDYSTONE_SERVICE_UUID || !(frame_type == EDDYSTONE_FRAME_TYPE_UID || frame_type == EDDYSTONE_FRAME_TYPE_URL ||
                   frame_type == EDDYSTONE_FRAME_TYPE_TLM)) {
                    return -1;
                }
                res.common.srv_data_type = type;
                res.common.frame_type = frame_type;
                break;
            }
            default:
                break;
        }
    }
    return eddystoneScanner_get_inform(buf+pos, len-pos, res);
}

esp_err_t internalHardwareSubsystem::bluetooth::eddystoneScanner::eddystoneScanner_uid_received(const uint8_t* buf, uint8_t len, esp_eddystone_result_t& res)
{
    uint8_t pos = 0;
    //1-byte Ranging Data + 10-byte Namespace + 6-byte Instance
    if((len != EDDYSTONE_UID_DATA_LEN) && (len != (EDDYSTONE_UID_RFU_LEN+EDDYSTONE_UID_DATA_LEN)))
    {
        //ERROR:uid len wrong
        return -1;
    }
    res.inform.uid.ranging_data = buf[pos++];
    for(int i=0; i<EDDYSTONE_UID_NAMESPACE_LEN; i++)
    {
        res.inform.uid.namespace_id[i] = buf[pos++];
    }
    for(int i=0; i<EDDYSTONE_UID_INSTANCE_LEN; i++)
    {
        res.inform.uid.instance_id[i] = buf[pos++];
    }
    return 0;
}

esp_err_t internalHardwareSubsystem::bluetooth::eddystoneScanner::eddystoneScanner_url_received(const uint8_t* buf, uint8_t len, esp_eddystone_result_t& res)
{
    char *url_res = NULL;
    uint8_t pos = 0;
    if(len-EDDYSTONE_URL_TX_POWER_LEN > EDDYSTONE_URL_MAX_LEN)
    {
        //ERROR:too long url
        return -1;
    }
    res.inform.url.tx_power = buf[pos++];
    url_res = eddystoneScanner_resolve_url_scheme(buf+pos, buf+len-1);
    memcpy(&(res.inform.url.url), url_res, strlen(url_res));
    res.inform.url.url[strlen(url_res)] = '\0';
    return 0;
}
char* internalHardwareSubsystem::bluetooth::eddystoneScanner::eddystoneScanner_resolve_url_scheme(const uint8_t* url_start, const uint8_t* url_end)
{
    int pos = 0;
    static char url_buf[100] = {0};
    const uint8_t *p = url_start;
    
    const char* eddystone_url_prefix[4] = {"http://www.","https://www.","http://","https://"}; 
    static const char* eddystone_url_encoding[14] =
    {
        ".com/",".org/",".edu/",".net/",".info/",".biz/",".gov/",
        ".com",".org",".edu",".net",".info",".biz",".gov"
    };

    pos += sprintf(&url_buf[pos], "%s", eddystone_url_prefix[*p++]);

    for (; p <= url_end; p++)
    {
        if (esp_eddystone_is_char_invalid((*p)))
        {
            pos += sprintf(&url_buf[pos], "%s", eddystone_url_encoding[*p]);
        } 
        else
        {
            pos += sprintf(&url_buf[pos], "%c", *p);
        }
    }
    return url_buf;
}
esp_err_t internalHardwareSubsystem::bluetooth::eddystoneScanner::eddystoneScanner_tlm_received(const uint8_t* buf, uint8_t len, esp_eddystone_result_t& res)
{
    uint8_t pos = 0;
    if(len > EDDYSTONE_TLM_DATA_LEN)
    {
        //ERROR:TLM too long
        return -1;
    }
    res.inform.tlm.version = buf[pos++];
    res.inform.tlm.battery_voltage = big_endian_read_16(buf, pos);
    pos += 2;
    uint16_t temp = big_endian_read_16(buf, pos);
    int8_t temp_integral = (int8_t)((temp >> 8) & 0xff);
    float temp_decimal = (temp & 0xff) / 256.0;
    res.inform.tlm.temperature = temp_integral + temp_decimal;
    pos += 2;
    res.inform.tlm.adv_count = big_endian_read_32(buf, pos);
    pos += 4;
    res.inform.tlm.time = big_endian_read_32(buf, pos);
    return 0;
}
esp_err_t internalHardwareSubsystem::bluetooth::eddystoneScanner::eddystoneScanner_get_inform(const uint8_t* buf, uint8_t len, esp_eddystone_result_t& res)
{
    static esp_err_t ret = -1;
    switch(res.common.frame_type)
    {
        case EDDYSTONE_FRAME_TYPE_UID: {
            ret = eddystoneScanner_uid_received(buf, len, res);
            break;
        }
        case EDDYSTONE_FRAME_TYPE_URL: {
            ret = eddystoneScanner_url_received(buf, len, res);
            break;
        }
        case EDDYSTONE_FRAME_TYPE_TLM: {
            ret = eddystoneScanner_tlm_received(buf, len, res);
            break;
        }
        default:
            break;
    }
    return ret;
}


inline uint16_t internalHardwareSubsystem::bluetooth::eddystoneScanner::little_endian_read_16(const uint8_t *buffer, uint8_t pos)
{
    return ((uint16_t)buffer[pos]) | (((uint16_t)buffer[(pos)+1]) << 8);
}

inline uint16_t internalHardwareSubsystem::bluetooth::eddystoneScanner::big_endian_read_16(const uint8_t *buffer, uint8_t pos)
{
    return (((uint16_t)buffer[pos]) << 8) | ((uint16_t)buffer[(pos)+1]);
}

inline uint32_t internalHardwareSubsystem::bluetooth::eddystoneScanner::big_endian_read_32(const uint8_t *buffer, uint8_t pos)
{
    return (((uint32_t)buffer[pos]) << 24) | (((uint32_t)buffer[(pos)+1]) << 16) | (((uint32_t)buffer[(pos)+2]) << 8) | ((uint32_t)buffer[(pos)+3]);
}

inline bool internalHardwareSubsystem::bluetooth::eddystoneScanner::esp_eddystone_is_char_invalid(int ch)
{
    return (ch >= 0x00 && ch <= 0x20) || (ch >= 0x7f && ch <= 0xff);
}