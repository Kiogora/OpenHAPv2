#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sys.h"
#include "wifiManager.hpp"

static const char* TAG = "internalHardwareSubsystem::wifi";

internalHardwareSubsystem::wifi::wifiManager::wifiManager(supportedWifiModes startupMode)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    if (startupMode == supportedWifiModes::accessPointMode)
    {
        ESP_ERROR_CHECK(esp_netif_init());
        ESP_ERROR_CHECK(esp_event_loop_create_default());
        esp_netif_create_default_wifi_ap();

        wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
        ESP_ERROR_CHECK(esp_wifi_init(&cfg));

        ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                            ESP_EVENT_ANY_ID,
                                                            &accessPointEventHandler,
                                                            NULL,
                                                            NULL));
        uint8_t baseMacAddress[6] = {0};
        //Get base MAC address from EFUSE BLK0(default option)
        // esp_err_t ret = esp_efuse_mac_get_default(baseMacAddress);
        // if (ret != ESP_OK)
        // {
        //     ESP_LOGE(TAG, "Failed to get base MAC address from EFUSE BLK0. (%s)", esp_err_to_name(ret));
        //     ESP_LOGE(TAG, "Aborting");
        //     abort();
        // }
        // else
        // {
        //     ESP_LOGI(TAG, "Base MAC Address read from EFUSE BLK0");
        // }

        // char ssid[32];
        // snprintf(ssid, sizeof(ssid), "OpenHAP_%02x:%02x:%02x:%02x:%02x:%02x\0",
        //          baseMacAddress[0], baseMacAddress[1], baseMacAddress[2], 
                //  baseMacAddress[3], baseMacAddress[4], baseMacAddress[5]);

        wifi_config_t wifi_config = {};
        strcpy((char*)wifi_config.ap.ssid, "OpenHAP");
        wifi_config.ap.ssid_len = strlen("Openhap");
        wifi_config.ap.channel = 6;
        strcpy((char*)wifi_config.ap.password, "");;
        wifi_config.ap.max_connection = 1;
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;

        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
        ESP_ERROR_CHECK(esp_wifi_start());
    }
}


void internalHardwareSubsystem::wifi::wifiManager::accessPointEventHandler(void* arg, esp_event_base_t event_base,
                                                                                 int32_t event_id, void* event_data)
{
    if (event_id == WIFI_EVENT_AP_STACONNECTED)
    {
        wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
        ESP_LOGI(TAG, "station connected, AID=%d", event->aid);
    } 
    else if (event_id == WIFI_EVENT_AP_STADISCONNECTED)
    {
        wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
        ESP_LOGI(TAG, "station disconnected, AID=%d", event->aid);
    }
}

