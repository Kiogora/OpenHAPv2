#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"

#include <algorithm>
#include <ctime>

#include "gpio.hpp"
#include "mlx90641.hpp"
#include "sds011.hpp"
#include "internalFlash.hpp"
#include "ds3231.hpp"
#include "time.hpp"

static const char *TEST_TAG = "TEST";
static const char *RESULT_TAG = "RESULT";

extern "C" void app_main()
{   /* Construct thermal imager object*/
    externalHardwareSubsystem::thermalImaging::MLX90641 thermalImager;
    /* Add DS3231 to thermal imager's I2C bus*/
    externalHardwareSubsystem::timekeeping::DS3231 ds3231(thermalImager);

#ifdef CONFIG_ENABLE_USER_LED_TEST_HARDWARE_VALIDATION
    externalHardwareInterface::gpio warningLed(GPIO_NUM_18, externalHardwareInterface::gpio::output); 
    ESP_LOGI(TEST_TAG, "WARNING LED - OBSERVE");
    for(int count = 0, max = 10; count < max; ++count)
    {
        ESP_LOGI(RESULT_TAG, "warning LED state %d of %d", count, max);
        warningLed.toggle();
        vTaskDelay(1000/portTICK_RATE_MS);
    }
#endif

#ifdef CONFIG_ENABLE_VISUALIZE_I2C_BUS_HARDWARE_VALIDATION
    ESP_LOGI(TEST_TAG, "I2C BUS DEVICE DETECTION");
    #ifdef CONFIG_ENABLE_THERMAL_IMAGER_TEST_HARDWARE_VALIDATION
        thermalImager.scanBusAddresses();
    #elif CONFIG_ENABLE_RTC_TEST_HARDWARE_VALIDATION
        ds3231.scanBusAddresses();
    #else
        ESP_LOGI(RESULT_TAG, "Error: No I2C device activated for test");
    #endif
#endif

#ifdef CONFIG_ENABLE_RTC_TEST_HARDWARE_VALIDATION
#ifdef CONFIG_ENABLE_SNTP_RTC_TEST_HARDWARE_VALIDATION
    std::time_t  now;
    while (ds3231.get_time(now) != ESP_OK)
    {
        ESP_LOGE(RESULT_TAG, "Could not get time from RTC");
    }
    ESP_LOGI(RESULT_TAG, "RTC time obtained prior to validation exercise is: UTC %s", std::ctime(&now));
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(example_connect());
    std::tm timeinfo;
    systemUtils::obtainSystemTimeFromSntp(timeinfo);
    now = std::mktime(&timeinfo);
#endif    
#ifdef CONFIG_ENABLE_STATIC_RTC_TEST_HARDWARE_VALIDATION
    std::time_t unixtime_now = 1555425481;
#endif
    ESP_LOGI(TEST_TAG, "EXTERNAL RTC");
    while (ds3231.set_time(now) != ESP_OK)
    {
        ESP_LOGI(RESULT_TAG, "Could not set RTC time");
        vTaskDelay(250 / portTICK_PERIOD_MS);
    }
    ESP_LOGI(RESULT_TAG, "Set RTC time to: UTC %s", std::ctime(&now));
    ESP_LOGI(RESULT_TAG, "Waiting for a few seconds then read back RTC time...");
    vTaskDelay(1000/portTICK_RATE_MS);
    while (ds3231.get_time(now) != ESP_OK)
    {
        ESP_LOGE(RESULT_TAG, "Could not get RTC time");
    }
    ESP_LOGI(RESULT_TAG, "RTC time read after time is set is: UTC %s", std::ctime(&now));
#endif

#ifdef CONFIG_ENABLE_THERMAL_IMAGER_TEST_HARDWARE_VALIDATION
    float maxThermalTemperature = 0.; 
    ESP_LOGI(TEST_TAG, "THERMOPILE SENSOR");
    const float* imageBuffer = thermalImager.GetImage();
    maxThermalTemperature = *std::max_element(imageBuffer, imageBuffer+thermalImager.pixelCount);
    ESP_LOGI(RESULT_TAG, "Max thermopile sensor array pixel temperature: %f??C", maxThermalTemperature);
#endif

#ifdef CONFIG_ENABLE_PM_SENSOR_TEST_HARDWARE_VALIDATION
    float pollutantConcentration = 0;
    externalHardwareSubsystem::particulateSensor::SDS011 particulateSensor; 
    ESP_LOGI(TEST_TAG, "PARTICULATE SENSOR AND LOADSWITCH");
    particulateSensor.powerState.on();
    particulateSensor.getParticulateMeasurement(pollutantConcentration);
    ESP_LOGI(RESULT_TAG, "PM 2.5 value: %f ??g/m??", pollutantConcentration);
    particulateSensor.powerState.off();
#endif

    internalHardwareSubsystem::storage::spiFlashFilesystem internalStorage;
    internalStorage.writeCSVEntry(
                                  "measurement.csv"
                                  #ifdef CONFIG_ENABLE_RTC_TEST_HARDWARE_VALIDATION
                                  ,now
                                  #else
                                  ,(std::time_t) 0
                                  #endif
                                  /*Not supported yet in hardware validation*/
                                  , "", -1
                                  #ifdef CONFIG_ENABLE_PM_SENSOR_TEST_HARDWARE_VALIDATION
                                  , pollutantConcentration
                                  #else
                                  ,0.
                                  #endif
                                  #ifdef CONFIG_ENABLE_THERMAL_IMAGER_TEST_HARDWARE_VALIDATION
                                  , maxThermalTemperature 
                                  #else
                                  ,0.
                                  #endif
                                );
    internalStorage.printFilesOnDisk();
    //internalStorage.deleteFile("measurement.csv");
    //internalStorage.printFilesOnDisk();

#ifdef CONFIG_ENABLE_DEFAULT_MAC_ADDRESS_RETRIEVAL_HARDWARE_VALIDATION
    uint8_t base_mac_addr[6] = {0};
    char macStr[18];

    esp_err_t ret = esp_efuse_mac_get_default(base_mac_addr);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TEST_TAG, "Failed to get base MAC address from EFUSE BLK0. (%s)", esp_err_to_name(ret));
    }

    snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",base_mac_addr[0], 
                                                                     base_mac_addr[1], 
                                                                     base_mac_addr[2], 
                                                                     base_mac_addr[3], 
                                                                     base_mac_addr[4], 
                                                                     base_mac_addr[5]);

    ESP_LOGI(TEST_TAG, "MAC address: %02x:%02x:%02x:%02x:%02x:%02x", base_mac_addr[0], 
                                                                     base_mac_addr[1],
                                                                     base_mac_addr[2],
                                                                     base_mac_addr[3],
                                                                     base_mac_addr[4],
                                                                     base_mac_addr[5]);
#endif

    ESP_LOGI(RESULT_TAG, "End of test!");
    while (1)
    {
        vTaskDelay(1000/portTICK_RATE_MS);
    }
}