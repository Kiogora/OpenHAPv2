#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "esp_err.h"
#include "esp_log.h"

#include <algorithm>
#include <ctime>

#include "gpio.hpp"
#include "time.hpp"
#include "ds3231.hpp"
#include "sds011.hpp"
#include "mlx90641.hpp"
#include "wifiManager.hpp"
#include "internalFlash.hpp"
#include "eddystoneScanner.hpp"

static const char *TAG = "FIELD_MEASUREMENT";

bool startMeasurement = false;

extern "C" void app_main()
{
    externalHardwareSubsystem::thermalImaging::MLX90641 thermalImager;
    externalHardwareSubsystem::timekeeping::DS3231 ds3231(thermalImager);
    externalHardwareSubsystem::particulateSensor::SDS011 particulateSensor; 
    externalHardwareInterface::gpio warningLed(GPIO_NUM_18, externalHardwareInterface::gpio::output); 
    
    /*Setup internal storage*/
    internalHardwareSubsystem::storage::spiFlashFilesystem internalStorage;
    internalStorage.printFilesOnDisk();

    /*Setup non-volatile settings*/
    internalHardwareSubsystem::storage::nonVolatileStorageSettings settings;
    
    /*Setup eddystone scanner*/
    internalHardwareSubsystem::bluetooth::eddystoneScanner activityDetector;

    /*Setup wifi access point*/
    internalHardwareSubsystem::wifi::wifiManager wifiSetup;

    /*Setup server on port 80 within wifi network - accessible via 192.168.4.1*/
    wifiSetup.startServer(activityDetector, settings, internalStorage, thermalImager, ds3231, particulateSensor, warningLed);

    while (1)
    {
        while(settings.isMeasurementActive() == true)
        {
            ESP_LOGI(TAG, "Sampling thermal camera...");
            const float* imageBuffer = thermalImager.GetImage();
            float maxThermalTemperature = *std::max_element(imageBuffer, imageBuffer+thermalImager.pixelCount);
            ESP_LOGI(TAG, "Completed sampling thermal camera");
            
            ESP_LOGI(TAG, "Sampling particulates...");
            float pollutantConcentration = 0;
            particulateSensor.powerState.on();
            particulateSensor.getParticulateMeasurement(pollutantConcentration);
            particulateSensor.powerState.off();
            ESP_LOGI(TAG, "Completed sampling particulates");

            ESP_LOGI(TAG, "Getting RTC time...");
            std::time_t  now;
            ds3231.get_time(now);
            ESP_LOGI(TAG, "Completed getting RTC time");

            int16_t packet_num;
            float rssi_average;
            activityDetector.get_average_rssi(packet_num, rssi_average);
            internalStorage.writeCSVEntry((std::string(wifiSetup.ssid)+".csv").c_str(), 
                                          now, packet_num, rssi_average, pollutantConcentration, 
                                          maxThermalTemperature);

            for(int count = 0, max = 4; count < max; ++count)
            {
                warningLed.toggle();
                vTaskDelay(300/portTICK_RATE_MS);
            }
            warningLed.off();
            vTaskDelay(60000/portTICK_RATE_MS);
        }
        ESP_LOGI(TAG, "Inactive measurement!");
        vTaskDelay(5000/portTICK_RATE_MS);
    }
}