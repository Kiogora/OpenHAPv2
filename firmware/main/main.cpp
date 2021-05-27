/* Blink Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include "esp_log.h"
#include "gpio.hpp"
//#include "sdmmc.hpp"
//#include "ds3231.hpp"
#include "mlx90641.hpp"
#include "sds011.hpp"
#include "statistics.hpp"

static const char *TAG = "Main";

extern "C" void app_main()
{
    /*I2C bus mutex to avoid bus contention*/
    SemaphoreHandle_t i2cBusAccessMutex{xSemaphoreCreateMutex()};

    /*Thermal imager max temperature variable*/
    float maxThermalTemperature = 0.;    
    /*PM 2.5 measurement variable*/
    uint16_t pollutantConcentration = 0;

    //externalHardwareInterface::gpio warningLed(GPIO_NUM_18, externalHardwareInterface::gpio::output);
    externalHardwareSubsystem::particulateSensor::SDS011 particulateSensor(GPIO_NUM_22, GPIO_NUM_23, GPIO_NUM_26);
    externalHardwareSubsystem::thermalImaging::MLX90641 thermalImager(i2cBusAccessMutex);

    ESP_LOGI(TAG, "Thermal imager refresh rate: %f Hz", thermalImager.getPrintableRefreshRate());
    ESP_LOGI(TAG, "Thermal imager resolution: %d bit", thermalImager.getPrintableResolution());
    
    while (1)
    {
        maxThermalTemperature = softwareUtilities::stats::findMax(thermalImager.getAndPrintImage(), static_cast<size_t>(thermalImager.pixelCount));
        ESP_LOGI(TAG, "Max thermal temperature: %f°C", maxThermalTemperature);

        particulateSensor.getParticulateMeasurement(pollutantConcentration);
        ESP_LOGI(TAG, "PM 2.5 value: %u μg/m³", pollutantConcentration);
    }
}