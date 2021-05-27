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

static const char *TAG = "Main";

extern "C" void app_main()
{
    SemaphoreHandle_t i2cBusAccessMutex{xSemaphoreCreateMutex()};
    bool unifiedGpioState = false;
    /*Thermal image buffer*/
    float thermalImage[externalHardwareSubsystem::thermalImaging::MLX90641::pixelCount] = {0};
    /*PM 2.5 measurement variable*/
    uint16_t pollutantConcentration;
    float maxThermalTemperature;

    //externalHardwareInterface::gpio warningLed(GPIO_NUM_18, externalHardwareInterface::gpio::output);
    externalHardwareSubsystem::particulateSensor::SDS011 particulateSensor(GPIO_NUM_22, GPIO_NUM_23, GPIO_NUM_26);
    externalHardwareSubsystem::thermalImaging::MLX90641 thermalImager(i2cBusAccessMutex);

    thermalImager.getAndPrintImage(thermalImage);
    while (1) {
        //unifiedGpioState? warningLed.on() : warningLed.off();
        particulateSensor.getMeasurementPoint(pollutantConcentration);

        /* Toggle the LED state */
        //unifiedGpioState = !unifiedGpioState;
        vTaskDelay(5000/portTICK_RATE_MS);
    }
}