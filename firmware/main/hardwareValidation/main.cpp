#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <algorithm>
#include <ctime>
#include "esp_log.h"
#include "gpio.hpp"
#include "mlx90641.hpp"
#include "sds011.hpp"
#include "statistics.hpp"
#include "internalFlash.hpp"
#include "ds3231.hpp"

static const char *TAG = "Main";

extern "C" void app_main()
{
    /*Thermal imager max temperature variable*/
    float maxThermalTemperature = 0.;    
    /*PM 2.5 measurement variable*/
    uint16_t pollutantConcentration = 0;

    externalHardwareInterface::gpio warningLed(GPIO_NUM_18, externalHardwareInterface::gpio::output);

    externalHardwareSubsystem::particulateSensor::SDS011 particulateSensor;
    externalHardwareSubsystem::thermalImaging::MLX90641 thermalImager;
    externalHardwareSubsystem::timekeeping::DS3231 ds3231(thermalImager);

    ESP_LOGI(TAG, "*****Warning LED test - observation*****");
    bool warningLedState = false;
    int count = 0;
    while(count < 10)
    {
        warningLed.write(warningLedState);
        warningLedState = !warningLedState;
        vTaskDelay(1000/portTICK_RATE_MS);
        ++count;
    }

    ESP_LOGI(TAG, "*****I2C bus devices detection test*****");
    thermalImager.scanBusAddresses();

    ESP_LOGI(TAG, "*****RTC test*****");
    std::time_t unixtime_now = 1555425481;

    while (ds3231.set_time(unixtime_now) != ESP_OK)
    {
        ESP_LOGI(TAG, "Could not set time\n");
        vTaskDelay(250 / portTICK_PERIOD_MS);
    }
    ESP_LOGI(TAG, "Set time to: UTC %s", std::ctime(&unixtime_now));

    while (ds3231.get_time(unixtime_now) != ESP_OK)
    {
        ESP_LOGE(TAG, "Could not get time");
    }
    ESP_LOGI(TAG, "Time obtained is: UTC %s", std::ctime(&unixtime_now));

    ESP_LOGI(TAG, "*****Thermopile sensor test*****");
    const float* imageBuffer = thermalImager.GetImage();
    maxThermalTemperature = *std::max_element(imageBuffer, imageBuffer+thermalImager.pixelCount);
    ESP_LOGI(TAG, "Max thermopile sensor array pixel temperature: %f°C", maxThermalTemperature);

    ESP_LOGI(TAG, "*****Particulate sensor and loadswitch LED test*****");
    particulateSensor.powerState.on();
    particulateSensor.getParticulateMeasurement(pollutantConcentration);
    ESP_LOGI(TAG, "PM 2.5 value: %u μg/m³", pollutantConcentration);
    particulateSensor.powerState.off();
    ESP_LOGI(TAG, "End of test!");
    while (1)
    {
        vTaskDelay(1000/portTICK_RATE_MS);
    }
}