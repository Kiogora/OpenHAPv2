#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <algorithm>
#include <ctime>
#include "esp_log.h"
#include "gpio.hpp"
#include "mlx90641.hpp"
#include "sds011.hpp"
#include "internalFlash.hpp"
#include "ds3231.hpp"

static const char *TEST_TAG = "TEST";
static const char *RESULT_TAG = "RESULT";

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

    ESP_LOGI(TEST_TAG, "WARNING LED - OBSERVE\n");

    for(int count = 0, max = 10; count < max; ++count)
    {
        ESP_LOGI(RESULT_TAG, "warning LED state %d of %d", count, max);
        warningLed.toggle();
        vTaskDelay(1000/portTICK_RATE_MS);
    }

    ESP_LOGI(TEST_TAG, "I2C BUS DEVICE DETECTION\n");

    thermalImager.scanBusAddresses();

    ESP_LOGI(TEST_TAG, "EXTERNAL RTC\n");
    
    std::time_t unixtime_now = 1555425481;
    while (ds3231.set_time(unixtime_now) != ESP_OK)
    {
        ESP_LOGI(RESULT_TAG, "Could not set time\n");
        vTaskDelay(250 / portTICK_PERIOD_MS);
    }
    ESP_LOGI(RESULT_TAG, "Set time to: UTC %s", std::ctime(&unixtime_now));
    while (ds3231.get_time(unixtime_now) != ESP_OK)
    {
        ESP_LOGE(RESULT_TAG, "Could not get time\n");
    }
    ESP_LOGI(RESULT_TAG, "Time obtained is: UTC %s", std::ctime(&unixtime_now));

    ESP_LOGI(TEST_TAG, "THERMOPILE SENSOR\n");

    const float* imageBuffer = thermalImager.GetImage();
    maxThermalTemperature = *std::max_element(imageBuffer, imageBuffer+thermalImager.pixelCount);
    ESP_LOGI(RESULT_TAG, "Max thermopile sensor array pixel temperature: %f°C\n", maxThermalTemperature);

    ESP_LOGI(TEST_TAG, "PARTICULATE SENSOR AND LOADSWITCH\n");
    
    particulateSensor.powerState.on();
    particulateSensor.getParticulateMeasurement(pollutantConcentration);
    ESP_LOGI(RESULT_TAG, "PM 2.5 value: %u μg/m³\n", pollutantConcentration);
    particulateSensor.powerState.off();
    ESP_LOGI(RESULT_TAG, "End of test!\n");
    while (1)
    {
        vTaskDelay(1000/portTICK_RATE_MS);
    }
}