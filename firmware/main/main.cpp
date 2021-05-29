#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "esp_log.h"
#include "gpio.hpp"
#include "mlx90641.hpp"
#include "sds011.hpp"
#include "statistics.hpp"
#include "spiffs.hpp"

static const char *TAG = "Main";

extern "C" void app_main()
{
    /*Thermal imager max temperature variable*/
    float maxThermalTemperature = 0.;    
    /*PM 2.5 measurement variable*/
    uint16_t pollutantConcentration = 0;

    //externalHardwareInterface::gpio warningLed(GPIO_NUM_18, externalHardwareInterface::gpio::output);
    externalHardwareSubsystem::particulateSensor::SDS011 particulateSensor;
    externalHardwareSubsystem::thermalImaging::MLX90641 thermalImager;
    softwareSubsystem::filesystems::spiffs internalSpiffsFilesystem;

    ESP_LOGI(TAG, "Thermal imager refresh rate: %.1f Hz", thermalImager.getPrintableRefreshRate());
    ESP_LOGI(TAG, "Thermal imager resolution: %d bit", thermalImager.getPrintableResolution());

    while (1)
    {
        maxThermalTemperature = softwareUtilities::stats::findMax(thermalImager.getAndPrintImage(), static_cast<size_t>(thermalImager.pixelCount));
        ESP_LOGI(TAG, "Max thermal temperature: %f°C", maxThermalTemperature);

        particulateSensor.getParticulateMeasurement(pollutantConcentration);
        ESP_LOGI(TAG, "PM 2.5 value: %u μg/m³", pollutantConcentration);
    }
}