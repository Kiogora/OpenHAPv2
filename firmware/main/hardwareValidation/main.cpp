#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "esp_log.h"
#include "gpio.hpp"
#include "mlx90641.hpp"
#include "sds011.hpp"
#include "statistics.hpp"
#include "internalFlash.hpp"
#include "csv.hpp"

static const char *TAG = "Main";

extern "C" void app_main()
{
    /*Testing filename*/
    const std::string filename = "/data.csv";

    /*Thermal imager max temperature variable*/
    float maxThermalTemperature = 0.;    
    /*PM 2.5 measurement variable*/
    uint16_t pollutantConcentration = 0;

    //externalHardwareInterface::gpio warningLed(GPIO_NUM_18, externalHardwareInterface::gpio::output);
    externalHardwareSubsystem::particulateSensor::SDS011 particulateSensor;
    externalHardwareSubsystem::thermalImaging::MLX90641 thermalImager;
    internalHardwareSubsystem::storage::spiFlashFilesystem internalFilesystem;

    ESP_LOGI(TAG, "Thermal imager refresh rate: %.1f Hz", thermalImager.getPrintableRefreshRate());
    ESP_LOGI(TAG, "Thermal imager resolution: %d bit", thermalImager.getPrintableResolution());


    // softwareUtilities::csv::csvfile csv(internalFilesystem.mountPoint+filename);
    // //csv << "X" << "VALUE" << softwareUtilities::csv::endrow;
    // // Data
    // int i = 1;
    // //csv << i++ << "String value" << softwareUtilities::csv::endrow;
    // csv << i++ << 123 << csv.endrow();
    // //csv << i++ << 1.f << softwareUtilities::csv::endrow;
    // csv << i++ << 1.2 << softwareUtilities::csv::endrow<<softwareUtilities::csv::flush;
    // //csv << i++ << "One more string" << softwareUtilities::csv::endrow;
    // //csv << i++ << "\"Escaped\"" << softwareUtilities::csv::endrow<<

    internalFilesystem.printBytesAvailable();


    while (1)
    {
        maxThermalTemperature = softwareUtilities::stats::findMax(thermalImager.getAndPrintImage(), static_cast<size_t>(thermalImager.pixelCount));
        ESP_LOGI(TAG, "Max thermal temperature: %f°C", maxThermalTemperature);

        particulateSensor.getParticulateMeasurement(pollutantConcentration);
        ESP_LOGI(TAG, "PM 2.5 value: %u μg/m³", pollutantConcentration);
    }
}