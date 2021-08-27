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
    
    /*Setup eddystone scanner*/
    internalHardwareSubsystem::bluetooth::eddystoneScanner activityDetector;

    /*Setup wifi access point*/
    internalHardwareSubsystem::wifi::wifiManager wifiSetup;

    /*Setup server on port 80 within wifi network - accessible via 192.168.4.1*/
    wifiSetup.startServer(activityDetector, internalStorage, thermalImager, ds3231, particulateSensor, warningLed);

    while (1)
    {
        while(startMeasurement == true)
        {

        }
        vTaskDelay(1000/portTICK_RATE_MS);
    }
}