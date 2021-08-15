#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "esp_err.h"
#include "esp_log.h"

#include "eddystoneScanner.hpp"
#include "internalFlash.hpp"
#include "wifiManager.hpp"

static const char *TAG = "FIELD_MEASUREMENT";

extern "C" void app_main()
{
    /*Setup internal storage*/
    internalHardwareSubsystem::storage::spiFlashFilesystem internalStorage;
    internalStorage.printFilesOnDisk();
    /*Setup eddystone scanner*/
    internalHardwareSubsystem::bluetooth::eddystoneScanner activityDetector;
    /*Setup wifi access point*/
    internalHardwareSubsystem::wifi::wifiManager wifiSetup;
    /*Setup server on port 80 within wifi network - accessible via 192.168.4.1*/
    wifiSetup.startServer(internalStorage.mountPoint);

    while (1)
    {
        vTaskDelay(1000/portTICK_RATE_MS);
    }
}