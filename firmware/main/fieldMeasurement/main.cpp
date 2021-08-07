#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "esp_err.h"
#include "esp_log.h"

#include "eddystoneScanner.hpp"
#include "wifiManager.hpp"

static const char *TAG = "FIELD_MEASUREMENT";

extern "C" void app_main()
{
    /*Setup eddystone scanner*/
    internalHardwareSubsystem::bluetooth::eddystoneScanner activityDetector;
    internalHardwareSubsystem::wifi::wifiManager wifiSetup;

    while (1)
    {
        vTaskDelay(1000/portTICK_RATE_MS);
    }
}