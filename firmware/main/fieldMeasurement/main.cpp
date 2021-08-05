#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "esp_err.h"
#include "esp_log.h"

#include "eddystoneScanner.hpp"

static const char *TAG = "FIELD_MEASUREMENT";
extern "C" void app_main()
{
    ESP_LOGI(TAG, "Registration ok");
    /*Setup eddystone scanner*/
    internalHardwareSubsystem::bluetooth::eddystoneScanner activityDetector;

    while (1)
    {
        vTaskDelay(1000/portTICK_RATE_MS);
    }
}