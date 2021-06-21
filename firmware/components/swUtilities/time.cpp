#include <ctime>
#include <sys/time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "time.hpp"
#include "esp_sntp.h"
#include "sntp.h"

#define TAG "system::time"

static void initialize_sntp(void)
{
    ESP_LOGI(TAG, "Initializing SNTP");
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, "pool.ntp.org");
    sntp_init();
}

void systemUtils::obtainSystemTimeFromSntp(std::tm& timeinfo)
{
    initialize_sntp();

    std::time_t now = 0;
    const int retry_count = 10;

    for(int retry = 0; sntp_get_sync_status() == SNTP_SYNC_STATUS_RESET && retry < retry_count; ++retry)
    {
        ESP_LOGI(TAG, "Waiting for system time to be set... (%d/%d)", retry, retry_count);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
    time(&now);
    timeinfo = *std::gmtime(&now);
}

bool systemUtils::isSystemTimeInvalid()
{
    std::time_t now;
    std::tm* timeinfo;
    time(&now);
    timeinfo = std::gmtime(&now);
    return (timeinfo->tm_year < (2016 - 1900));
}