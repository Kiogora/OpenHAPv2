#include <cstddef>
#include <string>
#include "esp_err.h"
#include "esp_log.h"
#include "esp_spiffs.h"
#include "internalFlash.hpp"
#include <fstream>

internalHardwareSubsystem::storage::spiFlashFilesystem::spiFlashFilesystem(std::string mountPoint, size_t maxFiles, bool formatIfMountFails)
: mountPoint(mountPoint)
{
    ESP_LOGI(TAG.c_str(), "Initializing SPIFFS");

    esp_vfs_spiffs_conf_t fsConfiguration =
    {
    .base_path = mountPoint.c_str(),
    .partition_label = NULL,
    .max_files = maxFiles,
    .format_if_mount_failed = formatIfMountFails
    };

    esp_err_t ret = esp_vfs_spiffs_register(&fsConfiguration);

    if (ret != ESP_OK) 
    {
        if (ret == ESP_FAIL) 
        {
            ESP_LOGE(TAG.c_str(), "Failed to mount or format filesystem");
        } else if (ret == ESP_ERR_NOT_FOUND)
        {
            ESP_LOGE(TAG.c_str(), "Failed to find SPIFFS partition");
        } else
        {
            ESP_LOGE(TAG.c_str(), "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
        }
    }   

    printBytesAvailable();
}

esp_err_t internalHardwareSubsystem::storage::spiFlashFilesystem::getBytesAvailable(size_t& used, size_t& total)
{
    esp_err_t ret = esp_spiffs_info(NULL, &total, &used);
    return ret;
}

esp_err_t internalHardwareSubsystem::storage::spiFlashFilesystem::printBytesAvailable()
{
    size_t used, total;
    esp_err_t ret = esp_spiffs_info(NULL, &total, &used);
    if (ret != ESP_OK) 
    {
        ESP_LOGE(TAG.c_str(), "Failed to get SPIFFS partition information (%s)", esp_err_to_name(ret));
    } 
    else 
    {
        ESP_LOGI(TAG.c_str(), "Partition size: total: %d, used: %d", total, used);
    }
    return ret;
}
