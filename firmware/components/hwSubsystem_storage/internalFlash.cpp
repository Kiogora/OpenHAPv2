#include <cstddef>
#include <string>
#include <sys/stat.h>
#include <sys/unistd.h>

#include "esp_err.h"
#include "esp_log.h"
#include "esp_spiffs.h"
#include "internalFlash.hpp"

static const char* TAG = "internalHardwareSubsystem::storage";

internalHardwareSubsystem::storage::spiFlashFilesystem::spiFlashFilesystem(std::string mountPoint, size_t maxFiles, bool formatIfMountFails)
: mountPoint(mountPoint)
{
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
            ESP_LOGE(TAG, "Failed to mount or format filesystem");
        } else if (ret == ESP_ERR_NOT_FOUND)
        {
            ESP_LOGE(TAG, "Failed to find SPIFFS partition");
        } else
        {
            ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
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
        ESP_LOGE(TAG, "Failed to get SPIFFS partition information (%s)", esp_err_to_name(ret));
    } 
    else 
    {
        ESP_LOGI(TAG, "Partition size: total: %d, used: %d", total, used);
    }
    return ret;
}

esp_err_t internalHardwareSubsystem::storage::spiFlashFilesystem::writeCSV(uint32_t time_now, std::string macAddress, int16_t averageRssi, float particulateConcentration, float maxTemp)
{
    int i;
    struct stat st;
    FILE* file = NULL;
    bool fileExists = false;
    const char* fileName = (mountPoint+"/measurement.csv").c_str();

    /*Open file by append mode, if file does not exist, create it*/
    if (stat(fileName, &st) == 0)
    {
        /*File exists, set integer to be checked*/
        ESP_LOGI(TAG, "File exists, appending to file");
        fileExists = true; 
    }
    else
    {
        /*File exists, set integer to be checked*/
        ESP_LOGW(TAG, "File does not exist, creating it...");
        fileExists = true;           
    }

    if(fileExists)
    {
        file = fopen(fileName, "a");
        if (file == NULL)
        {
            
            ESP_LOGE(TAG, "Failed to open file for writing");
            return ESP_FAIL;
        }
        i = fprintf(file, "\"%u\",\"%s\",\"%d\",\"%.2f\",\"%.2f\"", time_now, macAddress.c_str(), averageRssi, 
                                                                      particulateConcentration, maxTemp);

        if(i < 0)
        {
            ESP_LOGE(TAG, "Failed to write to opened file");
            return ESP_FAIL;
        }
    }
    else
    {
        file = fopen(fileName, "a");
        if (file == NULL)
        {
            
            ESP_LOGE(TAG, "Failed to open file for writing");
            return ESP_FAIL;
        }
        i = fprintf(file, "\"Unix time\",\"Tag\",\"Mean RSSI\",\"PM 2.5\",\"Max viewable temperature\"\n");

        if(i < 0)
        {
            ESP_LOGE(TAG, "Failed to write to opened file");
            return ESP_FAIL;
        }
    }
    return ESP_OK;
}
