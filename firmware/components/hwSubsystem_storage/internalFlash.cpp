#include <cstddef>
#include <string>
#include <sstream>
#include <dirent.h>
#include <sys/stat.h>
#include <sys/param.h>
#include <sys/unistd.h>

#include "esp_err.h"
#include "esp_log.h"
#include "esp_vfs.h"
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
    ESP_LOGI(TAG, "Setup internal filesystem successfully");
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

esp_err_t internalHardwareSubsystem::storage::spiFlashFilesystem::printFilesOnDisk()
{
    char entrypath[ESP_VFS_PATH_MAX + CONFIG_SPIFFS_OBJ_NAME_LEN];
    struct dirent* entry;
    char entrysize[16];
    const char* entrytype;
    struct stat entry_stat;
    unsigned int filesFound = 0;

    const char* root = (mountPoint+"/").c_str();

    DIR *dir = opendir(root);
    const size_t dirpath_len = strlen(root);

    /* Retrieve the base path of file storage to construct the full path */
    strlcpy(entrypath, root, sizeof(entrypath));  

    ESP_LOGI(TAG, "Printing all files on internal filesystem...");
    /* Iterate over all files / folders and fetch their names and sizes */
    while ((entry = readdir(dir)) != NULL)
    {
        ++filesFound;
        entrytype = (entry->d_type == DT_DIR ? "directory" : "file");
        strlcpy(entrypath + dirpath_len, entry->d_name, sizeof(entrypath) - dirpath_len);
        if (stat(entrypath, &entry_stat) == -1)
        {
            ESP_LOGE(TAG, "Failed to stat %s : %s", entrytype, entry->d_name);
            continue;
        }
        sprintf(entrysize, "%ld", entry_stat.st_size);
        ESP_LOGI(TAG, "Found %s : %s (%s bytes)", entrytype, entry->d_name, entrysize);
    }
    if (filesFound == 0)
    {
        ESP_LOGI(TAG, "No files found on internal disk");
    }
    return ESP_OK;
}

void internalHardwareSubsystem::storage::spiFlashFilesystem::deleteFile(const char* fileName)
{

    char pathQualifiedFilename[ESP_VFS_PATH_MAX + CONFIG_SPIFFS_OBJ_NAME_LEN];
    const char* root = (mountPoint+"/").c_str();
    const size_t rootLen = strlen(root);
    strlcpy(pathQualifiedFilename, root, sizeof(pathQualifiedFilename));
    strlcpy(pathQualifiedFilename+rootLen, fileName, sizeof(pathQualifiedFilename)-rootLen);

    ESP_LOGI(TAG, "Deleting %s...", pathQualifiedFilename);
    unlink(pathQualifiedFilename);
}

esp_err_t internalHardwareSubsystem::storage::spiFlashFilesystem::writeCSVEntry(const char* fileName, std::time_t time_now = 0, 
                                                                                std::string macAddress = "", int16_t averageRssi = 0, 
                                                                                float particulateConcentration = 0., float maxTemp=0.)
{
    int i;
    struct stat st;
    FILE* file = NULL;
    bool fileExists = false;

    char pathQualifiedFilename[ESP_VFS_PATH_MAX + CONFIG_SPIFFS_OBJ_NAME_LEN];
    const char* root = (mountPoint+"/").c_str();
    const size_t rootLen = strlen(root);
    strlcpy(pathQualifiedFilename, root, sizeof(pathQualifiedFilename));
    strlcpy(pathQualifiedFilename+rootLen, fileName, sizeof(pathQualifiedFilename)-rootLen);

    /*Open file by append mode, if file does not exist, create it*/
    if (stat(pathQualifiedFilename, &st) == 0)
    {
        ESP_LOGW(TAG, "File exists");
        fileExists = true;
    }
    else
    {
        ESP_LOGW(TAG, "File does not exist, creating it...");
    }

    file = fopen(pathQualifiedFilename, "a");
    if (file == NULL)
    {
        ESP_LOGE(TAG, "Failed to open file in append mode");
        return ESP_FAIL;
    }

    if(!fileExists)
    {
        ESP_LOGI(TAG, "Writing CSV header to %s", fileName);
        i = fprintf(file, "\"Unix time\",\"Tag\",\"Mean RSSI\",\"PM 2.5\",\"Max viewable temperature\"\n");
        if(i < 0)
        {
            ESP_LOGE(TAG, "Failed to write to opened file");
            fclose(file);
            return ESP_FAIL;
        }
        fflush(file);
    }      

    ESP_LOGI(TAG, "Writing CSV data to %s", fileName);
    i = fprintf(file, "\"%u\",\"%s\",\"%d\",\"%.2f\",\"%.2f\"", (uint32_t) time_now, macAddress.c_str(), averageRssi, 
                                                                particulateConcentration, maxTemp);
    if(i < 0)
    {
        ESP_LOGE(TAG, "Failed to write to opened file");
        fclose(file);
        return ESP_FAIL;
    }
    fflush(file);
    fclose(file);
    return ESP_OK;
}
