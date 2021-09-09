#ifndef  INTERNALFLASH_HPP
#define  INTERNALFLASH_HPP

#include <string>
#include <ctime>
#include "esp_spiffs.h"

namespace internalHardwareSubsystem
{
    namespace storage
    {
    
    class spiFlashFilesystem
    {
    public:
        const std::string mountPoint;

        spiFlashFilesystem(const std::string mountPoint = "/spiffs", size_t maxFiles = 5, bool formatIfMountFails = true);
        esp_err_t getBytesAvailable(size_t& usedBytes, size_t& totalBytes);
        esp_err_t printBytesAvailable();
        esp_err_t printFilesOnDisk();
        void deleteFile(const char* fileName);
        esp_err_t writeCSVEntry(const char* fileName, std::time_t time_now, int16_t packet_num, float averageRssi, float ParticulateConcentration, float maxTemp);

    private:
    };
    class nonVolatileStorageSettings
    {
    public:
        nonVolatileStorageSettings();

        static constexpr int32_t inactive = 0x00;
        static constexpr int32_t active = 0x01;
        
        bool isMeasurementActive();
        esp_err_t setMeasurementFlag(int32_t desiredMode);
    };
    }
}
#endif