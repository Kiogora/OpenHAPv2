#ifndef  INTERNALFLASH_HPP
#define  INTERNALFLASH_HPP

#include <string>
#include "esp_spiffs.h"

namespace internalHardwareSubsystem
{
    namespace storage
    {
    
    class spiFlashFilesystem
    {
    public:
        const std::string mountPoint;

        spiFlashFilesystem(const std::string mountPoint = "/internal", size_t maxFiles = 5, bool formatIfMountFails = true);
        esp_err_t getBytesAvailable(size_t& usedBytes, size_t& totalBytes);
        esp_err_t printBytesAvailable();
        esp_err_t writeCSV(uint32_t time_now, std::string macAddress, int16_t averageRssi, float ParticulateConcentration, float maxTemp);

    private:
    

    };
    }

}
#endif