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
        std::string TAG = "internalHardwareSubystem::filesystems";

        spiFlashFilesystem(const std::string mountPoint = "/internal", size_t maxFiles = 1, bool formatIfMountFails = true);

        esp_err_t getBytesAvailable(size_t& usedBytes, size_t& totalBytes);

    protected:
        esp_vfs_spiffs_conf_t fsConfiguration;
    };
    }

}
#endif