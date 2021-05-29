#ifndef  SPIFFS_HPP
#define  SPIFFS_HPP

#include <string>
#include "esp_spiffs.h"

namespace softwareSubsystem
{
    namespace filesystems
    {
    class spiffs
    {
    public:
        std::string TAG = "softwareSubystem::filesystems";

        /*Add to this enumeration as more memories become supported*/
        enum struct supportedMemories{internalSPIFlash};

        spiffs(supportedMemories setupOnThisMemory = supportedMemories::internalSPIFlash, const std::string mountPoint = "/internal", size_t maxFiles = 1, bool formatIfMountFails = true);

        esp_err_t getBytesAvailable(size_t& usedBytes, size_t& totalBytes);

    protected:
        esp_vfs_spiffs_conf_t fsConfiguration;
        supportedMemories thisMemory;
    };
    }

}
#endif