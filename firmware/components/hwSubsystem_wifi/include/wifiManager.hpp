#ifndef  WIFI_MANAGER_HPP
#define  WIFI_MANAGER_HPP

#include "esp_event.h"

namespace internalHardwareSubsystem
{
    namespace wifi
    {
    class wifiManager
    {
    public:
    
    enum struct supportedWifiModes: uint8_t{accessPointMode = 0x00, clientMode = 0x01};

    /*Constructor method*/
    wifiManager(supportedWifiModes startupMode = supportedWifiModes::accessPointMode);
        
    private:
    static void accessPointEventHandler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);
    };
    }
}
#endif