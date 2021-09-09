#ifndef  WIFI_MANAGER_HPP
#define  WIFI_MANAGER_HPP

#include <string>
#include "esp_vfs.h"
#include "esp_event.h"
#include "esp_http_server.h"

#include "gpio.hpp"
#include "ds3231.hpp"
#include "sds011.hpp"
#include "mlx90641.hpp"
#include "internalFlash.hpp"
#include "eddystoneScanner.hpp"

namespace internalHardwareSubsystem
{
    namespace wifi
    {
    class wifiManager
    {
    public:
    enum struct supportedWifiModes: uint8_t{accessPointMode = 0x00};

    char ssid[32];

    /*Constructor method*/
    wifiManager(supportedWifiModes startupMode = supportedWifiModes::accessPointMode);
    /*Server start method*/
    esp_err_t startServer(internalHardwareSubsystem::bluetooth::eddystoneScanner &bleScanner,
                          internalHardwareSubsystem::storage::nonVolatileStorageSettings &settings,
                          internalHardwareSubsystem::storage::spiFlashFilesystem &storage,
                          externalHardwareSubsystem::thermalImaging::MLX90641 &thermalImager,
                          externalHardwareSubsystem::timekeeping::DS3231 &ds3231,
                          externalHardwareSubsystem::particulateSensor::SDS011 &particulateSensor,
                          externalHardwareInterface::gpio &warningLed);
    private:
    /*Access point specifics*/
    static void accessPointEventHandler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);

    /*Starts local DNS server*/
    esp_err_t startMDNS();

    /*Socket specific open close handlers*/
    static esp_err_t on_open_sock(httpd_handle_t hd, int sockfd);
    static void on_close_sock(httpd_handle_t hd, int sockfd);

    /*Websocket specific for (a)synchronous send*/
    struct async_resp_arg_sensor_data
    {
        httpd_handle_t hd;
        int fd;
        externalHardwareSubsystem::thermalImaging::MLX90641* thermalImagerRef;
        externalHardwareSubsystem::timekeeping::DS3231* ds3231Ref;
        externalHardwareSubsystem::particulateSensor::SDS011* particulateSensorRef;
        externalHardwareInterface::gpio* warningLedRef;
    };

    struct async_resp_arg_ble_data
    {
        httpd_handle_t hd;
        int fd;
        internalHardwareSubsystem::bluetooth::eddystoneScanner* bleScannerRef;
    };
    static void ws_async_send_sensor_data(void *arg);
    static void ws_async_send_ble_data(void *arg);
    static esp_err_t trigger_async_send_sensor_data(httpd_handle_t handle, httpd_req_t *req);
    static esp_err_t trigger_async_send_ble_data(httpd_handle_t handle, httpd_req_t *req);
    static esp_err_t websocket_handler(httpd_req_t *req);
    static esp_err_t send_unified_header(httpd_req_t *req);
    static esp_err_t ble_view_handler(httpd_req_t *req);
    static esp_err_t sensor_view_handler(httpd_req_t *req);

    /*File server specifics from this point*/
    static constexpr int SCRATCH_BUFSIZE = 8192;
    struct server_data 
    {
        /* Base path of file storage */
        char base_path[ESP_VFS_PATH_MAX + 1];
        /* Scratch buffer for temporary storage during file transfer */
        char scratch[SCRATCH_BUFSIZE];
        /*Peripherals accessible to server callback functions*/
        internalHardwareSubsystem::bluetooth::eddystoneScanner* bleScannerRef;
        internalHardwareSubsystem::storage::nonVolatileStorageSettings* settingsRef;
        internalHardwareSubsystem::storage::spiFlashFilesystem* storageRef;
        externalHardwareSubsystem::thermalImaging::MLX90641*     thermalImagerRef;
        externalHardwareSubsystem::timekeeping::DS3231* ds3231Ref;
        externalHardwareSubsystem::particulateSensor::SDS011* particulateSensorRef;
        externalHardwareInterface::gpio* warningLedRef;
    };

    static esp_err_t index_html_get_handler(httpd_req_t *req);
    static esp_err_t favicon_get_handler(httpd_req_t *req);
    static esp_err_t mp3_get_handler(httpd_req_t *req);
    static esp_err_t plotter_get_handler(httpd_req_t *req);
    static esp_err_t measurement_handler(httpd_req_t *req);
    static esp_err_t http_resp_dir_html(httpd_req_t *req, const char *dirpath);
    static esp_err_t set_content_type_from_file(httpd_req_t *req, const char *filename);
    static const char* get_path_from_uri(char *dest, const char *base_path, const char *uri, size_t destsize);
    static esp_err_t download_get_handler(httpd_req_t *req);
    static esp_err_t delete_post_handler(httpd_req_t *req);
    };
    }
}
#endif