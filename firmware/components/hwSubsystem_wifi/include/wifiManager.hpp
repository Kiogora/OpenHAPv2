#ifndef  WIFI_MANAGER_HPP
#define  WIFI_MANAGER_HPP

#include <string>
#include "esp_vfs.h"
#include "esp_event.h"
#include "esp_http_server.h"

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
    /*Server start method*/
    esp_err_t startServer(std::string filesystemMountPoint);
    private:
    /*Access point specifics*/
    static void accessPointEventHandler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);

    /*Starts local DNS server*/
    esp_err_t startMDNS();

    /*Socket specific open close handlers*/
    static esp_err_t on_open_sock(httpd_handle_t hd, int sockfd);
    static void on_close_sock(httpd_handle_t hd, int sockfd);

    /*Websocket specific for (a)synchronous send*/
    struct async_resp_arg
    {
        httpd_handle_t hd;
        int fd;
    };
    static void ws_async_send(void *arg);
    static esp_err_t trigger_async_send(httpd_handle_t handle, httpd_req_t *req);
    static esp_err_t websocket_handler(httpd_req_t *req);
    static esp_err_t sensor_view_handler(httpd_req_t *req);

    /*File server specifics from this point*/
    static constexpr int SCRATCH_BUFSIZE = 8192;
    struct server_data 
    {
        /* Base path of file storage */
        char base_path[ESP_VFS_PATH_MAX + 1];
        /* Scratch buffer for temporary storage during file transfer */
        char scratch[SCRATCH_BUFSIZE];
    };

    static esp_err_t index_html_get_handler(httpd_req_t *req);
    static esp_err_t favicon_get_handler(httpd_req_t *req);
    static esp_err_t mp3_get_handler(httpd_req_t *req);
    static esp_err_t http_resp_dir_html(httpd_req_t *req, const char *dirpath);
    static esp_err_t set_content_type_from_file(httpd_req_t *req, const char *filename);
    static const char* get_path_from_uri(char *dest, const char *base_path, const char *uri, size_t destsize);
    static esp_err_t download_get_handler(httpd_req_t *req);
    static esp_err_t delete_post_handler(httpd_req_t *req);
    };
    }
}
#endif