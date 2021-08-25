#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include <lwip/err.h>
#include <lwip/sockets.h>
#include <lwip/sys.h>
#include <lwip/netdb.h>
#include <lwip/dns.h>
#include "lwip/err.h"
#include "lwip/sys.h"

#include <sys/param.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include <dirent.h>

#include "esp_vfs.h"
#include "esp_spiffs.h"
#include "esp_http_server.h"

#include "mdns.h"
#include "cJSON.h"

#include "mlx90641.hpp"
#include <algorithm>

#include "wifiManager.hpp"

static const char* TAG = "internalHardwareSubsystem::wifi";

internalHardwareSubsystem::wifi::wifiManager::wifiManager(supportedWifiModes startupMode)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    if (startupMode == supportedWifiModes::accessPointMode)
    {
        ESP_ERROR_CHECK(esp_netif_init());
        ESP_ERROR_CHECK(esp_event_loop_create_default());
        esp_netif_create_default_wifi_ap();

        wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
        ESP_ERROR_CHECK(esp_wifi_init(&cfg));

        ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                            ESP_EVENT_ANY_ID,
                                                            &accessPointEventHandler,
                                                            NULL,
                                                            NULL));

        uint8_t baseMacAddress[6] = {0};
        /*Get base MAC address from EFUSE BLK0(default option)*/
        esp_err_t ret = esp_efuse_mac_get_default(baseMacAddress);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to get base MAC address from EFUSE BLK0. (%s)", esp_err_to_name(ret));
            ESP_LOGE(TAG, "Aborting, setting default all zero MAC address");
        }
        else
        {
            ESP_LOGI(TAG, "Base MAC Address read from EFUSE BLK0");
        }

        char ssid[32];
        snprintf(ssid, sizeof(ssid), "OpenHAP_%02X:%02X:%02X:%02X:%02X:%02X",
                 baseMacAddress[0], baseMacAddress[1], baseMacAddress[2], 
                 baseMacAddress[3], baseMacAddress[4], baseMacAddress[5]);

        wifi_config_t wifi_config = {};
        strcpy((char*)wifi_config.ap.ssid, ssid);
        wifi_config.ap.ssid_len = strlen(ssid);
        wifi_config.ap.channel = 6;
        strcpy((char*)wifi_config.ap.password, "");;
        wifi_config.ap.max_connection = 3;
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;

        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
        ESP_ERROR_CHECK(esp_wifi_start());
    }
}

esp_err_t internalHardwareSubsystem::wifi::wifiManager::startServer(std::string filesystemMountPoint)
{
    static struct server_data *server_data = NULL;

    const char* base_path = filesystemMountPoint.c_str();

    /* Validate file storage base path */
    if (!base_path || strcmp(base_path, "/spiffs") != 0)
    {
        ESP_LOGE(TAG, "File server presently supports only '/spiffs' as base path");
        return ESP_ERR_INVALID_ARG;
    }

    if (server_data)
    {
        ESP_LOGE(TAG, "File server already started");
        return ESP_ERR_INVALID_STATE;
    }

    /* Allocate memory for server data */
    server_data = (struct server_data*) calloc(1, sizeof(struct server_data));
    if (!server_data)
    {
        ESP_LOGE(TAG, "Failed to allocate memory for server data");
        return ESP_ERR_NO_MEM;
    }
    strlcpy(server_data->base_path, base_path, sizeof(server_data->base_path));

    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    /* Use the URI wildcard matching function in order to
     * allow the same handler to respond to multiple different
     * target URIs which match the wildcard scheme */
    config.uri_match_fn = httpd_uri_match_wildcard;
    config.stack_size = 16384;
    config.open_fn = on_open_sock;
    config.close_fn = on_close_sock;

    ESP_LOGI(TAG, "Starting HTTP Server on port: '%d'", config.server_port);
    if (httpd_start(&server, &config) != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to start server!");
        return ESP_FAIL;
    }

    /* URI handler for getting uploaded files */
    httpd_uri_t file_download = 
    {
        .uri       = "/files/*",  // Match all URIs of type /path/to/file
        .method    = HTTP_GET,
        .handler   = download_get_handler,
        .user_ctx  = server_data    // Pass server data as context
    };
    httpd_register_uri_handler(server, &file_download);

    /* URI handler for deleting files from server */
    httpd_uri_t file_delete = 
    {
        .uri       = "/delete/*",   // Match all URIs of type /delete/path/to/file
        .method    = HTTP_POST,
        .handler   = delete_post_handler,
        .user_ctx  = server_data    // Pass server data as context
    };
    httpd_register_uri_handler(server, &file_delete);

    /* URI handler for deleting files from server */
    httpd_uri_t root = 
    {
        .uri       = "/",   
        .method    = HTTP_GET,
        .handler   = index_html_get_handler,
        .user_ctx  = server_data    // Pass server data as context
    };
    httpd_register_uri_handler(server, &root);

        /* URI handler for deleting files from server */
    httpd_uri_t index_html = 
    {
        .uri       = "/index.html",
        .method    = HTTP_GET,
        .handler   = index_html_get_handler,
        .user_ctx  = NULL
    };
    httpd_register_uri_handler(server, &index_html);

        /* URI handler for deleting files from server */
    httpd_uri_t favicon = 
    {
        .uri       = "/favicon.ico",
        .method    = HTTP_GET,
        .handler   = favicon_get_handler,
        .user_ctx  = NULL
    };
    httpd_register_uri_handler(server, &favicon);

    httpd_uri_t mp3 = 
    {
        .uri       = "/notification.mp3",
        .method    = HTTP_GET,
        .handler   = mp3_get_handler,
        .user_ctx  = NULL
    };
    httpd_register_uri_handler(server, &mp3);

    /* URI handler for deleting files from server */
    httpd_uri_t sensor_view = 
    {
        .uri       = "/sensors/",   // Match all URIs of type /delete/path/to/file
        .method    = HTTP_GET,
        .handler   = sensor_view_handler,
        .user_ctx   = NULL,
    };
    httpd_register_uri_handler(server, &sensor_view);

    /* URI handler for deleting files from server */
    httpd_uri_t websockets = 
    {
        .uri       = "/ws",   // Match all URIs of type /delete/path/to/file
        .method    = HTTP_GET,
        .handler   = websocket_handler,
        .user_ctx   = NULL,
        .is_websocket = true,
        .handle_ws_control_frames = true
    };
    httpd_register_uri_handler(server, &websockets);

    startMDNS();

    return ESP_OK;
}

void internalHardwareSubsystem::wifi::wifiManager::accessPointEventHandler(void* arg, esp_event_base_t event_base,
                                                                           int32_t event_id, void* event_data)
{
    if (event_id == WIFI_EVENT_AP_STACONNECTED)
    {
        wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
        ESP_LOGI(TAG, "station connected, AID=%d", event->aid);
    } 
    else if (event_id == WIFI_EVENT_AP_STADISCONNECTED)
    {
        wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
        ESP_LOGI(TAG, "station disconnected, AID=%d", event->aid);
    }
}

esp_err_t internalHardwareSubsystem::wifi::wifiManager::startMDNS()
{
    esp_err_t ret = mdns_init();
    if(ret!= ESP_OK)
    {
        return ret;
    }
    /*set mDNS hostname (required if you want to advertise services)*/
    ret = mdns_hostname_set("openhap" );
    if(ret!= ESP_OK)
    {
        return ret;
    }
    /*set default mDNS instance name*/
    mdns_instance_name_set("Openhap datalogger");
    if(ret!= ESP_OK)
    {
        return ret;
    }
    /*Initialize service*/
    mdns_service_add("Openhap-WebServer", "_http", "_tcp", 80, NULL, 0);
    if(ret!= ESP_OK)
    {
        return ret;
    }

    return ESP_OK;
}

/*Socket specific open close handlers*/
esp_err_t internalHardwareSubsystem::wifi::wifiManager::on_open_sock(httpd_handle_t hd, int sockfd)
{
    ESP_LOGI(TAG, "Socket has been opened");
    return ESP_OK;
}
void internalHardwareSubsystem::wifi::wifiManager::on_close_sock(httpd_handle_t hd, int sockfd)
{
    ESP_LOGI(TAG, "Socket has been closed");
}

void internalHardwareSubsystem::wifi::wifiManager::ws_async_send(void *arg)
{
    struct async_resp_arg *resp_arg = (struct async_resp_arg *)arg;
    httpd_handle_t hd = resp_arg->hd;
    int fd = resp_arg->fd;

    externalHardwareSubsystem::thermalImaging::MLX90641 thermalImager;
    const float* imageBuffer = thermalImager.GetImage();
    float maxThermalTemperature = *std::max_element(imageBuffer, imageBuffer+thermalImager.pixelCount);
    
    cJSON *response = cJSON_CreateObject();
    if (response  == NULL){ cJSON_Delete(response);}
    cJSON *maxTemp = cJSON_AddNumberToObject(response, "maxTemp", maxThermalTemperature);
    if (maxTemp  == NULL){ cJSON_Delete(response);}
    char* response_string = cJSON_Print(response);

    httpd_ws_frame_t ws_pkt;
    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    ws_pkt.payload = (uint8_t*)response_string;
    ws_pkt.len = strlen(response_string);
    ws_pkt.type = HTTPD_WS_TYPE_TEXT;

    httpd_ws_send_frame_async(hd, fd, &ws_pkt);
    cJSON_Delete(response);
    free(resp_arg);
}

esp_err_t internalHardwareSubsystem::wifi::wifiManager::trigger_async_send(httpd_handle_t handle, httpd_req_t *req)
{
    struct async_resp_arg *resp_arg = (struct async_resp_arg *)malloc(sizeof(struct async_resp_arg));
    resp_arg->hd = req->handle;
    resp_arg->fd = httpd_req_to_sockfd(req);
    return httpd_queue_work(handle, ws_async_send, resp_arg);
}

/* Handler to handle incoming websocket data from connected client*/
esp_err_t internalHardwareSubsystem::wifi::wifiManager::websocket_handler(httpd_req_t *req)
{
    if (req->method == HTTP_GET)
    {
        ESP_LOGI(TAG, "Handshake done, the new connection was opened");
        return ESP_OK;
    }
    httpd_ws_frame_t ws_pkt;
    uint8_t buf[128] = { 0 };
    
    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    ws_pkt.payload = buf;
    ws_pkt.type = HTTPD_WS_TYPE_TEXT;
    
    esp_err_t ret = httpd_ws_recv_frame(req, &ws_pkt, 128);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "httpd_ws_recv_frame failed to get frame len with %d", ret);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Got packet with message: %s", ws_pkt.payload);
    ESP_LOGI(TAG, "Packet type: %d", ws_pkt.type);

    if(strcmp((const char*)ws_pkt.payload, "sample") == 0)
    {
        return trigger_async_send(req->handle, req);
    }
    return ESP_OK;
}

esp_err_t internalHardwareSubsystem::wifi::wifiManager::sensor_view_handler(httpd_req_t *req)
{
    /* Get handle to embedded file upload script */
    extern const unsigned char sensors_html_start[] asm("_binary_sensors_html_start");
    extern const unsigned char sensors_html_end[]   asm("_binary_sensors_html_end");
    const size_t sensors_html_size = (sensors_html_end - sensors_html_start);

    /* Add file upload form and script which on execution sends a POST request to /upload */
    httpd_resp_send_chunk(req, (const char *)sensors_html_start, sensors_html_size);
    return ESP_OK;
}

/* Handler to redirect incoming GET request for /index.html*/
esp_err_t internalHardwareSubsystem::wifi::wifiManager::index_html_get_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "Index handler");
    httpd_resp_set_status(req, "307 Temporary Redirect");
    httpd_resp_set_hdr(req, "Location", "/files/");
    httpd_resp_send(req, NULL, 0);  // Response body can be empty
    return ESP_OK;
}

/* Handler to respond with an icon file embedded in flash.
 * Browsers expect to GET website icon at URI /favicon.ico.
 * This can be overridden by uploading file with same name */
esp_err_t internalHardwareSubsystem::wifi::wifiManager::favicon_get_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "favicon handler");
    extern const unsigned char favicon_ico_start[] asm("_binary_favicon_ico_start");
    extern const unsigned char favicon_ico_end[]   asm("_binary_favicon_ico_end");
    const size_t favicon_ico_size = (favicon_ico_end - favicon_ico_start);
    httpd_resp_set_type(req, "image/x-icon");
    httpd_resp_send(req, (const char *)favicon_ico_start, favicon_ico_size);
    return ESP_OK;
}

esp_err_t internalHardwareSubsystem::wifi::wifiManager::mp3_get_handler(httpd_req_t *req)
{
    extern const unsigned char mp3_start[] asm("_binary_notification_mp3_start");
    extern const unsigned char mp3_end[]   asm("_binary_notification_mp3_end");
    const size_t mp3_size = (mp3_end - mp3_start);
    httpd_resp_set_type(req, "audio/mpeg");
    httpd_resp_send(req, (const char *)mp3_start, mp3_size);
    return ESP_OK;  
}

/* Send HTTP response with a run-time generated html consisting of
 * a list of all files and folders under the requested path.
 * In case of SPIFFS this returns empty list when path is any
 * string other than '/', since SPIFFS doesn't support directories */
esp_err_t internalHardwareSubsystem::wifi::wifiManager::http_resp_dir_html(httpd_req_t *req, const char *dirpath)
{
    char entrypath[ESP_VFS_PATH_MAX + CONFIG_SPIFFS_OBJ_NAME_LEN];
    char entrysize[16];
    const char *entrytype;

    struct dirent *entry;
    struct stat entry_stat;

    DIR *dir = opendir(dirpath);
    const size_t dirpath_len = strlen(dirpath);

    /* Retrieve the base path of file storage to construct the full path */
    strlcpy(entrypath, dirpath, sizeof(entrypath));

    if (!dir)
    {
        ESP_LOGE(TAG, "Failed to stat dir : %s", dirpath);
        /* Respond with 404 Not Found */
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "Directory does not exist");
        return ESP_FAIL;
    }

    /* Send HTML file header */
    httpd_resp_sendstr_chunk(req, "<!DOCTYPE html><html><body>");

    extern const unsigned char index_html_start[] asm("_binary_index_html_start");
    extern const unsigned char index_html_end[]   asm("_binary_index_html_end");
    const size_t index_html_size = (index_html_end - index_html_start);
    httpd_resp_send_chunk(req, (const char *)index_html_start, index_html_size);

    /* Send file-list table definition and column labels */
    httpd_resp_sendstr_chunk(req,
        "<table class=\"fixed\" border=\"1\">"
        "<col width=\"800px\" /><col width=\"300px\" /><col width=\"300px\" /><col width=\"100px\" />"
        "<thead><tr><th>Name</th><th>Type</th><th>Size (Megabytes)</th><th>Delete</th></tr></thead>"
        "<tbody>");

    /* Iterate over all files / folders and fetch their names and sizes */
    while ((entry = readdir(dir)) != NULL) {
        entrytype = (entry->d_type == DT_DIR ? "directory" : "file");

        strlcpy(entrypath + dirpath_len, entry->d_name, sizeof(entrypath) - dirpath_len);
        if (stat(entrypath, &entry_stat) == -1) {
            ESP_LOGE(TAG, "Failed to stat %s : %s", entrytype, entry->d_name);
            continue;
        }
        sprintf(entrysize, "%.2f", entry_stat.st_size/(1024.*1024.));
        ESP_LOGI(TAG, "Found %s : %s (%f Megabytes)", entrytype, entry->d_name, entry_stat.st_size/(1024.*1024.));

        /* Send chunk of HTML file containing table entries with file name and size */
        httpd_resp_sendstr_chunk(req, "<tr><td><a href=\"");
        httpd_resp_sendstr_chunk(req, req->uri);
        httpd_resp_sendstr_chunk(req, entry->d_name);
        if (entry->d_type == DT_DIR)
        {
            httpd_resp_sendstr_chunk(req, "/");
        }
        httpd_resp_sendstr_chunk(req, "\">");
        httpd_resp_sendstr_chunk(req, entry->d_name);
        httpd_resp_sendstr_chunk(req, "</a></td><td>");
        httpd_resp_sendstr_chunk(req, entrytype);
        httpd_resp_sendstr_chunk(req, "</td><td>");
        httpd_resp_sendstr_chunk(req, entrysize);
        httpd_resp_sendstr_chunk(req, "</td><td>");
        httpd_resp_sendstr_chunk(req, "<form method=\"post\" action=\"/files/delete");
        httpd_resp_sendstr_chunk(req, req->uri);
        httpd_resp_sendstr_chunk(req, entry->d_name);
        httpd_resp_sendstr_chunk(req, "\"><button type=\"submit\">Delete</button></form>");
        httpd_resp_sendstr_chunk(req, "</td></tr>\n");
    }
    closedir(dir);

    /* Finish the file list table */
    httpd_resp_sendstr_chunk(req, "</tbody></table>");

    /* Send remaining chunk of HTML file to complete it */
    httpd_resp_sendstr_chunk(req, "</body></html>");

    /* Send empty chunk to signal HTTP response completion */
    httpd_resp_sendstr_chunk(req, NULL);
    return ESP_OK;
}

static inline bool is_file_extension(const char* filename, const char* ext)
{
    return (strcasecmp(&filename[strlen(filename) - sizeof(ext) + 1], ext) == 0);
}


/* Set HTTP response content type according to file extension */
esp_err_t internalHardwareSubsystem::wifi::wifiManager::set_content_type_from_file(httpd_req_t *req, const char *filename)
{
    if (is_file_extension(filename, ".pdf"))
    {
        return httpd_resp_set_type(req, "application/pdf");
    }
    else if (is_file_extension(filename, ".html"))
    {
        return httpd_resp_set_type(req, "text/html");
    }
    else if (is_file_extension(filename, ".jpeg"))
    {
        return httpd_resp_set_type(req, "image/jpeg");
    }
    else if (is_file_extension(filename, ".ico"))
    {
        return httpd_resp_set_type(req, "image/x-icon");
    }
    else if (is_file_extension(filename, ".mp3"))
    {
        return httpd_resp_set_type(req, "audio/mpeg");
    }
    /* This is a limited set only */
    /* For any other type always set as plain text */
    return httpd_resp_set_type(req, "text/plain");
}

/* Copies the full path into destination buffer and returns
 * pointer to path (skipping the preceding base path) */
const char* internalHardwareSubsystem::wifi::wifiManager::get_path_from_uri(char *dest, const char *base_path, const char *uri, size_t destsize)
{
    const size_t base_pathlen = strlen(base_path);
    size_t pathlen = strlen(uri);

    const char *quest = strchr(uri, '?');
    if (quest)
    {
        pathlen = ((pathlen)<(quest - uri))?(pathlen):((quest - uri));
    }
    const char *hash = strchr(uri, '#');
    if (hash) 
    {
        pathlen = ((pathlen)<(hash - uri))?(pathlen):((hash - uri));;
    }

    if (base_pathlen + pathlen + 1 > destsize) {
        /* Full path string won't fit into destination buffer */
        return NULL;
    }

    /* Construct full path (base + path) */
    strcpy(dest, base_path);
    strlcpy(dest + base_pathlen, uri, pathlen + 1);

    /* Return pointer to path, skipping the base */
    return dest + base_pathlen;
}

/* Handler to download a file kept on the server */
esp_err_t internalHardwareSubsystem::wifi::wifiManager::download_get_handler(httpd_req_t *req)
{
    char filepath[ESP_VFS_PATH_MAX + CONFIG_SPIFFS_OBJ_NAME_LEN];
    FILE *fd = NULL;
    struct stat file_stat;

    const char *filename = get_path_from_uri(filepath, ((struct server_data *)req->user_ctx)->base_path,
                                             req->uri + sizeof("/files") - 1, sizeof(filepath));
    if (!filename)
    {
        ESP_LOGE(TAG, "Filename is too long");
        /* Respond with 500 Internal Server Error */
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Filename too long");
        return ESP_FAIL;
    }

    /* If name has trailing '/', respond with directory contents */
    if (filename[strlen(filename) - 1] == '/')
    {
        return http_resp_dir_html(req, filepath);
    }   

    if (stat(filepath, &file_stat) == -1)
    {
        ESP_LOGE(TAG, "Failed to stat file : %s", filepath);
        /* Respond with 404 Not Found */
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "File does not exist");
        return ESP_FAIL;
    }

    fd = fopen(filepath, "r");
    if (!fd) {
        ESP_LOGE(TAG, "Failed to read existing file : %s", filepath);
        /* Respond with 500 Internal Server Error */
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to read existing file");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Sending file : %s (%ld bytes)...", filename, file_stat.st_size);
    set_content_type_from_file(req, filename);

    /* Retrieve the pointer to scratch buffer for temporary storage */
    char *chunk = ((struct server_data *)req->user_ctx)->scratch;
    size_t chunksize;
    do {
        /* Read file in chunks into the scratch buffer */
        chunksize = fread(chunk, 1, SCRATCH_BUFSIZE, fd);

        if (chunksize > 0) {
            /* Send the buffer contents as HTTP response chunk */
            if (httpd_resp_send_chunk(req, chunk, chunksize) != ESP_OK) {
                fclose(fd);
                ESP_LOGE(TAG, "File sending failed!");
                /* Abort sending file */
                httpd_resp_sendstr_chunk(req, NULL);
                /* Respond with 500 Internal Server Error */
                httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to send file");
               return ESP_FAIL;
           }
        }

        /* Keep looping till the whole file is sent */
    } while (chunksize != 0);

    /* Close file after sending complete */
    fclose(fd);
    ESP_LOGI(TAG, "File sending complete");

    /* Respond with an empty chunk to signal HTTP response completion */
#ifdef CONFIG_EXAMPLE_HTTPD_CONN_CLOSE_HEADER
    httpd_resp_set_hdr(req, "Connection", "close");
#endif
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

/* Handler to delete a file from the server */
esp_err_t internalHardwareSubsystem::wifi::wifiManager::delete_post_handler(httpd_req_t *req)
{
    char filepath[ESP_VFS_PATH_MAX + CONFIG_SPIFFS_OBJ_NAME_LEN];
    struct stat file_stat;

    /* Skip leading "/delete" from URI to get filename */
    /* Note sizeof() counts NULL termination hence the -1 */
    const char *filename = get_path_from_uri(filepath, ((struct server_data *)req->user_ctx)->base_path,
                                             req->uri  + sizeof("/files/delete") - 1, sizeof(filepath));
    if (!filename)
    {
        /* Respond with 500 Internal Server Error */
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Filename too long");
        return ESP_FAIL;
    }

    /* Filename cannot have a trailing '/' */
    if (filename[strlen(filename) - 1] == '/') {
        ESP_LOGE(TAG, "Invalid filename : %s", filename);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Invalid filename");
        return ESP_FAIL;
    }

    if (stat(filepath, &file_stat) == -1) {
        ESP_LOGE(TAG, "File does not exist : %s", filename);
        /* Respond with 400 Bad Request */
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "File does not exist");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Deleting file : %s", filename);
    /* Delete file */
    unlink(filepath);

    /* Redirect onto root to see the updated file list */
    httpd_resp_set_status(req, "303 See Other");
    httpd_resp_set_hdr(req, "Location", "/");
#ifdef CONFIG_EXAMPLE_HTTPD_CONN_CLOSE_HEADER
    httpd_resp_set_hdr(req, "Connection", "close");
#endif
    httpd_resp_sendstr(req, "File deleted successfully");
    return ESP_OK;
}

