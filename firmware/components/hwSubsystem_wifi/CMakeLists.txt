idf_component_register(SRCS "wifiManager.cpp"
                    INCLUDE_DIRS "include"
                    REQUIRES nvs_flash spiffs esp_http_server newlib
                    EMBED_FILES "favicon.ico" "upload_script.html")
