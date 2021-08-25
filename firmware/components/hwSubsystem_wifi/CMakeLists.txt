idf_component_register(SRCS "wifiManager.cpp"
                    INCLUDE_DIRS "include"
                    REQUIRES nvs_flash spiffs esp_http_server newlib mdns json hwSubsystem_thermalFrameGrabber
                    EMBED_FILES "favicon.ico" "index.html" "sensors.html" "notification.mp3")
