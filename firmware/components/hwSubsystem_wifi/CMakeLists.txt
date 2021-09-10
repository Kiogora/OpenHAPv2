idf_component_register(SRCS "wifiManager.cpp"
                    INCLUDE_DIRS "include"
                    REQUIRES nvs_flash spiffs esp_http_server newlib mdns json hwSubsystem_thermalFrameGrabber
                    hwSubsystem_particulateSensor hwSubsystem_bluetooth_le hwSubsystem_timekeeping hwSubsystem_storage
                    EMBED_FILES "www/favicon.ico" "www/index.html" "www/sensors.html" "www/notification.mp3" "www/d3.min.js"
                    "www/wristband.html" "www/w3.css" "www/fontawesome.min.css")
