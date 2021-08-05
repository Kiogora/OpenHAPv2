#ifndef  EDDYSTONE_SCANNER_HPP
#define  EDDYSTONE_SCANNER_HPP

#include "stdint.h"
#include "esp_err.h"

namespace internalHardwareSubsystem
{
    namespace bluetooth
    {
    class eddystoneScanner
    {
    public:

    /*Constructor method*/
    eddystoneScanner();
        
    private:

    /*******************************************************************/
    /*BLE scan parameters and eddystone event callbacks from this point*/
    /*******************************************************************/

    esp_ble_scan_params_t ble_scan_params = 
    {
        .scan_type              = BLE_SCAN_TYPE_ACTIVE,
        .own_addr_type          = BLE_ADDR_TYPE_PUBLIC,
        .scan_filter_policy     = BLE_SCAN_FILTER_ALLOW_ALL,
        .scan_interval          = 0x50,
        .scan_window            = 0x30,
        .scan_duplicate         = BLE_SCAN_DUPLICATE_DISABLE
    };

    void eddystoneScanner_event_callback(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t* param);

    /**************************************************/
    /*Eddystone frame level definition from this point*/
    /**************************************************/

    /* Eddystone UID frame */
    struct esp_eddystone_uid_t
    {
        int8_t    ranging_data;     /*<! calibrated Tx power at 0m */
        uint8_t   namespace_id[10];
        uint8_t   instance_id[6];
        uint8_t   reserved[2];
    } __attribute__((packed));

    /* Eddystone URL frame */
    struct esp_eddystone_url_t
    {
        int8_t    tx_power;         /*<! calibrated Tx power at 0m */
        uint8_t   url_scheme;       /*<! encoded scheme prefix */
        uint8_t   encoded_url[0];   /*<! length 1-17 */
    } __attribute__((packed));

    /* Eddystone TLM frame */
    struct esp_eddystone_tlm_t
    {
        uint8_t    version;        /*<! TLM version,0x00 for now */
        uint16_t   batt;           /*<! battery voltage, 1mV/bit */
        uint16_t   temp;           /*<! beacon temperature */
        uint32_t   adv_count;      /*<! adv pdu count since power-on or reboot */
        uint32_t   time;           /*<! time sence power-on or reboot, a 0.1 second resolution counter */
    } __attribute__((packed));

    /*  AD Structure of flags */
    struct esp_eddystone_flags_t
    {
        uint8_t     len;
        uint8_t     type;
        uint8_t     flags;
    } __attribute__((packed)) ;

    /* AD Structure of complete 16-bit service uuid */
    struct esp_eddystone_uuid_t
    {
        uint8_t     len;
        uint8_t     type;
        uint16_t    uuid;       /*<! complete list of 16-bit service UUIDs data type value */
    } __attribute__((packed)) ;

    /* AD Structure of eddystone frame*/
    struct esp_eddystone_frame_t
    {
        uint8_t     len;        /*<! length of eddystone data */
        uint8_t     type;       /*<! service data type,must be 0x16 */
        uint16_t    uuid;       /*<! 16-bit eddystone uuid */
        uint8_t     frame_type;
        union 
        {
            esp_eddystone_uid_t     uid;
            esp_eddystone_url_t     url;
            esp_eddystone_tlm_t     tlm;
        } u[0];
    } __attribute__((packed)) ;

    /* eddystone packet type */
    struct esp_eddystone_packet_t
    {
        esp_eddystone_flags_t   flags;
        esp_eddystone_uuid_t    uuid;
        esp_eddystone_frame_t   frame;
    } __attribute__((packed));

    static constexpr int EDDYSTONE_SERVICE_UUID = 0xFEAA;

    static constexpr int EDDYSTONE_FRAME_TYPE_UID = 0x00;
    static constexpr int EDDYSTONE_FRAME_TYPE_URL = 0x10;
    static constexpr int EDDYSTONE_FRAME_TYPE_TLM =0x20;
    static constexpr int EDDYSTONE_FRAME_TYPE_EID = 0x30;

    //UID
    static constexpr int EDDYSTONE_UID_RANG_DATA_LEN = 1;
    static constexpr int EDDYSTONE_UID_NAMESPACE_LEN = 10;
    static constexpr int EDDYSTONE_UID_INSTANCE_LEN = 6;
    static constexpr int EDDYSTONE_UID_RFU_LEN = 2;
    static constexpr int EDDYSTONE_UID_DATA_LEN = (EDDYSTONE_UID_RANG_DATA_LEN + 
                                                   EDDYSTONE_UID_NAMESPACE_LEN + 
                                                   EDDYSTONE_UID_INSTANCE_LEN);
    //TLM
    static constexpr int EDDYSTONE_TLM_VERSION_LEN = 1;
    static constexpr int EDDYSTONE_TLM_BATTERY_VOLTAGE_LEN = 2;
    static constexpr int EDDYSTONE_TLM_TEMPERATURE_LEN = 2;
    static constexpr int EDDYSTONE_TLM_ADV_COUNT_LEN = 4;
    static constexpr int EDDYSTONE_TLM_TIME_LEN = 4;
    static constexpr int EDDYSTONE_TLM_DATA_LEN = (EDDYSTONE_TLM_VERSION_LEN + 
                                                   EDDYSTONE_TLM_BATTERY_VOLTAGE_LEN + 
                                                   EDDYSTONE_TLM_TEMPERATURE_LEN + 
                                                   EDDYSTONE_TLM_ADV_COUNT_LEN + 
                                                   EDDYSTONE_TLM_TIME_LEN);
    //URL
    static constexpr int EDDYSTONE_URL_SCHEME_LEN = 1;
    static constexpr int EDDYSTONE_URL_ENCODED_MAX_LEN = 17;
    static constexpr int EDDYSTONE_URL_MAX_LEN = (EDDYSTONE_URL_SCHEME_LEN + EDDYSTONE_URL_ENCODED_MAX_LEN);
    static constexpr int EDDYSTONE_URL_TX_POWER_LEN = 1;
    
    /**********************************************************/
    /*Eddystone frame level processing results from this point*/
    /**********************************************************/

    /*The result of Eddystone UID frame processing*/
    struct uid_result_t
    {
        int8_t  ranging_data;     /*<! calibrated Tx power at 0m */
        uint8_t namespace_id[10];
        uint8_t instance_id[6];
    };

    /*The result of Eddystone URL frame processing*/
    struct url_result_t
    {
        int8_t  tx_power;                    /*<! calibrated Tx power at 0m */
        char    url[EDDYSTONE_URL_MAX_LEN];  /*<! the decoded URL */
    };

    /*The result of Eddystone TLM frame processing*/
    struct tlm_result_t
    {
        uint8_t   version;           /*<! TLM version,0x00 for now */
        uint16_t  battery_voltage;   /*<! battery voltage in mV */
        float     temperature;       /*<! beacon temperature in degrees Celsius */
        uint32_t  adv_count;         /*<! adv pdu count since power-up */
        uint32_t  time;              /*<! time since power-up, a 0.1 second resolution counter */
    };

    struct frame_metadata_t
    {
        uint8_t   flags;          /*<! AD flags data */
        uint16_t  srv_uuid;       /*<! complete list of 16-bit service uuid*/
        uint16_t  srv_data_type;  /*<! service data type */
        uint8_t   frame_type;     /*<! Eddystone UID, URL or TLM */
    };

    struct esp_eddystone_result_t
    {
        frame_metadata_t common;
        union
        {
            uid_result_t uid;
            url_result_t url;
            tlm_result_t tlm;
        } inform;
    };

    esp_eddystone_result_t res;

    char* eddystoneScanner_resolve_url_scheme(const uint8_t* url_start, const uint8_t* url_end);
    esp_err_t eddystoneScanner_url_received(const uint8_t* buf, uint8_t len);

    esp_err_t eddystoneScanner_get_inform(const uint8_t* buf, uint8_t len);
    esp_err_t eddystoneScanner_uid_received(const uint8_t* buf, uint8_t len);
    esp_err_t eddystoneScanner_tlm_received(const uint8_t* buf, uint8_t len);

    esp_err_t eddystoneScanner_decode(const uint8_t* buf, uint8_t len);

    /***************************/
    /*Utilities from this point*/
    /***************************/
    inline uint16_t little_endian_read_16(const uint8_t *buffer, uint8_t pos);
    inline uint16_t big_endian_read_16(const uint8_t *buffer, uint8_t pos);
    inline uint32_t big_endian_read_32(const uint8_t *buffer, uint8_t pos);
    inline bool esp_eddystone_is_char_invalid(int ch);
    };
    }
}
#endif