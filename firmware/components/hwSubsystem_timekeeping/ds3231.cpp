#include <ctime>
#include <driver/i2c.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "ds3231.hpp"


static const char* TAG = "externalHardwareSubsystem::timekeeping::DS3231";

/*Ctor to attach the device to an uninitialized bus object*/
externalHardwareSubsystem::timekeeping::DS3231::DS3231(uint8_t address, uint32_t timeout): m_address{address}, m_timeoutms{timeout} {}

/*Ctor to attach the device to an existing bus object*/
externalHardwareSubsystem::timekeeping::DS3231::DS3231(const i2cBus& otherBusDevice, uint8_t address, uint32_t timeout): i2cBus(otherBusDevice), m_address{address}, m_timeoutms{timeout} {}


/*Create concrete class from abstract i2c class -   How to detect presence of this device*/
bool externalHardwareSubsystem::timekeeping::DS3231::isPresent()
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (m_address << 1) | I2C_MASTER_WRITE, ACK_CHECK_ENABLED);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(m_portNum, cmd, 50 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret == ESP_OK;
}

esp_err_t externalHardwareSubsystem::timekeeping::DS3231::get_time(std::tm* time)
{
    uint8_t data[7] = {0};

    /* read time */
    esp_err_t ret = read_reg(reg_time, data, 7);
    if(ret != ESP_OK)
    {
        return ret;
    }

    /* convert to unix time structure */
    time->tm_sec = bcd2dec(data[0]);
    time->tm_min = bcd2dec(data[1]);
    if (data[2] & flag_12hour)
    {
        /* 12H */
        time->tm_hour = bcd2dec(data[2] & mask_12hour) - 1;

        /* AM/PM? */
        if (data[2] & flag_PM)
        {
            time->tm_hour += 12;
        }
    }
    else
    {
        time->tm_hour = bcd2dec(data[2]); /* 24H */
    }

    time->tm_wday = bcd2dec(data[3]) - 1;
    time->tm_mday = bcd2dec(data[4]);
    time->tm_mon  = bcd2dec(data[5] & mask_month) - 1;
    time->tm_year = (bcd2dec(data[6])+2000)-1900;
    time->tm_isdst = 0;
    return ESP_OK;
}

esp_err_t externalHardwareSubsystem::timekeeping::DS3231::get_time(std::time_t& unixtime)
{
    std::tm ptimeStruct;
    esp_err_t ret = get_time(&ptimeStruct);
    unixtime = std::mktime(&ptimeStruct);
    return ret;
}

esp_err_t externalHardwareSubsystem::timekeeping::DS3231::set_time(const struct tm *time)
{
    uint8_t data[7] = {0};

    /* time/date data */
    data[0] = dec2bcd(time->tm_sec);
    data[1] = dec2bcd(time->tm_min);
    data[2] = dec2bcd(time->tm_hour);
    /* The week data must be in the range 1 to 7, and to keep the start on the
     * same day as for tm_wday have it start at 1 on Sunday. */
    data[3] = dec2bcd(time->tm_wday + 1);
    data[4] = dec2bcd(time->tm_mday);
    data[5] = dec2bcd(time->tm_mon + 1);
    data[6] = dec2bcd((time->tm_year+1900)-2000);

    esp_err_t ret = write_reg(reg_time, data, 7);

    return ret;
}

esp_err_t externalHardwareSubsystem::timekeeping::DS3231::set_time(const std::time_t& unixtime)
{
    return set_time(std::gmtime(&unixtime));
}

uint8_t externalHardwareSubsystem::timekeeping::DS3231::bcd2dec(uint8_t val)
{   
    return (val >> 4) * 10 + (val & 0x0f);
}

uint8_t externalHardwareSubsystem::timekeeping::DS3231::dec2bcd(uint8_t val)
{
    return ((val / 10) << 4) + (val % 10);
}

inline esp_err_t externalHardwareSubsystem::timekeeping::DS3231::read_reg(uint8_t reg, void *in_data, size_t in_size)
{
    return read(&reg, 1, in_data, in_size);
}

esp_err_t externalHardwareSubsystem::timekeeping::DS3231::read(const void *out_data, size_t out_size, void *in_data, size_t in_size)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    if (out_data && out_size)
    {
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, m_address << 1, true);
        i2c_master_write(cmd, (uint8_t*)out_data, out_size, true);
    }
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, m_address << 1 | 1, true);
    i2c_master_read(cmd, (uint8_t*)in_data, in_size, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    xSemaphoreTake(m_busMutex, portMAX_DELAY);
    esp_err_t ret = i2c_master_cmd_begin(m_portNum, cmd, busTimeout/portTICK_RATE_MS);
    xSemaphoreGive(m_busMutex);
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Could not read from ds3231 device at address [0x%02x at %d]: returned %d", m_address, m_portNum, ret);
    }

    return ret;  

}

inline esp_err_t externalHardwareSubsystem::timekeeping::DS3231::write_reg(uint8_t reg, const void *out_data, size_t out_size)
{
    return  write(&reg, 1, out_data, out_size);
}

esp_err_t externalHardwareSubsystem::timekeeping::DS3231::write(const void *out_reg, size_t out_reg_size, const void *out_data, size_t out_size)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, m_address << 1, true);
    if (out_reg && out_reg_size)
    {
        i2c_master_write(cmd, (uint8_t *)out_reg, out_reg_size, true);
    }
    i2c_master_write(cmd, (uint8_t *)out_data, out_size, true);
    i2c_master_stop(cmd);
    xSemaphoreTake(m_busMutex, portMAX_DELAY);
    esp_err_t ret = i2c_master_cmd_begin(m_portNum, cmd, m_timeoutms/portTICK_RATE_MS);
    xSemaphoreGive(m_busMutex);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Could not write to  DS3231 device at address [0x%02x at %d]: returned %d", m_address, m_portNum, ret);
    }

    return ret; 
}