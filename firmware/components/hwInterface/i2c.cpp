/*
 * i2c.cpp
 *
 *  Created on: Feb 24, 2017
 *      Author: kolban
 */
#include <driver/i2c.h>
#include <esp_err.h>
#include <stdint.h>
#include <sys/types.h>
#include <esp_log.h>
#include <freertos/semphr.h>
#include "i2c.hpp"

static const char* TAG = "externalHardwareInterface::i2cBus";

/**
 * @brief Create an instance of an %i2c object.
 * @return N/A.
 */
externalHardwareInterface::i2cBus::i2cBus(gpio_num_t sdaPin, gpio_num_t sclPin, uint32_t clockSpeed, i2c_port_t portNum, bool pullupsState):
m_sdaPin(sdaPin), m_sclPin(sclPin), m_portNum(portNum)
{
	ESP_LOGD(TAG, ">> i2cDevice::i2c. sda=%d, scl=%d, clockSpeed=%d, portNum=%d", sdaPin, sclPin, clockSpeed, portNum);
	assert(portNum < I2C_NUM_MAX);

	i2c_config_t conf;
	conf.mode             = I2C_MODE_MASTER;
	conf.sda_io_num       = sdaPin;
	conf.scl_io_num       = sclPin;
	conf.sda_pullup_en    = (pullupsState == pullupsEnable) ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE;
	conf.scl_pullup_en    = (pullupsState == pullupsDisable) ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE;
	conf.master.clk_speed = clockSpeed;
	//conf.clk_flags        = 0; /*Choose a clock only according to desired frequencies*/
	esp_err_t err = i2c_param_config(m_portNum, &conf);
	if (err != ESP_OK)
    {
		ESP_LOGE(TAG, "I2C_param_config: rc=%d %s", err, esp_err_to_name(err));
	}

    err = i2c_driver_install(m_portNum, I2C_MODE_MASTER, 0, 0, 0);
    if (err != ESP_OK)
	{
        ESP_LOGE(TAG, "i2c_driver_install: rc=%d %s", err, esp_err_to_name(err));
    }
	
	/*I2C bus mutex to avoid bus contention*/
    m_busMutex={(SemaphoreHandle_t)xSemaphoreCreateMutex()};
}

void externalHardwareInterface::i2cBus::scanBusAddresses()
{
	printf("Data Pin: %d, Clock Pin: %d\n", m_sdaPin, m_sclPin);
    uint8_t address;
    printf("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\r\n");
    for (int i = 0; i < 128; i += 16) {
        printf("%02x: ", i);
        for (int j = 0; j < 16; j++) {
            fflush(stdout);
            address = i + j;
            i2c_cmd_handle_t cmd = i2c_cmd_link_create();
            i2c_master_start(cmd);
            i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, ACK_CHECK_ENABLED);
            i2c_master_stop(cmd);
            esp_err_t ret = i2c_master_cmd_begin(m_portNum, cmd, 50 / portTICK_RATE_MS);
            i2c_cmd_link_delete(cmd);
            if (ret == ESP_OK) 
			{
                printf("%02x ", address);
            } 
			else if (ret == ESP_ERR_TIMEOUT)
			{
                printf("UU ");
            }
			else
			{
                printf("-- ");
            }
        }
        printf("\r\n");
    }
}

bool externalHardwareInterface::i2cBus::isPresent(uint8_t address)
{
	i2c_cmd_handle_t cmd = ::i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, ACK_CHECK_ENABLED);
	i2c_master_stop(cmd);

	esp_err_t ret = ::i2c_master_cmd_begin(m_portNum, cmd, 50 / portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);
	return ret == 0;  // Return true if the slave is present and false otherwise.
} 