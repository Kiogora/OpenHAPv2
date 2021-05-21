/*
 * i2c.cpp
 *
 *  Created on: Feb 24, 2017
 *      Author: kolban
 */
#include <driver/gpio.h>
#include <driver/i2c.h>
#include <esp_err.h>
#include <stdint.h>
#include <sys/types.h>
#include "i2c.hpp"
#include "sdkconfig.h"
#include <esp_log.h>

static const char* LOG_TAG = "i2c";

/**
 * @brief Create an instance of an %i2c object.
 * @return N/A.
 */
hardwareInterface::i2cBus::i2cBus(gpio_num_t sdaPin, gpio_num_t sclPin, uint32_t clockSpeed, i2c_port_t portNum, bool pullupsState, uint8_t address, const SemaphoreHandle_t& i2cBusMutex): 
m_address(address), m_sdaPin(sdaPin), m_sclPin(sclPin), m_portNum(portNum), m_busMutex(i2cBusMutex)
{
	ESP_LOGD(LOG_TAG, ">> i2cDevice::i2c.  address=%d, sda=%d, scl=%d, clockSpeed=%d, portNum=%d", address, sdaPin, sclPin, clockSpeed, portNum);
	assert(portNum < I2C_NUM_MAX);

	i2c_config_t conf;
	conf.mode             = I2C_MODE_MASTER;
	conf.sda_io_num       = sdaPin;
	conf.scl_io_num       = sclPin;
	conf.sda_pullup_en    = (pullupsState == pullupsEnable) ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE;
	conf.scl_pullup_en    = (pullupsState == pullupsDisable) ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE;
	conf.master.clk_speed =  clockSpeed;
	esp_err_t err = i2c_param_config(m_portNum, &conf);
	if (err != ESP_OK)
    {
		ESP_LOGE(LOG_TAG, "I2C_param_config: rc=%d %s", err, esp_err_to_name(err));
	}
	
    err = i2c_driver_install(m_portNum, I2C_MODE_MASTER, 0, 0, 0);
    if (err != ESP_OK) {
        ESP_LOGE(LOG_TAG, "i2c_driver_install: rc=%d %s", err, esp_err_to_name(err));
    }
	//Throw an exception if i2c device fails
}