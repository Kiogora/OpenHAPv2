/*
 * gpio.cpp
 *
 *  Created on: Feb 28, 2017
 *      Author: kolban
 */

#include "gpio.hpp"
#include "driver/gpio.h"
#include "esp_log.h"
#include <esp_err.h>

static const char* LOG_TAG = "gpio";

hardwareInterface::gpio::gpio(gpio_num_t pin, gpioDirection direction):m_pin(pin), m_direction(direction)
{	
	if(direction == input)
	{
		gpio_set_direction(pin, GPIO_MODE_INPUT);
	}

	if(direction == output)
	{
		gpio_set_direction(pin, GPIO_MODE_OUTPUT);
	}
}

bool hardwareInterface::gpio::read()
{
	return gpio_get_level(m_pin) == 1;
}

esp_err_t hardwareInterface::gpio::write(bool value)
{
	//ESP_LOGD(LOG_TAG, ">> write: pin: %d, value: %d", pin, value);
	esp_err_t err = gpio_set_level(m_pin, value ? 1 : 0);
	if (err != ESP_OK)
	{
		ESP_LOGE(LOG_TAG, "<< gpio_set_level: pin=%d, err=%d %s", m_pin, err, esp_err_to_name(err));
		return err;
	}
	return ESP_OK;
}

const gpio_num_t& hardwareInterface::gpio::getPin()
{
	return m_pin;
}