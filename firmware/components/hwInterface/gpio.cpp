/*
 * gpio.cpp
 *
 *  Created on: Feb 28, 2017
 *      Author: kolban
 */

#include "driver/gpio.h"
#include "esp_log.h"
#include <esp_err.h>
#include "gpio.hpp"

static const char* TAG = "externalHardwareInterface::gpio";

externalHardwareInterface::gpio::gpio(gpio_num_t pin, gpioDirection direction, bool inverted):m_pin(pin), m_direction(direction), inverted(inverted)
{	
	if(direction == input)
	{
		gpio_set_direction(pin, GPIO_MODE_INPUT);
	}

	if(direction == output)
	{
		gpio_set_direction(pin, GPIO_MODE_INPUT_OUTPUT);
	}
}

bool externalHardwareInterface::gpio::read() const
{
	return inverted? gpio_get_level(m_pin) == 0 : gpio_get_level(m_pin) == 1;
}

esp_err_t externalHardwareInterface::gpio::write(bool value) const
{
	esp_err_t err = inverted? gpio_set_level(m_pin, value ? 0 : 1): gpio_set_level(m_pin, value ? 1: 0);
	if (err != ESP_OK)
	{
		ESP_LOGE(TAG, "<< gpio_set_level: pin=%d, err=%d %s", m_pin, err, esp_err_to_name(err));
		return err;
	}
	return ESP_OK;
}

esp_err_t externalHardwareInterface::gpio::on() const
{
	return write(true);
}

esp_err_t externalHardwareInterface::gpio::off() const
{
	return write(false);
}

esp_err_t externalHardwareInterface::gpio::toggle() const
{
	return write(!read());
}

gpio_num_t externalHardwareInterface::gpio::getPin() const
{
	return m_pin;
}