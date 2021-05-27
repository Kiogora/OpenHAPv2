/*
 * GPIO.hpp
 *
 *  Created on: Feb 28, 2017
 *  Author: kolban
 * 
 *  Modified for OpenHAPv2 by Alois Mbutura
 * 
 */

#ifndef  GPIO_HPP
#define  GPIO_HPP
#include <driver/gpio.h>

namespace externalHardwareInterface
{
enum class gpioDirection
{
	input,
	output
};


/**
 * @brief Interface to %GPIO functions.
 *
 * The %GPIO functions encapsulate the %GPIO access.  The GPIOs available to us are
 * 0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,21,23,25,26,27,32,33,34,35,36,37,38,39.
 *
 * The GPIOs of 34,35,36,37,38 and 39 are input only.
 *
 * Note that we must not use `int` values for the pin numbers but instead use the `gpio_num_t`.  There
 * are constants defined for these of the form `GPIO_NUM_xx`.
 *
 * To toggle a pin we might code:
 *
 * @code{.cpp}
 * hardware::gpio::setOutput(pin);
 * hardware::gpio::write(pin, false);
 * hardware::gpio::write(pin, true);
 * @endcode
 */

class gpio
{
public:
	static constexpr gpioDirection input{gpioDirection::input};
	static constexpr gpioDirection output{gpioDirection::output};

	gpio(gpio_num_t pin, gpioDirection direction, bool inverted = false);

	bool read();
	esp_err_t write(bool value);

	esp_err_t on();
	esp_err_t off();
	const gpio_num_t& getPin();
private:
	const gpio_num_t m_pin;
	gpioDirection m_direction;
	bool inverted;
};

}
#endif