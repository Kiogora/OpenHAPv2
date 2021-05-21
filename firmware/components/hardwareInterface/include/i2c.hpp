 /*
 *  i2c.hpp
 *
 *  Created on: Feb 24, 2017
 *  Author: kolban
 * 
 *  Modified for OpenHAPv2 by Alois Mbutura
 * 
 */

#ifndef UTILS_I2C_HPP
#define UTILS_I2C_HPP
#include <stdint.h>
#include <sys/types.h>
#include <driver/i2c.h>
#include <driver/gpio.h>

namespace hardwareInterface
{

/**
 * @brief Interface to %I2C bus functions.
 */
class i2cBus
{
public:

    static constexpr bool pullupsDisable{false}; 
    static constexpr bool pullupsEnable{true};

    /*Parameterized constructor*/
    i2cBus(uint8_t address, const SemaphoreHandle_t& i2cBusMutex, gpio_num_t sdaPin = GPIO_NUM_21, gpio_num_t sclPin = GPIO_NUM_19, uint32_t clockSpeed = 100000, 
    i2c_port_t portNum = I2C_NUM_0, bool pullupsState = pullupsDisable);

    /*Pure virtual read method - Must be overriden by derived class*/
    virtual void read() = 0;

    /*Pure virtual write method - Must be overriden by derived class*/
    virtual void write() = 0;

private:
    uint8_t		               m_address;
    gpio_num_t	               m_sdaPin;
    gpio_num_t	               m_sclPin;
    i2c_port_t	               m_portNum;
    SemaphoreHandle_t          m_busMutex;

};
}

#endif