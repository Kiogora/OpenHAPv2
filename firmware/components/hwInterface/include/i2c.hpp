 /*
 *  i2c.hpp
 *
 *  Created on: Feb 24, 2017
 *  Author: kolban
 * 
 *  Modified for OpenHAPv2 by Alois Mbutura
 * 
 */

#ifndef I2C_HPP
#define I2C_HPP

#include <stdint.h>
#include <driver/i2c.h>

namespace externalHardwareInterface
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
    i2cBus(gpio_num_t sdaPin = GPIO_NUM_21, gpio_num_t sclPin = GPIO_NUM_19, uint32_t clockSpeed = 100000, 
           i2c_port_t portNum = I2C_NUM_0, bool pullupsState = pullupsDisable);

    void scanBusAddresses();
    virtual bool isPresent() = 0;

protected:
    static constexpr bool  ACK_CHECK_ENABLED   {0x1};
    static constexpr bool  ACK_CHECK_DISABLED  {0x0};

    const gpio_num_t     m_sdaPin;
    const gpio_num_t	 m_sclPin;
    i2c_port_t	         m_portNum;
    SemaphoreHandle_t   m_busMutex;
};
}

#endif