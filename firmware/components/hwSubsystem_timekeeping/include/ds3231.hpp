#ifndef  DS3231_HPP
#define  DS3231_HPP


#include <ctime>
#include <stdint.h>
#include <esp_err.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "i2c.hpp"

namespace externalHardwareSubsystem
{
    namespace timekeeping
    {
    class DS3231: public externalHardwareInterface::i2cBus
    {
    public:
        DS3231(uint8_t address=factorySetAddress, uint32_t timeout = busTimeout);
        DS3231(const externalHardwareInterface::i2cBus& otherBusDevice, uint8_t address=factorySetAddress, uint32_t timeout = busTimeout);

        bool isPresent();

        esp_err_t get_time(std::time_t& unixtime);
        esp_err_t set_time(const std::time_t& unixtime);

        esp_err_t get_time(std::tm* time);
        esp_err_t set_time(const std::tm* time);

    protected:
        /*Default DS3231 I2C address*/
        static constexpr uint8_t factorySetAddress{0x68};
        /*Compile time assertions for private driver data based on nature of the hardware*/
        static_assert(factorySetAddress==0x68U, "DS3231 I2C address not default. Check datasheet");

        static constexpr int busTimeout{1000};

        /*regAddress*/
        static constexpr uint8_t  reg_time     {0x00};
        /*flags*/
        static constexpr uint8_t  flag_12hour  {0x40};
        static constexpr uint8_t  flag_PM      {0x20};
        /*masks*/
        static constexpr uint8_t  mask_12hour  {0x1f};
        static constexpr uint8_t  mask_month   {0x1f};

        uint8_t dec2bcd(uint8_t val);
        uint8_t bcd2dec(uint8_t val);

        inline esp_err_t write_reg(uint8_t reg, const void *out_data, size_t out_size);    
        inline esp_err_t read_reg(uint8_t reg, void *in_data, size_t in_size);

        esp_err_t read(const void *out_data, size_t out_size, void *in_data, size_t in_size);
        esp_err_t write(const void *out_reg, size_t out_reg_size, const void *out_data, size_t out_size);

    private:
        uint8_t m_address;
        uint32_t m_timeoutms;
    };
    }
}
#endif