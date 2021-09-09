#ifndef  SDS011_HPP
#define  SDS011_HPP

#include "driver/gpio.h"
#include "driver/uart.h"
#include "gpio.hpp"

namespace externalHardwareSubsystem
{
    namespace particulateSensor
    {
    /* Class is final and can't be overriden. */
    class SDS011
    {
    public:
    static constexpr uint32_t minimumRxFIFOLen{512};
    static constexpr uint32_t recommendedQueryDelayMs{3000};
    static constexpr uint32_t measurementStabilityMs{30000};
    static constexpr uint32_t dataIntervalMs{1000};

    static constexpr int activeReportingMeasurementLength{10};
    static constexpr int packetsToAverage{3};
    static constexpr int readBufferByteSize{packetsToAverage*activeReportingMeasurementLength};
    
    static constexpr uint32_t defaultBaudrate{9600};

    /*Compile time assertions based on nature of the hardware*/
    static_assert(defaultBaudrate == 9600U, "Baudrate not default. Check: Laser_Dust_Sensor_Control_Protocol_V1.3 PDF");
    static_assert(recommendedQueryDelayMs == 3000U, "Minimum sampling interval not default. Check: Laser_Dust_Sensor_Control_Protocol_V1.3 PDF");
    static_assert(measurementStabilityMs == 30000U, "Data stability delay is not default. Check: Laser_Dust_Sensor_Control_Protocol_V1.3 PDF");
    static_assert(dataIntervalMs== 1000U, "Serial data output interval not default. Check: Laser PM2.5 Sensor specification PDF");

    static_assert(readBufferByteSize/activeReportingMeasurementLength >= packetsToAverage, "Buffer will collect less packets than needed to average");
        
    externalHardwareInterface::gpio powerState;

    /*Constructor method*/
    SDS011(gpio_num_t rxPin = GPIO_NUM_22, gpio_num_t txPin = GPIO_NUM_23, gpio_num_t loadswitchGpio = GPIO_NUM_26, uart_port_t uartPort = UART_NUM_1);
        
    esp_err_t getParticulateMeasurement(float& PM2_5, const size_t numPacketsToAverage = packetsToAverage, bool useExistingNormalizedResponse = true);
        
    private:
    uart_port_t uartPort;
    uint8_t readBuffer[readBufferByteSize];

    int performDataAcquisition(uint32_t waitTimeMs);
    esp_err_t parseBuffer(const int bytesRead, float& measurement_sum, size_t& numReadingsFound);
    esp_err_t applyCorrectionFactors(float& PM2_5);
    uint16_t inline conv_int_8_16(uint8_t msb, uint8_t lsb);

    };
    }
}
#endif
