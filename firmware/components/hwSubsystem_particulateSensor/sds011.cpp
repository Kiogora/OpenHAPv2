#include <cstring>
#include "esp_err.h"
#include "esp_log.h"
#include "sds011.hpp"
#include "gpio.hpp"

#define TAG "externalHardwareSubsystem::particulateSensor"

externalHardwareSubsystem::particulateSensor::SDS011::SDS011(gpio_num_t rxPin, gpio_num_t txPin, gpio_num_t loadswitchGpio, uart_port_t uartPort):
powerState(GPIO_NUM_26, externalHardwareInterface::gpio::output), uartPort(uartPort)
{   
    const uart_config_t uart_config = 
    {
    .baud_rate = defaultBaudrate,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    .source_clk = UART_SCLK_APB,
    };
        
    uart_driver_install(uartPort, 2 * readBufferByteSize, 0, 0, NULL, 0);
    uart_param_config(uartPort, &uart_config);
    uart_set_pin(uartPort, txPin, rxPin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

esp_err_t externalHardwareSubsystem::particulateSensor::SDS011::getParticulateMeasurement(uint16_t& PM2_5)
{
    const int bytesRead  = performDataAcquisition();
    
    ESP_LOGD(TAG, "Read %d bytes", bytesRead);
    if (bytesRead != activeReportingMeasurementLength)
    {
        memset(readBuffer, 0, readBufferByteSize);
        return ESP_ERR_INVALID_SIZE;
    }

    performDataProcessing(PM2_5);
    memset(readBuffer, 0, readBufferByteSize);
    return ESP_OK;
}

int externalHardwareSubsystem::particulateSensor::SDS011::performDataAcquisition()
{
    vTaskDelay(recommendedQueryDelayMs /portTICK_RATE_MS);
    uart_flush(uartPort);
    int bytesRead = uart_read_bytes(uartPort, readBuffer, readBufferByteSize, dataIntervalMs / portTICK_RATE_MS);
    ESP_LOG_BUFFER_HEXDUMP(TAG, readBuffer, readBufferByteSize, ESP_LOG_INFO);
    return bytesRead;
}

void externalHardwareSubsystem::particulateSensor::SDS011::performDataProcessing(uint16_t& PM2_5)
{
    const uint8_t PM_25_MSB_POSITION{3};
    const uint8_t PM_25_LSB_POSITION{2};
    PM2_5 = conv_int_8_16(readBuffer[PM_25_MSB_POSITION], readBuffer[PM_25_LSB_POSITION]);
}

uint16_t inline externalHardwareSubsystem::particulateSensor::SDS011::conv_int_8_16(uint8_t msb, uint8_t lsb)
{
    uint16_t combined = msb << 8 | lsb;
    return combined;
}