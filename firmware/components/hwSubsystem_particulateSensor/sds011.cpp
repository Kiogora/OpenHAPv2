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
    memset(readBuffer, 0, readBufferByteSize);
}

esp_err_t externalHardwareSubsystem::particulateSensor::SDS011::applyCorrectionFactors(float& PM2_5)
{
    /*A temporary placeholder*/
    return ESP_OK;
}

esp_err_t externalHardwareSubsystem::particulateSensor::SDS011::getParticulateMeasurement(float& PM2_5, size_t numPacketsToAverage)
{
    if (powerState.read() == powerState.inactive)
    {
        ESP_LOGE(TAG, "SDS011 sensor not activated!");
        return ESP_ERR_INVALID_STATE;
    }
    
    const int bytesRead  = performDataAcquisition((1.5*dataIntervalMs)*numPacketsToAverage);
    ESP_LOGD(TAG, "Read %d bytes", bytesRead);
    if (bytesRead == -1)
    {
        return ESP_ERR_FAIL;
    }
    if (bytesRead < activeReportingMeasurementLength)
    {
        memset(readBuffer, 0, bytesRead);
        return ESP_ERR_INVALID_SIZE;
    }

    float measurementSum = 0.;
    uint32_t numReadingsFound = 0;
    
    esp_err_t ret = parseBuffer(bytesRead, measurementSum, numReadingsFound)    
    if (ret != ESP_OK)
    {
        memset(readBuffer, 0, bytesRead);
        return ret;
    }
    PM2_5 = measurementSum/numReadingsFound;
    applyCorrectionFactors(PM2_5);
    memset(readBuffer, 0, bytesRead);
    return ESP_OK;
}

int externalHardwareSubsystem::particulateSensor::SDS011::performDataAcquisition(uint32_t waitTimeMs)
{
    vTaskDelay(recommendedQueryDelayMs /portTICK_RATE_MS);
    uart_flush(uartPort);
    int bytesRead = uart_read_bytes(uartPort, readBuffer, readBufferByteSize, waitTimeMs / portTICK_RATE_MS);
    ESP_LOG_BUFFER_HEXDUMP(TAG, readBuffer, readBufferByteSize, ESP_LOG_DEBUG);
    return bytesRead;
}

esp_err_t externalHardwareSubsystem::particulateSensor::SDS011::parseBuffer(const int bytesRead, float& measurement_sum, size_t& numReadingsFound)
{
    uint8_t _step = 0;
    float measurement = 0.;

    size_t checksum = 0;
    uint8_t packet_checksum = 0;

    uint8_t pm_2_5_msb = 0;
    uint8_t pm_2_5_lsb = 0;

    measurement_sum = 0;
    numReadingsFound = 0;
    for (size_t i = 0; i < bytesRead; ++i)
    {
        const auto b = readBuffer[i];
        switch (_step)
        {
            /*Head*/
            case 0:
                if (b != 0xAA)
                {
                    _step = 0;
                    continue;
                }
                ESP_LOGI(TAG, "Parsing step 1/4: Found packet header, continuing parsing...");
                ++_step;
                break;

            /*Command ID*/
            case 1:
                if (b != 0xC0)
                {
                    ESP_LOGI(TAG, "Parsing step 2/4: Found invalid command ID 0x%X, continuing parsing...", b);
                    _step = 0;
                    continue;
                }
                ESP_LOGI(TAG, "Parsing step 2/4: Found valid command ID, continuing parsing...");
                ++_step;
                break;

            /*Checksum - Not including Head, command ID  or tail*/
            case 2:
                checksum = 0;
                packet_checksum = readBuffer[i+7];
                for(size_t checksum_input = 0; checksum_input < 6; ++checksum_input)
                {
                    checksum_input += readBuffer[i+checksum_input];
                }
                checksum %= 0xFF;
                if(checksum == packet_checksum)
                {
                    ESP_LOGI(TAG, "Parsing step 3/4: Checksum invalid, packet checksum is 0x%X, found 0x%X...continuing parsing...", packet_checksum, checksum);
                    _step = 0;
                    continue;
                }
                ESP_LOGI(TAG, "Parsing step 3/4: checksum match");
                ++_step;
                break;

            /*Obtain the PM2.5 reading*/
            case 3:
                pm_2_5_msb = readBuffer[i+1];
                pm_2_5_lsb = readBuffer[i];
                measurement = ( conv_int_8_16(pm_2_5_msb, pm_2_5_lsb) / 10 );
                ESP_LOGI(TAG, "Parsing step 4/4: Found measurement of %f ug/m3...continuing parsing...", measurement);
                measurement_sum += measurement;
                ++numReadingsFound;
                _step = 0;
                continue;
        }
    }
    if(numReadingsFound == 0)
    {
        return ESP_ERR_NOT_FOUND;
    }
    return ESP_OK;
}

uint16_t inline externalHardwareSubsystem::particulateSensor::SDS011::conv_int_8_16(uint8_t msb, uint8_t lsb)
{
    uint16_t combined = msb << 8 | lsb;
    return combined;
}
