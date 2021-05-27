#include <math.h>
#include <driver/i2c.h>
#include "mlx90641.hpp"
#include "esp_err.h"
#include "esp_log.h"

constexpr int externalHardwareSubsystem::thermalImaging::MLX90641::refreshratesTable[];
constexpr float externalHardwareSubsystem::thermalImaging::MLX90641::resolutionsTable[];

/*Ctor to attach the device to an uninitialized bus object*/
externalHardwareSubsystem::thermalImaging::MLX90641::MLX90641(SemaphoreHandle_t& i2cBusMutex, uint8_t address, uint32_t timeout): i2cBus(i2cBusMutex), m_address{address}, m_timeoutms{timeout} {}

/*Ctor to attach the device to an existing bus object*/
externalHardwareSubsystem::thermalImaging::MLX90641::MLX90641(const i2cBus& otherBusDevice, uint8_t address, uint32_t timeout): i2cBus(otherBusDevice), m_address{address}, m_timeoutms{timeout} {}

int externalHardwareSubsystem::thermalImaging::MLX90641::read(uint16_t startAddress, uint16_t nMemAddressRead, uint16_t *data)
{                          
    int cnt = 0;
    int i = 0;

    uint8_t mlx_cmd[2] = {0,0};
    uint8_t data_rd[1664] = {0};
    uint16_t *p = data;
    
    /*Change data orientation on the wire*/
    mlx_cmd[0] = (uint8_t)(startAddress >> 8);
    mlx_cmd[1] = (uint8_t)(startAddress & 0x00FF);

    i2c_cmd_handle_t i2c_cmd = i2c_cmd_link_create();
    i2c_master_start(i2c_cmd);
    i2c_master_write_byte(i2c_cmd, ( m_address << 1 ) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write(i2c_cmd, mlx_cmd, 2, ACK_CHECK_EN);
    i2c_master_start(i2c_cmd);
    i2c_master_write_byte(i2c_cmd, ( m_address << 1 ) | I2C_MASTER_READ, ACK_CHECK_EN);
    if ((nMemAddressRead*2) > 1)
    {
        i2c_master_read(i2c_cmd, data_rd, (2*nMemAddressRead)-1, I2C_MASTER_ACK);
    }

    i2c_master_read_byte(i2c_cmd, data_rd+((2*nMemAddressRead)-1), I2C_MASTER_LAST_NACK);
    i2c_master_stop(i2c_cmd);
    xSemaphoreTake(getMutex(), portMAX_DELAY);
    esp_err_t ret = i2c_master_cmd_begin(getPort(), i2c_cmd, m_timeoutms / portTICK_RATE_MS);
    xSemaphoreGive(getMutex());
    i2c_cmd_link_delete(i2c_cmd);
    if (ret != ESP_OK)
    {
        return -1;
    }

    ESP_LOG_BUFFER_HEX_LEVEL("RAW I2C BUFFER", data_rd, 2*nMemAddressRead, ESP_LOG_DEBUG);
    for(cnt=0; cnt < nMemAddressRead; cnt++)
    {
        i = cnt << 1;
        *p++ =  (uint16_t)data_rd[i+1] + (uint16_t)data_rd[i]*256;
    }

    ESP_LOG_BUFFER_HEXDUMP("RECONSTRUCTED UINT16 DATA:", data, 2*nMemAddressRead, ESP_LOG_DEBUG);
    return 0;  
}

int externalHardwareSubsystem::thermalImaging::MLX90641::write(uint16_t writeAddress, uint16_t data)
{
    uint8_t mlx_cmd[4] = {0};
    uint16_t dataCheck;

    mlx_cmd[0] = writeAddress >> 8;
    mlx_cmd[1] = writeAddress & 0x00FF;
    mlx_cmd[2] = data >> 8;
    mlx_cmd[3] = data & 0x00FF;

    i2c_cmd_handle_t i2c_cmd = i2c_cmd_link_create();
    i2c_master_start(i2c_cmd);
    i2c_master_write_byte(i2c_cmd, ( m_address << 1 ) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write(i2c_cmd, mlx_cmd, 4, ACK_CHECK_EN);
    i2c_master_stop(i2c_cmd);
    xSemaphoreTake(getMutex(), portMAX_DELAY);
    esp_err_t ret = i2c_master_cmd_begin(getPort(), i2c_cmd, m_timeoutms / portTICK_RATE_MS);
    xSemaphoreGive(getMutex());
    i2c_cmd_link_delete(i2c_cmd);
    if (ret != ESP_OK)
    {
        return -1;
    }   
    
    /*Read-back verification*/
    read(writeAddress, 1, &dataCheck);
    if ( dataCheck != data)
    {
        return -2;
    }    
    return 0;
}

int externalHardwareSubsystem::thermalImaging::MLX90641::generalReset()
{    
    uint8_t mlx_cmd[2] = {0,0};
    
    mlx_cmd[0] = 0x00;
    mlx_cmd[1] = 0x06;

    i2c_cmd_handle_t i2c_cmd = i2c_cmd_link_create();
    i2c_master_start(i2c_cmd);
    i2c_master_write_byte(i2c_cmd, ( m_address << 1 ) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write(i2c_cmd, mlx_cmd, 2, ACK_CHECK_EN);
    i2c_master_stop(i2c_cmd);
    xSemaphoreTake(getMutex(), portMAX_DELAY);
    esp_err_t ret = i2c_master_cmd_begin(getPort(), i2c_cmd, m_timeoutms / portTICK_RATE_MS);
    xSemaphoreGive(getMutex());
    i2c_cmd_link_delete(i2c_cmd);
    if (ret != ESP_OK)
    {
        return -1;
    } 
    return 0;   
}

int externalHardwareSubsystem::thermalImaging::MLX90641::DumpEE(uint16_t *eeData)
{
     int error = 1;
     error = read(0x2400, 832, eeData);
     if (error == 0)
     {
        error = HammingDecode(eeData);  
     }
         
     return error;
}

//------------------------------------------------------------------------------

int externalHardwareSubsystem::thermalImaging::MLX90641::HammingDecode(uint16_t *eeData)
{
    int error = 0;
    int16_t parity[5];
    int8_t D[16];
    int16_t check;
    uint16_t data;
    uint16_t mask;
    
    for (int addr=16; addr<832; addr++)
    {   
        parity[0] = -1;
        parity[1] = -1;
        parity[2] = -1;
        parity[3] = -1;
        parity[4] = -1;
        
        data = eeData[addr];
        
        mask = 1;
        for( int i = 0; i < 16; i++)
        {          
          D[i] = (data & mask) >> i;
          mask = mask << 1;
        }
        
        parity[0] = D[0]^D[1]^D[3]^D[4]^D[6]^D[8]^D[10]^D[11];
        parity[1] = D[0]^D[2]^D[3]^D[5]^D[6]^D[9]^D[10]^D[12];
        parity[2] = D[1]^D[2]^D[3]^D[7]^D[8]^D[9]^D[10]^D[13];
        parity[3] = D[4]^D[5]^D[6]^D[7]^D[8]^D[9]^D[10]^D[14];
        parity[4] = D[0]^D[1]^D[2]^D[3]^D[4]^D[5]^D[6]^D[7]^D[8]^D[9]^D[10]^D[11]^D[12]^D[13]^D[14]^D[15];
        
        
        if ((parity[0]!=0) || (parity[1]!=0) || (parity[2]!=0) || (parity[3]!=0) || (parity[4]!=0))
        {        
            check = (parity[0]<<0) + (parity[1]<<1) + (parity[2]<<2) + (parity[3]<<3) + (parity[4]<<4);
    
            if ((check > 15)&&(check < 32))
            {
                switch (check)
                {    
                    case 16:
                        D[15] = 1 - D[15];
                        break;
                    
                    case 24:
                        D[14] = 1 - D[14];
                        break;
                        
                    case 20:
                        D[13] = 1 - D[13];
                        break;
                        
                    case 18:
                        D[12] = 1 - D[12];
                        break;                                
                        
                    case 17:
                        D[11] = 1 - D[11];
                        break;
                        
                    case 31:
                        D[10] = 1 - D[10];
                        break;
                        
                    case 30:
                        D[9] = 1 - D[9];
                        break;
                    
                    case 29:
                        D[8] = 1 - D[8];
                        break;                
                    
                    case 28:
                        D[7] = 1 - D[7];
                        break;
                        
                    case 27:
                        D[6] = 1 - D[6];
                        break;
                            
                    case 26:
                        D[5] = 1 - D[5];
                        break;    
                        
                    case 25:
                        D[4] = 1 - D[4];
                        break;     
                        
                    case 23:
                        D[3] = 1 - D[3];
                        break; 
                        
                    case 22:
                        D[2] = 1 - D[2];
                        break; 
                            
                    case 21:
                        D[1] = 1 - D[1];
                        break; 
                        
                    case 19:
                        D[0] = 1 - D[0];
                        break;     
                                     
                }
               
                if(error == 0)
                {
                    error = -9;
                   
                }
                
                data = 0;
                mask = 1;
                for( int i = 0; i < 16; i++)
                {                    
                    data = data + D[i]*mask;
                    mask = mask << 1;
                }
       
            }
            else
            {
                error = -10;                
            }   
        }
        
        eeData[addr] = data & 0x07FF;
    }
    
    return error;
}

//------------------------------------------------------------------------------

int externalHardwareSubsystem::thermalImaging::MLX90641::SynchFrame()
{
    uint16_t dataReady = 0;
    uint16_t statusRegister;
    int error = 1;
    
    error = write(0x8000, 0x0030);
    if(error == -1)
    {
        return error;
    }
    
    while(dataReady == 0)
    {
        error = read(0x8000, 1, &statusRegister);
        if(error != 0)
        {
            return error;
        }    
        dataReady = statusRegister & 0x0008;
    }      
    
   return 0;   
}

//------------------------------------------------------------------------------

int externalHardwareSubsystem::thermalImaging::MLX90641::TriggerMeasurement()
{
    int error = 1;
    uint16_t ctrlReg;
    
    error = read(0x800D, 1, &ctrlReg);
    
    if ( error != 0) 
    {
        return error;
    }    
                                                
    ctrlReg |= 0x8000;
    error = write(0x800D, ctrlReg);
    
    if ( error != 0)
    {
        return error;
    }    
    
    error = externalHardwareSubsystem::thermalImaging::MLX90641::generalReset();
    
    if ( error != 0)
    {
        return error;
    }    
    
    error = read(0x800D, 1, &ctrlReg);
    
    if ( error != 0)
    {
        return error;
    }    
    
    if ((ctrlReg & 0x8000) != 0)
    {
        return -11;
    }
    
    return 0;    
}

//------------------------------------------------------------------------------

int externalHardwareSubsystem::thermalImaging::MLX90641::GetFrameData(uint16_t *frameData)
{
    uint16_t dataReady = 1;
    uint16_t controlRegister1;
    uint16_t statusRegister;
    uint16_t data[48];
    int error = 1;
    uint8_t cnt = 0;
    uint8_t subPage = 0;
    
    dataReady = 0;
    while(dataReady == 0)
    {
        error = read(0x8000, 1, &statusRegister);
        if(error != 0)
        {
            return error;
        }    
        dataReady = statusRegister & 0x0008;
    }   
    subPage = statusRegister & 0x0001;
        
    error = write(0x8000, 0x0030);
    if(error == -1)
    {
        return error;
    }
                
    if(subPage == 0)
    { 
        error = read(0x0400, 32, frameData); 
        if(error != 0)
        {
            return error;
        }
        error = read(0x0440, 32, frameData+32); 
        if(error != 0)
        {
            return error;
        }
        error = read(0x0480, 32, frameData+64); 
        if(error != 0)
        {
            return error;
        }
        error = read(0x04C0, 32, frameData+96); 
        if(error != 0)
        {
            return error;
        }
        error = read(0x0500, 32, frameData+128); 
        if(error != 0)
        {
            return error;
        }
        error = read(0x0540, 32, frameData+160); 
        if(error != 0)
        {
            return error;
        }
    }    
    else
    {
        error = read(0x0420, 32, frameData); 
        if(error != 0)
        {
            return error;
        }
        error = read(0x0460, 32, frameData+32); 
        if(error != 0)
        {
            return error;
        }
        error = read(0x04A0, 32, frameData+64); 
        if(error != 0)
        {
            return error;
        }
        error = read(0x04E0, 32, frameData+96); 
        if(error != 0)
        {
            return error;
        }
        error = read(0x0520, 32, frameData+128); 
        if(error != 0)
        {
            return error;
        }
        error = read(0x0560, 32, frameData+160); 
        if(error != 0)
        {
            return error;
        }
    }   
    
    error = read(0x0580, 48, data); 
    if(error != 0)
    {
        return error;
    }            
    
    
    error = read(0x800D, 1, &controlRegister1);
    frameData[240] = controlRegister1;
    frameData[241] = subPage;

    if(error != 0)
    {
        return error;
    }
    
    error = ValidateAuxData(data);
    if(error == 0)
    {
        for(cnt=0; cnt<48; cnt++)
        {
            frameData[cnt+pixelCount] = data[cnt];
        }
    }        
    
    error = ValidateFrameData(frameData);
    if (error != 0)
    {
        return error;
    }                                 
    
    return frameData[241];    
}

int externalHardwareSubsystem::thermalImaging::MLX90641::ValidateFrameData(uint16_t *frameData)
{
    uint8_t line = 0;
    
    for(int i=0; i<pixelCount; i+=16)
    {
        if(frameData[i] == 0x7FFF) return -8;
        line = line + 1;
    }    
        
    return 0;    
}

int externalHardwareSubsystem::thermalImaging::MLX90641::ValidateAuxData(uint16_t *auxData) 
{
    
    if(auxData[0] == 0x7FFF) return -8;    
    
    for(int i=8; i<19; i++)
    {
        if(auxData[i] == 0x7FFF) return -8;
    }
    
    for(int i=20; i<23; i++)
    {
        if(auxData[i] == 0x7FFF) return -8;
    }
    
    for(int i=24; i<33; i++)
    {
        if(auxData[i] == 0x7FFF) return -8;
    }
    
    for(int i=40; i<48; i++)
    {
        if(auxData[i] == 0x7FFF) return -8;
    }
    
    return 0;
    
}

//------------------------------------------------------------------------------

int externalHardwareSubsystem::thermalImaging::MLX90641::ExtractParameters(uint16_t *eeData)
{
    int error = CheckEEPROMValid(eeData);
    
    if(error == 0)
    {
        ExtractVDDParameters(eeData);
        ExtractPTATParameters(eeData);
        ExtractGainParameters(eeData);
        ExtractTgcParameters(eeData);
        ExtractEmissivityParameters(eeData);
        ExtractResolutionParameters(eeData);
        ExtractKsTaParameters(eeData);
        ExtractKsToParameters(eeData);
        ExtractCPParameters(eeData);
        ExtractAlphaParameters(eeData);
        ExtractOffsetParameters(eeData);
        ExtractKtaPixelParameters(eeData);
        ExtractKvPixelParameters(eeData);        
        error = ExtractDeviatingPixels(eeData);  
    }
    
    return error;

}

//------------------------------------------------------------------------------

int externalHardwareSubsystem::thermalImaging::MLX90641::SetResolution(supportedResolutions resolution)
{
    uint16_t controlRegister1;
    int value;
    int error;
    
    value = (static_cast<int>(resolution) & 0x03) << 10;
    
    error = read(0x800D, 1, &controlRegister1);
    
    if(error == 0)
    {
        value = (controlRegister1 & 0xF3FF) | value;
        error = write(0x800D, value);        
    }    
    
    return error;
}

//------------------------------------------------------------------------------

int externalHardwareSubsystem::thermalImaging::MLX90641::GetCurResolution()
{
    uint16_t controlRegister1;
    int resolutionRAM;
    int error;
    
    error = read(0x800D, 1, &controlRegister1);
    if(error != 0)
    {
        return error;
    }    
    resolutionRAM = (controlRegister1 & 0x0C00) >> 10;
    
    return resolutionRAM; 
}

//------------------------------------------------------------------------------

int externalHardwareSubsystem::thermalImaging::MLX90641::SetRefreshRate(supportedRefreshRates refreshRate)
{
    uint16_t controlRegister1;
    int value;
    int error;
    
    value = (static_cast<int>(refreshRate) & 0x07)<<7;
    
    error = read(0x800D, 1, &controlRegister1);
    if(error == 0)
    {
        value = (controlRegister1 & 0xFC7F) | value;
        error = write(0x800D, value);
    }    
    
    return error;
}

//------------------------------------------------------------------------------

int externalHardwareSubsystem::thermalImaging::MLX90641::GetRefreshRate()
{
    uint16_t controlRegister1;
    int refreshRate;
    int error;
    
    error = read(0x800D, 1, &controlRegister1);
    if(error != 0)
    {
        return error;
    }    
    refreshRate = (controlRegister1 & 0x0380) >> 7;
    
    return refreshRate;
}

//------------------------------------------------------------------------------
float externalHardwareSubsystem::thermalImaging::MLX90641::getPrintableRefreshRate()
{
    int refreshRate = GetRefreshRate();
    return (refreshRate < 0)? 0. : refreshratesTable[refreshRate];
}
//------------------------------------------------------------------------------
int externalHardwareSubsystem::thermalImaging::MLX90641::getPrintableResolution()
{
    int resolution = GetCurResolution();
    return (resolution < 0)? -1 : resolutionsTable[resolution];
}

//------------------------------------------------------------------------------
void externalHardwareSubsystem::thermalImaging::MLX90641::CalculateTo(uint16_t *frameData, float emissivity, float tr, float *result) const
{
    float vdd;
    float ta;
    float ta4;
    float tr4;
    float taTr;
    float gain;
    float irDataCP;
    float irData;
    float alphaCompensated;
    float Sx;
    float To;
    float alphaCorrR[8];
    int8_t range;
    uint16_t subPage;
    float ktaScale;
    float kvScale;
    float alphaScale;
    float kta;
    float kv;
    
    subPage = frameData[241];
    vdd = externalHardwareSubsystem::thermalImaging::MLX90641::GetVdd(frameData);
    ta = externalHardwareSubsystem::thermalImaging::MLX90641::GetTa(frameData);    
    ta4 = (ta + 273.15);
    ta4 = ta4 * ta4;
    ta4 = ta4 * ta4;
    tr4 = (tr + 273.15);
    tr4 = tr4 * tr4;
    tr4 = tr4 * tr4;
    
    taTr = tr4 - (tr4-ta4)/emissivity;
    
    ktaScale = pow(2,(double)paramsMLX90641.ktaScale);
    kvScale = pow(2,(double)paramsMLX90641.kvScale);
    alphaScale = pow(2,(double)paramsMLX90641.alphaScale);
    
    alphaCorrR[1] = 1 / (1 + paramsMLX90641.ksTo[1] * 20);
    alphaCorrR[0] = alphaCorrR[1] / (1 + paramsMLX90641.ksTo[0] * 20);
    alphaCorrR[2] = 1 ;
    alphaCorrR[3] = (1 + paramsMLX90641.ksTo[2] * paramsMLX90641.ct[3]);
    alphaCorrR[4] = alphaCorrR[3] * (1 + paramsMLX90641.ksTo[3] * (paramsMLX90641.ct[4] - paramsMLX90641.ct[3]));
    alphaCorrR[5] = alphaCorrR[4] * (1 + paramsMLX90641.ksTo[4] * (paramsMLX90641.ct[5] - paramsMLX90641.ct[4]));
    alphaCorrR[6] = alphaCorrR[5] * (1 + paramsMLX90641.ksTo[5] * (paramsMLX90641.ct[6] - paramsMLX90641.ct[5]));
    alphaCorrR[7] = alphaCorrR[6] * (1 + paramsMLX90641.ksTo[6] * (paramsMLX90641.ct[7] - paramsMLX90641.ct[6]));
    
//------------------------- Gain calculation -----------------------------------    
    gain = frameData[202];
    if(gain > 32767)
    {
        gain = gain - 65536;
    }
    
    gain = paramsMLX90641.gainEE / gain; 
  
//------------------------- To calculation -------------------------------------        
    irDataCP = frameData[200];  
    if(irDataCP > 32767)
    {
        irDataCP = irDataCP - 65536;
    }
    irDataCP = irDataCP * gain;

    irDataCP = irDataCP - paramsMLX90641.cpOffset * (1 + paramsMLX90641.cpKta * (ta - 25)) * (1 + paramsMLX90641.cpKv * (vdd - 3.3));
    
    for( int pixelNumber = 0; pixelNumber < pixelCount; pixelNumber++)
    {      
        irData = frameData[pixelNumber];
        if(irData > 32767)
        {
            irData = irData - 65536;
        }
        irData = irData * gain;
        
        kta = (float)paramsMLX90641.kta[pixelNumber]/ktaScale;
        kv = (float)paramsMLX90641.kv[pixelNumber]/kvScale;
            
        irData = irData - paramsMLX90641.offset[subPage][pixelNumber]*(1 + kta*(ta - 25))*(1 + kv*(vdd - 3.3));                
    
        irData = irData - paramsMLX90641.tgc * irDataCP;
        
        irData = irData / emissivity;
        
        alphaCompensated = SCALEALPHA*alphaScale/paramsMLX90641.alpha[pixelNumber];
        alphaCompensated = alphaCompensated*(1 + paramsMLX90641.KsTa * (ta - 25));
        
        Sx = alphaCompensated * alphaCompensated * alphaCompensated * (irData + alphaCompensated * taTr);
        Sx = sqrt(sqrt(Sx)) * paramsMLX90641.ksTo[2];
        
        To = sqrt(sqrt(irData/(alphaCompensated * (1 - paramsMLX90641.ksTo[2] * 273.15) + Sx) + taTr)) - 273.15;
                
        if(To < paramsMLX90641.ct[1])
        {
            range = 0;
        }
        else if(To < paramsMLX90641.ct[2])   
        {
            range = 1;            
        }   
        else if(To < paramsMLX90641.ct[3])
        {
            range = 2;            
        }
        else if(To < paramsMLX90641.ct[4])
        {
            range = 3;            
        }
        else if(To < paramsMLX90641.ct[5])
        {
            range = 4;            
        }
        else if(To < paramsMLX90641.ct[6])
        {
            range = 5;            
        }
        else if(To < paramsMLX90641.ct[7])
        {
            range = 6;            
        }
        else
        {
            range = 7;            
        }      
        
        To = sqrt(sqrt(irData / (alphaCompensated * alphaCorrR[range] * (1 + paramsMLX90641.ksTo[range] * (To - paramsMLX90641.ct[range]))) + taTr)) - 273.15;
        
        result[pixelNumber] = To;
    }
}

float* externalHardwareSubsystem::thermalImaging::MLX90641::getAndPrintImage()
{
    float* frameBuffer = GetImage();
    printf("[[");
    for(int i=0; i<pixelCount; ++i)
    {
        if (i%16 == 0 && i != 0)
        {
            printf(",],\n[%f,", frameBuffer [i]);
        }
        else
        {
            printf("%f ", frameBuffer[i]);
        }
    }
    printf("]]\n");

    return frameBuffer;
}

//------------------------------------------------------------------------------

float* externalHardwareSubsystem::thermalImaging::MLX90641::GetImage()
{
    uint16_t mlx90641Frame[242] = {0};
    uint16_t eeMLX90641[832] = {0};

    int status = DumpEE(eeMLX90641);
    status = ExtractParameters(eeMLX90641);
    status = GetFrameData(mlx90641Frame);
    CalculateTo(mlx90641Frame, GetEmissivity(), defaultBaselineTemperature, frameBuffer);
    return frameBuffer;
}

//------------------------------------------------------------------------------

float externalHardwareSubsystem::thermalImaging::MLX90641::GetVdd(uint16_t *frameData) const
{
    float vdd;
    float resolutionCorrection;
    
    int resolutionRAM;    
    
    vdd = frameData[234];
    if(vdd > 32767)
    {
        vdd = vdd - 65536;
    }
    resolutionRAM = (frameData[240] & 0x0C00) >> 10;
    resolutionCorrection = pow(2, (double)paramsMLX90641.resolutionEE) / pow(2, (double)resolutionRAM);
    vdd = (resolutionCorrection * vdd - paramsMLX90641.vdd25) / paramsMLX90641.kVdd + 3.3;
    
    return vdd;
}

    //------------------------------------------------------------------------------

float externalHardwareSubsystem::thermalImaging::MLX90641::GetTa(uint16_t *frameData) const
{
    float ptat;
    float ptatArt;
    float vdd;
    float ta;
    
    vdd = GetVdd(frameData);
    
    ptat = frameData[224];
    if(ptat > 32767)
    {
        ptat = ptat - 65536;
    }
    
    ptatArt = frameData[pixelCount];
    if(ptatArt > 32767)
    {
        ptatArt = ptatArt - 65536;
    }
    ptatArt = (ptat / (ptat * paramsMLX90641.alphaPTAT + ptatArt)) * pow(2, (double)18);
    
    ta = (ptatArt / (1 + paramsMLX90641.KvPTAT * (vdd - 3.3)) - paramsMLX90641.vPTAT25);
    ta = ta / paramsMLX90641.KtPTAT + 25;
    
    return ta;
}

//------------------------------------------------------------------------------

int externalHardwareSubsystem::thermalImaging::MLX90641::GetSubPageNumber(uint16_t *frameData)
{
    return frameData[241];    

}    

//------------------------------------------------------------------------------
void externalHardwareSubsystem::thermalImaging::MLX90641::BadPixelsCorrection(uint16_t pixel)
{   
    float ap[2];
    uint8_t line;
    uint8_t column;
    
    if(pixel<pixelCount)
    {
        line = pixel>>4;
        column = pixel - (line<<4);
               
        if(column == 0)
        {
            frameBuffer[pixel] = frameBuffer[pixel+1];            
        }
        else if(column == 1 || column == 14)
        {
            frameBuffer[pixel] = (frameBuffer[pixel-1]+frameBuffer[pixel+1])/2.0;                
        } 
        else if(column == 15)
        {
            frameBuffer[pixel] = frameBuffer[pixel-1];
        } 
        else
        {            
            ap[0] = frameBuffer[pixel+1] - frameBuffer[pixel+2];
            ap[1] = frameBuffer[pixel-1] - frameBuffer[pixel-2];
            if(fabs(ap[0]) > fabs(ap[1]))
            {
                frameBuffer[pixel] = frameBuffer[pixel-1] + ap[1];                        
            }
            else
            {
                frameBuffer[pixel] = frameBuffer[pixel+1] + ap[0];                        
            }
                    
        }                      
    }   
}    

//------------------------------------------------------------------------------
void externalHardwareSubsystem::thermalImaging::MLX90641::ExtractVDDParameters(uint16_t *eeData)
{
    int16_t kVdd;
    int16_t vdd25;
    
    kVdd = eeData[39];
    if(kVdd > 1023)
    {
        kVdd = kVdd - 2048;
    }
    kVdd = 32 * kVdd;
    
    vdd25 = eeData[38];
    if(vdd25 > 1023)
    {
        vdd25 = vdd25 - 2048;
    }
    vdd25 = 32 * vdd25;
    
    paramsMLX90641.kVdd = kVdd;
    paramsMLX90641.vdd25 = vdd25; 
}

//------------------------------------------------------------------------------

void externalHardwareSubsystem::thermalImaging::MLX90641::ExtractPTATParameters(uint16_t *eeData)
{
    float KvPTAT;
    float KtPTAT;
    int16_t vPTAT25;
    float alphaPTAT;
    
    KvPTAT = eeData[43];
    if(KvPTAT > 1023)
    {
        KvPTAT = KvPTAT - 2048;
    }
    KvPTAT = KvPTAT/4096;
    
    KtPTAT = eeData[42];
    if(KtPTAT > 1023)
    {
        KtPTAT = KtPTAT - 2048;
    }
    KtPTAT = KtPTAT/8;
    
    vPTAT25 = 32 * eeData[40] + eeData[41];
    
    alphaPTAT = eeData[44] / 128.0f;
    
    paramsMLX90641.KvPTAT = KvPTAT;
    paramsMLX90641.KtPTAT = KtPTAT;    
    paramsMLX90641.vPTAT25 = vPTAT25;
    paramsMLX90641.alphaPTAT = alphaPTAT;   
}

//------------------------------------------------------------------------------

void externalHardwareSubsystem::thermalImaging::MLX90641::ExtractGainParameters(uint16_t *eeData)
{
    int16_t gainEE;
    
    gainEE = 32 * eeData[36] + eeData[37];

    paramsMLX90641.gainEE = gainEE;    
}

//------------------------------------------------------------------------------

void externalHardwareSubsystem::thermalImaging::MLX90641::ExtractTgcParameters(uint16_t *eeData)
{
    float tgc;
    tgc = eeData[51] & 0x01FF;
    if(tgc > 255)
    {
        tgc = tgc - 512;
    }
    tgc = tgc / 64.0f;
    
    paramsMLX90641.tgc = tgc;        
}

//------------------------------------------------------------------------------

void externalHardwareSubsystem::thermalImaging::MLX90641::ExtractEmissivityParameters(uint16_t *eeData)
{
    float emissivity;
    emissivity = eeData[35];
       
    if(emissivity > 1023)
    {
        emissivity = emissivity - 2048;
    }
    emissivity = emissivity/512;
    
    paramsMLX90641.emissivityEE = emissivity;
}
    
//------------------------------------------------------------------------------

void externalHardwareSubsystem::thermalImaging::MLX90641::ExtractResolutionParameters(uint16_t *eeData)
{
    uint8_t resolutionEE;
    resolutionEE = (eeData[51] & 0x0600) >> 9;    
    
    paramsMLX90641.resolutionEE = resolutionEE;
}

//------------------------------------------------------------------------------

void externalHardwareSubsystem::thermalImaging::MLX90641::ExtractKsTaParameters(uint16_t *eeData)
{
    float KsTa;
    KsTa = eeData[34];
    if(KsTa > 1023)
    {
        KsTa = KsTa - 2048;
    }
    KsTa = KsTa / 32768.0f;
    
    paramsMLX90641.KsTa = KsTa;
}

//------------------------------------------------------------------------------

void externalHardwareSubsystem::thermalImaging::MLX90641::ExtractKsToParameters(uint16_t *eeData)
{
    int KsToScale;
    
    paramsMLX90641.ct[0] = -40;
    paramsMLX90641.ct[1] = -20;
    paramsMLX90641.ct[2] = 0;
    paramsMLX90641.ct[3] = 80;
    paramsMLX90641.ct[4] = 120;
    paramsMLX90641.ct[5] = eeData[58];
    paramsMLX90641.ct[6] = eeData[60];
    paramsMLX90641.ct[7] = eeData[62];
     
    KsToScale = eeData[52];
    KsToScale = 1 << KsToScale;
    
    paramsMLX90641.ksTo[0] = eeData[53];
    paramsMLX90641.ksTo[1] = eeData[54];
    paramsMLX90641.ksTo[2] = eeData[55];
    paramsMLX90641.ksTo[3] = eeData[56];
    paramsMLX90641.ksTo[4] = eeData[57];
    paramsMLX90641.ksTo[5] = eeData[59];
    paramsMLX90641.ksTo[6] = eeData[61];
    paramsMLX90641.ksTo[7] = eeData[63];
    
    
    for(int i = 0; i < 8; i++)
    {
        if(paramsMLX90641.ksTo[i] > 1023)
        {
            paramsMLX90641.ksTo[i] = paramsMLX90641.ksTo[i] - 2048;
        }
        paramsMLX90641.ksTo[i] = paramsMLX90641.ksTo[i] / KsToScale;
    } 
}

//------------------------------------------------------------------------------

void externalHardwareSubsystem::thermalImaging::MLX90641::ExtractAlphaParameters(uint16_t *eeData)
{
    float rowMaxAlphaNorm[6];
    uint16_t scaleRowAlpha[6];
    uint8_t alphaScale;
    float alphaTemp[pixelCount];
    float temp;
    int p = 0;

    scaleRowAlpha[0] = (eeData[25] >> 5) + 20;
    scaleRowAlpha[1] = (eeData[25] & 0x001F) + 20;
    scaleRowAlpha[2] = (eeData[26] >> 5) + 20;
    scaleRowAlpha[3] = (eeData[26] & 0x001F) + 20;
    scaleRowAlpha[4] = (eeData[27] >> 5) + 20;
    scaleRowAlpha[5] = (eeData[27] & 0x001F) + 20;

    
    for(int i = 0; i < 6; i++)
    {
        rowMaxAlphaNorm[i] = eeData[28 + i] / pow(2,(double)scaleRowAlpha[i]);
        rowMaxAlphaNorm[i] = rowMaxAlphaNorm[i] / 2047.0f;
    }

    for(int i = 0; i < 6; i++)
    {
        for(int j = 0; j < 32; j ++)
        {
            p = 32 * i +j;
            alphaTemp[p] = eeData[256 + p] * rowMaxAlphaNorm[i]; 
            alphaTemp[p] = alphaTemp[p] - paramsMLX90641.tgc * paramsMLX90641.cpAlpha;
            alphaTemp[p] = SCALEALPHA/alphaTemp[p];
        }
    }
    
    temp = alphaTemp[0];
    for(int i = 1; i < pixelCount; i++)
    {
        if (alphaTemp[i] > temp)
        {
            temp = alphaTemp[i];
        }
    }
    
    alphaScale = 0;
    while(temp < 32768)
    {
        temp = temp*2;
        alphaScale = alphaScale + 1;
    } 
    
    for(int i = 0; i < pixelCount; i++)
    {
        temp = alphaTemp[i] * pow(2,(double)alphaScale);        
        paramsMLX90641.alpha[i] = (temp + 0.5);        
        
    } 
    
    paramsMLX90641.alphaScale = alphaScale;      
}

//------------------------------------------------------------------------------

void externalHardwareSubsystem::thermalImaging::MLX90641::ExtractOffsetParameters(uint16_t *eeData)
{
    int scaleOffset;
    int16_t offsetRef;
    int16_t tempOffset; 
    
    scaleOffset = eeData[16] >> 5;
    scaleOffset = 1 << scaleOffset;

    offsetRef = 32 * eeData[17] + eeData[18];
    if (offsetRef > 32767)
    {
        offsetRef = offsetRef - 65536;
    }

    for(int i = 0; i < pixelCount; i++)
    {
        tempOffset = eeData[64 + i];
        if(tempOffset > 1023)
        {
           tempOffset = eeData[64 + i] - 2048; 
        }
        paramsMLX90641.offset[0][i] = tempOffset * scaleOffset + offsetRef;
        
        tempOffset = eeData[640 + i];
        if(tempOffset > 1023)
        {
           tempOffset = eeData[640 + i] - 2048; 
        }
        paramsMLX90641.offset[1][i] = tempOffset * scaleOffset + offsetRef;
    }
}

//------------------------------------------------------------------------------

void externalHardwareSubsystem::thermalImaging::MLX90641::ExtractKtaPixelParameters(uint16_t *eeData)
{
    uint8_t ktaScale1;
    uint8_t ktaScale2;
    int16_t ktaAvg;
    int16_t tempKta;
    float ktaTemp[pixelCount];
    float temp;

    ktaAvg = eeData[21];
    if (ktaAvg > 1023)
    {
        ktaAvg = ktaAvg - 2048;
    }
  
    ktaScale1 = eeData[22] >> 5;
    ktaScale2 = eeData[22] & 0x001F;

    for(int i = 0; i < pixelCount; i++)
    {
        tempKta = (eeData[448 + i] >> 5);
        if (tempKta > 31)
        {
            tempKta = tempKta - 64;
        }

        ktaTemp[i] = tempKta * pow(2,(double)ktaScale2);
        ktaTemp[i] = ktaTemp[i] + ktaAvg;
        ktaTemp[i] = ktaTemp[i] / pow(2,(double)ktaScale1);
    }
    
    temp = fabs(ktaTemp[0]);
    for(int i = 1; i < pixelCount; i++)
    {
        if (fabs(ktaTemp[i]) > temp)
        {
            temp = fabs(ktaTemp[i]);
        }
    }
    
    ktaScale1 = 0;
    while(temp < 64)
    {
        temp = temp*2;
        ktaScale1 = ktaScale1 + 1;
    }    
     
    for(int i = 0; i < pixelCount; i++)
    {
        temp = ktaTemp[i] * pow(2,(double)ktaScale1);
        if (temp < 0)
        {
            paramsMLX90641.kta[i] = (temp - 0.5);
        }
        else
        {
            paramsMLX90641.kta[i] = (temp + 0.5);
        }        
        
    } 
    
    paramsMLX90641.ktaScale = ktaScale1;
}

//------------------------------------------------------------------------------

void externalHardwareSubsystem::thermalImaging::MLX90641::ExtractKvPixelParameters(uint16_t *eeData)
{
    uint8_t kvScale1;
    uint8_t kvScale2;
    int16_t kvAvg;
    int16_t tempKv;
    float kvTemp[pixelCount];
    float temp;

    kvAvg = eeData[23];
    if (kvAvg > 1023)
    {
        kvAvg = kvAvg - 2048;
    }
  
    kvScale1 = eeData[24] >> 5;
    kvScale2 = eeData[24] & 0x001F;

    for(int i = 0; i < pixelCount; i++)
    {
        tempKv = (eeData[448 + i] & 0x001F);
        if (tempKv > 15)
        {
            tempKv = tempKv - 32;
        }

        kvTemp[i] = tempKv * pow(2,(double)kvScale2);
        kvTemp[i] = kvTemp[i] + kvAvg;
        kvTemp[i] = kvTemp[i] / pow(2,(double)kvScale1);
    }
    
    temp = fabs(kvTemp[0]);
    for(int i = 1; i < pixelCount; i++)
    {
        if (fabs(kvTemp[i]) > temp)
        {
            temp = fabs(kvTemp[i]);
        }
    }
    
    kvScale1 = 0;
    while(temp < 64)
    {
        temp = temp*2;
        kvScale1 = kvScale1 + 1;
    }    
     
    for(int i = 0; i < pixelCount; i++)
    {
        temp = kvTemp[i] * pow(2,(double)kvScale1);
        if (temp < 0)
        {
            paramsMLX90641.kv[i] = (temp - 0.5);
        }
        else
        {
            paramsMLX90641.kv[i] = (temp + 0.5);
        }        
        
    } 
    
    paramsMLX90641.kvScale = kvScale1;        
}

//------------------------------------------------------------------------------

void externalHardwareSubsystem::thermalImaging::MLX90641::ExtractCPParameters(uint16_t *eeData)
{
    float alphaCP;
    int16_t offsetCP;
    float cpKv;
    float cpKta;
    uint8_t alphaScale;
    uint8_t ktaScale1;
    uint8_t kvScale;

    alphaScale = eeData[46];
    
    offsetCP = 32 * eeData[47] + eeData[48];
    if (offsetCP > 32767)
    {
        offsetCP = offsetCP - 65536;
    }
       
    alphaCP = eeData[45];
    if (alphaCP > 1023)
    {
        alphaCP = alphaCP - 2048;
    }
    
    alphaCP = alphaCP /  pow(2,(double)alphaScale);
    
    
    cpKta = eeData[49] & 0x001F;
    if (cpKta > 31)
    {
        cpKta = cpKta - 64;
    }
    ktaScale1 = eeData[49] >> 6;    
    paramsMLX90641.cpKta = cpKta / pow(2,(double)ktaScale1);
    
    cpKv = eeData[50] & 0x001F;
    if (cpKv > 31)
    {
        cpKv = cpKv - 64;
    }
    kvScale = eeData[50] >> 6;
    paramsMLX90641.cpKv = cpKv / pow(2,(double)kvScale);
       
    paramsMLX90641.cpAlpha = alphaCP;
    paramsMLX90641.cpOffset = offsetCP;
}

//------------------------------------------------------------------------------

float externalHardwareSubsystem::thermalImaging::MLX90641::GetEmissivity() const
{
    return  paramsMLX90641.emissivityEE;
}

//------------------------------------------------------------------------------

int externalHardwareSubsystem::thermalImaging::MLX90641::ExtractDeviatingPixels(uint16_t *eeData)
{
    uint16_t pixCnt = 0;
    uint16_t brokenPixCnt = 0;

    int warn = 0;
    
    paramsMLX90641.brokenPixel = 0xFFFF;
        
    pixCnt = 0;    
    while (pixCnt < pixelCount && brokenPixCnt < 2)
    {
        if((eeData[pixCnt+64] == 0) && (eeData[pixCnt+256] == 0) && (eeData[pixCnt+448] == 0) && (eeData[pixCnt+640] == 0))
        {
            paramsMLX90641.brokenPixel = pixCnt;
            brokenPixCnt = brokenPixCnt + 1;
        }    
        
        pixCnt = pixCnt + 1;
    } 
    
    if(brokenPixCnt > 1)  
    {
        warn = -3;
    }         
    
    return warn;
       
}
 
 //------------------------------------------------------------------------------
 
 int externalHardwareSubsystem::thermalImaging::MLX90641::CheckEEPROMValid(uint16_t *eeData)  
 {
     int deviceSelect;
     deviceSelect = eeData[10] & 0x0040;
     if(deviceSelect != 0)
     {
         return 0;
     }
     
     return -7;    
 }        