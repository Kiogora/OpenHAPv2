#ifndef  MLX90641_HPP
#define  MLX90641_HPP

/**
 * @copyright (C) 2017 Melexis N.V.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */


#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "i2c.hpp"

namespace externalHardwareSubsystem
{
    namespace thermalImaging
    {
    /* Class is final and can't be overriden. */
    class MLX90641 : public externalHardwareInterface::i2cBus
    {
    public:
        static constexpr uint8_t pixelCount{192};
        /*Compile time assertions for public driver data based on nature of the hardware*/
        static_assert(pixelCount==192U, "MLX90641 pixel count not default. Check datasheet");

        /*Configurations - setting*/
        enum struct supportedRefreshRates: uint8_t{_0_5Hz = 0x00,_1Hz = 0x01,_2Hz = 0x02,_4Hz = 0x03,_8Hz = 0x04, _16Hz = 0x05,_32Hz = 0x06, _64Hz = 0x07};
        enum struct supportedResolutions: uint8_t{_16bit = 0x00,_17bit = 0x01,_18bit = 0x02,_19bit = 0x03};
    
        /*Constructor methods*/
        MLX90641(uint8_t address=factorySetAddress, uint32_t timeout = busTimeout);
        MLX90641(const externalHardwareInterface::i2cBus& otherBusDevice, uint8_t address=factorySetAddress, uint32_t timeout = busTimeout);

        /*Data acquisition functions*/
        const float* GetImage();
        const float* getAndPrintImage();
        void BadPixelsCorrection(uint16_t pixel);

        /*Configuration functions*/
        int SetResolution(supportedResolutions resolution);
        int GetCurResolution();
        int SetRefreshRate(supportedRefreshRates refreshRate);   
        int GetRefreshRate();

        /*Configuration helper functions for printing*/
        float getPrintableRefreshRate();
        int getPrintableResolution();

    private:
        struct operatingParams
        {
            int16_t kVdd;int16_t vdd25;float KvPTAT;float KtPTAT;uint16_t vPTAT25;
            float alphaPTAT;int16_t gainEE;float tgc;float cpKv;float cpKta;
            uint8_t resolutionEE;uint8_t calibrationModeEE;float KsTa;float ksTo[8];
            int16_t ct[8];uint16_t alpha[192];uint8_t alphaScale;int16_t offset[2][192];    
            int8_t kta[192];uint8_t ktaScale;int8_t kv[192];uint8_t kvScale;float cpAlpha;
            int16_t cpOffset;float emissivityEE;uint16_t brokenPixel;
        };

        static constexpr uint8_t factorySetAddress{0x33};
        /*Compile time assertions for private driver data based on nature of the hardware*/
        static_assert(factorySetAddress==0x33U, "MLX90641 I2C address not default. Check datasheet");

        static constexpr int busTimeout{1000};

        static constexpr float defaultBaselineTemperature{23.15};
        static constexpr float defaultEmissivity{0.95};
        static constexpr float SCALEALPHA     {0.000001};

        /*Configurations - getting*/
        static constexpr float refreshratesTable[] {0.5, 1., 2., 4, 8., 16., 32., 64.};
        static constexpr int resolutionsTable[] {16, 17, 18, 19};

        uint8_t m_address;
        uint32_t m_timeoutms;
        operatingParams paramsMLX90641;
        float frameBuffer[pixelCount];

        int read(uint16_t startAddress, uint16_t nMemAddressRead, uint16_t *data);
        int write(uint16_t writeAddress, uint16_t data);
        int generalReset();

        /*Utility methods*/
        int DumpEE(uint16_t *eeData);
        int SynchFrame();
        int TriggerMeasurement();
        int GetFrameData(uint16_t *frameData);
        int ExtractParameters(uint16_t *eeData);
        float GetVdd(uint16_t *frameData) const;
        float GetTa(uint16_t *frameData) const;
        void CalculateTo(uint16_t *frameData, float emissivity, float tr, float *result) const;
        int GetSubPageNumber(uint16_t *frameData);
        float GetEmissivity() const;

        int HammingDecode(uint16_t *eeData);
        int ValidateFrameData(uint16_t *frameData);
        int ValidateAuxData(uint16_t *auxData);
        void ExtractVDDParameters(uint16_t *eeData);
        void ExtractPTATParameters(uint16_t *eeData);
        void ExtractGainParameters(uint16_t *eeData);
        void ExtractTgcParameters(uint16_t *eeData);
        void ExtractEmissivityParameters(uint16_t *eeData);
        void ExtractResolutionParameters(uint16_t *eeData);
        void ExtractKsTaParameters(uint16_t *eeData);
        void ExtractKsToParameters(uint16_t *eeData);
        void ExtractCPParameters(uint16_t *eeData);
        void ExtractAlphaParameters(uint16_t *eeData);
        void ExtractOffsetParameters(uint16_t *eeData);
        void ExtractKtaPixelParameters(uint16_t *eeData);
        void ExtractKvPixelParameters(uint16_t *eeData);       
        int ExtractDeviatingPixels(uint16_t *eeData);
        int CheckEEPROMValid(uint16_t *eeData);
    };
    }
}
#endif