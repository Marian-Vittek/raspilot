/*

The MIT License

Copyright (c) 2014-2023 Korneliusz Jarzębski

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

*/

#ifndef MS5611_h
#define MS5611_h

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#define MS5611_ADDRESS                (0x77)

#define MS5611_CMD_ADC_READ           (0x00)
#define MS5611_CMD_RESET              (0x1E)
#define MS5611_CMD_CONV_D1            (0x40)
#define MS5611_CMD_CONV_D2            (0x50)
#define MS5611_CMD_READ_PROM          (0xA2)

typedef enum
{
    MS5611_ULTRA_HIGH_RES   = 0x08,
    MS5611_HIGH_RES         = 0x06,
    MS5611_STANDARD         = 0x04,
    MS5611_LOW_POWER        = 0x02,
    MS5611_ULTRA_LOW_POWER  = 0x00
} ms5611_osr_t;

class MS5611
{
    public:

        bool begin(char *path = (char*)"/dev/i2c-1", int devAddr = 0x77, ms5611_osr_t osr = MS5611_HIGH_RES);
	uint32_t readRawTemperature(void);
	uint32_t readRawPressure(void);
	double readTemperature(bool compensation = false);
	int32_t readPressure(bool compensation = false);
	double getAltitude(double pressure, double seaLevelPressure = 101325);
	double getSeaLevel(double pressure, double altitude);
	void setOversampling(ms5611_osr_t osr);
	ms5611_osr_t getOversampling(void);

    private:
	
	uint16_t fc[6];
	uint8_t ct;
	uint8_t uosr;
	int32_t TEMP2;
	int64_t OFF2, SENS2;

	void reset(void);
	void readPROM(void);
	int i2cFd;

	uint16_t readRegister16(uint8_t reg);
	uint32_t readRegister24(uint8_t reg);

};

#endif
