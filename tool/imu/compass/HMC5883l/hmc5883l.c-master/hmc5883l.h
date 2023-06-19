// HMC5883L compass
// Copyright 2016 Bloutiouf
// https://opensource.org/licenses/MIT

#ifndef HMC5883L_H
#define HMC5883L_H

#include <unistd.h>
#include <stdbool.h>

#define HMC5883L_ID 0x1E

enum HMC5883L_Gain
{
    HMC5883L_GAIN_1370 = 0x00,
    HMC5883L_GAIN_1090 = 0x20, // device default
    HMC5883L_GAIN_820 = 0x40,
    HMC5883L_GAIN_660 = 0x60,
    HMC5883L_GAIN_440 = 0x80,
    HMC5883L_GAIN_390 = 0xA0,
    HMC5883L_GAIN_330 = 0xC0,
    HMC5883L_GAIN_230 = 0xE0,
};

enum HMC5883L_MeasurementMode
{
    HMC5883L_MEASUREMENTMODE_NORMAL = 0x00, // device default
    HMC5883L_MEASUREMENTMODE_POSITIVEBIAS = 0x01,
    HMC5883L_MEASUREMENTMODE_NEGATIVEBIAS = 0x02,
};

enum HMC5883L_OutputRate
{
    HMC5883L_OUTPUTRATE_0_75 = 0x00,
    HMC5883L_OUTPUTRATE_1_5 = 0x04,
    HMC5883L_OUTPUTRATE_3 = 0x08,
    HMC5883L_OUTPUTRATE_7_5 = 0x0C,
    HMC5883L_OUTPUTRATE_15 = 0x10, // device default
    HMC5883L_OUTPUTRATE_30 = 0x14,
    HMC5883L_OUTPUTRATE_75 = 0x18,
};

enum HMC5883L_Samples
{
    HMC5883L_SAMPLES_1 = 0x00, // device default
    HMC5883L_SAMPLES_2 = 0x20,
    HMC5883L_SAMPLES_4 = 0x40,
    HMC5883L_SAMPLES_8 = 0x60,
};

struct HMC5883L
{
    enum HMC5883L_Gain gain;
    enum HMC5883L_MeasurementMode measurementMode;
    enum HMC5883L_OutputRate outputRate;
    enum HMC5883L_Samples samples;
};

int HMC5883L_Init(int file, unsigned char id, bool check);

int HMC5883L_Configure(int file, const struct HMC5883L *device);

int HMC5883L_SetContinuousMeasurement(int file);
int HMC5883L_SetIdle(int file);
int HMC5883L_SetSingleMeasurement(int file);

int HMC5883L_ReadData(int file, short *x, short *y, short *z);
int HMC5883L_ReadDataReady(int file, bool *ready);

#endif
