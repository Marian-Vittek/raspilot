#include <sys/ioctl.h>
#include <sys/stat.h>
#include <linux/i2c-dev.h>
#include <fcntl.h>

#include "hmc5883l.h"

#define READ(N) \
	if (read(file, buffer, N) != N) \
		return -1;

#define WRITE(N) \
	if (write(file, buffer, N) != N) \
		return -1;

int HMC5883L_Init(int file, unsigned char id, bool check)
{
    if (ioctl(file, I2C_SLAVE, id) < 0)
        return -1;

    if (check)
    {
        unsigned char buffer[3];

        buffer[0] = 0x0A;
        WRITE(1);

        READ(3);
        if (buffer[0] != 0x48 ||
            buffer[1] != 0x34 ||
            buffer[2] != 0x33)
            return -1;
    }

    return 0;
}

int HMC5883L_Configure(int file, const struct HMC5883L *device)
{
    unsigned char buffer[3];

    buffer[0] = 0x00;
    buffer[1] = (device->samples & 0x60) |
        (device->outputRate & 0x1C) |
        (device->measurementMode & 0x03);
    buffer[2] = (device->gain & 0xE0);
    WRITE(3);

    return 0;
}

int HMC5883L_SetContinuousMeasurement(int file)
{
    unsigned char buffer[2];

    buffer[0] = 0x02;
    buffer[1] = 0x00;
    WRITE(2);

    return 0;
}

int HMC5883L_SetIdle(int file)
{
    unsigned char buffer[2];

    buffer[0] = 0x02;
    buffer[1] = 0x03;
    WRITE(2);

    return 0;
}

int HMC5883L_SetSingleMeasurement(int file)
{
    unsigned char buffer[2];

    buffer[0] = 0x02;
    buffer[1] = 0x01;
    WRITE(2);

    return 0;
}

int HMC5883L_ReadData(int file, short *x, short *y, short *z)
{
    unsigned char buffer[6];

    buffer[0] = 0x03;
    WRITE(1);

    READ(6);
    *x = ((buffer[0]) << 8) | buffer[1];
    *y = ((buffer[4]) << 8) | buffer[5];
    *z = ((buffer[2]) << 8) | buffer[3];

    return 0;
}

int HMC5883L_ReadDataReady(int file, bool *ready)
{
    unsigned char buffer[1];

    buffer[0] = 0x09;
    WRITE(1);

    READ(1);
    *ready = (buffer[0] & 0x01);

    return 0;
}
