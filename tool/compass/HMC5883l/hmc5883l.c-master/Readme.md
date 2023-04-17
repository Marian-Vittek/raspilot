# HMC5883L.c

Simple wrapper for HMC5883L compass on I2C.

Tested on a Raspberry Pi 2.

## Build

Build, and optionally install, with [scons](http://scons.org/).

	scons
	sudo scons install

## Usage

All functions take a file handle as first parameter (obtained with `open` and released with `close`), and return `0` when successful or `-1` on failure.

Example:

	static int file;

	static int closeAndExit(int code)
	{
		close(file);
		return code;
	}

	int main(void)
	{
		file = open("/dev/i2c-1", O_RDWR);
		if (file < 0)
			return -1;

		if (HMC5883L_Init(file, HMC5883L_ID, true))
			return closeAndExit(-1);

	    struct HMC5883L conf = {
	        .gain = HMC5883L_GAIN_1090,
	        .measurementMode = HMC5883L_MEASUREMENTMODE_NORMAL,
	        .outputRate = HMC5883L_OUTPUTRATE_30,
	        .samples = HMC5883L_SAMPLES_2,
	    };
	    if (HMC5883L_Configure(file, &conf))
			return closeAndExit(-1);

	    if (HMC5883L_SetContinuousMeasurement(file))
			return closeAndExit(-1);

		short x, y, z;
		if (HMC5883L_ReadData(file, &x, &y, &z))
			return closeAndExit(-1);

		return closeAndExit(0);
	}


### HMC5883L_Init(int file, unsigned char id, bool check)

`id` is the device I2C address. It is usually `HMC5883L_ID` (0x1E).

If `check` is true, the function reads the device identification registers and checks against the expected values. This inherently checks that the device and the I2C bus are working.

## License

Copyright 2016 Bloutiouf

[MIT License](https://opensource.org/licenses/MIT)
