/*
 * @author 	Alexander Rüedlinger <a.rueedlinger@gmail.com>
 * @date 	26.02.2015
 *
 * A C driver for the sensor BMP180.
 *  
 */

/*
  Marian Vittek 2024: Modifications done to fit Raspilot and Raspberry Pi i2c.
 */


#ifndef __BMP180__
#define __BMP180__
#include <stdint.h>
#include "bmp180.h"
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <stdio.h>
#include <linux/i2c-dev.h>
#include <time.h>
#include <math.h>
// [MV] added
#include <signal.h>
#include "pi2c.h"
#endif


/* 
 * AC register
 */
#define BMP180_REG_AC1_H 0xAA
#define BMP180_REG_AC2_H 0xAC
#define BMP180_REG_AC3_H 0xAE
#define BMP180_REG_AC4_H 0xB0
#define BMP180_REG_AC5_H 0xB2
#define BMP180_REG_AC6_H 0xB4

/* 
 * B1 register
 */
#define BMP180_REG_B1_H 0xB6

/* 
 * B2 register
 */
#define BMP180_REG_B2_H 0xB8

/* 
 * MB register
 */
#define BMP180_REG_MB_H 0xBA

/* 
 * MC register
 */
#define BMP180_REG_MC_H 0xBC

/* 
 * MD register
 */
#define BMP180_REG_MD_H 0xBE

/* 
 * AC register
 */
#define BMP180_CTRL 0xF4

/* 
 * Temperature register
 */
#define BMP180_REG_TMP 0xF6

/* 
 * Pressure register
 */
#define BMP180_REG_PRE 0xF6

/*
 * Temperature read command
 */
#define BMP180_TMP_READ_CMD 0x2E

/*
 *  Waiting time in us for reading temperature values
 */
#define BMP180_TMP_READ_WAIT_US 5000

/*
 * Pressure oversampling modes
 */
#define BMP180_PRE_OSS0 0 // ultra low power
#define BMP180_PRE_OSS1 1 // standard
#define BMP180_PRE_OSS2 2 // high resolution
#define BMP180_PRE_OSS3 3 // ultra high resoultion

/*
 * Pressure read commands
 */
#define BMP180_PRE_OSS0_CMD 0x34
#define BMP180_PRE_OSS1_CMD 0x74
#define BMP180_PRE_OSS2_CMD 0xB4
#define BMP180_PRE_OSS3_CMD 0xF4

/* 
 * Waiting times in us for reading pressure values
 */
#define BMP180_PRE_OSS0_WAIT_US 5000
#define BMP180_PRE_OSS1_WAIT_US 8000
#define BMP180_PRE_OSS2_WAIT_US 14000
#define BMP180_PRE_OSS3_WAIT_US 26000

/*
 * Average sea-level pressure in hPa
 */
#define BMP180_SEA_LEVEL 1013.25


/*
 * Define debug function.
 */

//#define __BMP180_DEBUG__
#ifdef __BMP180_DEBUG__
#define DEBUG(...)	printf(__VA_ARGS__)
#else
#define DEBUG(...)
#endif


/*
 * Shortcut to cast void pointer to a bmp180_t pointer
 */
#define TO_BMP(x)	(bmp180_t*) x



/*
 * Basic structure for the bmp180 sensor
 */
typedef struct {
	/* file descriptor */
	int file;

	/* i2c device address */
	int address;
	
	/* BMP180 oversampling mode */
	int oss;
	
	/* i2c device file path */
	char *i2c_device;
	
	/* Eprom values */
	int32_t ac1;
	int32_t ac2;
	int32_t ac3;
	int32_t ac4;
	int32_t ac5;
	int32_t ac6;
	int32_t b1;
	int32_t b2;
	int32_t mb;
	int32_t mc;
	int32_t md;
} bmp180_t;


/*
 * Lookup table for BMP180 register addresses
 */
int32_t bmp180_register_table[11][2] = {
		{BMP180_REG_AC1_H, 1},
		{BMP180_REG_AC2_H, 1},
		{BMP180_REG_AC3_H, 1},
		{BMP180_REG_AC4_H, 0},
		{BMP180_REG_AC5_H, 0},
		{BMP180_REG_AC6_H, 0},
		{BMP180_REG_B1_H, 1},
		{BMP180_REG_B2_H, 1},
		{BMP180_REG_MB_H, 1},
		{BMP180_REG_MC_H, 1},
		{BMP180_REG_MD_H, 1}
};


/*
 * Prototypes for helper functions
 */
void bmp180_read_eprom_reg(void *_bmp, int32_t *_data, uint8_t reg, int32_t sign);
void bmp180_read_eprom(void *_bmp);
int32_t bmp180_read_raw_pressure(void *_bmp, uint8_t oss);
int32_t bmp180_read_raw_temperature(void *_bmp);
void bmp180_init_error_cleanup(void *_bmp);


/*
 * Implemetation of the helper functions
 */


/*
 * Frees allocated memory in the init function.
 * 
 * @param bmp180 sensor
 */
void bmp180_init_error_cleanup(void *_bmp) {
	bmp180_t* bmp = TO_BMP(_bmp);
	
	if(bmp->i2c_device != NULL) {
		free(bmp->i2c_device);
		bmp->i2c_device = NULL;
	}
	
	free(bmp);
	bmp = NULL;
}



/*
 * Reads a single calibration coefficient from the BMP180 eprom.
 * 
 * @param bmp180 sensor
 */
void bmp180_read_eprom_reg(void *_bmp, int32_t *_store, uint8_t reg, int32_t sign) {
	bmp180_t *bmp = TO_BMP(_bmp);
	
	// int32_t data = i2c_smbus_read_word_data(bmp->file, reg) & 0xFFFF;
	uint16_t data;
	pi2cReadBytes(bmp->file, reg, 2, (uint8_t*)&data);
	
	// i2c_smbus_read_word_data assumes little endian 
	// but ARM uses big endian. Thus the ordering of the bytes is reversed.
	// data = 	 0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15   bit position
	//          |      lsb      |          msb        |  
	
	//                 msb           +     lsb
	*_store = ((data << 8) & 0xFF00) + (data >> 8);
	
	if(sign && (*_store > 32767)) {
		*_store -= 65536;
	}
}


/*
 * Reads the eprom of this BMP180 sensor.
 * 
 * @param bmp180 sensor
 */
void bmp180_read_eprom(void *_bmp) {
	bmp180_t *bmp = TO_BMP(_bmp);	
	
	int32_t *bmp180_register_addr[11] = {
		&bmp->ac1, &bmp->ac2, &bmp->ac3, &bmp->ac4, &bmp->ac5, &bmp->ac6,
		&bmp->b1, &bmp->b2, &bmp->mb, &bmp->mc, &bmp->md
	};
	
	uint8_t sign, reg;
	int32_t *data;
	int i;
	for(i = 0; i < 11; i++) {
		reg = (uint8_t) bmp180_register_table[i][0];
		sign = (uint8_t) bmp180_register_table[i][1];
		data = bmp180_register_addr[i];
		bmp180_read_eprom_reg(_bmp, data, reg, sign);
	}
}


/*
 * Returns the raw measured temperature value of this BMP180 sensor.
 * 
 * @param bmp180 sensor
 */
int32_t bmp180_read_raw_temperature(void *_bmp) {
	bmp180_t* bmp = TO_BMP(_bmp);
	//i2c_smbus_write_byte_data(bmp->file, BMP180_CTRL, BMP180_TMP_READ_CMD);
	pi2cWriteByteToReg(bmp->file, BMP180_CTRL, BMP180_TMP_READ_CMD);

	usleep(BMP180_TMP_READ_WAIT_US);
	//int32_t data = i2c_smbus_read_word_data(bmp->file, BMP180_REG_TMP) & 0xFFFF;
	
	uint16_t data;
	pi2cReadBytes(bmp->file, BMP180_REG_TMP, 2, (uint8_t*)&data);
	
	data = ((data << 8) & 0xFF00) + (data >> 8);
	
	return data;
}


/*
 * Returns the raw measured pressure value of this BMP180 sensor.
 * 
 * @param bmp180 sensor
 */
int32_t bmp180_read_raw_pressure(void *_bmp, uint8_t oss) {
	bmp180_t* bmp = TO_BMP(_bmp);
	uint16_t wait;
	uint8_t cmd;
	
	switch(oss) {
		case BMP180_PRE_OSS1:
			wait = BMP180_PRE_OSS1_WAIT_US; cmd = BMP180_PRE_OSS1_CMD;
			break;
		
		case BMP180_PRE_OSS2:
			wait = BMP180_PRE_OSS2_WAIT_US; cmd = BMP180_PRE_OSS2_CMD;
			break;
		
		case BMP180_PRE_OSS3:
			wait = BMP180_PRE_OSS3_WAIT_US; cmd = BMP180_PRE_OSS3_CMD;
			break;
		
		case BMP180_PRE_OSS0:
		default:
			wait = BMP180_PRE_OSS0_WAIT_US; cmd = BMP180_PRE_OSS0_CMD;
			break;
	}
	
	// i2c_smbus_write_byte_data(bmp->file, BMP180_CTRL, cmd);
	pi2cWriteByteToReg(bmp->file, BMP180_CTRL, cmd);
	
	usleep(wait);
	
	int32_t msb, lsb, xlsb, data;
	//msb = i2c_smbus_read_byte_data(bmp->file, BMP180_REG_PRE) & 0xFF;
	//lsb = i2c_smbus_read_byte_data(bmp->file, BMP180_REG_PRE+1) & 0xFF;
	//xlsb = i2c_smbus_read_byte_data(bmp->file, BMP180_REG_PRE+2) & 0xFF;
	uint8_t dd;
	pi2cReadBytes(bmp->file, BMP180_REG_PRE, 1, &dd); msb = dd;
	pi2cReadBytes(bmp->file, BMP180_REG_PRE+1, 1, &dd); lsb = dd;
	pi2cReadBytes(bmp->file, BMP180_REG_PRE+1, 1, &dd); xlsb = dd;
	
	data = ((msb << 16)  + (lsb << 8)  + xlsb) >> (8 - bmp->oss);
	
	return data;
}

/*
 * Implementation of the interface functions
 */

/**
 * Dumps the eprom values of this BMP180 sensor.
 * 
 * @param bmp180 sensor
 * @param bmp180 eprom struct
 */
void bmp180_dump_eprom(void *_bmp, bmp180_eprom_t *eprom) {
	bmp180_t *bmp = TO_BMP(_bmp);
	eprom->ac1 = bmp->ac1;
	eprom->ac2 = bmp->ac2;
	eprom->ac3 = bmp->ac3;
	eprom->ac4 = bmp->ac4;
	eprom->ac5 = bmp->ac5;
	eprom->ac6 = bmp->ac6;
	eprom->b1 = bmp->b1;
	eprom->b2 = bmp->b2;
	eprom->mb = bmp->mb;
	eprom->mc = bmp->mc;
	eprom->md = bmp->md;
}


/**
 * Creates a BMP180 sensor object.
 *
 * @param i2c device address
 * @param i2c device file path
 * @return bmp180 sensor
 */
void *bmp180_init(int address, const char* i2c_device_filepath) {
	DEBUG("device: init using address %#x and i2cbus %s\n", address, i2c_device_filepath);
	
	// setup BMP180
	void *_bmp = malloc(sizeof(bmp180_t));
	if(_bmp == NULL)  {
		DEBUG("error: malloc returns NULL pointer\n");
		return NULL;
	}

	bmp180_t *bmp = TO_BMP(_bmp);
	bmp->address = address;

	// setup i2c device path
	bmp->i2c_device = (char*) malloc(strlen(i2c_device_filepath) * sizeof(char));
	if(bmp->i2c_device == NULL) {
		DEBUG("error: malloc returns NULL pointer!\n");
		bmp180_init_error_cleanup(bmp);
		return NULL;
	}

	// copy string
	strcpy(bmp->i2c_device, i2c_device_filepath);
	
	// open i2c device
	int file;
	file = pi2cOpen(bmp->i2c_device,  bmp->address);
	bmp->file = file;

	// setup i2c device
	bmp180_read_eprom(_bmp);
	bmp->oss = 0;
	
	DEBUG("device: open ok\n");

	return _bmp;
}


/**
 * Closes a BMP180 object.
 * 
 * @param bmp180 sensor
 */
void bmp180_close(void *_bmp) {
	if(_bmp == NULL) {
		return;
	}
	
	DEBUG("close bmp180 device\n");
	bmp180_t *bmp = TO_BMP(_bmp);
	
	if(close(bmp->file) < 0) {
		DEBUG("error: %s close() failed\n", bmp->i2c_device);
	}
	
	free(bmp->i2c_device); // free string
	bmp->i2c_device = NULL;
	free(bmp); // free bmp structure
	_bmp = NULL;
} 


/**
 * Returns the measured temperature in celsius.
 * 
 * @param bmp180 sensor
 * @return temperature
 */
static float bmp180_raw_temp_to_temp(bmp180_t * bmp, long UT) {
    float T;
    long B5, X2, X1;

    X1 = ((UT - bmp->ac6) * bmp->ac5) >> 15;
    X2 = (bmp->mc << 11) / (X1 + bmp->md);
    B5 = X1 + X2;
    T = ((B5 + 8) >> 4) / 10.0;
	

    return(T);
}

float bmp180_temperature(void *_bmp) {
	bmp180_t* bmp = TO_BMP(_bmp);
	long UT, X1, X2, B5;
	float T;
	
	UT = bmp180_read_raw_temperature(_bmp);
	
	DEBUG("UT=%lu\n",UT);
	T = bmp180_raw_temp_to_temp(bmp, UT);
	return T;
}


long bmp180_raw_pressure_and_temp_to_pressure(void *_bmp, long UT, long UP) {
	bmp180_t* bmp = TO_BMP(_bmp);

	long B6, B5, X1, X2, X3, B3, p;
	unsigned long B4, B7;

	X1 = ((UT - bmp->ac6) * bmp->ac5) >> 15;
	X2 = (bmp->mc << 11) / (X1 + bmp->md);
	
	B5 = X1 + X2;
	
	B6 = B5 - 4000;
	
	X1 = (bmp->b2 * (B6 * B6) >> 12) >> 11;
	X2 = (bmp->ac2 * B6) >> 11;
	X3 = X1 + X2;
	
	B3 = ((((bmp->ac1 * 4) + X3) << bmp->oss) + 2) / 4;
	X1 = (bmp->ac3 * B6) >> 13;
	X2 = (bmp->b1 * ((B6 * B6) >> 12)) >> 16;
	X3 = ((X1 + X2) + 2) >> 2;
	
	
	B4 = bmp->ac4 * (unsigned long)(X3 + 32768) >> 15;
	B7 = ((unsigned long) UP - B3) * (50000 >> bmp->oss);
	
	if(B7 < 0x80000000) {
		p = (B7 * 2) / B4;
	} else {
		p = (B7 / B4) * 2;
	}
	
	X1 = (p >> 8) * (p >> 8);
	X1 = (X1 * 3038) >> 16;
	X2 = (-7357 * p) >> 16;
	p = p + ((X1 + X2 + 3791) >> 4);
	
	return p;
}

/**
 * Returns the measured pressure in pascal.
 * 
 * @param bmp180 sensor
 * @return pressure
 */
long bmp180_pressure(void *_bmp) {
	bmp180_t* bmp = TO_BMP(_bmp);
	long UT, UP, p;

	// [MV] Hmm. Maybe we shall ignore the temperature, it is not the outside temp, but the one
	// of the device heated by CPU, Wifi and ESC.
	UT = bmp180_read_raw_temperature(_bmp);
	UP = bmp180_read_raw_pressure(_bmp, bmp->oss);

	p = bmp180_raw_pressure_and_temp_to_pressure(bmp, UT, UP);
	
	return p;
}


/**
 * Returns altitude in meters based on the measured pressure 
 * and temperature of this sensor.
 * 
 * @param bmp180 sensor
 * @return altitude
 */

static float bmp180_pressure_to_altitude(float p) {
    float alt;
    alt = 44330 * (1 - pow(( (p/100) / BMP180_SEA_LEVEL),1/5.255));

    return(alt);
}

float bmp180_altitude(void *_bmp) {
	float p, alt;
	p = bmp180_pressure(_bmp);
	alt = bmp180_pressure_to_altitude(p);	
	return alt;
}


/**
 * Sets the oversampling setting for this sensor.
 * 
 * @param bmp180 sensor
 * @param oversampling mode
 */
void bmp180_set_oss(void *_bmp, int oss) {
	bmp180_t* bmp = TO_BMP(_bmp);
	bmp->oss = oss;
}


/////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////
// [MV] Raspilot interface

static bmp180_t *bmp;
static bmp180_eprom_t eprom;

static inline double doubleGetTime() {
  struct timespec tt;
  clock_gettime(CLOCK_REALTIME, &tt);
  return(tt.tv_sec + tt.tv_nsec/1000000000.0);
}

static void taskStop(int signum) {
    printf("Info: %s exiting\n", __FILE__); fflush(stdout);
    exit(0);
}

static void deviceInit(char *path, int devAddr) {
    bmp = (bmp180_t *) bmp180_init(devAddr, path);
	
    bmp180_dump_eprom(bmp, &eprom);
    bmp180_set_oss(bmp, BMP180_PRE_OSS3);
}

static void deviceGetData(double *data) {
    long UT, UUT, UP, p;

    UT = UUT = bmp180_read_raw_temperature(bmp);
    UP = bmp180_read_raw_pressure(bmp, bmp->oss);
    
    // UUT = 31000;
    p = bmp180_raw_pressure_and_temp_to_pressure(bmp, UUT, UP);
    
    // printf("UT == %10d; UP == %10d, p == %10d, pp == %10d\n", UT, UP, p, bmp180_raw_pressure_and_temp_to_pressure(bmp, 31000, 75000));
    data[0] = p;
    data[1] = bmp180_pressure_to_altitude(p);
    data[2] = bmp180_raw_temp_to_temp(bmp, UT);
}


int main(int argc, char **argv) {
    double 	t0, t1, samplePeriod;
    int		i, usleepTime;

    int		optSharedI2cFlag;
    char	*optI2cPath;
    double	optRate;
    double	data[3];

#ifdef SHM
    struct raspilotInputBuffer 	*shmbuf;
    struct raspilotInputBuffer 	*shmbuf2;
#endif

    optSharedI2cFlag = 0;
    optI2cPath = (char*)"/dev/i2c-1";
    optRate = 100.0; 			// default rate 1kHz
    
    for(i=1; i<argc; i++) {
	if (strcmp(argv[i], "-s") == 0) {
	    // share i2c. Do not reset shared semaphores
	    optSharedI2cFlag = 1;
	} else if (strcmp(argv[i], "-r") == 0) {
	    // refresh rate in Hz
	    i++;
	    if (i<argc) optRate = strtod(argv[i], NULL);
	} else {
	    optI2cPath = argv[i];
	}
    }	

    if (optSharedI2cFlag) pi2cInit(optI2cPath, 1);

#ifdef SHM
    shmbuf = raspilotShmConnect((char *)"raspilot.gyro-mpu6050-magwick-shm.rpy");
    if (shmbuf == NULL) exit(-1);
#endif
    
    signal(SIGINT, taskStop);

    deviceInit(optI2cPath, 0x77);

    usleepTime = 1000000 / optRate;
    // It's around 20ms to get data from bmp
    usleepTime -= 20000;
    if (usleepTime < 0) usleepTime = 0;

    usleep(usleepTime);

    t0 = doubleGetTime();
    i = 0;
    for(;;) {
	
	deviceGetData(data);
	t1 = doubleGetTime();

#ifdef SHM
	shmbuf->confidence = 1.0;
	if (raspilotShmPush(shmbuf, t1, data, 3) != 0) taskStop(0);
	// printf("debug: pressure, altitude, temperature: %g: %9.7f %9.7f %9.7f\n", t1, data[0], data[1], data[2]);
#else
	printf("alt %9.7f\n", data[1]);
	printf("temp %9.7f\n", data[2]);
	fflush(stdout);
#endif	
	
	t0 = t1;
	if (samplePeriod > 1.0/optRate && usleepTime > 100) usleepTime-=100;
	else if (samplePeriod < 1.0/optRate) usleepTime+=100;
	usleep(usleepTime);

	// if (i++ % 1000 == 0) printf("debug usleepTime == %d\n", usleepTime);
    }

    taskStop(0);
}


