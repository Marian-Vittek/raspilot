LIBI2C=$(RASPILOT_HOME)/lib/pi2c
LIBMPU=$(RASPILOT_HOME)/lib/mpu6050
LIBFUSION=$(RASPILOT_HOME)/lib/FussionMagwick/Fusion

all: mpu6050 mpu6050-shm


mpu6050: mpu6050.c $(LIBMPU)/MPU6050.cpp $(LIBMPU)/I2Cdev.cpp $(LIBI2C)/pi2c.c $(LIBI2C)/pi2c.h
	g++ -o mpu6050 -g mpu6050.c -I$(LIBFUSION) -I$(LIBMPU) -I$(LIBI2C) $(LIBMPU)/MPU6050.cpp $(LIBMPU)/I2Cdev.cpp $(LIBI2C)/pi2c.c -L$(LIBFUSION)  -pthread -lFusion  -lm -lrt


mpu6050-shm: mpu6050.c $(LIBMPU)/MPU6050.cpp $(LIBMPU)/I2Cdev.cpp $(LIBI2C)/pi2c.c $(LIBI2C)/pi2c.h $(RASPILOT_HOME)/src/raspilotshm.h
	g++ -o mpu6050-shm -g mpu6050.c -DSHM -I$(RASPILOT_HOME)/src -I$(LIBFUSION) -I$(LIBMPU) -I$(LIBI2C) $(LIBMPU)/MPU6050.cpp $(LIBMPU)/I2Cdev.cpp $(LIBI2C)/pi2c.c -L$(LIBFUSION)  -pthread -lFusion  -lm -lrt

clean: always
	rm -f a.out mpu6050 mpu6050-shm *~

.PHONY: always
