LIBI2C=$(RASPILOT_HOME)/lib/pi2c
LIBMPU=$(RASPILOT_HOME)/lib/mpu6050
LIBFUSION=$(RASPILOT_HOME)/lib/FussionMagwick/Fusion

all: fusion


fusion: fusion-simple.c $(LIBMPU)/MPU6050.cpp $(LIBMPU)/I2Cdev.cpp $(LIBI2C)/pi2c.c $(LIBI2C)/pi2c.h
	g++ -o fusion -g fusion-simple.c -I$(LIBFUSION) -I$(LIBMPU) -I$(LIBI2C) $(LIBMPU)/MPU6050.cpp $(LIBMPU)/I2Cdev.cpp $(LIBI2C)/pi2c.c -L$(LIBFUSION) -lFusion  -lm -pthread

clean: always
	rm -f a.out fusion *~

.PHONY: always
