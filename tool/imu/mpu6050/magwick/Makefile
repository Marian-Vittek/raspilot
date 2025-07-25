SRC=$(RASPILOT_HOME)/src
LIBMPU=$(RASPILOT_HOME)/lib/mpu6050
LIBFUSION=$(RASPILOT_HOME)/lib/FussionMagwick/Fusion

all: mpu6050


mpu6050: mpu6050.c $(LIBMPU)/MPU6050.cpp $(LIBMPU)/I2Cdev.cpp $(SRC)/common.h $(SRC)/raspilot_tlib.c
	g++ -o mpu6050 -g mpu6050.c -I$(LIBFUSION) -I$(LIBMPU) -I$(SRC) $(LIBMPU)/MPU6050.cpp $(LIBMPU)/I2Cdev.cpp $(SRC)/raspilot_tlib.o++ -L$(LIBFUSION)  -pthread -lFusion  -lm -lrt


clean: always
	rm -f a.out mpu6050 mpu6050-shm *~

.PHONY: always
