LIBI2C=$(RASPILOT_HOME)/lib/pi2c
LIBMPU=$(RASPILOT_HOME)/lib/mpu6050
LIBFUSION=$(RASPILOT_HOME)/lib/FussionMagwick/Fusion

# TODO:finish others!
all: mpu6050 mpu6050-shm hmc5883l qmc5883l bmp180


mpu6050: mpu6050.c $(LIBMPU)/MPU6050.cpp $(LIBMPU)/I2Cdev.cpp $(LIBI2C)/pi2c.c $(LIBI2C)/pi2c.h
	g++ -o mpu6050 -g mpu6050.c -I$(LIBFUSION) -I$(LIBMPU) -I$(LIBI2C) $(LIBMPU)/MPU6050.cpp $(LIBMPU)/I2Cdev.cpp $(LIBI2C)/pi2c.c -L$(LIBFUSION) -lFusion -pthread -lm -lrt

mpu6050-shm: mpu6050.c $(LIBMPU)/MPU6050.cpp $(LIBMPU)/I2Cdev.cpp $(LIBI2C)/pi2c.c $(LIBI2C)/pi2c.h
	g++ -o mpu6050-shm -g mpu6050.c -DSHM -I$(RASPILOT_HOME)/src -I$(LIBFUSION) -I$(LIBMPU) -I$(LIBI2C) $(LIBMPU)/MPU6050.cpp $(LIBMPU)/I2Cdev.cpp $(LIBI2C)/pi2c.c -L$(LIBFUSION) -lFusion -pthread -lm -lrt

hmc5883l: hmc5883l.c $(LIBI2C)/pi2c.c $(LIBMPU)/MPU6050.cpp $(LIBMPU)/I2Cdev.cpp $(LIBI2C)/pi2c.h 
	g++ -o hmc5883l -g hmc5883l.c -I$(LIBMPU) -I$(LIBI2C) $(LIBMPU)/MPU6050.cpp $(LIBMPU)/I2Cdev.cpp $(LIBI2C)/pi2c.c -lm -pthread

qmc5883l: qmc5883l.c $(LIBI2C)/pi2c.c $(LIBMPU)/MPU6050.cpp $(LIBMPU)/I2Cdev.cpp $(LIBI2C)/pi2c.h 
	g++ -o qmc5883l -g qmc5883l.c -I$(LIBMPU) -I$(LIBI2C) $(LIBMPU)/MPU6050.cpp $(LIBMPU)/I2Cdev.cpp $(LIBI2C)/pi2c.c -lm -pthread

bmp180: bmp180.c $(LIBI2C)/pi2c.c $(LIBMPU)/I2Cdev.cpp $(LIBI2C)/pi2c.h 
	gcc -o bmp180 -g bmp180.c -I$(LIBI2C) $(LIBI2C)/pi2c.c -lm -pthread

clean: always
	rm -f mpu6050+hmc5883l hmc5883l qmc5883l bmp180 *~

.PHONY: always

