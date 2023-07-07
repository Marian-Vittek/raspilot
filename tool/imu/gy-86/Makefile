LIBI2C=$(RASPILOT_HOME)/lib/pi2c
LIBMPU=$(RASPILOT_HOME)/lib/mpu6050
LIBMS5611=$(RASPILOT_HOME)/lib/ms5611
LIBFUSION=$(RASPILOT_HOME)/lib/FussionMagwick/Fusion

all: mpu6050+hmc5883l ms5611 hmc5883l


mpu6050+hmc5883l: mpu6050+hmc5883l.c $(LIBMPU)/MPU6050.cpp $(LIBMPU)/I2Cdev.cpp $(LIBI2C)/pi2c.c $(LIBI2C)/pi2c.h
	g++ -o mpu6050+hmc5883l -g mpu6050+hmc5883l.c -I$(LIBFUSION) -I$(LIBMPU) -I$(LIBI2C) $(LIBMPU)/MPU6050.cpp $(LIBMPU)/I2Cdev.cpp $(LIBI2C)/pi2c.c -L$(LIBFUSION) -lFusion  -lm -pthread

ms5611: ms5611.c $(LIBI2C)/pi2c.c $(LIBI2C)/pi2c.h  $(LIBMS5611)/MS5611.cpp $(LIBMS5611)/MS5611.h
	g++ -o ms5611 -g ms5611.c -I$(LIBMS5611) -I$(LIBI2C) $(LIBMS5611)/MS5611.cpp $(LIBI2C)/pi2c.c -lm -pthread

hmc5883l: hmc5883l.c $(LIBI2C)/pi2c.c $(LIBMPU)/MPU6050.cpp $(LIBMPU)/I2Cdev.cpp $(LIBI2C)/pi2c.h 
	g++ -o hmc5883l -g hmc5883l.c -I$(LIBMS5611) -I$(LIBMPU) -I$(LIBI2C) $(LIBMPU)/MPU6050.cpp $(LIBMPU)/I2Cdev.cpp $(LIBI2C)/pi2c.c -lm -pthread

clean: always
	rm -f mpu6050+hmc5883l ms5611 hmc5883l *~

.PHONY: always

