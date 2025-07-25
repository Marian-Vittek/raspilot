SRC=$(RASPILOT_HOME)/src
LIBMPU=$(RASPILOT_HOME)/lib/mpu6050
LIBFUSION=$(RASPILOT_HOME)/lib/FussionMagwick/Fusion

# TODO:finish others!
all: mpu6050 hmc5883l qmc5883l bmp180


mpu6050: mpu6050.c $(LIBMPU)/MPU6050.cpp $(LIBMPU)/I2Cdev.cpp $(SRC)/common.h $(SRC)/raspilot_tlib.c
	g++ -o mpu6050 -g mpu6050.c -I$(LIBFUSION) -I$(LIBMPU) -I$(SRC) $(LIBMPU)/MPU6050.cpp $(LIBMPU)/I2Cdev.cpp $(SRC)/raspilot_tlib.o++ -L$(LIBFUSION) -lFusion -pthread -lm -lrt

hmc5883l: hmc5883l.c $(LIBMPU)/MPU6050.cpp $(LIBMPU)/I2Cdev.cpp $(SRC)/common.h $(SRC)/raspilot_tlib.c 
	g++ -o hmc5883l -g hmc5883l.c -I$(LIBMPU) -I$(SRC) $(LIBMPU)/MPU6050.cpp $(LIBMPU)/I2Cdev.cpp $(SRC)/raspilot_tlib.o++ -lm -pthread

qmc5883l: qmc5883l.c $(LIBMPU)/MPU6050.cpp $(LIBMPU)/I2Cdev.cpp $(SRC)/common.h $(SRC)/raspilot_tlib.c 
	g++ -o qmc5883l -g qmc5883l.c -I$(LIBMPU) -I$(SRC) $(LIBMPU)/MPU6050.cpp $(LIBMPU)/I2Cdev.cpp $(SRC)/raspilot_tlib.o++ -lm -pthread

bmp180: bmp180.c $(LIBMPU)/I2Cdev.cpp $(SRC)/common.h $(SRC)/raspilot_tlib.c
	gcc -o bmp180 -g bmp180.c -I$(SRC) $(SRC)/raspilot_tlib.o -lm -pthread

clean: always
	rm -f mpu6050 hmc5883l qmc5883l bmp180 *~

.PHONY: always

