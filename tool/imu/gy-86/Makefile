FUSION_DIR=$(RASPILOT_HOME)/tool/Fusion/Fusion
PI2C_DIR=$(RASPILOT_HOME)/tool/pi2c

all: gy86


gy86: gy86.c ../mpu6050/MPU6050.cpp ../mpu6050/I2Cdev.cpp $(PI2C_DIR)/pi2c.c
	g++ -o gy86 -g gy86.c -I$(FUSION_DIR) -I.. -I$(PI2C_DIR) ../mpu6050/MPU6050.cpp ../mpu6050/I2Cdev.cpp $(PI2C_DIR)/pi2c.c -L$(FUSION_DIR) -lFusion  -lm -pthread

clean: always
	rm -f gy86 *~

.PHONY: always

