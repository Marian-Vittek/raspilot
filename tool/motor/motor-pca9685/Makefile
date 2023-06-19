
PI2C_DIR=../../pi2c

all: motor-pca9685


motor-pca9685: motor-pca9685.cpp ../motor-common.c ../motor-common.h Adafruit_PWMServoDriver.h Adafruit_PWMServoDriver.cpp $(PI2C_DIR)/pi2c.c
	g++ -Wall -I $(PI2C_DIR) -o motor-pca9685 ../motor-common.c motor-pca9685.cpp Adafruit_PWMServoDriver.cpp $(PI2C_DIR)/pi2c.c -pthread

.PHONY: always



