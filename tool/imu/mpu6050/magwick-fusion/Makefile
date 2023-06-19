FUSION_DIR=$(RASPILOT_HOME)/tool/Fusion/Fusion
PI2C_DIR=$(RASPILOT_HOME)/tool/pi2c

all: fusion


fusion: fusion-simple.c ../MPU6050.cpp ../I2Cdev.cpp $(PI2C_DIR)/pi2c.c $(PI2C_DIR)/pi2c.h
	g++ -o fusion -g fusion-simple.c -I$(FUSION_DIR) -I.. -I$(PI2C_DIR) ../MPU6050.cpp ../I2Cdev.cpp $(PI2C_DIR)/pi2c.c -L$(FUSION_DIR) -lFusion  -lm -pthread

clean: always
	rm -f a.out fusion *~

.PHONY: always
