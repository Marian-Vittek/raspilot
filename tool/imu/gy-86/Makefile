FUSION_DIR=../../../Fusion/Fusion
PI2C_DIR=../../../pi2c

all: gy86


gy86: gy86.c ../MPU6050.cpp ../I2Cdev.cpp $(PI2C_DIR)/pi2c.c
	g++ -o gy86 -g gy86.c -I$(FUSION_DIR) -I.. -I$(PI2C_DIR) ../MPU6050.cpp ../I2Cdev.cpp $(PI2C_DIR)/pi2c.c -L$(FUSION_DIR) -lFusion  -lm -pthread

clean:
	rm -f gy86 *~
