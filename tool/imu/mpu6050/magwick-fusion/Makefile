FUSION_DIR=../../../Fusion/Fusion
PI2C_DIR=../../../pi2c

all: fusion


fusion: fusion-simple.c ../MPU6050.cpp ../I2Cdev.cpp $(PI2C_DIR)/pi2c.c
	g++ -o fusion -g fusion-simple.c -I$(FUSION_DIR) -I.. -I$(PI2C_DIR) ../MPU6050.cpp ../I2Cdev.cpp $(PI2C_DIR)/pi2c.c -L$(FUSION_DIR) -lFusion  -lm -pthread

clean:
	rm -f a.out fusion *~
