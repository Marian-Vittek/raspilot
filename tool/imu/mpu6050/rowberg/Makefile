# You need to "sudo apt-get install libgtkmm-3.0-dev" to build the demo_3d binary

all: demo_dmp

HDRS = ../helper_3dmath.h ../I2Cdev.h MPU6050_6Axis_MotionApps20.h ../MPU6050.h
SRCS = demo_dmp.cpp ../I2Cdev.cpp ../MPU6050.cpp ../../../pi2c/pi2c.c

# Set DMP FIFO rate to 20Hz to avoid overflows on 3d demo.  See comments in
# MPU6050_6Axis_MotionApps20.h for details.
# I got approximately the following:
# DMP_FIFO_RATE=3 gives 50Hz
# DMP_FIFO_RATE=2 gives 75Hz
# DMP_FIFO_RATE=1 gives 100Hz
# DMP_FIFO_RATE=0 gives 200Hz? // giving completely random values
# If you change DMP_FIFO_RATE, do full recompile with "make clean" !!!!
CXXFLAGS = -DDMP_FIFO_RATE=1 -DMPU6050_INCLUDE_DMP_MOTIONAPPS20=1 -Wall -g -I.. -I../../../pi2c

demo_dmp: $(HDRS) $(SRCS)
	g++ -o demo_dmp $(SRCS) $(CXXFLAGS) -lm -pthread

clean:
	rm -f demo_dmp
