
all: demo_dmp

LIBI2C=$(RASPILOT_HOME)/lib/pi2c
LIBMPU=$(RASPILOT_HOME)/lib/mpu6050

HDRS = $(LIBMPU)/helper_3dmath.h $(LIBMPU)/I2Cdev.h $(LIBMPU)/MPU6050_6Axis_MotionApps20.h $(LIBMPU)/MPU6050.h $(LIBI2C)/pi2c.h
SRCS = demo_dmp.cpp $(LIBMPU)/I2Cdev.cpp $(LIBMPU)/MPU6050.cpp $(LIBI2C)/pi2c.c

# Set DMP FIFO rate to 20Hz to avoid overflows on 3d demo.  See comments in
# MPU6050_6Axis_MotionApps20.h for details.
# I got approximately the following:
# DMP_FIFO_RATE=3 gives 50Hz
# DMP_FIFO_RATE=2 gives 75Hz
# DMP_FIFO_RATE=1 gives 100Hz
# DMP_FIFO_RATE=0 gives 200Hz? // giving completely random values
# If you change DMP_FIFO_RATE, do full recompile with "make clean" !!!!
CXXFLAGS = -O2 -DDMP_FIFO_RATE=3 -DMPU6050_INCLUDE_DMP_MOTIONAPPS20=1 -Wall -g -I$(LIBMPU) -I$(LIBI2C)

demo_dmp: $(HDRS) $(SRCS)
	g++ -o demo_dmp $(SRCS) $(CXXFLAGS) -lm -pthread

clean:
	rm -f demo_dmp
