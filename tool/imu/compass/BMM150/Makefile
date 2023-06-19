PI2C_DIR=$(RASPILOT_HOME)/tool/pi2c

all: compass calibration

compass: compass.cpp bmm150.cpp bmm150.h bmm150_defs.h $(PI2C_DIR)/pi2c.c $(PI2C_DIR)/pi2c.h
	g++ -o compass compass.cpp bmm150.cpp $(PI2C_DIR)/pi2c.c 

calibration: calibration.cpp bmm150.cpp bmm150.h bmm150_defs.h $(PI2C_DIR)/pi2c.c $(PI2C_DIR)/pi2c.h
	g++ -o calibration calibration.cpp bmm150.cpp $(PI2C_DIR)/pi2c.c

clean: always
	rm -f *~ compass calibration
	
.PHONY: always

