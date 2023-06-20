LIBI2C=$(RASPILOT_HOME)/lib/pi2c


all: compass calibrate

calibrate: calibrate.cpp LIS3MDL.cpp LIS3MDL.h $(LIBI2C)/pi2c.c $(LIBI2C)/pi2c.h
	   g++ -o calibrate calibrate.cpp LIS3MDL.cpp -I$(LIBI2C) $(LIBI2C)/pi2c.c -pthread

compass: compass.cpp LIS3MDL.cpp LIS3MDL.h $(LIBI2C)/pi2c.c $(LIBI2C)/pi2c.h
	   g++ -o compass compass.cpp LIS3MDL.cpp -I$(LIBI2C) $(LIBI2C)/pi2c.c -pthread

clean: always
	rm -f compass calibrate 

.PHONY: always
