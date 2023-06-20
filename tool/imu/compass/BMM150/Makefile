LIBI2C=$(RASPILOT_HOME)/lib/pi2c

all: compass calibration

compass: compass.cpp bmm150.cpp bmm150.h bmm150_defs.h $(LIBI2C)/pi2c.c $(LIBI2C)/pi2c.h
	g++ -o compass -I$(LIBI2C) compass.cpp bmm150.cpp $(LIBI2C)/pi2c.c -pthread

calibration: calibration.cpp bmm150.cpp bmm150.h bmm150_defs.h $(LIBI2C)/pi2c.c $(LIBI2C)/pi2c.h
	g++ -o calibration -I$(LIBI2C) calibration.cpp bmm150.cpp $(LIBI2C)/pi2c.c -pthread

clean: always
	rm -f *~ compass calibration
	
.PHONY: always

