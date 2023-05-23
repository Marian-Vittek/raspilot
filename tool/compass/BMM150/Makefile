
all: compass calibration

compass: compass.cpp bmm150.cpp bmm150.h bmm150_defs.h pi2c.c pi2c.h
	g++ -o compass compass.cpp bmm150.cpp pi2c.c

calibration: calibration.cpp bmm150.cpp bmm150.h bmm150_defs.h pi2c.c pi2c.h
	g++ -o calibration calibration.cpp bmm150.cpp pi2c.c