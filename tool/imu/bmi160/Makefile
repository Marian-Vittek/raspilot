LIBI2C=$(RASPILOT_HOME)/lib/pi2c
LIBBMI=$(RASPILOT_HOME)/lib/bmi160/DFRobot
LIBFUSION=$(RASPILOT_HOME)/lib/FussionMagwick/Fusion

all: bmi160 bmi160-shm


bmi160: bmi160.c $(LIBBMI)/DFRobot_BMI160.cpp $(LIBBMI)/DFRobot_BMI160.h $(LIBI2C)/pi2c.c $(LIBI2C)/pi2c.h
	g++ -o bmi160 -g bmi160.c -I$(LIBFUSION) -I$(LIBBMI) -I$(LIBI2C) $(LIBBMI)/DFRobot_BMI160.cpp $(LIBI2C)/pi2c.c -L$(LIBFUSION) -pthread -lFusion -lm -lrt

bmi160-shm: bmi160.c $(LIBBMI)/DFRobot_BMI160.cpp $(LIBBMI)/DFRobot_BMI160.h $(LIBI2C)/pi2c.c $(LIBI2C)/pi2c.h $(RASPILOT_HOME)/src/raspilotshm.h
	g++ -o bmi160-shm -g bmi160.c -DSHM -I$(RASPILOT_HOME)/src -I$(LIBFUSION) -I$(LIBBMI) -I$(LIBI2C) $(LIBBMI)/DFRobot_BMI160.cpp $(LIBI2C)/pi2c.c -L$(LIBFUSION) -lFusion -pthread -lm -lrt 

clean: always
	rm -f bmi160 bmi160-shm *~

.PHONY: always

