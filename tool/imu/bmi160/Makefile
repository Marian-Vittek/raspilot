SRC=$(RASPILOT_HOME)/src
LIBBMI=$(RASPILOT_HOME)/lib/bmi160/DFRobot
LIBFUSION=$(RASPILOT_HOME)/lib/FussionMagwick/Fusion

all: bmi160


bmi160: bmi160.c $(LIBBMI)/DFRobot_BMI160.cpp $(LIBBMI)/DFRobot_BMI160.h $(SRC)/common.h $(SRC)/raspilot_tlib.c
	g++ -o bmi160 -g bmi160.c -I$(LIBFUSION) -I$(LIBBMI) -I$(SRC) $(LIBBMI)/DFRobot_BMI160.cpp $(SRC)/raspilot_tlib.o++ -L$(LIBFUSION) -pthread -lFusion -lm -lrt

clean: always
	rm -f bmi160 *~

.PHONY: always

