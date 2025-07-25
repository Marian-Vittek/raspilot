SRC=$(RASPILOT_HOME)/src

all: sonar-gy-us42

sonar-gy-us42: sonar-gy-us42.c $(SRC)/common.h $(SRC)/raspilot_tlib.c
	g++ -O2 -o sonar-gy-us42 sonar-gy-us42.c $(SRC)/raspilot_tlib.o++ -I$(SRC) -pthread -lm -lrt

run:
	sudo ./sonar-gy-us42

clean: always
	rm -f sonar-gy-us42

.PHONY: always
