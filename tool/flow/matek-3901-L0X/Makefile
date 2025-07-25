SRC=$(RASPILOT_HOME)/src

all: matek-3901-l0x

matek-3901-l0x: matek-3901-l0x.c msp_protocol.h $(SRC)/common.h $(SRC)/raspilot_tlib.c
	gcc -o matek-3901-l0x -I$(SRC) matek-3901-l0x.c $(SRC)/raspilot_tlib.o -lm


