

all: motor-pigpio


motor-pigpio: motor-pigpio.c ../motor-common.c ../motor-common.h
	g++ -Wall -o motor-pigpio ../motor-common.c motor-pigpio.c -lpigpio

.PHONY: always

