
all: hw-pwm


hw-pwm: hw-pwm.c 
	g++ -Wall -o hw-pwm hw-pwm.c -l bcm2835

clean: always
	rm -f *~ hw-pwm

.PHONY: always

