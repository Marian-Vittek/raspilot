SENSOR=bmp180

all: altimeter test

test:
	#gcc -Wall -c $(SENSOR).c -o $(SENSOR).o -lm
	gcc -Wall $(SENSOR).c smbus.c test.c -o test -lm

altimeter: $(SENSOR).c smbus.c altimeter.c smbus.h $(SENSOR).h
	gcc -O2 -Wall $(SENSOR).c smbus.c altimeter.c -o altimeter -lm 


clean:
	rm *.o > /dev/null 2>&1 &
