
all: test test2


test: test.c motor-dshot.c 
	gcc -O2 -S -o motor-dshot.s motor-dshot.c
	gcc -O2 -Wall -o test test.c motor-dshot.c

test2: test2.c motor-dshot.c 
	gcc -O2 -Wall -o test2 test2.c motor-dshot.c

clean: always
	rm -f test test2 motor-dshot.s

run: always
	sudo chrt -f 99 ./test


.PHONY: always



