
all: test


test: test.c motor-dshot.c 
	gcc -O2 -S -o motor-dshot.s motor-dshot.c
	gcc -O2 -Wall -o test test.c motor-dshot.c

clean: always
	rm -f test motor-dshot.s

run: always
	sudo chrt -f 99 ./test


.PHONY: always



