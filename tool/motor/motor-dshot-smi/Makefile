
all: test


test: test.c motor-dshot-smi.c rpi_dma_utils.c rpi_dma_utils.h
	gcc -Wall -O2 -o test test.c motor-dshot-smi.c rpi_dma_utils.c

clean: always
	rm -f test

run: always
	sudo ./test


.PHONY: always



