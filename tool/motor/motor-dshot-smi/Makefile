
all: test


test: test.c motor-dshot-smi.c rpi_dma_utils.c rpi_dma_utils.h
	gcc -Wall -O2 -o test test.c motor-dshot-smi.c rpi_dma_utils.c

test2: test2.c motor-dshot-smi.c rpi_dma_utils.c rpi_dma_utils.h
	gcc -Wall -O2 -o test2 test2.c motor-dshot-smi.c rpi_dma_utils.c

clean: always
	rm -f test test2

run: always
	sudo ./test 16 19 20 21


.PHONY: always



