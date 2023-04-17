#######################################################
# raspilot makefile

CFLAGS= -g -DEDEBUG -Wall -Wno-format-truncation -Wno-unused-variable -Wno-pointer-sign -Wno-unused-but-set-variable -L/usr/local/lib
LINKFLAGS=-lm

INCLUDES=common.h expmem.h sglib.h linmath.h linmath2.h
OBJS=main.o common.o expmem.o json.o baio.o config.o pilot.o mission.o

########################################################

#dev
HOST=d250
#HOST=piw
#HOST=pc
#HOST=piz2
###

#set IGNORE_BUILD_ERROR to true if you want to ignore build errors
ifeq ($(HOST),pc)
IGNORE_BUILD_ERROR=true
else
IGNORE_BUILD_ERROR=false
endif


all: raspilot mpucompile motorcompile altimetercompile sonarcompile

viz: always
	make -C ../tool/pose-vizualization || $(IGNORE_BUILD_ERROR)

april-mipi:
	make -C ../tool/april-mipi/src || $(IGNORE_BUILD_ERROR)

raspiaprilcompile:
	make -C ../tool/raspicam-april || $(IGNORE_BUILD_ERROR)

librealsensecompile:
	make -C ../tool/librealsense/build/examples/pose  || $(IGNORE_BUILD_ERROR)

mpucompile: always
	make -C ../tool/MPU6050-Pi demo_dmp  || $(IGNORE_BUILD_ERROR)

altimetercompile: always
	make -C ../tool/altimeter/baro/bmp180/bmp180-master/src altimeter  || $(IGNORE_BUILD_ERROR)

sonarcompile: always
	make -C ../tool/sonar-hcsr04  || $(IGNORE_BUILD_ERROR)

motorcompile: always
	make -C ../tool/motors/motor-pca9685 motor-pca9685  || $(IGNORE_BUILD_ERROR)
	make -C ../tool/motors/motor-pigpio motor-pigpio  || $(IGNORE_BUILD_ERROR)
	make -C ../tool/motors/motor-dshot motor-dshot  -f Makefile.raspilot || $(IGNORE_BUILD_ERROR)

raspilot: $(OBJS) $(INCLUDES)
	gcc -o raspilot $(OBJS) $(LINKFLAGS)

picopy: dist
	scp ../../raspilot.tgz $(HOST):dev/
	ssh $(HOST) "cd dev && tar xfz raspilot.tgz"

# ordering of motors shall be
#
#   3       0
#    \     /
#     --^--
#     |   |
#     -----
#    /     \
#   2       1
#
# in my case it gives gpins 8 25 24 23

stop: always
	sudo bash -c "killall -q raspilot; sleep 0.5; killall -9 -q raspilot" || true

prestartchecks: always
	@ ( if test "`sudo swapon -s`" != ""; then echo "Warning: Swap enabled! Do not fly with this system setting !"; fi ) 
	@ echo

prestart: prestartchecks stop
	# clear caches, to avoid freezing
	sudo bash -c "sync; echo 3 > /proc/sys/vm/drop_caches"

start: prestart
	sudo bash -c "ulimit -c unlimited ; chrt -r 20 ./raspilot -p `echo $$SSH_CLIENT | awk '{ print $$1}'` -d 1"
	echo; echo; cat statistics.txt

startdebug: prestart
	sudo bash -c "ulimit -c unlimited ; chrt -r 20 ./raspilot -d 33 -p `echo $$SSH_CLIENT | awk '{ print $$1}'`"
	echo; echo; cat statistics.txt

starttolog: prestart
	ln -f -s ../log/log-`date "+%Y-%m-%d---%H:%M:%S.txt"` currentlog.txt
	echo "Start at `date`"
	sudo bash -c "ulimit -c unlimited ; chrt -r 20 ./raspilot -d 33 " -p `echo $$SSH_CLIENT | awk '{ print $$1}'` | tee currentlog.txt | grep -i "Error\|Warning\|waypoint"
	echo; echo; cat statistics.txt
	echo "Exit at `date`"
	
replay: always
	scp $(HOST):dev/raspilot/src/trajectory.dat .
	gnuplot ../tool/replay-trajectory/replay-trajectory.gnuplot

piall: picopy
	ssh $(HOST) "make -C dev/raspilot/src all"

picompile: picopy
	ssh $(HOST) "make -C dev/raspilot/src raspilot"

pistart: always
	ssh $(HOST) "make -C dev/raspilot/src start"

pistartdebug: always
	ssh $(HOST) "make -C dev/raspilot/src startdebug"

pistarttolog: always
	ssh $(HOST) "make -C dev/raspilot/src starttolog"

clean: always
	rm -r -f *.o core raspilot a.out currentsubmsgs.txt

#######################################################


%.o : %.c $(INCLUDES)
	gcc -c $(CFLAGS) $< -o $@

distclean: stop clean
	rm -f *~
	find ../../raspilot -name "*~" | xargs rm -f
	rm -f ../tool/motor/*.o ../tool/motor/motor ../tool/motor/calibrator
	rm -f ../tool/raspicam-april/{*.o,raspicam-april}
	rm -f ../tool/pose-vizualization/{*.o,raspicam-april}
	rm -f ../tool/MPU6050-Pi/*.o ../tool/MPU6050-Pi/demo_dmp ../tool/MPU6050-Pi/demo_raw 

dist:   distclean
	(cd ../.. && tar cfHz raspilot.tgz posix --exclude='raspilot/.git' --exclude='raspilot/misc' --exclude='raspilot/log' raspilot && cp raspilot.tgz raspilot-`date "+%Y-%m-%d"`.tgz)

#######################################################

test:always
	# ssh $(HOST) "cd dev/raspilot/tool/raspicam-april && killall -9 raspicam-april ; ./raspicam-april -x 1 -t 1 -m 480 -n 360" | ../tool/pose-vizualization/viz
	# ssh $(HOST) "cd dev/raspilot/tool/MPU6050-Pi/ && sudo killall -9 demo_dmp ; sudo ./demo_dmp" | ../tool/pose-vizualization/viz 
	# ssh $(HOST) "cd dev/raspilot/tool/librealsense/build/examples/pose && sudo killall -9 rs-pose ; sudo ./rs-pose" | ../tool/pose-vizualization/viz 
	# ssh $(HOST) "cd dev/raspilot/tool/april-mipi/src && sudo killall -9 raspiapril ; ./raspiapril -r 10" | ../tool/pose-vizualization/viz 
	# ssh $(HOST) "cd dev/raspilot/src ; make start" | ../tool/pose-vizualization/viz 
	# ssh $(HOST) "cd dev/raspilot/src ; make start"
	# ssh piz2 "dev/raspilot/tool/april-mipi/src/raspiapril -m 3 -s 0.027 -l 3.45 -u 3.896 -v 2.453 -r 8 -x 2" | ../tool/pose-vizualization/viz 


.PHONY: always

