#######################################################
# raspilot makefile

CFLAGS= -g -DEDEBUG -Wall -Wno-format-truncation -Wno-unused-variable -Wno-pointer-sign -Wno-unused-but-set-variable -L/usr/local/lib
LINKFLAGS=-lm

INCLUDES=common.h expmem.h sglib.h linmath.h linmath2.h
OBJS=main.o common.o expmem.o json.o baio.o config.o pilot.o mission.o

########################################################

#dev
#DRONE=d250
#DRONE=piw
#DRONE=pc
DRONE=piz2
###

#set IGNORE_BUILD_ERROR to true if you want to ignore build errors
ifeq ($(DRONE),pc)
IGNORE_BUILD_ERROR=true
else
IGNORE_BUILD_ERROR=false
endif


all: tools raspilot

raspilot: $(OBJS) $(INCLUDES)
	gcc -o raspilot $(OBJS) $(LINKFLAGS)

tools: always
	export RASPILOT_HOME=`pwd`/..; find ../tool -name "Makefile.raspilot" -execdir make -f Makefile.raspilot \; || $(IGNORE_BUILD_ERROR)

toolsclean: always
	export RASPILOT_HOME=`pwd`/..; find ../tool -name "Makefile.raspilot" -execdir make -f Makefile.raspilot clean \; || $(IGNORE_BUILD_ERROR)

viz: always
	make -C ../misc/pose-vizualization || $(IGNORE_BUILD_ERROR)

librealsensecompile:
	make -C ../tool/librealsense/build/examples/pose  || $(IGNORE_BUILD_ERROR)

motor: always
	make -C ../tool/motor/motor-pca9685 motor-pca9685  || $(IGNORE_BUILD_ERROR)
	make -C ../tool/motor/motor-pigpio-pwm motor-pigpio  || $(IGNORE_BUILD_ERROR)
	make -C ../tool/motor/motor-dshot motor-dshot  -f Makefile.raspilot || $(IGNORE_BUILD_ERROR)

#######################################################

stop: always
	sudo bash -c "killall -q raspilot; sleep 0.5; killall -9 -q raspilot" || true

prestartchecks: always
	@ ( if test "`sudo swapon -s`" != ""; then echo "Warning: Swap enabled! Do not fly with this system setting !"; fi ) 
	@ echo

prestart: prestartchecks stop
	sudo bash -c "sync; echo 3 > /proc/sys/vm/drop_caches"

start: prestart
	sudo bash -c "ulimit -c unlimited ; chrt -r 20 ./raspilot -p `echo $$SSH_CLIENT | awk '{ print $$1}'` -d 1"

startdebug: prestart
	sudo bash -c "ulimit -c unlimited ; chrt -r 20 ./raspilot -d 33 -p `echo $$SSH_CLIENT | awk '{ print $$1}'`"

starttolog: prestart
	ln -f -s ../log/log-`date "+%Y-%m-%d---%H:%M:%S.txt"` currentlog.txt
	echo "Start at `date`"
	sudo bash -c "ulimit -c unlimited ; chrt -r 20 ./raspilot -d 33 " -p `echo $$SSH_CLIENT | awk '{ print $$1}'` | tee currentlog.txt | grep -i "Error\|Warning\|waypoint\|waiting"
	echo "Exit at `date`"

replay: always
	scp $(DRONE):dev/raspilot/src/trajectory.dat .
	gnuplot ../misc/replay-trajectory/replay-trajectory.gnuplot

#######################################################

picopy: dist
	scp ../../raspilot.tgz $(DRONE):dev/
	ssh $(DRONE) "cd dev && tar xfz raspilot.tgz"

piall: picopy
	ssh $(DRONE) "make -C dev/raspilot/src all"

#######################################################


%.o : %.c $(INCLUDES)
	gcc -c $(CFLAGS) $< -o $@

clean: always
	rm -r -f *.o core raspilot a.out currentsubmsgs.txt

distclean: stop clean toolsclean
	rm -f *~
	find ../../raspilot -name "*~" | xargs rm -f
	rm -f ../misc/pose-vizualization/{*.o,raspicam-april}

dist:   distclean
	(cd ../.. && tar cfHz raspilot.tgz posix --exclude='raspilot/.git' --exclude='raspilot/misc' --exclude='raspilot/log' raspilot && cp raspilot.tgz raspilot-`date "+%Y-%m-%d"`.tgz)

#######################################################

test:always
	# ssh $(DRONE) "cd dev/raspilot/tool/raspicam-april && killall -9 raspicam-april ; ./raspicam-april -x 1 -t 1 -m 480 -n 360" | ../misc/pose-vizualization/viz
	# ssh $(DRONE) "cd dev/raspilot/tool/imu/mpu6050/rowberg && sudo killall -9 demo_dmp ; sudo ./demo_dmp" | ../misc/pose-vizualization/viz 
	# ssh $(DRONE) "cd dev/raspilot/tool/imu/mpu6050/magwick-fusion && sudo killall -9 fusion ; sudo ./fusion" | ../misc/pose-vizualization/viz 
	# ssh $(DRONE) "cd dev/raspilot/tool/librealsense/build/examples/pose && sudo killall -9 rs-pose ; sudo ./rs-pose" | ../misc/pose-vizualization/viz 
	# ssh $(DRONE) "cd dev/raspilot/tool/april-mipi/src && sudo killall -9 raspiapril ; ./raspiapril -r 10" | ../misc/pose-vizualization/viz 
	# ssh $(DRONE) "cd dev/raspilot/src ; make start" | ../misc/pose-vizualization/viz 
	# ssh $(DRONE) "cd dev/raspilot/src ; make start"
	# ssh piz2 "dev/raspilot/tool/april-mipi/src/raspiapril -m 3 -s 0.027 -l 3.45 -u 3.896 -v 2.453 -r 8 -x 2" | ../misc/pose-vizualization/viz 
	# ssh piz2 "dev/raspilot/tool/imu/mpu6050/fusion/fusion /dev/i2c-1"

.PHONY: always

