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


all: raspilot motor altimeter sonar mpurowberg mpu-magwick-fusion mpu-magwick-rowberg

viz: always
	make -C ../misc/pose-vizualization || $(IGNORE_BUILD_ERROR)

april-mipi:
	make -C ../tool/april-mipi/src || $(IGNORE_BUILD_ERROR)

raspiaprilcompile:
	make -C ../tool/raspicam-april || $(IGNORE_BUILD_ERROR)

librealsensecompile:
	make -C ../tool/librealsense/build/examples/pose  || $(IGNORE_BUILD_ERROR)

mpurowberg: always
	make -C ../tool/gyro/mpu6050/rowberg demo_dmp  || $(IGNORE_BUILD_ERROR)

mpu-magwick-fusion: always
	#(cd ../tool/Fusion && cmake .)
	#make -C ../tool/Fusion/Fusion  || $(IGNORE_BUILD_ERROR)
	make -C ../tool/gyro/mpu6050/magwick-fusion all  || $(IGNORE_BUILD_ERROR)

mpu-magwick-rowberg: always
	#(cd ../tool/Fusion && cmake .)
	#make -C ../tool/Fusion/Fusion  || $(IGNORE_BUILD_ERROR)
	make -C ../tool/gyro/mpu6050/magwick+rowberg all  || $(IGNORE_BUILD_ERROR)

altimeter: always
	make -C ../tool/altimeter/baro/bmp180/bmp180-master/src altimeter  || $(IGNORE_BUILD_ERROR)

sonar: always
	make -C ../tool/sonar/sonar-hcsr04  || $(IGNORE_BUILD_ERROR)

motor: always
	make -C ../tool/motor/motor-pca9685 motor-pca9685  || $(IGNORE_BUILD_ERROR)
	make -C ../tool/motor/motor-pigpio-pwm motor-pigpio  || $(IGNORE_BUILD_ERROR)
	make -C ../tool/motor/motor-dshot motor-dshot  -f Makefile.raspilot || $(IGNORE_BUILD_ERROR)

raspilot: $(OBJS) $(INCLUDES)
	gcc -o raspilot $(OBJS) $(LINKFLAGS)

picopy: dist
	scp ../../raspilot.tgz $(DRONE):dev/
	ssh $(DRONE) "cd dev && tar xfz raspilot.tgz"

# ordering of motors shall be:
#
# front side of the drone
#
#   3       0
#    \     /
#     --^--
#     |   |
#     -----
#    /     \
#   2       1
#

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

piall: picopy
	ssh $(DRONE) "make -C dev/raspilot/src all"

picompile: picopy
	ssh $(DRONE) "make -C dev/raspilot/src raspilot"

pistart: always
	ssh $(DRONE) "make -C dev/raspilot/src start"

pistartdebug: always
	ssh $(DRONE) "make -C dev/raspilot/src startdebug"

pistarttolog: always
	ssh $(DRONE) "make -C dev/raspilot/src starttolog"

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
	rm -f ../misc/pose-vizualization/{*.o,raspicam-april}
	rm -f ../tool/gyro/MPU6050-Pi/*.o ../tool/gyro/MPU6050-Pi/demo_dmp ../tool/gyro/MPU6050-Pi/demo_raw 

dist:   distclean
	(cd ../.. && tar cfHz raspilot.tgz posix --exclude='raspilot/.git' --exclude='raspilot/misc' --exclude='raspilot/log' raspilot && cp raspilot.tgz raspilot-`date "+%Y-%m-%d"`.tgz)

#######################################################

test:always
	# ssh $(DRONE) "cd dev/raspilot/tool/raspicam-april && killall -9 raspicam-april ; ./raspicam-april -x 1 -t 1 -m 480 -n 360" | ../misc/pose-vizualization/viz
	# ssh $(DRONE) "cd dev/raspilot/tool/gyro/mpu6050/rowberg && sudo killall -9 demo_dmp ; sudo ./demo_dmp" | ../misc/pose-vizualization/viz 
	# ssh $(DRONE) "cd dev/raspilot/tool/gyro/mpu6050/magwick-fusion && sudo killall -9 fusion ; sudo ./fusion" | ../misc/pose-vizualization/viz 
	# ssh $(DRONE) "cd dev/raspilot/tool/librealsense/build/examples/pose && sudo killall -9 rs-pose ; sudo ./rs-pose" | ../misc/pose-vizualization/viz 
	# ssh $(DRONE) "cd dev/raspilot/tool/april-mipi/src && sudo killall -9 raspiapril ; ./raspiapril -r 10" | ../misc/pose-vizualization/viz 
	# ssh $(DRONE) "cd dev/raspilot/src ; make start" | ../misc/pose-vizualization/viz 
	# ssh $(DRONE) "cd dev/raspilot/src ; make start"
	# ssh piz2 "dev/raspilot/tool/april-mipi/src/raspiapril -m 3 -s 0.027 -l 3.45 -u 3.896 -v 2.453 -r 8 -x 2" | ../misc/pose-vizualization/viz 
	# ssh piz2 "dev/raspilot/tool/gyro/mpu6050/fusion/fusion /dev/i2c-1"

.PHONY: always

