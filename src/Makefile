#######################################################
# raspilot makefile

CFLAGS= -g -DEDEBUG -Wall -Wno-format-truncation -Wno-unused-variable -Wno-pointer-sign -Wno-unused-but-set-variable -I../lib/pi2c -I../lib -L/usr/local/lib
LINKFLAGS= -pthread -lm -lrt 

INCLUDES=common.h raspilotshm.h expmem.h sglib.h linmath.h linmath2.h ../lib/pi2c/pi2c.h
OBJS=main.o common.o expmem.o json.o baio.o config.o device.o pilot.o mission.o mavlink.o ../lib/pi2c/pi2c.c

########################################################

#DRONE=d250
#DRONE=p2
#DRONE=192.168.0.220
#DRONE=pifly
#DRONE=pc
#DRONE=pi5
DRONE=openhd@pc:2233

#SET IGNORE_BUILD_ERROR to true if you want to ignore build errors
ifeq ($(DRONE),pc)
IGNORE_BUILD_ERROR=true
else
IGNORE_BUILD_ERROR=false
endif

all: libs tools raspilot

raspilot: $(OBJS) $(INCLUDES)
	gcc -o raspilot $(OBJS) $(LINKFLAGS)

# generic raspilot src compile rule
%.o : %.c $(INCLUDES)
	gcc -c $(CFLAGS) $< -o $@

# go through '../tool' and make every directory containing Makefile.raspilot
tools: libs
	export RASPILOT_HOME=`dirname $$PWD`; find ../tool -name "Makefile.raspilot" -execdir make -f Makefile.raspilot \; || $(IGNORE_BUILD_ERROR)

# go through '../lib' and make every directory containing Makefile.raspilot
libs: always
	# for libraries like FussionMagwick, first run cmake if not yet done 
	export RASPILOT_HOME=`dirname $$PWD`; find ../lib -name "CMakeLists.txt" -execdir bash -c "test -f Makefile || cmake ." \; || $(IGNORE_BUILD_ERROR)
	export RASPILOT_HOME=`dirname $$PWD`; find ../lib -name "Makefile.raspilot" -execdir make -f Makefile.raspilot \; || $(IGNORE_BUILD_ERROR)
	
# go recursively through '../tool' and '../lib' and clean every directory containing Makefile.raspilot
dirsclean: always
	export RASPILOT_HOME=`dirname $$PWD`; find ../tool -name "Makefile.raspilot" -execdir make -f Makefile.raspilot clean \; || $(IGNORE_BUILD_ERROR)
	export RASPILOT_HOME=`dirname $$PWD`; find ../lib -name "Makefile.raspilot" -execdir make -f Makefile.raspilot clean \; || $(IGNORE_BUILD_ERROR)

viz: always
	make -C ../misc/pose-vizualization || $(IGNORE_BUILD_ERROR)

librealsensecompile:
	make -C ../tool/librealsense/build/examples/pose  || $(IGNORE_BUILD_ERROR)

#######################################################

stop: always
	sudo bash -c "killall -q raspilot; sleep 0.5; killall -9 -q raspilot" || true

prestartchecks: always
	@ ( if test "`sudo swapon -s`" != ""; then echo "Warning: Swap enabled! Do not fly with this system setting !"; fi ) 
	@ echo

prestart: prestartchecks stop
	sudo bash -c "sync; echo 3 > /proc/sys/vm/drop_caches"

start: prestart
	sudo bash -c "ulimit -c unlimited ; chrt -r 20 taskset -c 0 ./raspilot  -d 1 -ll 6 "

autostart: prestart
	sudo bash -c "ulimit -c unlimited ; chrt -r 20 taskset -c 0 ./raspilot  -auto -d 0 -ll 6 "
# it is supposed that the autostart goal is called from crontab a few seconds after reboot, supposed crontab line is:
# @reboot         nohup /home/openhd/raspilot/src/autostart.sh &

starttolog: prestart
	sudo bash -c "ulimit -c unlimited ; chrt -r 20 taskset -c 0 ./raspilot -d 6 -ll 33 "
	# Copy the log on the ssh server
	# cat currentlog.txt | gzip | ssh `echo $$SSH_CLIENT | awk '{ print $$1}'` "cat > dev/raspilot/log/`readlink currentlog.txt`.gz"
	# scp `readlink currentlog.txt` `echo vittek@$$SSH_CLIENT | awk '{ print $$1}'`:dev/raspilot/log/

startdebug: prestart
	sudo bash -c "ulimit -c unlimited ; chrt -r 20 taskset -c 0 ./raspilot -d 66 -ll 66"

replay: always
	scp scp://$(DRONE)/~/dev/raspilot/src/trajectory.dat .
	gnuplot ../misc/replay-trajectory/replay-trajectory.gnuplot

#######################################################

picopy: dist
	scp ../../raspilot.tgz scp://$(DRONE)/~/dev/
	ssh ssh://$(DRONE) "cd dev && tar xfz raspilot.tgz"

piall: picopy
	ssh ssh://$(DRONE) "make -C dev/raspilot/src all"

#######################################################


clean: dirsclean
	rm -r -f *.o core raspilot a.out currentsubmsgs.txt

distclean: stop clean
	rm -f *~
	find ../../raspilot -name "*~" | xargs rm -f
	rm -f ../misc/pose-vizualization/{*.o,raspicam-april}

dist:   distclean
	(cd ../.. && tar cfHz raspilot.tgz posix --exclude='raspilot/.git' --exclude='raspilot/log/*' raspilot && cp raspilot.tgz raspilot-`date "+%Y-%m-%d"`.tgz)

#######################################################

test:always
	# ssh ssh://$(DRONE) "cd dev/raspilot/tool/raspicam-april && killall -9 raspicam-april ; ./raspicam-april -x 1 -t 1 -m 480 -n 360" | ../misc/pose-vizualization/viz
	# ssh ssh://$(DRONE) "cd dev/raspilot/tool/imu/mpu6050/rowberg && sudo killall -9 demo_dmp ; sudo ./demo_dmp" | ../misc/pose-vizualization/viz 
	# ssh ssh://$(DRONE) "cd dev/raspilot/tool/imu/mpu6050/magwick-fusion && sudo killall -9 fusion ; sudo ./fusion" | ../misc/pose-vizualization/viz 
	# ssh ssh://$(DRONE) "cd dev/raspilot/tool/librealsense/build/examples/pose && sudo killall -9 rs-pose ; sudo ./rs-pose" | ../misc/pose-vizualization/viz 
	# ssh ssh://$(DRONE) "cd dev/raspilot/tool/april-mipi/src && sudo killall -9 raspiapril ; ./raspiapril -r 10" | ../misc/pose-vizualization/viz 
	# ssh ssh://$(DRONE) "cd dev/raspilot/src ; make start" | ../misc/pose-vizualization/viz 
	# ssh ssh://$(DRONE) "cd dev/raspilot/src ; make start"
	# ssh piz2 "dev/raspilot/tool/april-mipi/src/raspiapril -m 3 -s 0.027 -l 3.45 -u 3.896 -v 2.453 -r 8 -x 2" | ../misc/pose-vizualization/viz 
	# ssh piz2 "dev/raspilot/tool/imu/mpu6050/fusion/fusion /dev/i2c-1"

.PHONY: always

