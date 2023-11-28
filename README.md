

Drone flight controller and autopilot for Raspberry Pi (Zero 2). It
does not require any additional flight controller, the Raspberry
Pi pinout is connected directly to ESCs and sensors. It has been
developed from the scratch, the code for stabilisation and reaching
waypoints is original.  Here is a
[video](https://www.youtube.com/watch?v=454NIqCr8b4) of one of the
first succesfull flights.

### Why to do it?

This project began as a professional assignment. The aim was to
develop a flight controller and autopilot where it is easy to connect
various hardware components such as experimental gyroscopes, distance
sensors, positioning modules etc. and perform series of tests. Another
goal was potentially unlimited precision (sub centimeter) of flight.

Unfortunately, the company that commissioned the software cancelled
the project before it had reached its first milestone. By that time
the most of the autopilot code had been written. I published it and
maintain it on my own now.



### Why raspberry pi?


Raspberry Pi fits the purpose of the project. It is powerful enough to
support sensors that require a Linux operating system such as Intel
Intellisense devices. The computer is light enough to be mounted on
small drones. Raspbery Pi natively supports double precision
floating-point arithmetic which we use for all our calculations.

Of course, there are obvious drawbacks to using Raspberry Pi. After
connecting the battery, you have to wait until Linux boots. You have
to shut down Linux before you disconnect the battery. And, of course,
Linux is not a real-time operating system, and precise timing can be a
problem.


However, none of these points outweigh the advantages we have
gained. Raspberry Pi offers plenty of processing power and a standard
development and debugging environment. Connected via wifi we can edit,
compile, run and debug the code directly on the drone. No need to
flash the firmware every time we change the code. We can fly with new
versions of our software again and again without physically touching
the drone. We can put any debugging output to logs with no effort. We
can connect any new sensor through a pipe immediately after having
compiled its factory provided demo software.




### What does raspilot provide?

Raspilot is a small software at the moment. It implements the
necessary to allow a drone to fly and carry out its mission.  The
autopilot provides basic stabilisation, it reads sensors and sends
commands to the motors.  The overall architecture is similar to that
of the PX4. Specialised routines that manipulate the sensors and
motors are running in separate processes and are connected to the
autopilot via Linux pipes and/or shared memory. Software runs
asynchronously and the input is read as it comes.  The autopilot
itself runs an infinite loop at an adjustable frequency. Frequency is
configurable and ranges between 50Hz and 5kHz. Less stable designs
with a centre of gravity above propellers require higher frequency.
Most of our drones (small and large) were operated at around 200 Hz.
Software that physically controls ESCs runs in a separate process
too. This architecture makes it easy to integrate specific sensors,
ESC protocols or specific motor hardware.


From the user point of view, the whole autopilot behaves like a
library. The main program initialises the autopilot and then executes
a mission for the drone to follow.  The mission is a C function coded
by the user. It basically calls autopilot functions like
"goto_waypoint(X,Y,Z,Yaw)", which means that the drone will go to the
point X,Y,Z and approach it with the yaw orientation. When the
waypoint is reached, the function returns and the mission can
continue. Such an architecture gives the user (who must be a C
programmer) full control over all flight variables and configurations
to perform any sophisticated computations he may wish during the
flight.

The physics that Raspilot uses to stabilise drones is very
rudimentary. Only inertia rules are taken into account. Changes in
rotation and speed of movement are controlled by PID
controllers. 



### Hardware

At the moment Raspilot supports T265 Intel intellisense positioning
and orientation sensor; MPU-6050 family of gyroscopes; BMI160
gyroscope; HC-SR04 distance sensor; any NMEA GPS sensor and others. It
implements PWM and DSHOT 150 protocols to control motor ESCs.  To see
all supported hardware go through subdirectories under 'tool'
directory. If your hardware is not there, it is quite easy to add it.
All you need to do is to hack a demo example that comes with the
sensor and make it to print measurements to the standard output.

### Getting Started

You need to have at least a basic understanding of Linux and C
programming in order to use Raspilot. If you have the courage to try
it then:

1.) Clone Raspilot to your Raspberry Pi

``` cd your_destination_directory
    git clone https://github.com/Marian-Vittek/raspilot.git
```

2.) Create/Edit configuration file for your drone in the directory
"cfg". There are a few working configurations which can be used as
templates. Then create a symbolic link to your configuration file in
the "src" directory. The name of the link shall be "config.json".

```
    cd raspilot/src
    ln -s -f ../cfg/raspilot-myconfiguration.json config.json
```

3.) Possibly edit the file "mission.c" and change the body of the
function "mission" to execute your mission. In the beginning you
probably just want to perfom test of motors rotating one motor after
another. The code will be:

```
  void mission() {
    missionMotorTest(-1);
  }
```

There are a few precoded missions in mission.c. You can inspect and
use them.  For example, to perform a simple square mission making the
drone to fly in a square of 20cm at the altitude 10cm use the following
code:

```
  void mission() {
    raspilotPreLaunchSequence();
    missionSquare(0.20, 0.10, 1.0, 1);
  }
```


4.) Compile Raspilot

```
    make all
```

5.) Launch the autopilot with

```
   make starttolog
```

Once the mission is completed, the log from the last flight can be
seen in the file currentlog.txt. All logs from previous flights can be
found in ../log directory. If you do not need log files, you can
launch autopilot with:

```
  make start
```


Good luck and do not hesitate to contact me.




