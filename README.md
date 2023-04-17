

Raspilot is a drone flight controller and autopilot for Raspberry
Pi. It does not require an additional computer on board, the Raspberry
Pi pinout is connected directly to the ESCs. 


### Why to do it?

This project began as a professional assignment. The aim was to
develop a flight controller and autopilot where it is easy to connect
various hardware components such as gyroscopes, GPS and distance
sensors and perform series of tests. Another goal was potentially
unlimited precision (sub centimeter) of flight.

Unfortunately, the company that commissioned the software cancelled
the project before it had reached its first milestone. By that time
the most of the autopilot code had been written. I published the code
and continue working on it on my own. Here is a [video of one of the
first succesfull flight](https://www.youtube.com/watch?v=454NIqCr8b4)



### Why raspberry pi?


Raspberry Pi fits the purpose of the project. It is powerful enough to
support sensors that require a Linux operating system such as Intel
Intellisense devices. The computer is light enough to be mounted on
small drones. Raspbery Pi natively supports double precision
floating-point arithmetic, which we use for all our calculations.

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
the drone. Linux provides a file system with buffered I/O, allowing us
to store and edit our configurations, logs, videos, etc.

Though booting and shutting down may be annoying we are professional
developers and we are used to it. Also, our experiments show that
Linux user space processes can be perfectly timed with a resolution
better than 1ms. This is largely sufficient for the autopilot, where
the main loop is executed every 10ms. Things that require better
timing (such as the dshot protocol) are implemented using busy wait
loops.




### What does raspilot provide?

Raspilot is a small software at the moment. It implements the
necessary to allow a drone to fly and carry out its mission.  The
autopilot provides basic stabilisation, it reads sensors and sends
commands to the motors.  The overall architecture is similar to that
of the PX4. Specialised routines that manipulate the sensors are
running in separate processes and are connected to the autopilot via
Linux pipes. Software runs asynchronously and the input is read as it
comes.  The autopilot itself runs an infinite loop at an adjustable
frequency, usually 100Hz. Software that physically controls motors (or
ESCs) runs in a separate process that is also connected through Linux
pipes as well. It makes it easy to integrate specific ESC protocols or
specific motor hardware.


From the user point of view, the whole autopilot behaves like a
library. The main program initialises the autopilot and then executes
a mission for the drone to follow.  The mission is a C function coded
by the user. It basically does what it wants to do and calls autopilot
functions like "goto_waypoint(X,Y,Z,Yaw)", which means that the drone
will go to the point X,Y,Z and approach it with the yaw
orientation. When the waypoint is reached, the function returns and
the mission can continue. Such an architecture gives the user (who
must be a C programmer) full control over all flight variables and
configurations to perform any sophisticated computations he may wish
during the flight.

The physics that Raspilot uses to stabilise drones is very
rudimentary. Only inertia rules are taken into account. Changes in
rotation and speed of movement are controlled by PID controllers. This
model is able to stabilise the drone very well at 100Hz (main loop
frequency). We were able to fly drones with frequencies between 20Hz
and 400Hz. At lower frequencies the drone is unstable, higher
frequencies do not allow to save full log of the flight.



### Hardware

For the moment Raspilot supports T265 Intel intellisense positioning
and orientation sensor; MPU-6050 family of gyroscopes; HC-SR04
distance sensor; any NMEA GPS sensor and HMC5883 compass.

