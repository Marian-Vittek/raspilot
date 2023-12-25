//////////////////////////////////////////////////////////////////////////////////////////
// This file contaions raspilot missions
// For your own mission, edit this file, in particular the
// function 'mission' at the bottom of the file.

#include "common.h"

void missionLoiter(double loiterTime, double altitude) {

    raspilotLaunch(uu->config.drone_min_altitude + 0.02);

    lprintf(1, "%s: mission: Begin loiter(%g, %g)!\n", PPREFIX(), loiterTime, altitude);
    
    // loiter 10 seconds
    raspilotWaypointSet(0, 0, altitude, 0);
    raspilotBusyWait(loiterTime);

    lprintf(1, "%s: mission: End   loiter(%g, %g)!\n", PPREFIX(), loiterTime, altitude);

    // Hmm. if we are too low, it may hit the ground and fail roll,pitch goals
    // raspilotGotoWaypoint(0, 0, 0.10, 0);
    raspilotLand(0, 0);    
}

void missionTestYawLoiter(double altitude) {

    raspilotLaunch(uu->config.drone_min_altitude + 0.02);

    lprintf(1, "%s: mission: Begin testYawLoiter(%g)!\n", PPREFIX(), altitude);
    
    raspilotWaypointSet(0, 0, altitude, 0);
    raspilotBusyWait(3.0);

    lprintf(1, "%s: mission: Turn left!\n", PPREFIX(), altitude);
    raspilotWaypointSet(0, 0, altitude, 0.4);
    raspilotBusyWait(3.0);

    lprintf(1, "%s: mission: Turn right!\n", PPREFIX(), altitude);
    raspilotWaypointSet(0, 0, altitude, -0.4);
    raspilotBusyWait(3.0);

    lprintf(1, "%s: mission: Turn back!\n", PPREFIX(), altitude);
    raspilotWaypointSet(0, 0, altitude, 0);
    raspilotBusyWait(3.0);

    lprintf(1, "%s: mission: End   testYawLoiter(%g)!\n", PPREFIX(), altitude);
    raspilotLand(0, 0);    
}

void missionSquare(double squareSize, double altitude, double waitOnWaypoint, int repeat) {
    double 	x,y;
    int		i;
    
    raspilotLaunch(uu->config.drone_min_altitude + 0.02);

    lprintf(1, "%s: mission: Begin square(%g, %g)!\n", PPREFIX(), squareSize, altitude);

    // go up
    raspilotGotoWaypoint(0, 0, altitude, 0);

    x = y = squareSize/2;

    for(i=0; i<repeat; i++) {
	raspilotGotoWaypoint(x, y, altitude, 0);
	raspilotBusyWait(waitOnWaypoint);
	raspilotGotoWaypoint(x, -y, altitude, 0);
	raspilotBusyWait(waitOnWaypoint);
	raspilotGotoWaypoint(-x, -y, altitude, 0);
	raspilotBusyWait(waitOnWaypoint);
	raspilotGotoWaypoint(-x, y, altitude, 0);
	raspilotBusyWait(waitOnWaypoint);
    }
    
    raspilotGotoWaypoint(x, y, altitude, 0);
    raspilotBusyWait(waitOnWaypoint);

    // goto landing point
    raspilotGotoWaypoint(0, 0, altitude, 0);

    lprintf(1, "%s: mission: End   square(%g, %g)!\n", PPREFIX(), squareSize, altitude);

    // Try "precise" landing.
    raspilotGotoWaypoint(0, 0, 0.10, 0);
    raspilotLand(0, 0);
    
}

void missionLandImmediately() {
    if (uu->flyStage < FS_FLY) {
	// if we did not launch yet, just shutdown pilot
	lprintf(0, "%s: Interactive: shutdown.\n", PPREFIX());
	raspilotShutDownAndExit();
    } else {
	// we are flying, do landing
	lprintf(0, "%s: Interactive: Landing based on interactive command.\n", PPREFIX());
	pilotImmediateLanding();
    }
}

void missionProcessInteractiveInput(int c) {
    static double 	tinc = 0;
    int			r;

    // initialize increment to 10% if ci
    if (tinc == 0) tinc = uu->pidAltitude.constant.ci / 10;
    switch(c) {
    case '+':
	tinc *= 2;
	break;
    case '-':
	tinc /= 2;
	break;
    case 'w':
	uu->pidAltitude.constant.ci += tinc;
	uu->pidAltitude.constant.ci = truncateToRange(uu->pidAltitude.constant.ci, 0, 1, NULL);
	printf("%s: Interactive: thrust == %f\n", PPREFIX(), uu->pidAltitude.constant.ci);
	break;
    case 's':
	uu->pidAltitude.constant.ci -= tinc;
	uu->pidAltitude.constant.ci = truncateToRange(uu->pidAltitude.constant.ci, 0, 1, NULL);
	printf("%s: Interactive: thrust == %f\n", PPREFIX(), uu->pidAltitude.constant.ci);
	break;
    case 'l':
    case 'L':
	// normal interactive landing
	missionLandImmediately();
	break;
    default:
	// emergency turn off nmotors
	lprintf(0, "%s: Interactive: shutdown.\n", PPREFIX());
	raspilotShutDownAndExit();
    }
}

void missionInteractive(double missionTime) {
    // Hmm. we can not 'launch' without working altitude sensor
    raspilotLaunch(0);
    raspilotWaypointSet(0, 0, 0, 0);
    lprintf(1, "%s: Starting Interactive\n", PPREFIX());
    raspilotBusyWait(9999);
}

void missionJoystick(double loiterTime) {
    // We suppose that the waypoint has been set by joystick yet
    raspilotLaunch(uu->currentWaypoint.position[2]);
    lprintf(1, "%s: mission: Begin following the joystick!\n", PPREFIX());
    raspilotBusyWait(loiterTime);
    lprintf(1, "%s: mission: End   following the joystick!\n", PPREFIX());
    raspilotLand(uu->currentWaypoint.position[0], uu->currentWaypoint.position[1]);    
}

// if motorIndex < 0 then all motors are set together
void missionMotorPwmCalibrationAndExit(int motorIndex) {
    int i, c;

    printf("\n\n\n");
    printf("%s: Make sure that the power for ESCs is turned off !!!\n", PPREFIX());
    printf("%s: If not, motors will launch on max speed  !!!\n", PPREFIX());
    printf("%s: Type 'c' if ESCs are off and we can continue.\n", PPREFIX());
    stdbaioStdinClearBuffer();
    timeLineInsertEvent(UTIME_AFTER_MSEC(1), pilotRegularSpecialModeTick, NULL);
    
    while ((c=stdbaioStdinMaybeGetPendingChar()) == -1) raspilotPoll();
    if (c != 'c') {
	printf("%s: 'c' not pressed. Exiting calibration.\n", PPREFIX());
	raspilotShutDownAndExit();
    }
    
    printf("%s: OK. Sending max pulses to GPIO.\n", PPREFIX());
    fflush(stdout);
    motorThrustSetAndSend(motorIndex, 1.0);

    printf("%s: Turn ESCs on and press 'c' one more time!\n", PPREFIX());
    stdbaioStdinClearBuffer();
    while ((c=stdbaioStdinMaybeGetPendingChar()) == -1) raspilotPoll();
    if (c != 'c') {
	motorThrustSetAndSend(motorIndex, 0.0);
	printf("%s: 'c' not pressed. Exiting calibration.\n", PPREFIX());
	raspilotShutDownAndExit();
    }

    printf("%s: OK. Sending min pulses for 5 seconds.\n", PPREFIX());
    fflush(stdout);
    motorThrustSetAndSend(motorIndex, 0.0);
    raspilotBusyWait(5.0);

    printf("%s: Calibration done. Spinning slowly for 5 seconds\n", PPREFIX());
    fflush(stdout);
    motorThrustSetAndSend(motorIndex, uu->config.motor_thrust_min_spin);
    raspilotBusyWait(5.0);

    printf("%s: All done.\n", PPREFIX());
    fflush(stdout);
    raspilotShutDownAndExit();
}

void missionSingleMotorTest(int motorIndex) {
    motorsThrustSetAndSend(0);
    lprintf(0, "%s: Warning: testing motor %d\n", PPREFIX(), motorIndex);
    motorThrustSetAndSend(motorIndex, uu->config.motor_thrust_min_spin);
    raspilotBusyWait(1.0);
    motorThrustSetAndSend(motorIndex, 0);
    raspilotBusyWait(1.0);
}

void missionMotorTest(int i) {
    timeLineInsertEvent(UTIME_AFTER_MSEC(1), pilotRegularSpecialModeTick, NULL);
    // Do a longer wait for case if someting is not initialized immediately.
    raspilotBusyWait(10.0);
    if (i < 0) {
	missionSingleMotorTest(0) ;
	missionSingleMotorTest(1) ;
	missionSingleMotorTest(2) ;
	missionSingleMotorTest(3) ;
    } else {
	missionSingleMotorTest(i) ;
    }
    raspilotShutDownAndExit();
}

///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////

// The mission to be executed by the drone
void mission() {

    if (0) {
	// calibrate one or all motor ESC (depending on argument)
	missionMotorPwmCalibrationAndExit(1);
    } else if (0) {
	// rotate motors one after another
	missionMotorTest(-1);
    } else if (1) {
	// Joystick conrolled.
	// Flight controller only. (TODO: To be finished).
	uu->manual.altitude = -0.1;
	timeLineInsertEvent(UTIME_AFTER_MSEC(10), pilotRegularManualControl, NULL);
	raspilotPreLaunchSequence(1);
	if (uu->manual.altitude <= 0) {
	    lprintf(0, "%s: Error: Launch altitude not set during prefly. Exiting!\n", PPREFIX());
	    raspilotShutDownAndExit();
	}
	uu->flyStage = FS_FLY;
	// raspilotBusyWait(15);
	lprintf(0, "%s: Info: launched.\n", PPREFIX());
	for(;;) raspilotPoll();
    } else {
	// autopilot mission mode
	raspilotPreLaunchSequence(0);
	missionLoiter(120.0, 0.60);
	// missionJoystick(10.0);
	// missionTestYawLoiter(0.10);
	// missionSquare(0.10, 0.10, 1.0, 1);
	// missionSquare(0.30, 0.15, 1.0, 1);
	// missionInteractive(0.1);
    }
}

