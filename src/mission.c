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
    raspilotBusyWaitUntilTimeoutOrStandby(loiterTime);

    lprintf(1, "%s: mission: End   loiter(%g, %g)!\n", PPREFIX(), loiterTime, altitude);

    // Hmm. if we are too low, it may hit the ground and fail roll,pitch goals
    // raspilotGotoWaypoint(0, 0, 0.10, 0);
    raspilotLand(0, 0);    
}

void missionTestYawLoiter(double altitude) {

    raspilotLaunch(uu->config.drone_min_altitude + 0.02);

    lprintf(1, "%s: mission: Begin testYawLoiter(%g)!\n", PPREFIX(), altitude);
    
    raspilotWaypointSet(0, 0, altitude, 0);
    raspilotBusyWaitUntilTimeoutOrStandby(3.0);

    lprintf(1, "%s: mission: Turn left!\n", PPREFIX(), altitude);
    raspilotWaypointSet(0, 0, altitude, 0.4);
    raspilotBusyWaitUntilTimeoutOrStandby(3.0);

    lprintf(1, "%s: mission: Turn right!\n", PPREFIX(), altitude);
    raspilotWaypointSet(0, 0, altitude, -0.4);
    raspilotBusyWaitUntilTimeoutOrStandby(3.0);

    lprintf(1, "%s: mission: Turn back!\n", PPREFIX(), altitude);
    raspilotWaypointSet(0, 0, altitude, 0);
    raspilotBusyWaitUntilTimeoutOrStandby(3.0);

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
	raspilotBusyWaitUntilTimeoutOrStandby(waitOnWaypoint);
	raspilotGotoWaypoint(x, -y, altitude, 0);
	raspilotBusyWaitUntilTimeoutOrStandby(waitOnWaypoint);
	raspilotGotoWaypoint(-x, -y, altitude, 0);
	raspilotBusyWaitUntilTimeoutOrStandby(waitOnWaypoint);
	raspilotGotoWaypoint(-x, y, altitude, 0);
	raspilotBusyWaitUntilTimeoutOrStandby(waitOnWaypoint);
    }
    
    raspilotGotoWaypoint(x, y, altitude, 0);
    raspilotBusyWaitUntilTimeoutOrStandby(waitOnWaypoint);

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
    switch(c) {
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
    raspilotBusyWaitUntilTimeoutOrStandby(9999);
}

void missionJoystick(double loiterTime) {
    // We suppose that the waypoint has been set by joystick yet
    raspilotLaunch(uu->currentWaypoint.position[2]);
    lprintf(1, "%s: mission: Begin following the joystick!\n", PPREFIX());
    raspilotBusyWaitUntilTimeoutOrStandby(loiterTime);
    lprintf(1, "%s: mission: End   following the joystick!\n", PPREFIX());
    raspilotLand(uu->currentWaypoint.position[0], uu->currentWaypoint.position[1]);    
}

///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////

void mission() {
    // autopilot mission mode
    // timeLineInsertEvent(UTIME_AFTER_SECONDS(1), pilotRegularManualControl, NULL); // because of joystick test with openHd
    raspilotPreLaunchSequence(0);

    // raspilotBusyWait(9999999);
    missionLoiter(200.0, 0.20);
	
    // missionTestYawLoiter(0.20);
    
    // missionSquare(0.30, 0.20, 1.0, 1);
    
    // missionSquare(0.30, 0.15, 1.0, 1);
    // missionInteractive(0.1);
}

