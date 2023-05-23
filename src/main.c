
#include "common.h"

//////////////////////////////////////////////////////////////////////////////////////////
// TODO: probably create a new file for those functions
enum statisticsActionEnum {
    STATISTIC_NONE,
    STATISTIC_INIT,
    STATISTIC_PRINT,
    STATISTIC_MAX,
};

void mainStatisticsMotors(FILE *ff, int action) {
    int 	i;
    double 	sum, max, tt;

    sum = 0; max = -1;
    for(i=0; i<uu->motor_number; i++) {
	if (action == STATISTIC_INIT) {
	    uu->motor[i].totalWork = 0;
	}
	tt = uu->motor[i].totalWork;
	sum += tt;
	if (tt > max) max = tt;
    }
    
    if (action != STATISTIC_PRINT) return;
    
    fprintf(ff, "%s: Average motor thrust: %g\n", PPREFIX(), sum / uu->motor_number / (currentTime.dtime - uu->flyStartTime));

    if (0) {
	fprintf(ff, "%s: Proposed New motor_esc_corrections: ", PPREFIX());
	for(i=0; i<uu->motor_number; i++) {
	    tt = uu->motor[i].totalWork;
	    if (i==0) {
		fprintf(ff, "[");
	    } else {
		fprintf(ff, ",");
	    }
	    fprintf(ff, "%f", tt/max);
	}
	fprintf(ff, "],\n");
    }
}

void mainStatisticsPoseSensors(FILE *ff, int action) {
    int 			i, j, k;
    struct deviceData		*dd;
    struct deviceDataData	*ddd;
    char			*sep;
    
    if (action == STATISTIC_PRINT) {
	fprintf(ff, "%s:\n", PPREFIX());
	fprintf(ff, "%s: Sensors / Devices:\n", PPREFIX());
    }
    for(i=0; i<uu->deviceMax; i++) {
	dd = uu->device[i];
	if (dd != NULL) {
	    if (action == STATISTIC_PRINT) fprintf(ff, "%s:  %s\n", PPREFIX(), dd->name);
	    for(j=0; j<dd->ddtMax; j++) {
		ddd = dd->ddt[j];
		if (ddd != NULL) {
		    if (action == STATISTIC_PRINT) fprintf(ff, "%s:   %30s: ", PPREFIX(), ddd->name);
		    switch (ddd->type) {
		    case DT_VOID:
			if (action == STATISTIC_PRINT) fprintf(ff, "%d records received", ddd->totalNumberOfRecordsReceivedForStatistics);
			break;
		    case DT_DEBUG:
			// no statistics for debugging info?
			if (action == STATISTIC_PRINT) fprintf(ff, "%d records received", ddd->totalNumberOfRecordsReceivedForStatistics);
			break;
		    case DT_PONG:
			if (action == STATISTIC_PRINT) fprintf(ff, "average round trip latency: %g ms from %d pings", 1000.0*ddd->pongTotalTimeForStatistics/ddd->totalNumberOfRecordsReceivedForStatistics, ddd->totalNumberOfRecordsReceivedForStatistics);
			break;
		    default:
			sep = "average: [";
			for(k=0; k<DIM(ddd->history.totalSumForStatistics); k++) {
			    if (action == STATISTIC_INIT) ddd->history.totalSumForStatistics[k]=0;
			    if (action == STATISTIC_PRINT && ddd->history.n != 0) fprintf(ff, "%s%g", sep, ddd->history.totalSumForStatistics[k] / ddd->history.n);
			    sep = ", ";
			}
			// Do not do this. It would clear current position and orientation
			// if (action == STATISTIC_INIT) ddd->history.n = 0;
			if (action == STATISTIC_PRINT) fprintf(ff, "]");
			break;
		    }
		    if (action == STATISTIC_PRINT) fprintf(ff, "\n");
		}
	    }
	}
    }
    if (action == STATISTIC_PRINT) fprintf(ff, "%s: End\n", PPREFIX());   
}


void mainStatisticsPids(FILE *ff, int action) {
    int 			i, j, k;
    struct deviceData		*dd;
    struct deviceDataData	*ddd;
    char			*sep;


    if (action == STATISTIC_INIT) {
	pidControllerReset(&uu->pidX);
	pidControllerReset(&uu->pidY);
	pidControllerReset(&uu->pidAltitude);
	pidControllerReset(&uu->pidRoll);
	pidControllerReset(&uu->pidPitch);
	pidControllerReset(&uu->pidYaw);
    }
    
    if (action == STATISTIC_PRINT) {
	fprintf(ff, "%s:\n", PPREFIX());
	fprintf(ff, "%s: PID Roll:     %s\n", PPREFIX(), pidControllerStatistics(&uu->pidRoll, 1));
	fprintf(ff, "%s: PID Pitch:    %s\n", PPREFIX(), pidControllerStatistics(&uu->pidPitch, 1));
	fprintf(ff, "%s: PID Yaw:      %s\n", PPREFIX(), pidControllerStatistics(&uu->pidYaw, 1));
	fprintf(ff, "%s: PID Altitude: %s\n", PPREFIX(), pidControllerStatistics(&uu->pidAltitude, 1));
	fprintf(ff, "%s: PID X:        %s\n", PPREFIX(), pidControllerStatistics(&uu->pidX, 0));
	fprintf(ff, "%s: PID Y:        %s\n", PPREFIX(), pidControllerStatistics(&uu->pidY, 0));
	fprintf(ff, "%s:\n", PPREFIX());
    }
}

void mainStatistics(int action) {
    FILE *ff;

    ff = NULL;
    if (action == STATISTIC_INIT) uu->flyStartTime = currentTime.dtime;
    if (action == STATISTIC_PRINT) {
	ff = fopen("statistics.txt", "w");
	if (ff == NULL) ff = stdout;
	fprintf(ff, "%s: \n", PPREFIX());
	fprintf(ff, "%s: STATISTICS:\n", PPREFIX());
	fprintf(ff, "%s: Mission time: %gs\n", PPREFIX(), currentTime.dtime - uu->flyStartTime);
    }
    
    mainStatisticsMotors(ff, action);
    mainStatisticsPoseSensors(ff, action);
    mainStatisticsPids(ff, action);
    
    if (action == STATISTIC_PRINT) {
	fprintf(ff, "\n");
	if (ff != stdout) fclose(ff);
    }
}

// this is emergency shutdown
void shutdown() {
    // to be implemented
    motorsEmmergencyLand();
}

void mainExit(void *d) {
    printf("%s: Exiting.\n", PPREFIX());
    if (1) mainStatistics(STATISTIC_PRINT);
    fflush(stdout);
    exit(0);
}

void mainStandardShutdown(void *d) {
    lprintf(0, "%s: Raspilot is going down\n", PPREFIX());
    trajectoryLogClose();
    pingToHostClose();
    stdbaioClose();
    fflush(stdout);
    motorsStop(NULL);
    motorsStandby(NULL) ;
    shutDownInProgress = 1;
    terminalResume();
    timeLineRemoveAllEvents();
    timeLineInsertEvent(UTIME_AFTER_MSEC(500), mainExit, d);
}

//////////////////////////////////////////////////////////

void pilotInterruptHandler(int sig) {
    static unsigned fatal_error_in_progress = 0;
    char	*ss;

    if (fatal_error_in_progress ++) {
	raise(sig);
	exit(-1);
    }
    
    // try to print info message
    ss = signalInterruptNames[sig];
    if (ss == NULL) ss = "";
    printf("%s: FATAL ERROR: Signal %s (%d) received. Aborting!\n", PPREFIX(), signalInterruptNames[sig], sig);
    fflush(stdout);

    // TODO: Remove this exception for production, i.e. do safe landing on interrupt
    if (sig == SIGINT || sig == SIGTERM) {
	// user interrupted (probably during debug), just stop motors and shutdown pilot
	mainStandardShutdown(NULL);
    } else {
	// unexpected signal (like segmentation fault, floating point exception ...) probably bug.
	// send motors safe-land command and core dump
	motorsEmmergencyLand();
	terminalResume();
	signal(SIGABRT, SIG_DFL);
	abort();
    }
}

static void zombieHandler(int s) {
    waitpid((pid_t)(-1), 0, WNOHANG);
}

static void sigPipeHandler(int s) {
    // TODO: Do some more informal message. Implement something to detect closed pipe
    // and printing some informative message which device crashed.
    lprintf(0, "%s: Warning: SIGPIPE received!\n", PPREFIX());
}

////////////////////////////////////////////////////////////

static void mainPrintUsage() {
    printf("Usage:     raspilot [options]\n"
	   "\n"
	   "options:\n"
	   " -c <config_file> : load configuration from config_file\n"
	   " -d <number>      : set verbosity of debug output. Range 0 - 1000\n"
	   " -h               : this help\n"
	   " -p <ip_address>  : during a fly, regularly ping this host. Land if the connection is lost.\n"
	   "\n"
	);
}

int mainProcessCommandLineArgs(int argc, char **argv) {
    int i;

    // task name
    if (argc > 0 && strlen(argv[0]) >= 4 && strcmp(argv[0]+strlen(argv[0])-4, "stop") == 0) {
	// if invoked as stop engine, stop the engine
	lprintf(0, "%s: Stopping engine!\n", PPREFIX());
	debugLevel = 0;
    } else {
	// otherwise execute mission
	debugLevel = DEFAULT_DEBUG_LEVEL;
    }

    for(i=1; i<argc && argv[i][0] == '-'; i++) {
	if (strcmp(argv[i], "-c") == 0) {
	    // cfg file 
	    if (i+1 >= argc) {
		lprintf(0, "%s: Error: Command line: -c shall be followed by configuration file\n", PPREFIX());
	    } else {
		i++;
		uu->cfgFileName = strDuplicate(argv[i]);
		lprintf(0, "%s: Info: Configuration file is %s\n", PPREFIX(), uu->cfgFileName);
	    }
	} else if (strcmp(argv[i], "-d") == 0) {
	    // debug level
	    if (i+1 >= argc) {
		lprintf(0, "%s: Error: Command line: -d shall be followed by debug level argument\n", PPREFIX());
	    } else {
		i++;
		debugLevel = atoi(argv[i]);
		if (debugLevel > 0) lprintf(0, "%s: Info: debug level: %d\n", PPREFIX(), debugLevel);
	    }
	} else if (strcmp(argv[i], "-p") == 0) {
	    // ping to host
	    // if the connection to this host is broken, return to home or land!
	    if (i+1 >= argc) {
		lprintf(0, "%s: Error: Command line: -p shall be followed by IP address of host to ping\n", PPREFIX());
	    } else {
		i++;
		uu->pingToHost = argv[i];
		if (1 || debugLevel > 0) lprintf(0, "%s: Info: ping to host %s\n", PPREFIX(), uu->pingToHost);
	    }
	} else {
	    if (argv[i][1] != 'h') {
		lprintf(0, "Error: unknown option %s\n\n", argv[i]);
	    }
	    mainPrintUsage();
	    exit(0);
	}
    }

    if (argc != i) {
	lprintf(0, "%s: Error: Command line: wrong number of arguments!\n", PPREFIX());
	mainPrintUsage();
	exit(0);
    }
    
    return(i);
}
	
static void initTask() {
    int i;

    // the very first initializations of the system
    
    memset(uu, 0, sizeof(*uu));
    enumNamesInit();
    setCurrentTime();
    uu->pilotStartingTime = currentTime.dtime;
    baioLibraryInit(0);

    signal(SIGABRT, pilotInterruptHandler);
    signal(SIGFPE,  pilotInterruptHandler);
    signal(SIGILL,  pilotInterruptHandler);
    signal(SIGINT,  pilotInterruptHandler);
    signal(SIGSEGV, pilotInterruptHandler);
    signal(SIGTERM, pilotInterruptHandler);

    signal(SIGPIPE, sigPipeHandler);				// ignore SIGPIPE Signals
    signal(SIGCHLD, zombieHandler);				// avoid system of keeping child zombies

}

static void initConfiguredPilot() {
    int			i;
    struct baio 	*bb;
    struct deviceData	*dd;

    uu->pilotLaunchTime = currentTime.dtime;
    poseHistoryInit(&uu->shortHistoryPose, uu->config.short_history_seconds * uu->stabilization_loop_Hz);
    sprintf(uu->shortHistoryPose.name, "short pose history");
    poseHistoryInit(&uu->shortHistoryVelo, uu->config.short_history_seconds * uu->stabilization_loop_Hz);
    sprintf(uu->shortHistoryPose.name, "short velo history");
    
    for(i=0; i<uu->motor_number; i++) {
	uu->motor[i].thrust = 0.0;
    }
    
    for(i=0; i<uu->deviceMax; i++) {
	deviceInitiate(i);
    }

    // Just for now
    assert(uu->device[0]->name != NULL);
    assert(strcmp(uu->device[0]->name, "motors") == 0);
    uu->motorBaioMagic = uu->device[0]->baioMagic;
}
    
//////////////////////////////////////////////////////////////////////////////////////////

int raspilotPoll() {
    struct timeval 	tv;
    int 		r;

    setCurrentTime();
    timeLineTimeToNextEvent(&tv, 1);
    // execute I/O operations
    r = baioPoll(tv.tv_sec*1000000+tv.tv_usec);
    setCurrentTime();
    // execute planned operations
    r += timeLineExecuteScheduledEvents(0);
    return(r);
}

void raspilotBusyWait(double sleeptime) {
    double tt;
    
    setCurrentTime();
    tt = currentTime.dtime;
    while (currentTime.dtime < tt + sleeptime) raspilotPoll();
}

int raspilotInit(int argc, char **argv) {
    double 	tt;

    stdbaioInit();
    initTask();
    mainProcessCommandLineArgs(argc, argv);
    configloadFile();
    initConfiguredPilot();

    trajectoryLogInit();
    if (uu->pingToHost != NULL) pingToHostInit();
    

    // the first ping wakes up motors
    timeLineInsertEvent(UTIME_AFTER_MSEC(5000), pilotRegularMotorPing, NULL);
    // read standard input for interactive commands
    timeLineInsertEvent(UTIME_AFTER_MSEC(10), pilotInteractiveInputRegularCheck, NULL);
    // start saving trajectory
    timeLineInsertEvent(UTIME_AFTER_MSEC(30), pilotRegularSaveTrajectory, NULL);

    // check that ping to master host is still alive, give him some time before the first check
    if (uu->pingToHost != NULL) timeLineInsertEvent(UTIME_AFTER_SECONDS(5), pingToHostRegularCheck, NULL);

    // Sleep at leat 1 second. It is the usual time for killing subprocesses and restarting new one in config
    // file
    raspilotBusyWait(1.10);
    return(0);
}

void raspilotPreLaunchSequence() {
    double	thrust;
    int		i, n;
    int64_t	nextTickUsec;
    double 	tt, td;
    double 	thrust_base;
    
    sendToAllDevices("info: init\n");
    timeLineInsertEvent(UTIME_AFTER_MSEC(1), pilotRegularPreLaunchTick, NULL);

    lprintf(1, "%s: Info: Starting prefly sequence.\n", PPREFIX());
    while (! pilotAreAllDevicesReady()) raspilotPoll();
    
    lprintf(1, "%s: Info: All sensors/devices ready.\n", PPREFIX());

    motorsThrustSet(0);
    raspilotBusyWait(PILOT_WARMING_WARNING_ROTATIONS_TO_LAUNCH);
    lprintf(1, "%s: Warning: First warning motor rotation!\n", PPREFIX());
    motorsThrustSet(uu->config.motor_thrust_min_spin);
    raspilotBusyWait(PILOT_WARMING_WARNING_ROTATION_TIME);
    motorsThrustSet(0);
    raspilotBusyWait(PILOT_WARMING_WARNING_ROTATIONS_DELAY);
    pilotStoreLaunchPose(NULL);
    lprintf(1, "%s: Warning: Second warning rotation!\n", PPREFIX());
    motorsThrustSet(uu->config.motor_thrust_min_spin);
    raspilotBusyWait(PILOT_WARMING_WARNING_ROTATION_TIME);
    motorsThrustSet(0);
    raspilotBusyWait(PILOT_WARMING_WARNING_ROTATIONS_TO_LAUNCH);
    
    lprintf(1, "%s: Info: Prefly rotation!\n", PPREFIX());

    // start flying motor rotation
    // rotate up to 80% of loitering rotation. Do not rotate until 100%, because you may start flying here
    // which would be a disaster without doing any correction.
    motorsThrustSet(uu->config.motor_thrust_min_spin);
    raspilotBusyWait(PILOT_WARMING_WARNING_ROTATION_TIME);

    // end of pre-launch events
    timeLineRemoveEventAtUnknownTimeAndArg(pilotRegularPreLaunchTick);
    
    lprintf(5, "%s: Info: Prefly sequence done.\n", PPREFIX());

    mainStatistics(STATISTIC_INIT);
    pilotUpdatePositionHistoryAndRecomputeMotorThrust();		// initiate PIDs
}

void raspilotLaunch(double altitude) {
    struct pose 	cc, *cpose;
    double 		tt;
    struct config	savedConfig;
    struct waypoint     savedWaypoint;

    if (altitude != 0 && altitude < uu->config.drone_min_altitude) {
	lprintf(1, "%s: Warning: Launch altitude smaller than drone_min_altitude!\n", PPREFIX());
	// prefer not to overwrite because of 'joystick' fly
	// altitude = uu->config.drone_min_altitude;
    }
    
    lprintf(1, "%s: Warning: launch: Going to fly!\n", PPREFIX());
    sendToAllDevices("info: takeoff\n");

    // schedule regular flight events
    timeLineInsertEvent(UTIME_AFTER_MSEC(1), pilotRegularStabilizationLoopTick, NULL);

    savedConfig = uu->config;
    savedWaypoint = uu->currentWaypoint;
    
#if 1
    poseHistoryEstimatePoseForTimeByLinearRegression(&uu->shortHistoryPose, currentTime.dtime, &cc);
    cpose = &cc;
#else    
    cpose = POSE_HISTORY_LAST(&uu->shortHistoryPose);
#endif
    
    memcpy(uu->currentWaypoint.position, cpose->pr, sizeof(uu->currentWaypoint.position));

    // set a bit higher altitude, so that we do not stop there
    if (altitude == 0) {
	uu->currentWaypoint.position[2] = altitude;
    } else {
	uu->currentWaypoint.position[2] = altitude + uu->config.drone_waypoint_reached_range + 0.01;
    }
    uu->config.drone_min_altitude = -99999.0;
	
    // Remove the speed constraint, so that we launch a bit quicker than normal flight
    // Otherwise the drone is drifting on minimal altitude and stumble about something
    uu->config.drone_max_speed = PILOT_LAUNCH_SPEED;
    uu->config.pilot_reach_goal_orientation_time = PILOT_LAUNCH_GOAL_ORIENTATION_TIME;
    uu->config.pilot_reach_goal_position_time = PILOT_LAUNCH_GOAL_POSITION_TIME;
    
    // initiate PID
    pilotUpdatePositionHistoryAndRecomputeMotorThrust();
    
    tt = currentTime.dtime;
    while(raspilotCurrentAltitude() < altitude && currentTime.dtime < tt + PILOT_LAUNCH_MAX_TIME) raspilotPoll();

    // ok we are flying and launch altitude, recover configuration and continue user's program
    uu->config = savedConfig;
    uu->currentWaypoint = savedWaypoint;

    if (currentTime.dtime >= tt + PILOT_LAUNCH_MAX_TIME) {
	lprintf(1, "%s: Error: Can't launch within %g seconds. Aborting mission!\n", PPREFIX(), PILOT_LAUNCH_MAX_TIME);
	raspilotLand(0, 0);
	raspilotShutDownAndExit();
    }
    lprintf(1, "%s: Info: launched.\n", PPREFIX());
}

void raspilotLand(double x, double y) {
    double 		tt;
    struct config	savedConfig;
    struct waypoint     savedWaypoint;
    
    lprintf(1, "%s: Warning: landing!\n", PPREFIX());
    
    savedConfig = uu->config;
    savedWaypoint = uu->currentWaypoint;
        
    // go strait down on normal speed
    uu->currentWaypoint.position[0] = x;
    uu->currentWaypoint.position[1] = y;
    uu->currentWaypoint.position[2] = -999999.0;
    uu->config.drone_min_altitude = -999999.0;

    // Hmm. how to know that I am landed if I do not have sensors for it?
    // go strait to altitude (which is relative to the launch point) 1 m
    while(raspilotCurrentAltitude() > PILOT_LAND_ALTITUDE) raspilotPoll();

    // then go down on landing speed
    uu->config.drone_max_speed = PILOT_LAND_SPEED;
    if (1) {
	// landing with working sensors
	while(raspilotCurrentAltitude() > 0.01) raspilotPoll();
    } else {
	// a very safe (blind) land
	raspilotBusyWait(PILOT_LAND_ALTITUDE/PILOT_LAND_SPEED*1.5);
    }

    motorsStop(NULL);
    motorsStandby(NULL) ;

    uu->config = savedConfig;
    // uu->currentWaypoint = savedWaypoint;
}


void raspilotWaypointSet(double x, double y, double z, double yaw) {
    if (z < uu->config.drone_min_altitude) {
	lprintf(0, "%s: Error: waypoint altitude %g is lower than drone_min_altitude %g.\n",
		PPREFIX(), uu->currentWaypoint.position[2], uu->config.drone_min_altitude
	    );
	// z = uu->config.drone_min_altitude;
    }
    if (z > uu->config.drone_max_altitude) {
	lprintf(0, "%s: Error: waypoint altitude %g is higher than drone_max_altitude %g.\n",
		PPREFIX(), uu->currentWaypoint.position[2], uu->config.drone_min_altitude
	    );
	// z = uu->config.drone_max_altitude;
    }
    if (yaw < -M_PI || yaw > M_PI) {
	lprintf(0, "%s: Error: waypoint yaw %g out of range -PI .. PI. Normalizing it.\n", PPREFIX(), yaw);
	yaw = normalizeToRange(yaw, -M_PI, M_PI);
    }
    uu->currentWaypoint.position[0] = x;
    uu->currentWaypoint.position[1] = y;
    uu->currentWaypoint.position[2] = z;
    uu->currentWaypoint.yaw = yaw;
}

double raspilotCurrentAltitude() {
    struct pose		cpose;
    poseHistoryEstimatePoseForTimeByLinearRegression(&uu->shortHistoryPose, currentTime.dtime, &cpose);
    return(cpose.pr[2]);
}

int raspilotWaypointReached() {
    vec3		dv;
    double		distance, distanceYaw;
    struct pose 	cpose;

    poseHistoryEstimatePoseForTimeByLinearRegression(&uu->shortHistoryPose, currentTime.dtime, &cpose);
    vec3_sub(dv, uu->currentWaypoint.position, &cpose.pr[0]);
    distance = vec3_len(dv);
    if (distance > uu->config.drone_waypoint_reached_range) return(0);
    // TODO: Correct this
    distanceYaw = fabs(uu->currentWaypoint.yaw - cpose.pr[9]);
    // TODO: Uncomment me
    // if (distanceYaw > uu->config.drone_target_reached_angle) return(0);
    
    return(1);
}

void raspilotGotoWaypoint(double x, double y, double z, double yaw) {
    lprintf(1, "%s: Info: next    waypoint (%7.2g,%7.2g,%7.2g ), yaw %5.2g\n", PPREFIX(), x, y, z, yaw);
    raspilotWaypointSet(x, y, z, yaw);
    while (! raspilotWaypointReached()) raspilotPoll();
    if (debugLevel > 10)  lprintf(1, "\n\n\n");
    lprintf(1, "%s: Info: reached waypoint (%7.2g,%7.2g,%7.2g ), yaw %5.2g !\n", PPREFIX(), x, y, z, yaw);
    // lprintf(1, "%s: Warning: reached waypoint (%7.2g,%7.2g,%7.2g ), yaw %5.2g !\n", PPREFIX(), x, y, z, yaw);
    if (debugLevel > 10)  lprintf(1, "\n\n\n");
}

int raspilotShutDownAndExit() {
    mainStandardShutdown(NULL);
    // exit will be called from poll
    while(1) raspilotPoll();
    assert(0);
}


///////////////////////////////////////////////////////////////////////////////////////////
// This is the main raspilot entry function
// Do not modify this function for your mission. Instead edit the function
// 'mission' in the file 'mission.c' and setup your mission there

int main(int argc, char **argv) {

    raspilotInit(argc, argv);
    mission();
    raspilotShutDownAndExit();
}
