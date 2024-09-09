
#include "common.h"

//////////////////////////////////////////////////////////////////////////////////////////
// TODO: probably create a new file for those functions

enum statisticsActionEnum {
    STATISTIC_NONE,
    STATISTIC_INIT,
    STATISTIC_PRINT,
    STATISTIC_MAX,
};

void mainLoadPreviousFlyTime() {
    FILE * ff;
    uu->previousTotalFlyTime = 0;
    ff = fopen("totalFlyTime.txt", "r");
    if (ff != NULL) {
	fscanf(ff, "%lf", &uu->previousTotalFlyTime);
	fclose(ff);
    }
}

void mainSavePreviousFlyTime() {
    FILE * ff;

    // do not touch if did not fly at all
    if (uu->flyStage < FS_FLY) return;
    
    ff = fopen("totalFlyTime.txt", "w");
    if (ff != NULL) {
	fprintf(ff, "%f\n", uu->previousTotalFlyTime);
	fclose(ff);
    }
}

void mainStatisticsMotors(int action) {
    int 	i;
    double 	sum, max, tt, flightTime;

    flightTime = (currentTime.dtime - uu->flyStartTime);
    // avoid division by zero
    if (flightTime == 0) flightTime = 1;
    
    sum = 0; max = -1;
    if (action == STATISTIC_PRINT) lprintf(0, "%s: Average motor thrusts: ", PPREFIX());
    for(i=0; i<uu->motor_number; i++) {
	if (action == STATISTIC_INIT) {
	    uu->motor[i].totalWork = 0;
	}
	tt = uu->motor[i].totalWork;
	if (action == STATISTIC_PRINT) lprintf(0, "%g ", tt / flightTime);
	sum += tt;
	if (tt > max) max = tt;
    }
    if (action == STATISTIC_PRINT) lprintf(0, "\n");
    
    if (action != STATISTIC_PRINT) return;
    
    lprintf(0, "%s: Total average motor thrust: %g\n", PPREFIX(), sum / uu->motor_number / flightTime);

    if (0) {
	lprintf(0, "%s: Proposed New motor_esc_corrections: ", PPREFIX());
	for(i=0; i<uu->motor_number; i++) {
	    tt = uu->motor[i].totalWork;
	    if (i==0) {
		lprintf(0, "[");
	    } else {
		lprintf(0, ",");
	    }
	    lprintf(0, "%f", tt/max);
	}
	lprintf(0, "],\n");
    }
}

void mainStatisticsSensors(int action) {
    int 			i, j, k;
    struct deviceData		*dd;
    struct deviceStreamData	*ddd;
    char			*sep;
    
    if (action == STATISTIC_PRINT) {
	lprintf(0, "%s:\n", PPREFIX());
	lprintf(0, "%s: Sensors / Devices:\n", PPREFIX());
    }
    for(i=0; i<uu->deviceMax; i++) {
	dd = uu->device[i];
	if (dd != NULL) {
	    if (action == STATISTIC_PRINT) lprintf(0, "%s:  %s\n", PPREFIX(), dd->name);
	    for(j=0; j<dd->ddtMax; j++) {
		ddd = dd->ddt[j];
		if (ddd != NULL) {
		    if (action == STATISTIC_INIT) ddd->totalNumberOfRecordsReceivedForStatistics = 0;
		    if (action == STATISTIC_PRINT) lprintf(0, "%s:   %30s: ", PPREFIX(), ddd->name);
		    switch (ddd->type) {
		    case DT_VOID:
			if (action == STATISTIC_PRINT) lprintf(0, "%d records received", ddd->totalNumberOfRecordsReceivedForStatistics);
			break;
		    case DT_DEBUG:
			// no statistics for debugging info?
			if (action == STATISTIC_PRINT) lprintf(0, "%d records received", ddd->totalNumberOfRecordsReceivedForStatistics);
			break;
		    case DT_PONG:
			if (action == STATISTIC_INIT) ddd->pongTotalTimeForStatistics = 0;
			if (action == STATISTIC_PRINT) lprintf(0, "average round trip latency: %g ms from %d pings", 1000.0*ddd->pongTotalTimeForStatistics/ddd->totalNumberOfRecordsReceivedForStatistics, ddd->totalNumberOfRecordsReceivedForStatistics);
			break;
		    default:
			sep = "average: [";
			for(k=0; k<ddd->outputBuffer.vectorsize; k++) {
			    if (action == STATISTIC_INIT) ddd->outputBuffer.totalSumForStatistics[k]=0;
			    if (action == STATISTIC_PRINT && ddd->outputBuffer.totalElemsForStatistics != 0) lprintf(0, "%s%g", sep, ddd->outputBuffer.totalSumForStatistics[k] / ddd->outputBuffer.totalElemsForStatistics);
			    sep = ", ";
			}
			if (action == STATISTIC_INIT) ddd->outputBuffer.totalElemsForStatistics = 0;
			if (action == STATISTIC_PRINT) lprintf(0, "]");
			break;
		    }
		    if (action == STATISTIC_PRINT) lprintf(0, "\n");
		}
	    }
	}
    }
    if (action == STATISTIC_PRINT) lprintf(0, "%s: End\n", PPREFIX());   
}


void mainStatisticsPids(int action) {
    int 			i, j, k;
    struct deviceData		*dd;
    struct deviceStreamData	*ddd;
    char			*sep;


    if (action == STATISTIC_INIT) {
	pidControllerReset(&uu->pidX, 1/uu->autopilot_loop_Hz);
	pidControllerReset(&uu->pidY, 1/uu->autopilot_loop_Hz);
	pidControllerReset(&uu->pidAltitude, 1.0/uu->stabilization_loop_Hz);
	pidControllerReset(&uu->pidAccAltitude, 1.0/uu->stabilization_loop_Hz);
	pidControllerReset(&uu->pidRoll, 1.0/uu->stabilization_loop_Hz);
	pidControllerReset(&uu->pidPitch, 1.0/uu->stabilization_loop_Hz);
	pidControllerReset(&uu->pidYaw, 1.0/uu->stabilization_loop_Hz);
    }
    
    if (action == STATISTIC_PRINT) {
	lprintf(0, "%s:\n", PPREFIX());
	lprintf(0, "%s: PID Roll:     %s\n", PPREFIX(), pidControllerStatistics(&uu->pidRoll, 1));
	lprintf(0, "%s: PID Pitch:    %s\n", PPREFIX(), pidControllerStatistics(&uu->pidPitch, 1));
	lprintf(0, "%s: PID Yaw:      %s\n", PPREFIX(), pidControllerStatistics(&uu->pidYaw, 1));
	lprintf(0, "%s: PID Altitude: %s\n", PPREFIX(), pidControllerStatistics(&uu->pidAltitude, 1));
	lprintf(0, "%s: PID X:        %s\n", PPREFIX(), pidControllerStatistics(&uu->pidX, 0));
	lprintf(0, "%s: PID Y:        %s\n", PPREFIX(), pidControllerStatistics(&uu->pidY, 0));
	lprintf(0, "%s:\n", PPREFIX());
    }
}

void mainStatistics(int action) {
    if (action == STATISTIC_INIT) {
	uu->flyStartTime = currentTime.dtime;
    }
    if (action == STATISTIC_PRINT) {
	double flyTime;

	if (uu->flyStartTime == 0) {
	    flyTime = 0;
	} else {
	    flyTime = currentTime.dtime - uu->flyStartTime;
	}
	lprintf(0, "%s: \n", PPREFIX());
	lprintf(0, "%s: STATISTICS:\n", PPREFIX());
	lprintf(0, "%s: Mission time: %gs\n", PPREFIX(), flyTime);
	uu->previousTotalFlyTime += flyTime;
	lprintf(0, "%s: Fly time since reset: %gs\n", PPREFIX(), uu->previousTotalFlyTime);
    }
    
    mainStatisticsMotors(action);
    mainStatisticsSensors(action);
    mainStatisticsPids(action);
    
    if (action == STATISTIC_PRINT) {
	lprintf(0, "\n");
    }
}

// this is emergency shutdown
void shutdown() {
    // to be implemented
    motorsEmmergencyLand();
}

void mainExit(void *d) {
    printf("%s: Exiting.\n", PPREFIX());
    fflush(stdout);
    exit(0);
}

void mainStandardShutdown(void *d) {
    int 	i;
    
    lprintf(0, "%s: Raspilot is going down\n", PPREFIX());
    motorsStop(NULL);

    motorsStandby(NULL);
    if (uu->deviceMotors >= 0 && uu->device[uu->deviceMotors] != NULL && uu->device[uu->deviceMotors]->shutdownExit) motorsExit(NULL);

    if (1) mainStatistics(STATISTIC_PRINT);
    // save flight time after statistics where it is updated

    // do not change flight status before savign total flight time
    mainSavePreviousFlyTime();
    uu->flyStage = FS_SHUTDOWN;
    
    
    shutDownInProgress = 1;

    // Deinitialize/close all devices
    for(i=0; i<uu->deviceMax; i++) deviceFinalize(i);

    trajectoryLogClose();
    pingToHostClose();
    logbaioClose();
    stdbaioClose();
    fflush(stdout);
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
#if 1
	// strangely, abort corrupts my stack.
	signal(SIGSEGV, SIG_DFL);
	DO_SIGSEGV();
#else
	signal(SIGABRT, SIG_DFL);
	abort();
#endif	
    }
}

static void zombieHandler(int s) {
    waitpid((pid_t)(-1), 0, WNOHANG);
}

static void sigPipeHandler(int s) {
    // TODO: Do some more informal message. Implement something to detect closed pipe
    // and printing some informative message which device crashed.
    lprintf(0, "%s: Warning: SIGPIPE received!\n", PPREFIX());
#if 1    
    lprintf(0, "%s: Core dumped for later debug.\n", PPREFIX()); CORE_DUMP();
#endif    
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
	motorsEmmergencyShutdown();
	stdbaioClose();
	exit(0);
    } else {
	// otherwise execute mission
	debugLevel = DEFAULT_DEBUG_LEVEL;
	logLevel = DEFAULT_LOG_LEVEL;
    }

    for(i=1; i<argc; i++) {
	// printf("Checking %s\n", argv[i]);
	if (strcmp(argv[i], "-c") == 0) {
	    // cfg file 
	    if (i+1 >= argc) {
		lprintf(0, "%s: Error: Command line: -c shall be followed by configuration file\n", PPREFIX());
	    } else {
		i++;
		uu->cfgFileName = strDuplicate(argv[i]);
		lprintf(0, "%s: Info: Configuration file is %s\n", PPREFIX(), uu->cfgFileName);
	    }
	} else if (strcmp(argv[i], "-d") == 0 || strcmp(argv[i], "-dl") == 0) {
	    // debug level
	    if (i+1 >= argc) {
		lprintf(0, "%s: Error: Command line: -d and -dl shall be followed by debug level argument\n", PPREFIX());
	    } else {
		i++;
		debugLevel = atoi(argv[i]);
		lprintf(0, "%s: Info: debug level: %d\n", PPREFIX(), debugLevel);
	    }
	} else if (strcmp(argv[i], "-ll") == 0) {
	    // log level
	    if (i+1 >= argc) {
		lprintf(0, "%s: Error: Command line: -ll shall be followed by log level argument\n", PPREFIX());
	    } else {
		i++;
		logLevel = baseLogLevel = atoi(argv[i]);
		if (logLevel > 0) lprintf(0, "%s: Info: log level: %d\n", PPREFIX(), logLevel);
	    }
	} else if (strcmp(argv[i], "-l") == 0) {
	    // log file
	    if (i+1 >= argc) {
		lprintf(0, "%s: Error: Command line: -l shall be followed by log file name\n", PPREFIX());
	    } else {
		i++;
		uu->logFileName = argv[i];
		lprintf(0, "%s: Info: log file: %s\n", PPREFIX(), uu->logFileName);
	    }
	} else if (strcmp(argv[i], "-p") == 0) {
	    // ping to host
	    // if the connection to this host is broken, return to home or land!
	    // TODO: This -p option and ping config shall be moved to the config file!!
	    // for example openhd & mavlink is not dependent on ssh ping connection.
	    if (i+1 >= argc) {
		lprintf(0, "%s: Error: Command line: -p shall be followed by IP address of host to ping\n", PPREFIX());
	    } else {
		i++;
		uu->pingToHost = argv[i];
		lprintf(0, "%s: Info: ping to host %s\n", PPREFIX(), uu->pingToHost);
	    }
	} else if (strcmp(argv[i], "-auto") == 0) {
	    uu->autostart = 1;
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

void mainInitDeviceDataStreamVectorLengths(int motor_number) {
    int 	i;
    
    // Streams in general are providing a vector of doubles.
    // This is the length of that vector for the given stream
    for(i=0; i<DT_MAX; i++) deviceDataStreamVectorLength[i] = -1;    
    deviceDataStreamVectorLength[DT_NONE] = 0;
    deviceDataStreamVectorLength[DT_VOID] = 0;
    deviceDataStreamVectorLength[DT_DEBUG] = 0;
    deviceDataStreamVectorLength[DT_PONG] = 0;
    deviceDataStreamVectorLength[DT_POSITION_VECTOR] = 3;
    deviceDataStreamVectorLength[DT_BOTTOM_RANGE] = 1;
    deviceDataStreamVectorLength[DT_FLOW_XY] = 2;
    deviceDataStreamVectorLength[DT_ALTITUDE] = 1;
    deviceDataStreamVectorLength[DT_TEMPERATURE] = 1;
    deviceDataStreamVectorLength[DT_MAGNETIC_HEADING] = 3;
    deviceDataStreamVectorLength[DT_ORIENTATION_RPY] = 3;
    deviceDataStreamVectorLength[DT_EARTH_ACCELERATION] = 3;
    // deviceDataStreamVectorLength[DT_ORIENTATION_QUATERNION] = 4;
    deviceDataStreamVectorLength[DT_POSITION_NMEA] = 3;
    deviceDataStreamVectorLength[DT_MAGNETIC_HEADING_NMEA] = 1;
    deviceDataStreamVectorLength[DT_JSTEST] = 0;
    deviceDataStreamVectorLength[DT_POSITION_SHM] = 3;
    deviceDataStreamVectorLength[DT_EARTH_ACCELERATION_SHM] = 3;
    deviceDataStreamVectorLength[DT_ORIENTATION_RPY_SHM] = 3;

    deviceDataStreamVectorLength[DT_PING] = 1;
    deviceDataStreamVectorLength[DT_THRUST] = motor_number;
    deviceDataStreamVectorLength[DT_THRUST_SHM] = motor_number;
    deviceDataStreamVectorLength[DT_GIMBAL_X] = 1;
    deviceDataStreamVectorLength[DT_GIMBAL_Y] = 1;

    deviceDataStreamVectorLength[DT_MAVLINK_RC_CHANNELS_OVERRIDE] = 18;
    deviceDataStreamVectorLength[DT_MAVLINK_ATTITUDE] = 6;
    deviceDataStreamVectorLength[DT_MAVLINK_BATTERY_STATUS] = 6;
    deviceDataStreamVectorLength[DT_MAVLINK_GLOBAL_POSITION] = 7;
    deviceDataStreamVectorLength[DT_MAVLINK_HOME_POSITION] = 7;
    deviceDataStreamVectorLength[DT_MAVLINK_STATUSTEXT] = 0;
	
    // check that we did not forget anything.
    for(i=0; i<DT_MAX; i++) {
	if (deviceDataStreamVectorLength[i] == -1) {
	    fprintf(stderr, "%s: Internal Error: deviceDataTypeLength[%d] not set. Exiting!\n", PPREFIX(), i);
	}
    }
}

static void initTask() {
    int 	i, r;
    cpu_set_t 	set;

    // the very first initializations of the system

    memset(uu, 0, sizeof(*uu));
    uu->flyStage = FS_START;
    enumNamesInit();
    setCurrentTime();
    uu->pilotStartingTime = currentTime.dtime;

    // pre-init vector length konstants. Some values depend on configuration, so it will be "reinitialized"
    // from config.c one more time.
    mainInitDeviceDataStreamVectorLengths(0);

    baioLibraryInit(0);
    strtodninit() ;
    // Hardware i2c may be shared between processes
    pi2cInit("/dev/i2c-1", 0);
    
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
    char		*mem;
    
    uu->pilotLaunchTime = currentTime.dtime;
    regressionBufferInit(&uu->longBufferPosition, 3, uu->config.long_buffer_seconds * uu->autopilot_loop_Hz + 0.5, "long buffer pose");
    regressionBufferInit(&uu->longBufferRpy, 3, uu->config.long_buffer_seconds * uu->autopilot_loop_Hz + 0.5, "long buffer orientation");
    regressionBufferInit(&uu->shortBufferPosition, 3, uu->config.short_buffer_seconds * uu->autopilot_loop_Hz + 0.5, "short buffer pose");
    regressionBufferInit(&uu->shortBufferRpy, 3, uu->config.short_buffer_seconds * uu->autopilot_loop_Hz + 0.5, "short buffer orientation");
    regressionBufferInit(&uu->shortBufferAcceleration, 3, uu->config.short_buffer_seconds * uu->autopilot_loop_Hz + 0.5, "short buffer acceleration");
    // hold non regression history of [x,y,z,r,p,y] for 5 seconds. At hight rate it is too big. Store 1 second.
    ALLOCC(mem, RASPILOT_RING_BUFFER_SIZE(1 * uu->autopilot_loop_Hz + 0.5, 6), char);
    uu->historyPose = (struct raspilotRingBuffer *) mem;
    raspilotRingBufferInit(uu->historyPose, 6, 1 * uu->autopilot_loop_Hz + 0.5, "historyPose");
    for(i=0; i<uu->motor_number; i++) {
	uu->motor[i].thrust = 0.0;
    }

    
    // The main launch/initialization of all configured devices.
    for(i=0; i<uu->deviceMax; i++) {
	deviceInitiate(i);
    }

    motorsSendStreamThrustFactor(NULL);

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

void raspilotBusyWaitUntilTimeoutOrStandby(double sleeptime) {
    double tt;
    
    setCurrentTime();
    tt = currentTime.dtime;
    while (currentTime.dtime < tt + sleeptime && uu->flyStage != FS_STANDBY) raspilotPoll();
}

int raspilotInit(int argc, char **argv) {
    double 	tt;

    stdbaioInit();
    initTask();
    mainProcessCommandLineArgs(argc, argv);
    logbaioInit();
    configloadFile();

    if (uu->autostart && uu->config.pilot_main_mode != MODE_MANUAL_RC) {
	printf("%s: Raspilot is not auto starting, because the main mode is not MODE_MANUAL_RC.\n", PPREFIX());
	exit(1);
    }
    
    initConfiguredPilot();

    trajectoryLogInit();
    if (uu->pingToHost != NULL) pingToHostInit();
    
    // the first ping wakes up motors. ? Is this still true?
    timeLineInsertEvent(UTIME_AFTER_MSEC(5000), pilotRegularSendPings, NULL);
    // read standard input for interactive commands
    timeLineInsertEvent(UTIME_AFTER_MSEC(10), pilotInteractiveInputRegularCheck, NULL);
    // start saving trajectory
    timeLineInsertEvent(UTIME_AFTER_MSEC(30), pilotRegularSaveTrajectory, NULL);
    // TODO: only if gimbal configured
    timeLineInsertEvent(UTIME_AFTER_MSEC(20), pilotRegularSendGimbalPwm, NULL);
    
    // check that ping to master host is still alive, give him some time before the first check
    if (uu->pingToHost != NULL) timeLineInsertEvent(UTIME_AFTER_SECONDS(50), pingToHostRegularCheck, NULL);

    // Sleep at leat 1 second. It is the usual time for killing subprocesses and restarting new one in config
    // file
    // raspilotBusyWait(1.10);
    return(0);
}

void raspilotPreLaunchSequence(int flightControllerOnlyMode) {
    double	thrust;
    int		i, n;
    int64_t	nextTickUsec;
    double 	tt, td;
    double 	thrust_warning_spin;
    
    // deviceSendToAllDevices("info: init\n");
    setCurrentTime();

    // initial value for batteryStatusRpFactor
    uu->averageAltitudeThrust = uu->config.motor_altitude_thrust_hold;
    uu->batteryStatusRpyFactor = uu->averageAltitudeThrust;
    
    uu->flyStage = FS_WAITING_FOR_SENSORS;
    thrust_warning_spin = uu->config.motor_thrust_min_spin;

    lprintf(1, "%s: Info: Starting prefly sequence.\n", PPREFIX());
    mavlinkPrintfStatusTextToListeners("Starting prefly sequence");
    while (! pilotAreAllDevicesReady()) {
	raspilotPoll();
	if (uu->flyStage == FS_STANDBY) goto launchCanceled;
    }
    
    lprintf(1, "%s: Info: All sensors/devices ready.\n", PPREFIX());

    // This will arm motors
    uu->flyStage = FS_PRE_FLY;
    // wait until motor beeps (ARM)
    motorsThrustSet(0);
    raspilotBusyWaitUntilTimeoutOrStandby(PILOT_WARMING_WARNING_ROTATIONS_TO_LAUNCH);
    if (uu->flyStage == FS_STANDBY) goto launchCanceled;
    
    if (! flightControllerOnlyMode) {
	lprintf(1, "%s: Warning: First warning motor rotation!\n", PPREFIX());
	mavlinkPrintfStatusTextToListeners("Warning rotation");
	motorsThrustSet(thrust_warning_spin);
	raspilotBusyWaitUntilTimeoutOrStandby(PILOT_WARMING_WARNING_ROTATION_TIME);
	if (uu->flyStage == FS_STANDBY) goto launchCanceled;
	motorsThrustSet(0);
	raspilotBusyWaitUntilTimeoutOrStandby(PILOT_WARMING_WARNING_ROTATIONS_DELAY);
	if (uu->flyStage == FS_STANDBY) goto launchCanceled;
    }
    
    // launch pose has to be stored between rotations. To have enough of time to accumulate reasonable values
    // for lauch pose and also to get enough of time to accumulate real values  (with substracted launchpose)
    // before the real launch. 
    pilotLaunchPoseSet(NULL);

    // One more time wait until all regression buffers are full with launch pose corrected values
    while (! pilotAreAllDevicesReady()) {
	raspilotPoll();
	if (uu->flyStage == FS_STANDBY) goto launchCanceled;
    }

    lprintf(1, "%s: Warning: Second warning rotation!\n", PPREFIX());
    mavlinkPrintfStatusTextToListeners("Warning rotation");
    motorsThrustSet(thrust_warning_spin);
    raspilotBusyWaitUntilTimeoutOrStandby(PILOT_WARMING_WARNING_ROTATION_TIME);
    if (uu->flyStage == FS_STANDBY) goto launchCanceled;
    motorsThrustSet(0);
    raspilotBusyWaitUntilTimeoutOrStandby(PILOT_WARMING_WARNING_ROTATIONS_TO_LAUNCH);
    if (uu->flyStage == FS_STANDBY) goto launchCanceled;
    
    lprintf(1, "%s: Info: Prefly rotation!\n", PPREFIX());
    // start flying motor rotation
    // Maybe a bit more than min rotation would be ok
    motorsThrustSet(uu->config.motor_thrust_min_spin);
    raspilotBusyWaitUntilTimeoutOrStandby(PILOT_WARMING_WARNING_ROTATION_TIME);
    if (uu->flyStage == FS_STANDBY) goto launchCanceled;

    lprintf(5, "%s: Info: Prefly sequence done.\n", PPREFIX());

    mainLoadPreviousFlyTime();
    mainStatistics(STATISTIC_INIT);
    pilotInitiatePids();
    return;

launchCanceled:
    motorsThrustSet(0);
    return;
}

void raspilotLaunch(double altitude) {
    vec3		cpose;
    double 		tt;
    struct config	savedConfig;
    struct waypoint     savedWaypoint;

    if (altitude != 0 && altitude < uu->config.drone_min_altitude) {
	lprintf(1, "%s: Warning: Launch altitude smaller than drone_min_altitude!\n", PPREFIX());
	// prefer not to overwrite because of 'joystick' fly
	// altitude = uu->config.drone_min_altitude;
    }
    
    uu->flyStage = FS_FLY;
    lprintf(0, "%s: Warning: launch: Going to fly!\n", PPREFIX());
    // deviceSendToAllDevices("info: takeoff\n");

    savedConfig = uu->config;
    savedWaypoint = uu->currentWaypoint;
    
    regressionBufferEstimateForTime(&uu->longBufferPosition, currentTime.dtime, cpose);

    vec3_assign(uu->currentWaypoint.position, cpose);

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
    //uu->config.pilot_reach_goal_orientation_time = PILOT_LAUNCH_GOAL_ORIENTATION_TIME;
    //uu->config.pilot_reach_goal_position_time = PILOT_LAUNCH_GOAL_POSITION_TIME;
    
    pilotInitiatePids();
    
    tt = currentTime.dtime;
    while(0 && raspilotCurrentAltitude() < altitude && currentTime.dtime < tt + PILOT_LAUNCH_MAX_TIME) raspilotPoll();

    // ok we are flying and launch altitude, recover configuration and continue user's program
    uu->config = savedConfig;
    uu->currentWaypoint = savedWaypoint;

    if (currentTime.dtime >= tt + PILOT_LAUNCH_MAX_TIME) {
	lprintf(0, "%s: Error: Can't launch within %g seconds. Aborting mission!\n", PPREFIX(), PILOT_LAUNCH_MAX_TIME);
	raspilotLand(0, 0);
	raspilotShutDownAndExit();
    }
    lprintf(0, "%s: Info: launched.\n", PPREFIX());
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
    // landing with working sensors
    while(raspilotCurrentAltitude() > 0.01) raspilotPoll();
    // sometimes altimeter gives wrong value, so go down for a few more soconds blindly
    raspilotBusyWaitUntilTimeoutOrStandby(3.0);

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
    vec3		cpose;
    regressionBufferEstimateForTime(&uu->longBufferPosition, currentTime.dtime, cpose);
    return(cpose[2]);
}

int raspilotWaypointReached() {
    vec3	dv;
    double	distance, distanceYaw;
    vec3 	cpose;
    vec3 	crpy;

    regressionBufferEstimateForTime(&uu->longBufferPosition, currentTime.dtime, cpose);
    regressionBufferEstimateForTime(&uu->longBufferRpy, currentTime.dtime, crpy);
    vec3_sub(dv, uu->currentWaypoint.position, cpose);
    distance = vec3_len(dv);
    if (distance > uu->config.drone_waypoint_reached_range) return(0);
    // TODO: Correct this
    distanceYaw = fabs(uu->currentWaypoint.yaw - crpy[2]);
    // TODO: Uncomment me
    // if (distanceYaw > uu->config.drone_target_reached_angle) return(0);
    
    return(1);
}

void raspilotGotoWaypoint(double x, double y, double z, double yaw) {
    lprintf(1, "%s: Info: next    waypoint (%7.2g,%7.2g,%7.2g ), yaw %5.2g\n", PPREFIX(), x, y, z, yaw);
    raspilotWaypointSet(x, y, z, yaw);
    while (! raspilotWaypointReached()) raspilotPoll();
    lprintf(10, "\n\n\n");
    lprintf(1, "%s: Info: reached waypoint (%7.2g,%7.2g,%7.2g ), yaw %5.2g !\n", PPREFIX(), x, y, z, yaw);
    // lprintf(1, "%s: Warning: reached waypoint (%7.2g,%7.2g,%7.2g ), yaw %5.2g !\n", PPREFIX(), x, y, z, yaw);
    lprintf(10, "\n\n\n");
}

int raspilotShutDownAndExit() {
    mainStandardShutdown(NULL);
    // exit will be called from poll
    while(1) raspilotPoll();
    assert(0);
}

void raspilotGotoStandby() {
    motorsStop(0);
    motorsStandby(NULL);
    uu->flyStage = FS_STANDBY;
}

#if 0
void test() {
    int 		i;
    struct pose 	pp, rr, mm, mmr, rrm, rrr;
    struct poseHistory 	hh, hhr, hhm;
    
    double		x;
    
    poseHistoryInit(&hh, 5);
    poseHistoryInit(&hhr, 20);
    poseHistoryInit(&hhm, 20);
    for(x=0; x<12.6; x+=0.1) {
	pp.time = x;
	pp.pr[0] = sin(x) ; // + (rand()%1024-512)*0.0001;
	poseHistoryAddElem(&hh, &pp);
	poseHistoryEstimatePoseForTimeByLinearRegression(&hh, x, &rr);
	poseHistoryAddElem(&hhr, &rr);	
	poseHistoryGetMean(&hh, &mm);
	poseHistoryAddElem(&hhm, &mm);	
	if (x>6.28) {
	    poseHistoryGetMean(&hh, &mm);
	    poseHistoryEstimatePoseForTimeByLinearRegression(&hh, x, &rr);
	    poseHistoryGetMean(&hhr, &mmr);
	    poseHistoryEstimatePoseForTimeByLinearRegression(&hhm, x, &rrm);
	    poseHistoryEstimatePoseForTimeByLinearRegression(&hhr, x, &rrr);
	    printf("%g %g %g %g %g %g %g\n", x, pp.pr[0], rr.pr[0], mm.pr[0], mmr.pr[0], rrm.pr[0], rrr.pr[0]);
	}
    }
}
#endif

///////////////////////////////////////////////////////////////////////////////////////////
// pilot modes

// if motorIndex < 0 then all motors are set together
static void pilotModeMotorPwmCalibrationAndExit(int motorIndex) {
    int i, c;

    printf("\n\n\n");
    printf("%s: Make sure that the power for ESCs is turned off !!!\n", PPREFIX());
    printf("%s: If not, motors will launch on max speed  !!!\n", PPREFIX());
    printf("%s: Type 'c' if ESCs are off and we can continue.\n", PPREFIX());
    stdbaioStdinClearBuffer();
    timeLineInsertEvent(UTIME_AFTER_MSEC(1), pilotRegularMotorTestModeTick, NULL);
    
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
    raspilotBusyWaitUntilTimeoutOrStandby(5.0);

    printf("%s: Calibration done. Spinning slowly for 5 seconds\n", PPREFIX());
    fflush(stdout);
    motorThrustSetAndSend(motorIndex, uu->config.motor_thrust_min_spin);
    raspilotBusyWaitUntilTimeoutOrStandby(5.0);

    printf("%s: All done.\n", PPREFIX());
    fflush(stdout);
    raspilotShutDownAndExit();
}

static void pilotSingleMotorTest(int motorIndex) {
    motorsThrustSetAndSend(0);
    lprintf(0, "%s: Warning: testing motor %d\n", PPREFIX(), motorIndex);
    motorThrustSetAndSend(motorIndex, uu->config.motor_thrust_min_spin);
    raspilotBusyWaitUntilTimeoutOrStandby(2.0);
    motorThrustSetAndSend(motorIndex, 0);
    raspilotBusyWaitUntilTimeoutOrStandby(1.0);
}

static void pilotModeMotorTest(int i) {
    timeLineInsertEvent(UTIME_AFTER_MSEC(1), pilotRegularMotorTestModeTick, NULL);
    // Do a longer wait for case if someting is not initialized immediately.
    raspilotBusyWaitUntilTimeoutOrStandby(10.0);
    if (i < 0) {
	pilotSingleMotorTest(0) ;
	pilotSingleMotorTest(1) ;
	pilotSingleMotorTest(2) ;
	pilotSingleMotorTest(3) ;
    } else {
	pilotSingleMotorTest(i) ;
    }
    raspilotShutDownAndExit();
}

static int droneHasEmergencyLanded() {
    if (uu->flyStage != FS_EMERGENCY_LANDING) return(0);
    // If we are on low altitude, confitm land
    if (uu->droneLastPosition[2] < 0.005) return(1);
    // if thrust is very low, we are probably landed even if altimeters do not say so
    if (uu->averageAltitudeThrust <= uu->config.motor_thrust_min_spin) return(1);
    return(0);
}

void mainEmergencyLanding() {
    time_t	landingStartTime;
    
    lprintf(0, "%s: Info: Emergency landing. Waiting for land.\n", PPREFIX());
    mavlinkPrintfStatusTextToListeners("Emergency Landing!");
    landingStartTime = currentTime.sec;
    while (uu->flyStage == FS_EMERGENCY_LANDING
	   && ! droneHasEmergencyLanded()
	   && currentTime.sec - landingStartTime < uu->config.emergency_landing_max_time
	) {
	raspilotPoll();
    }
    // If we are landed
    lprintf(0, "%s: Info: Emergency landed.\n", PPREFIX());
}

static void pilotModeManualRc() {
    // In this mode the drone is controller by the joystick or similar
    manualControlInit(&uu->rc.roll, &uu->config.manual_rc_roll);
    manualControlInit(&uu->rc.pitch, &uu->config.manual_rc_pitch);
    manualControlInit(&uu->rc.yaw, &uu->config.manual_rc_yaw);
    manualControlInit(&uu->rc.altitude, &uu->config.manual_rc_altitude);
    timeLineInsertEvent(UTIME_AFTER_MSEC(10), manualControlRegularCheck, NULL);
    uu->flyStage = FS_STANDBY;
    timeLineInsertEvent(UTIME_AFTER_MSEC(2), pilotRegularStabilisationTick, NULL);
    // This is the main loop when raspilot is in manual rc mode
    while (uu->flyStage == FS_STANDBY) {
	uu->rc.altitude.value = -9999;
	lprintf(0, "%s: Standby\n", PPREFIX());
	mavlinkPrintfStatusTextToListeners("Standby mode");
	usleep(10000);
	pilotLaunchPoseClear(NULL);
	while (uu->flyStage == FS_STANDBY) {
	    raspilotPoll() ;
	}
	if (uu->flyStage == FS_COUNTDOWN) {
	    lprintf(0, "%s: Countdown!\n", PPREFIX());
	    // mavlinkPrintfStatusTextToListeners("Countdown");
	    raspilotPreLaunchSequence(1);
	    if (uu->flyStage == FS_STANDBY) {
		lprintf(0, "%s: Info: Launch sequence interrupted.\n", PPREFIX());		
	    } else if (uu->rc.altitude.value <= -9999) {
		lprintf(0, "%s: Error: Launch altitude not set during prefly. Interrupting!\n", PPREFIX());
		mavlinkPrintfStatusTextToListeners("Launch altitude not set during prefly. Interrupting!\n");
		uu->flyStage = FS_STANDBY;
	    } else {
		lprintf(0, "%s: Info: launched.\n", PPREFIX());
		mavlinkPrintfStatusTextToListeners("Launch");
		uu->flyStage = FS_FLY;
		while (uu->flyStage == FS_FLY) {
		    raspilotPoll();
		}
		if (uu->flyStage == FS_EMERGENCY_LANDING) {
		    mainEmergencyLanding();
		    if (uu->flyStage == FS_EMERGENCY_LANDING) raspilotGotoStandby();
		}
	    }
	}
    }
}


///////////////////////////////////////////////////////////////////////////////////////////
// This is the main raspilot entry function
// Do not modify this function for your mission. Instead edit the function
// 'mission' in the file 'mission.c' and setup your mission there

int main(int argc, char **argv) {
    raspilotInit(argc, argv);
    switch (uu->config.pilot_main_mode) {
    case MODE_MOTOR_PWM_CALIBRATION:
	pilotModeMotorPwmCalibrationAndExit(1);
	break;
    case MODE_MOTOR_TEST:
	pilotModeMotorTest(-1);
	break;
    case MODE_MANUAL_RC:
	pilotModeManualRc();
	break;
    case MODE_SINGLE_MISSION:
	timeLineInsertEvent(UTIME_AFTER_MSEC(1), pilotRegularMissionModeLoopTick, NULL);
	timeLineInsertEvent(UTIME_AFTER_MSEC(2), pilotRegularStabilisationTick, NULL);
	mission();
	break;
    default:
	lprintf(0, "%s: Error: unexpected main pilot mode %d\n", PPREFIX(), uu->config.pilot_main_mode);
	break;
    }
    raspilotShutDownAndExit();
}
