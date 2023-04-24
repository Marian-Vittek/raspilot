
#include "common.h"

void motorsThrustSend(void *d) {
    int 		i;
    struct baio		*bb;
    double		thrust;

    // do not send anything in shutdown sequence (otherwise motors are awaken)
    if (shutDownInProgress) return;

    if (debugLevel > 49) {
	lprintf(0, "%s: Motor thrust: ", PPREFIX());
	for(i=0; i<uu->motor_number; i++) lprintf(0, "%5.3f ", uu->motor[i].thrust);
	lprintf(0, "\n");
    }

    for(i=0; i<uu->motor_number; i++) {
	thrust = uu->motor[i].thrust ;
	uu->motor[i].rotationSpeed = sqrt(thrust) * 1000; 	// c1 + c2 * sqrt(thrust);
    }

    // if we are within 0.1s from the last motor sending, do not resend the same values
    if (uu->motorLastSendTime + 0.10 >= currentTime.dtime) {
	for(i=0; i<uu->motor_number; i++) {
	    if (uu->motor[i].rotationSpeed != uu->motor[i].lastSentRotationSpeed) break;
	}
	// exactly the same values were sent within last 0.1s, do nothing.
	// I wonder if this ever happens
	// TODO: TEST: Put me back!!!!
	// if (i == uu->motor_number) return;
	
    }
    
    for(i=0; i<uu->motor_number; i++) {
	// For statistics
	// Hmm. when flying too long, this will not add anything, because of the big difference.
	// For a long fly values will be biased.
	uu->motor[i].totalWork += uu->motor[i].lastSentThrust * (currentTime.dtime - uu->motorLastSendTime);
    }
    
    bb = baioFromMagic(uu->motorBaioMagic);
    if (bb == NULL) return;
    baioPrintfToBuffer(bb, "mt%d", uu->motor_number);
    for(i=0; i<uu->motor_number; i++) {
	uu->motor[i].lastSentRotationSpeed = uu->motor[i].rotationSpeed;
	uu->motor[i].lastSentThrust = uu->motor[i].thrust;
	baioPrintfToBuffer(bb, " %d", uu->motor[i].rotationSpeed);
    }
    baioPrintfToBuffer(bb, "\n");
    uu->motorLastSendTime = currentTime.dtime;

    if (debugLevel > 39) {
	lprintf(0, "%s: Sent motor rotation speed: ", PPREFIX());
	for(i=0; i<uu->motor_number; i++) lprintf(0, "%d ", uu->motor[i].rotationSpeed);
	lprintf(0, "\n");
    }

}

void motorsStandby(void *d) {
    struct baio	*bb;
    bb = baioFromMagic(uu->motorBaioMagic);
    if (bb == NULL) return;
    baioPrintfToBuffer(bb, "stan\n");
}

void motorsThrustSet(double thrust) {
    int i;
    for(i=0; i<uu->motor_number; i++) {
	uu->motor[i].thrust = thrust;
    }
}

// i is the index of the motor, if -1 then all motors are set and sent
void motorThrustSetAndSend(int i, double thrust) {
    if (i == MOTORS_ALL) {
	motorsThrustSet(thrust);
    } else if (i >= 0 && i < uu->motor_number) {
	uu->motor[i].thrust = thrust;
    } else {
	lprintf(0, "%s: Internal Error: motor index %d of of range 0 - %d\n", PPREFIX(), i, uu->motor_number-1);
    }
    motorsThrustSend(NULL);    
}

void motorsThrustSetAndSend(double thrust) {
    motorThrustSetAndSend(MOTORS_ALL, thrust);
    motorsThrustSend(NULL);    
}

void motorsStop(void *d) {
    motorsThrustSetAndSend(0);
}

// This is an emergency function which does not use baio library
// and sends command directly to motor fd, it is used in from interrupt handler
// supposing raspilot task crashed and is in inconsistent state.
void motorsEmergencySendSpecialCommand(char *command) {
    struct baio		*bb;
    bb = baioFromMagic(uu->motorBaioMagic);
    if (bb == NULL) return;
    if (bb->wfd < 0) return;
    write(bb->wfd, command, strlen(command));
    usleep(10000);
}

void motorsEmmergencyLand() {
    motorsEmergencySendSpecialCommand("\nland\n");
}

void motorsEmmergencyStanby() {
    motorsEmergencySendSpecialCommand("\nstan\n");
}

void pilotImmediateLanding() {
    struct pose cpose;
    
    sendToAllDevices("info: landing\n");
    // go strait down on interactive down 
    poseHistoryEstimatePoseForTimeByLinearRegression(&uu->shortHistoryPose, currentTime.dtime, &cpose);
    raspilotLand(cpose.pr[0], cpose.pr[1]);
    raspilotShutDownAndExit();
}

//////////////////////////////////////////////////////////////////////////////////////

// This function always return position 0,0,0
// It is used when we need to only stabilize the drone
static void pilotUpdateZeropose(struct deviceDataData *gg, quat droneOrientation) {
    struct pose			vv;
    int				i, r;
    double			time;
    double			roll, pitch, yaw;
    vec2			dd;
    
    if (gg == NULL) return;

    memset(&vv, 0, sizeof(vv));
    vv.time = currentTime.dtime;
    vv.pr[6] = 1.0;
    
    poseHistoryAddElem(&gg->history, &vv);
    // give me a small confidence, not to interfere with real devices
    gg->confidence = 0.001;
}

// This function computes an estimation of new drone position based on previous position, motor
// thrust and the drone orientation (returned by gyroscope)
static void pilotUpdateCurrentGyropose(struct deviceDataData *gg, quat droneOrientation) {
    struct pose			vv;
    int				i, r;
    double			time;
    double			roll, pitch, yaw;
    vec2			dd;
    
    time = currentTime.dtime;
    if (gg == NULL) return;

    //lprintf(0, "%s:  uu->shortHistory.n == %d\n",   PPREFIX(), uu->shortHistory.n);
    // hmm a small hack, if we do not have history yet (start) suppose pose 0,0,0
    if (uu->shortHistoryPose.n <= 1) {
	// return;
	memset(&vv, 0, sizeof(vv));
	vv.time = time;
    } else {
	r = poseHistoryEstimatePoseForTimeByLinearRegression(&uu->shortHistoryPose, time, &vv);
    }
    
    GET_ROLL_PITCH_YAW_FROM_POSE_VECTOR(vv.pr, roll, pitch, yaw);
    // we suppose that the increment in x,y due to roll pitch will be as follows
    dd[0] = GRAVITY_ACCELERATION * -sin(pitch) / uu->stabilization_loop_Hz;
    dd[1] = GRAVITY_ACCELERATION * -sin(roll) / uu->stabilization_loop_Hz;
    vec2Rotate(dd, dd, yaw);
    vec2_add(&vv.pr[0], &vv.pr[0], dd);
    
    poseHistoryAddElem(&gg->history, &vv);
    // give me a small confidence, not to interfere with real devices
    gg->confidence = 0.001;
}

static int pilotCheckDeviceForTimeout(struct deviceDataData *gg) {
    double lastdatatime;
    
    if (gg == NULL) return(-1);
    if (gg->history.n == 0) {
	lprintf(50, "%s: %s.%s: no history\n", PPREFIX(), gg->dd->name, gg->name);
	return(-1);
    }
    lastdatatime = gg->history.a[gg->history.ailast].time;
    // lprintf(0, "timeout check %f < %f %f %f == %f\n", lastdatatime, currentTime.dtime, gg->timeout, gg->latency, currentTime.dtime - (gg->timeout + gg->latency));
    if (lastdatatime < currentTime.dtime - (gg->timeout + gg->latency)) {
	lprintf(50, "%s: %s.%s: timeouted\n", PPREFIX(), gg->dd->name, gg->name);
	return(-1);
    }
    return(0);
}

static double pilotAddCurrentOrientationFromSensorToSum(quat qqsum, struct deviceDataData *gg) {
    struct pose			vv;
    quat			qq;
    int				r;
    double			weightconf;
    
    if (pilotCheckDeviceForTimeout(gg)) return(0);

    r = poseHistoryEstimatePoseForTimeByLinearRegression(&gg->history, currentTime.dtime, &vv);
    if (r) {
	lprintf(50, "%s: Warning: %s.%s pose estimation failed!\n", PPREFIX(), gg->dd->name, gg->name);
    }

    // TODO: simplify, we do not have to copy this
    // TODO: Maybe remove all the quaternion from here. Just take each of RPY angles, treat them as vectors and average them
    memcpy(qq, &vv.pr[3], sizeof(quat));
    if (gg->weight[0] != gg->weight[1] || gg->weight[0] != gg->weight[2]) {
	lprintf(0, "%s: warning: axis different weight do not apply for orientation!\n", PPREFIX());
    }
    weightconf = gg->weight[0] * gg->confidence;
    lprintf(PILOT_SENSOR_MERGE_DEBUG_LEVEL, "%s: add %10s.%-10s: weight %f, confidence %g, orientation %s\n", PPREFIX(), gg->dd->name, gg->name, gg->weight[0], gg->confidence, vecToString_st(qq));
    vec4_scale(qq, qq, weightconf);
    vec4_add(qqsum, qqsum, qq);
    return(weightconf);
}

static double pilotAddCurrentPositionFromSensorToSum(vec3 ppsum, vec3 weightsum, struct deviceDataData *gg, quat droneOrientation) {
    struct pose			vv;
    double				*devicePos;
    vec3				mm, dronepos, pbase;
    quat				ii;
    int					r;
    
    // take sensor's positions, use the orientation to translate it to the drone position
    // (based on sensor mount points) 

    // if (pilotCheckDeviceDataDataType(gg, DDT_VECTOR, 3)) return(-1);
    if (pilotCheckDeviceForTimeout(gg)) return(-1);

    r = poseHistoryEstimatePoseForTimeByLinearRegression(&gg->history, currentTime.dtime, &vv);
    if (r) {
	lprintf(50, "%s: Warning: %s.%s pose estimation failed!\n", PPREFIX(), gg->dd->name, gg->name);
    }

    // TODO: we do not need to copy it here
    memcpy(pbase, &vv.pr[0], sizeof(pbase));
    
    // translate from mount point to drone center
    quat_inverse(ii, droneOrientation);
    quat_mul_vec3(mm, ii, gg->dd->mount_position);
    vec3_add(dronepos, mm, pbase);
    vec3_sub(dronepos, dronepos, gg->dd->mount_position);
    
    lprintf(PILOT_SENSOR_MERGE_DEBUG_LEVEL,"%s: add %10s.%-10s: \tweights: %s, confidence %f, position: %s --> %s\n", PPREFIX(), gg->dd->name, gg->name, vecToString_st(gg->weight), gg->confidence, arrayWithDimToStr_st(vv.pr, 3), vecToString_st(dronepos));
    dronepos[0] *= gg->weight[0] * gg->confidence;
    dronepos[1] *= gg->weight[1] * gg->confidence;
    dronepos[2] *= gg->weight[2] * gg->confidence;
    vec3_add(ppsum, ppsum, dronepos);
    weightsum[0] += gg->weight[0] * gg->confidence;
    weightsum[1] += gg->weight[1] * gg->confidence;
    weightsum[2] += gg->weight[2] * gg->confidence;
    return(0);
}

static double pilotAddCurrentGroundDistanceFromSensorToSum(vec3 ppsum, vec3 weightsum, struct deviceDataData *gg, quat droneOrientation) {
    struct pose			pp, *vv;
    double			alt;
    int				r;
    
    // this is basically adding altitude from down radar

    // {static int cc = 0; if (cc++ % 300 != 0) return(-1);}
    
    if (pilotCheckDeviceForTimeout(gg)) return(-1);

    r = poseHistoryEstimatePoseForTimeByLinearRegression(&gg->history, currentTime.dtime, &pp);

    vv = &pp;
    // radar is getting vec1, which is [distance]
    if (vv->pr[2] < gg->min_range) return(-1);
    if (vv->pr[2] > gg->max_range) return(-1);

    
    // lprintf(0, "%s: Error: down radar not yet implemented\n", PPREFIX());

    // TODO: translate distance to altitude based on drone orientation
    alt = vv->pr[2];
    if (gg->launchPoseSetFlag) alt -= gg->launchPose.pr[2];
    
    ppsum[2] += alt * gg->weight[2];
    weightsum[2] += gg->weight[2];

    lprintf(PILOT_SENSOR_MERGE_DEBUG_LEVEL, "%s: add %10s.%-10s: \tweights: [0,0,%g], confidence %f, distance: %f --> [0,0,%f]\n", PPREFIX(), gg->dd->name, gg->name, gg->weight[2], 1.0, vv->pr[2], alt);
    
    return(0);
}

static double pilotAddCurrentOrientationFromDataTypeToSum(quat qqsum, int datatype) {
    struct deviceDataData 	*ddl;
    double			w;
    assert(datatype > DT_NONE && datatype < DT_MAX);
    w = 0;
    for(ddl=uu->deviceDataDataByType[datatype]; ddl!=NULL; ddl=ddl->nextWithSameType) {
	w += pilotAddCurrentOrientationFromSensorToSum(qqsum, ddl);
    }
    return(w);
}

static void pilotAddCurrentPositionFromDataTypeToSum(vec3 ppsum, vec3 weightsum, quat orientation, int datatype) {
    struct deviceDataData *ddl;
    
    for(ddl=uu->deviceDataDataByType[datatype]; ddl!=NULL; ddl=ddl->nextWithSameType) {
	pilotAddCurrentPositionFromSensorToSum(ppsum, weightsum, ddl, orientation);
    }
}


static int pilotCombineCurrentOrientationFromSensors(quat orientation) {
    double			w;
    quat			qqsum;
    int				i, j;
    
    // We are using quanternion representation to compute the average of reported orientations

    w = 0;
    memset(qqsum, 0, sizeof(quat));

    lprintf(PILOT_SENSOR_MERGE_DEBUG_LEVEL, "%s: Going to merge orientation from sensors\n", PPREFIX());

    w += pilotAddCurrentOrientationFromDataTypeToSum(qqsum, DT_ORIENTATION_QUATERNION);
    w += pilotAddCurrentOrientationFromDataTypeToSum(qqsum, DT_ORIENTATION_RPY_COMPASS);
    
    if (w == 0) {
	memset(orientation, 0, sizeof(quat));
	return(-1);
    }
    quat_scale(orientation, qqsum, 1.0/w);
    quat_safe_norm(orientation, orientation);
    return(0);
}

static int pilotCombineCurrentPositionFromSensors(vec3 position, quat orientation) {
    int				i,j;
    struct deviceData		*dd;
    vec3			ppsum, weightsum;
    struct deviceDataData	*ddl;

    // pilotComputeCurrentGyropose("gyropose", DT_POSITION_VECTOR, orientation);

    lprintf(PILOT_SENSOR_MERGE_DEBUG_LEVEL, "%s: Going to merge position from sensors\n", PPREFIX());

    memset(position, 0, sizeof(vec3));
    memset(ppsum, 0, sizeof(vec3));
    memset(weightsum, 0, sizeof(vec3));

    pilotAddCurrentPositionFromDataTypeToSum(ppsum, weightsum, orientation, DT_POSITION_VECTOR);
    pilotAddCurrentPositionFromDataTypeToSum(ppsum, weightsum, orientation, DT_POSITION_NMEA);
    for(ddl=uu->deviceDataDataByType[DT_GROUND_DISTANCE]; ddl!=NULL; ddl=ddl->nextWithSameType) {
	pilotAddCurrentGroundDistanceFromSensorToSum(ppsum, weightsum, ddl, orientation);
    }
    
    if (weightsum[0] == 0) return(-1);
    if (weightsum[1] == 0) return(-1);
    if (weightsum[2] == 0) return(-1);

    position[0] = ppsum[0]/weightsum[0];
    position[1] = ppsum[1]/weightsum[1];
    position[2] = ppsum[2]/weightsum[2];

    return(0);
}

static int pilotInternalDummyDevicesTick(int dataType, quat orientation) {
    int				i,j;
    struct deviceData		*dd;
    vec3			ppsum, weightsum;
    struct deviceDataData	*ddl;
    
    lprintf(60, "%s: Updating internal position devices\n", PPREFIX());

    // TODO: precompute a list of all internal devices
    for(ddl=uu->deviceDataDataByType[dataType]; ddl!=NULL; ddl=ddl->nextWithSameType) {
	dd = ddl->dd;
	switch (dd->connection.type) {
	case DCT_INTERNAL_ZEROPOSE:
	    pilotUpdateZeropose(ddl, orientation);
	    break;
	case DCT_INTERNAL_GYROPOSE:
	    pilotUpdateCurrentGyropose(ddl, orientation);
	    break;
	default:
	    break;
	}
    }
    
    return(0);
}

////////////

static int pilotGetCurrentPositionAndOrientationFromSensors(struct pose *pp) {
    struct deviceDataData 	*gg;
    quat 			orientation;
    vec3			position;
    int				r;
    double			yaw,pitch,roll;

    pilotInternalDummyDevicesTick(DT_ORIENTATION_QUATERNION, orientation);

    // first get and combine the orientation of the drone and use it to compute position
    r = pilotCombineCurrentOrientationFromSensors(orientation);
    if (r) {
	printf("%s: Error: No sensor providing orientation. Emergency landing!\n", PPREFIX());
	raspilotShutDownAndExit();
    }

    pilotInternalDummyDevicesTick(DT_POSITION_VECTOR, orientation);
    
    r = pilotCombineCurrentPositionFromSensors(position, orientation);
    if (r) {
	printf("%s: Error: No sensor providing position. Emergency landing!\n", PPREFIX());
	raspilotShutDownAndExit();
    }
    quatToYpr(orientation, &yaw, &pitch, &roll);

    lprintf(PILOT_SENSOR_MERGE_DEBUG_LEVEL - 10, "%s: Position from sensors: %s\n", PPREFIX(), vec3ToString_st(position));
    lprintf(PILOT_SENSOR_MERGE_DEBUG_LEVEL, "%s: Orientation from sensors: %s\n", PPREFIX(), quatToString_st(orientation));

    pp->time = currentTime.dtime;
    pp->pr[0] = position[0];
    pp->pr[1] = position[1];
    pp->pr[2] = position[2];
    pp->pr[3] = orientation[0];
    pp->pr[4] = orientation[1];
    pp->pr[5] = orientation[2];
    pp->pr[6] = orientation[3];
    pp->pr[7] = roll;
    pp->pr[8] = pitch;
    pp->pr[9] = yaw;
    
    return(roll);
}

////////////

void pilotStoreLaunchPose(void *d) {
    int				i, j;
    struct deviceData 		*dd;
    struct deviceDataData 	*ddd;

    lprintf(6, "%s: Storing launch pose.\n", PPREFIX());
    for(i=0; i<uu->deviceMax; i++) {
	dd = uu->device[i];
	for(j=0; j<dd->ddtMax; j++) {
	    ddd = dd->ddt[j];
	    poseHistoryGetMean(&ddd->history, &ddd->launchPose);
	    ddd->launchPoseSetFlag = 1;
	    lprintf(23, "%s: %s.%s launch pose %s.\n", PPREFIX(), dd->name, ddd->name, vecToString_st(ddd->launchPose.pr));
	}
    }
}

static int pilotGetDronePositionAndVelocityAndStoreInShortHistory() {
    struct pose		pose, previousPose, velo;
    struct pose		*cpose;
    int				r;
    double			ddt, tdTick;

    tdTick = 1.0/uu->stabilization_loop_Hz;
    // before storing the current pose, get the previous pose which we will use to compute velocity
    poseHistoryEstimatePoseForTimeByLinearRegression(&uu->shortHistoryPose, currentTime.dtime-tdTick, &previousPose);

    cpose = &pose;
    // move average from sensors to get the current pose their report
    r = pilotGetCurrentPositionAndOrientationFromSensors(cpose);

    // add the pose from sensor to regression buffer and use regression to estimate smooth current position
    poseHistoryAddElem(&uu->shortHistoryPose, cpose);    
    poseHistoryEstimatePoseForTimeByLinearRegression(&uu->shortHistoryPose, currentTime.dtime, &pose);

    // TODO: Implement some realistic restriction so that velocity does not exceed "normal values"
    // and acceleration / deceleration constraints

    // compute velocity from previous regressed pose and the current regressed pose
    ddt = cpose->time - previousPose.time;
    if (ddt < tdTick) ddt = tdTick;	// avoid division by zero
    poseVectorSubstract(&velo, cpose, &previousPose);
    poseVectorScale(&velo, &velo, 1.0/ddt);
    velo.time = currentTime.dtime;

    // put current velocity into a buffer (though velo history is not used in the current version of pilot)
    poseHistoryAddElem(&uu->shortHistoryVelo, &velo);

    return(r);
}

// Normalization is called when pilot requires a thrust out of bonds 0..1
// It means that either we need to rotate in opposite direction or more thrust than motor is able to provide.
// Anyway it means a problem which can not be resolved 100% safe. So, in order not to crash severly, we try 
// to put thrust into the range 0..1 in some way
static int pilotNormalizeMotorThrust() {
    int 	i;
    double 	d, dmin, dmax, avg, t, nt, tmin, tmax, tsum, shift;

    tmin = DBL_MAX; tmax = -DBL_MAX; tsum = 0;
    for(i=0; i<uu->motor_number; i++) {
	t = uu->motor[i].thrust;
	tsum += t;
	if (t < tmin) tmin = t;
	if (t > tmax) tmax = t;
    }
    lprintf(PILOT_THRUST_NORMALIZATION_DEBUG_LEVEL, "%s: Normalizing motor thrusts: ", PPREFIX());
#if 0
    for(i=0; i<uu->motor_number; i++) {
	t = uu->motor[i].thrust;
	nt = truncateToRange(t, 0, 1);
	lprintf(PILOT_THRUST_NORMALIZATION_DEBUG_LEVEL, "%d: %g->%g;  ", i, t, nt);
	uu->motor[i].thrust = nt;
    }
#elif 1
    // This normalizes motor thrust to the range 0..1 in the way that the average thrust is
    // kept. I.e. drone shall hold its altitude but will be destabilized.
    avg = tsum / uu->motor_number;
    if (avg <= 0 || avg >= 1) {
	lprintf(PILOT_THRUST_NORMALIZATION_DEBUG_LEVEL, "\n%s: Error: Thrust can not be normalized average thrust == %f.\n", PPREFIX(), avg);
	if (avg < 0) t = 0;
	else t = 1;
	for(i=0; i<uu->motor_number; i++) uu->motor[i].thrust = t;
	return(-1);
    }

    dmin = dmax = 0;
    if (tmin < 0) dmin = (avg - tmin) / avg;
    if (tmax > 1) dmax = (tmax - avg) / (1.0 - avg);
    d = MAX(dmin, dmax);
    if (d == 0) {
	// Hmm. This shall never happen
	InternalCheck(0);
	return(-1);
    }
    for(i=0; i<uu->motor_number; i++) {
	t = uu->motor[i].thrust;
	nt = avg + (t-avg) / d;
	if (nt <= 0) {
	    // this can be only epsilon due to rounding errors
	    assert(-nt < MOTOR_THRUST_EPSILON);
	    nt = 0.0;
	}
	if (nt >= 1) {
	    // this can be only epsilon due to rounding errors
	    assert(nt-1 < MOTOR_THRUST_EPSILON);
	    nt = 1.0;
	}
	lprintf(PILOT_THRUST_NORMALIZATION_DEBUG_LEVEL, "%d: %g->%g;  ", i, t, nt);
	uu->motor[i].thrust = nt;
    }
    // for(i=0; i<uu->motor_number; i++) assert(uu->motor[i].thrust >= 0 && uu->motor[i].thrust <= 1);
#else
    // This normalizes in the way to keep the difference between thrusts. I.e. drone shall stay stable
    // but may loose altitude.
    if (tmax - tmin > 1.0) {
	lprintf(PILOT_THRUST_NORMALIZATION_DEBUG_LEVEL, "\n%s: Error: Thrust can not be normalized tmax - tmin == %f.\n", PPREFIX(), tmax-tmin);
	for(i=0; i<uu->motor_number; i++) uu->motor[i].thrust = truncateToRange(uu->motor[i].thrust, 0, 1);
	return(-1);
    }
    shift = 0;
    if (tmin < 0) shift = -tmin;
    else if (tmax > 1) shift = tmax-1;
    for(i=0; i<uu->motor_number; i++) {
	t = uu->motor[i].thrust;
	nt = t + shift;
	lprintf(PILOT_THRUST_NORMALIZATION_DEBUG_LEVEL, "%d: %g->%g;  ", i, t, nt);
	uu->motor[i].thrust = nt;
    }
#endif
    lprintf(PILOT_THRUST_NORMALIZATION_DEBUG_LEVEL, "\n");
    return(0);
}

// This is the main function computing the thrust to be applied on each motor to reach/hold the waypoint
static void normalizeMotorThrustIfOutOfRange() {
    int 	i, r;
    double	t, tmin, tmax;
    static int 	normalizationFailedDuringLastSecondCounter;
    static int 	normalizationDuringLastSecondCounter;
    static int 	normalizationRequiredTimeSecond;
    
    tmin = DBL_MAX; tmax = DBL_MIN;
    for(i=0; i<uu->motor_number; i++) {
	t = uu->motor[i].thrust;
	if (t < tmin) tmin = t;
	if (t > tmax) tmax = t;
    }

    // Thrust of each motor shall be between 0 and 1. If it is out of bonds, then we have a problem
    if (normalizationDuringLastSecondCounter > 0 || tmin < 0 || tmax > 1) {
	if (normalizationRequiredTimeSecond != currentTime.sec && normalizationDuringLastSecondCounter != 0) {
	    // This is a warning during normal flight, may be normal during landing
	    // TODO: Do not show it during landing phase
	    if (normalizationFailedDuringLastSecondCounter >= 5 || normalizationDuringLastSecondCounter >= 5) {
		lprintf(0, "%s: Warning: %d thrust overflows during previous second. Maybe too aggressive drone setting.\n",
			PPREFIX(), normalizationDuringLastSecondCounter
		    );
	    }
	    if (normalizationFailedDuringLastSecondCounter >= 5) {
		lprintf(0, "%s: Error: Thrust can't be normalized %d times during previous second! Something is wrong!\n",
			PPREFIX(), normalizationFailedDuringLastSecondCounter
		    );
	    }
	    normalizationDuringLastSecondCounter = 0;
	    normalizationFailedDuringLastSecondCounter = 0;
	}

	if (tmin < 0 || tmax > 1) {
	    normalizationRequiredTimeSecond = currentTime.sec;
	    normalizationDuringLastSecondCounter ++;
	    r = pilotNormalizeMotorThrust();
	    if (r != 0) normalizationFailedDuringLastSecondCounter ++;
	}
    }
}

static void pilotSetMinimalMotorThrust() {
    int 	i;
    double 	t, nt, tmin, shift;

    tmin = DBL_MAX;
    for(i=0; i<uu->motor_number; i++) {
	t = uu->motor[i].thrust;
	if (t < tmin) tmin = t;
    }
    lprintf(PILOT_THRUST_NORMALIZATION_DEBUG_LEVEL, "%s: !!! Seting Minimal motor thrusts: ", PPREFIX());
    if (tmin < 0.0) {
	lprintf(5, "\n%s: Error: Thrust can not be decreased: tmin == %g.\n", PPREFIX(), tmin);
	for(i=0; i<uu->motor_number; i++) uu->motor[i].thrust = truncateToRange(uu->motor[i].thrust, 0, 1);
	return;
    }
    shift = -tmin;
    for(i=0; i<uu->motor_number; i++) {
	t = uu->motor[i].thrust;
	nt = t + shift;
	lprintf(PILOT_THRUST_NORMALIZATION_DEBUG_LEVEL, "%d: %g->%g;  ", i, t, nt);
	uu->motor[i].thrust = nt;
    }
    lprintf(PILOT_THRUST_NORMALIZATION_DEBUG_LEVEL, "\n");
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
// This is the main function computing thrust for motors to stabilize drone and
// move toward the next waypoint.
//
static void pilotComputeMotorThrustsForStabilizationAndWaypoint() {
    int 			i;
    vec3			targetPositionVector, targetPositionDroneFrame;
    double 			roll, pitch, yaw;
    double 			targetRoll, targetPitch, targetYaw;
    double			altitudeThrust, yawThrust, rollThrust, pitchThrust;
    double			tdTick, tdRpyFix;	// td stands for Time Delta
    struct pose		poseEstimatedAtHalfRpyFixTime;
    struct pose		veloEstimatedAtQuartRpyFixTime;
    vec3			movingVelocity;
    vec3			movingVelocityDroneFrame;
    vec3			targetGroundVelocity, targetRelativeVelocity, targetVelocityDroneFrame, diffVelocityDroneFrame;
    struct pose		ccpose, *cpose;
    struct pose		*cvelo;
    double			t;
    double			yawRotationSpeed, rollRotationSpeed, pitchRotationSpeed;
    double			targetYawRotationSpeed, targetRollRotationSpeed, targetPitchRotationSpeed;
    double			verticalSpeed, targetVerticalSpeed;
    double			dmax, dspeed;
    
    // if not enough of data do nothing
    if (uu->shortHistoryPose.n <= 1) return;
    if (uu->shortHistoryVelo.n <= 1) return;

    // Set some time quantums here
    // tdTick is the length of one pilot tick, it is used as PID controllers step.
    tdTick = 1.0/uu->stabilization_loop_Hz;
    // tdRpyFix is the timeframe in which we want to achieve targeted roll, pitch, yaw
    tdRpyFix = uu->config.pilot_reach_goal_orientation_time;
    
    // maximal distance on which we focus
    dmax = uu->config.drone_max_speed * uu->config.pilot_reach_goal_position_time;

    // get current drone pose and velocity
    poseHistoryEstimatePoseForTimeByLinearRegression(&uu->shortHistoryPose, currentTime.dtime, &ccpose);
    cpose = &ccpose;
    cvelo = &uu->shortHistoryVelo.a[uu->shortHistoryVelo.ailast];

    lprintf(30, "%s: cpose Position     : %s\n", PPREFIX(), vec3ToString_st(&cpose->pr[0]));
    lprintf(30, "%s: cvelo Velocity     : %s\n", PPREFIX(), vec3ToString_st(&cvelo->pr[0]));

    // We consider as starting position for our computation the estimated position/orientation at time in tdRpyFix/2.0 seconds,
    // i.e. when we suppose that the new orientation of the drone will start taking effect.
    poseHistoryEstimatePoseForTimeByLinearRegression(&uu->shortHistoryPose, currentTime.dtime + tdRpyFix/2.0, &poseEstimatedAtHalfRpyFixTime);

    // Get current roll, pitch and yaw from current position pose.
    GET_ROLL_PITCH_YAW_FROM_POSE_VECTOR(cpose->pr, roll, pitch, yaw);
    
    // And current velocities and speeds from cvelo pose.
    vec3_assign(movingVelocity, &cvelo->pr[0]);
    rollRotationSpeed = cvelo->pr[7]; 
    pitchRotationSpeed = cvelo->pr[8];
    yawRotationSpeed = cvelo->pr[9];
    
    // pitch - negative == nose down;      positive == nose up
    // roll  - negative == left wing down; positive == left wing up
    // yaw   - positive == rotated counterclockwise (view from up)

    // small hack, suppose that we never travel much more than max_speed to avoid dirty values.
    vec3TruncateToSize(movingVelocity, uu->config.drone_max_speed * 1.9, "movingVelocity");
    // rollRotationSpeed = truncateToRange(rollRotationSpeed, -1.9 * uu->drone_max_rotation_speed, 1.9 * uu->drone_max_rotation_speed);
    // pitchRotationSpeed = truncateToRange(pitchRotationSpeed, -1.9 * uu->drone_max_rotation_speed, 1.9 * uu->drone_max_rotation_speed);
    // yawRotationSpeed = truncateToRange(yawRotationSpeed, -1.9 * uu->drone_max_rotation_speed, 1.9 * uu->drone_max_rotation_speed);


    // Compute the velocity to go. In our simple model, to move to a target point X, the drone first
    // rotates to the goal pitch&roll (during tdRyFix time), then it travels to the middle of
    // the distance while accelerating, then he rotates to "braking" pitch&roll (another tdRpyFix time)
    // and then slowing down to the final speed zero when reaching the point X.

    vec3_sub(targetPositionVector, uu->currentWaypoint.position, poseEstimatedAtHalfRpyFixTime.pr);
    for(i=0; i<3; i++) {
	// Restrict targetPositionVector so that we do not need to go over max size in any direction
	// I think this is here to restrict vertical speed in particular, so that drone does not try to climb/descent
	// disproportionally and then eclipsing the necessity to hold X,Y position.
	targetPositionVector[i] = truncateToRange(targetPositionVector[i], -dmax, dmax);
	
	// Compute the max velocity we want to achieve toward the target point (which will be in the midway to the target).
	targetGroundVelocity[i] = 2.0 * targetPositionVector[i] / uu->config.pilot_reach_goal_position_time;
    }

    // Use PID controller to add "wind" velocity in order to get targetVelocity relative to the air.
    targetRelativeVelocity[0] = targetGroundVelocity[0] + pidControllerStep(&uu->pidX, targetGroundVelocity[0], movingVelocity[0], tdTick);
    targetRelativeVelocity[1] = targetGroundVelocity[1] + pidControllerStep(&uu->pidY, targetGroundVelocity[1], movingVelocity[1], tdTick);
    targetRelativeVelocity[2] = targetGroundVelocity[2];
    
    // Translate everything neccessary to drone frame!
    // Attention: What we currently consider as drone frame is not rotated by drone pitch and roll!!!
    // We compute the target position relative to the drone's position, gravity and drone's yaw.
    // Other way would be to translate it completely to drone frame (including pitch, roll), but then what?

    vec3_sub(targetPositionDroneFrame, uu->currentWaypoint.position, poseEstimatedAtHalfRpyFixTime.pr);
    vec2Rotate(targetPositionDroneFrame, targetPositionDroneFrame, -yaw);
    vec3_assign(movingVelocityDroneFrame, movingVelocity);
    vec2Rotate(movingVelocityDroneFrame, movingVelocityDroneFrame, -yaw);
    vec3_assign(targetVelocityDroneFrame, targetRelativeVelocity);
    vec2Rotate(targetVelocityDroneFrame, targetVelocityDroneFrame, -yaw);

    lprintf(30, "%s: Info: target position w.r.t. drone: %s, velocity to target: %s\n", PPREFIX(), vec3ToString_st(targetPositionDroneFrame), vec3ToString_st(targetVelocityDroneFrame));

    // Truncate target velocity to hold user configured constraints.
    vec3TruncateToSize(targetVelocityDroneFrame, uu->config.drone_max_speed, NULL);
    lprintf(50, "%s: Info: Target velocity normalized: %s\n", PPREFIX(), vec3ToString_st(targetVelocityDroneFrame));	

    // diffVelocityDroneFrame is the velocity we need to focus on.
    vec3_sub(diffVelocityDroneFrame, targetVelocityDroneFrame, movingVelocityDroneFrame);
    
    lprintf(30, "%s: Velocities: current: %s, target: %s, difference: %s\n", PPREFIX(), vecToString_st(movingVelocityDroneFrame), vecToString_st(targetVelocityDroneFrame), vecToString_st(diffVelocityDroneFrame));

    // Infer target pitch and roll from the diffVelocity. The diffVelocity corresponds to the horizontal velocity
    // we need to achieve in tdPositionFix/2.
    // We suppose that the average vertical thrust made by propellers equals to the gravity acceleration (9.8).
    // In normal conditions the drone is holding altitude which is ensured by the altitude PID controller (for the moment, we do not
    // consider cases where we are climbing or descending very fast). So, first compute the imaginative vertical speed
    // that the motor thrust is producing. I.e. the speed at the moment the drone shall have the maximal horizontal
    // velocity (diffVelocity) to the point X.
    // That imaginative speed is proportional to the gravity acceleration:
    dspeed = GRAVITY_ACCELERATION * 0.5 * uu->config.pilot_reach_goal_position_time;
    
    // Target pitch and roll is the angle between "gravity vertical speed" and drone "target horizontal speed".
    // Strictly speaking it is vector between accelerations, but it is not important now.
    targetPitch = - atan2(diffVelocityDroneFrame[0], dspeed);
    targetRoll = - atan2(diffVelocityDroneFrame[1], dspeed);	
    targetYaw = uu->currentWaypoint.yaw;

    // Normalize angles to <-Pi, Pi> range and apply user constraints from the configuration
    targetPitch = normalizeToRange(targetPitch, -M_PI, M_PI);
    targetRoll = normalizeToRange(targetRoll, -M_PI, M_PI);
    targetPitch = truncateToRange(targetPitch, -uu->config.drone_max_inclination, uu->config.drone_max_inclination);
    targetRoll = truncateToRange(targetRoll, -uu->config.drone_max_inclination, uu->config.drone_max_inclination);

    // Compute rotation speed to get to the target roll, pitch, yaw
    // Also apply user constraints from the configuration
    targetPitchRotationSpeed = angleSubstract(targetPitch, pitch) / tdRpyFix;
    targetPitchRotationSpeed =  truncateToRange(targetPitchRotationSpeed, -uu->config.drone_max_rotation_speed, uu->config.drone_max_rotation_speed);
    targetRollRotationSpeed = angleSubstract(targetRoll, roll) / tdRpyFix;
    targetRollRotationSpeed =  truncateToRange(targetRollRotationSpeed, -uu->config.drone_max_rotation_speed, uu->config.drone_max_rotation_speed);
    targetYawRotationSpeed = angleSubstract(targetYaw, yaw) / tdRpyFix;
    targetYawRotationSpeed =  truncateToRange(targetYawRotationSpeed, -uu->config.drone_max_rotation_speed, uu->config.drone_max_rotation_speed);

    lprintf(27, "%s: Info: Orientation: PRY == Current: %f %f %f  --> Target: %f %f %f \n", PPREFIX(), pitch, roll, yaw, targetPitch, targetRoll, targetYaw);
    lprintf(22, "%s: Info: Speeds     : PRY == Current: %f %f %f  --> Target: %f %f %f \n", PPREFIX(), pitchRotationSpeed, rollRotationSpeed, yawRotationSpeed, targetPitchRotationSpeed, targetRollRotationSpeed, targetYawRotationSpeed);

    // Use PID controllers to compute final motor thrusts to achieve target rotation speeds from current rotation speeds
    rollThrust = pidControllerStep(&uu->pidRoll, targetRollRotationSpeed, rollRotationSpeed, tdTick);
    pitchThrust = pidControllerStep(&uu->pidPitch, targetPitchRotationSpeed, pitchRotationSpeed, tdTick);
    yawThrust = pidControllerStep(&uu->pidYaw, targetYawRotationSpeed, yawRotationSpeed, tdTick);

    // finally get the Altitude thrust
    targetVerticalSpeed = targetVelocityDroneFrame[2];
    verticalSpeed = movingVelocityDroneFrame[2];
    altitudeThrust = pidControllerStep(&uu->pidAltitude, targetVerticalSpeed, verticalSpeed, tdTick);
    
    lprintf(22, "%s: Info: Thrust     : Altitude == %f;  PRY == %f %f %f\n", PPREFIX(), altitudeThrust, pitchThrust, rollThrust, yawThrust);

    // Combine everything into thrusts for each motor
    for(i=0; i<uu->motor_number; i++) {
	t = 0;
	t += altitudeThrust;
	t += uu->motor_yaw_forces[i] * yawThrust;
	t += uu->motor_pitch_forces[i] * pitchThrust;
	t += uu->motor_roll_forces[i] * rollThrust;
	uu->motor[i].thrust = t;
    }

    // It may happen that we need to turn motors in reverse direction (negative thrust) or more than the maximal rotation
    // speed of the motor (thrust over 1.0) in order to achieve configured performance. This means that the drone hardware
    // is unable to stabilize/move the drone and satisfy configuration. That is a problem. If this happens the following
    // function shall "normalize" thrust in the way that the drone does not crash immediately.
    normalizeMotorThrustIfOutOfRange();

    // Security exception. Hard prevent the drone to fly higher than drone_max_altitude.
    if (poseEstimatedAtHalfRpyFixTime.pr[2] > uu->config.drone_max_altitude) pilotSetMinimalMotorThrust();

#if CONNECTION_JITTER_TEST
    {
	// This code was used to test latency and throughoutput of the Linux pipe to motors.
	static int nn = 0;
	for(i=0; i<uu->motor_number; i++) uu->motor[i].thrust = ((double)rand())/RAND_MAX;
	if (nn++ < 10) lprintf(0, "%s: Warning: Motors are running at random thrust !!!!!!!!!!!!!!!!!!!!!!!!!!!!! Do not fly !!!!!!!!\n", PPREFIX());
    }
#endif	

    if (debugLevel > 15) {
	lprintf(16, "%s: Thrust: ", PPREFIX());
	for(i=0; i<uu->motor_number; i++) lprintf(16, "%5.3f ", uu->motor[i].thrust);
	lprintf(16, "\n");
    }
}

// This is the main stabilization function executed each PILOT_STABILIZATION_TICK_MSEC
void pilotUpdatePositionHistoryAndRecomputeMotorThrust() {
    int 	r, c;
    
    r = pilotGetDronePositionAndVelocityAndStoreInShortHistory();
    if (r) return;
    
    pilotComputeMotorThrustsForStabilizationAndWaypoint();
}

static int64_t pilotScheduleNextTick(double frequency, void (*tickfunction)(void *arg), void *arg) {
    int64_t 	nextTickUsec;
    
    setCurrentTime();
    nextTickUsec = currentTimeLineTimeUsec + 1000000LL / frequency;
    if (nextTickUsec <= currentTime.usec) {
	lprintf(0, "%s: Warning: Some code took longer than %f ms. Time skewed by %f ms!\n", PPREFIX(), 1000/frequency, (currentTime.usec - nextTickUsec) / 1000.0);
	nextTickUsec = currentTime.usec + 1000000LL / frequency;
    }
    timeLineRescheduleUniqueEvent(nextTickUsec, tickfunction, arg);
    return(nextTickUsec);
}

void pilotRegularStabilizationLoopTick(void *d) {
    lprintf(30, "\n");
    lprintf(100, "%s: Stabilization tick\n", PPREFIX());
    nextStabilizationTickUsec = pilotScheduleNextTick(uu->stabilization_loop_Hz, pilotRegularStabilizationLoopTick, NULL);
    pilotUpdatePositionHistoryAndRecomputeMotorThrust();
    motorsThrustSend(NULL);
}

void pilotRegularPreLaunchTick(void *d) {
    // pre launch tick are running on low frequency because launching pipes, etc. takes time
    pilotScheduleNextTick(PILOT_PRELAUNCH_FREQUENCY_HZ, pilotRegularPreLaunchTick, NULL);
    if (pilotAreAllDevicesReady()) pilotGetDronePositionAndVelocityAndStoreInShortHistory();
    motorsThrustSend(NULL);
}

void pilotRegularSpecialTick(void *d) {
    pilotScheduleNextTick(PILOT_PRELAUNCH_FREQUENCY_HZ, pilotRegularSpecialTick, NULL);
    motorsThrustSend(NULL);
}

void pilotRegularStabilizationLoopRescheduleToSoon() {
    int		r;
    uint64_t	tt;

    tt = currentTime.usec + 500;
    if (nextStabilizationTickUsec > tt) {
	r = timeLineRescheduleUniqueEventIfExisted(tt, pilotRegularStabilizationLoopTick, NULL);
	if (r == 0) nextStabilizationTickUsec = tt;
    }
}

void pilotRegularMotorPing(void *d) {
    struct baio	*bb;

    // do not send anything in shutdown sequence (otherwise motors are awaken)
    if (shutDownInProgress) return;
    
    bb = baioFromMagic(uu->motorBaioMagic);
    if (bb == NULL) return;
    baioPrintfToBuffer(bb, "ping%"PRId64"\n", currentTime.usec);
    lprintf(100, "%s: Pinging motor process\n", PPREFIX());
    timeLineInsertEvent(UTIME_AFTER_MSEC(2022), pilotRegularMotorPing, NULL);
}

void pilotRegularSaveTrajectory(void *d) {
    struct pose ccpose;

    if (uu->shortHistoryPose.n != 0) {
	poseHistoryEstimatePoseForTimeByLinearRegression(&uu->shortHistoryPose, currentTime.dtime, &ccpose);
	trajectoryLogPrintf("%g %g %g  %g %g %g\n", ccpose.pr[0], ccpose.pr[1], ccpose.pr[2], ccpose.pr[7], ccpose.pr[8], ccpose.pr[9]);
    }
    timeLineInsertEvent(TLINE_UTIME_AFTER_MSEC(100), pilotRegularSaveTrajectory, d);
}

void pilotInteractiveInputRegularCheck(void *d) {
    int		c;
    int64_t	nextTickUsec;

    timeLineRescheduleUniqueEvent(UTIME_AFTER_MSEC(100), pilotInteractiveInputRegularCheck, d);

    c = stdbaioStdinMaybeGetPendingChar();
    if (c >= 0) {
	missionProcessInteractiveInput(c);
	stdbaioStdinClearBuffer();
    }

}

int pilotAreAllDevicesReady() {
    static int64_t 	lastWaitingMsgMsec = 0;
    int 		i, j;
    struct deviceData	*dd;
    
    // here we are waiting for all essentials sensors to be initialized and running,
    // then we start autopilot tasks

    for(i=0; i<uu->deviceMax; i++) {
	dd = uu->device[i];

	for(j=0; j<dd->ddtMax; j++) {
	    if (
		(dd->ddt[j]->mandatory && dd->ddt[j]->history.n == 0)
		||
		(uu->pilotLaunchTime + dd->warming_time > currentTime.dtime)
		) {
		// {
		// if (dd->connection.type ras!= DCT_INTERNAL && dd->lastActivityTime <= uu->pilotStartingTime) {
		// this sensor did not send data yet, continue waiting
		if (debugLevel > 0 && currentTime.msec > lastWaitingMsgMsec + 2000) {
		    if (lastWaitingMsgMsec != 0) lprintf(1, "%s: Info: waiting for device: %s.\n", PPREFIX(), dd->name);
		    lastWaitingMsgMsec = currentTime.msec;
		}
		return(0);
	    }
	}
    }
    return(1);
}


