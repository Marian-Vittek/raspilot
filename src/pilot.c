
#include "common.h"

void motorsThrustSend(void *d) {
    int 		i;
    struct baio		*bb;
    double		thrust;

    // Hmm. I need to stop motors. do not send anything in shutdown sequence (otherwise motors are awaken)
    if (shutDownInProgress) return;

    lprintf(49, "%s: Motor thrust: ", PPREFIX());
    for(i=0; i<uu->motor_number; i++) lprintf(49, "%5.3f ", uu->motor[i].thrust);
    lprintf(49, "\n");

    for(i=0; i<uu->motor_number; i++) {
	thrust = uu->motor[i].thrust ;
	if (thrust >= 0) {
	    uu->motor[i].rotationSpeed = sqrt(thrust); 			// c1 + c2 * sqrt(thrust);
	} else {
	    // Those negative thrusts makes the drone shaking too much, it is unusable for stabilization
	    uu->motor[i].rotationSpeed = - sqrt(- thrust) / 100.0; 		// c3 - c4 * sqrt(-thrust);
	}
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
	baioPrintfToBuffer(bb, " %.4g", uu->motor[i].rotationSpeed);
    }
    baioPrintfToBuffer(bb, "\n");
    uu->motorLastSendTime = currentTime.dtime;

    lprintf(39, "%s: Sent motor rotation speed: ", PPREFIX());
    for(i=0; i<uu->motor_number; i++) lprintf(39, "%g ", uu->motor[i].rotationSpeed);
    lprintf(39, "\n");

}

void motorsStandby(void *d) {
    struct baio	*bb;
    bb = baioFromMagic(uu->motorBaioMagic);
    if (bb == NULL) return;
    baioPrintfToBuffer(bb, "stan\n");
}

void motorsExit(void *d) {
    struct baio	*bb;
    bb = baioFromMagic(uu->motorBaioMagic);
    if (bb == NULL) return;
    baioPrintfToBuffer(bb, "exit\n");
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
    int i;
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
    writeToFd(bb->wfd, command, strlen(command));
    usleep(10000);
}

void motorsEmmergencyLand() {
    motorsEmergencySendSpecialCommand("\nland\n");
}

void motorsEmmergencyShutdown() {
    motorsEmergencySendSpecialCommand("\nstop\n");
}

void pilotImmediateLanding() {
    vec3	cpose;
    
    sendToAllDevices("info: landing\n");
    // go strait down on interactive down 
    regressionBufferEstimatePoseForTimeByLinearRegression(&uu->shortHistoryPosition, currentTime.dtime, cpose);
    raspilotLand(cpose[0], cpose[1]);
    raspilotShutDownAndExit();
}

//////////////////////////////////////////////////////////////////////////////////////

// This function always return position 0,0,0
// It is used when we need to only stabilize the drone
static void pilotUpdateZeropose(struct deviceDataData *gg, vec3 rpy) {
    vec3			vv;
    int				i, r;
    double			time;
    double			roll, pitch, yaw;
    vec2			dd;
    
    if (gg == NULL) return;

    assert(gg->dataHistory.vectorsize == 3);
    memset(&vv, 0, sizeof(vv));
    regressionBufferAddElem(&gg->dataHistory, currentTime.dtime, vv);
    // give me a small confidence, not to interfere with real devices
    gg->confidence = 1e-300;
}

static int pilotCheckDeviceForTimeout(struct deviceDataData *gg) {
    double lastdatatime;
    
    if (gg == NULL) return(-1);
    if (gg->dataHistory.n == 0) {
	lprintf(50, "%s: %s.%s: no history\n", PPREFIX(), gg->dd->name, gg->name);
	return(-1);
    }
    lastdatatime = gg->dataHistory.time[gg->dataHistory.ailast];
    // lprintf(0, "timeout check %f < %f %f %f == %f\n", lastdatatime, currentTime.dtime, gg->timeout, gg->latency, currentTime.dtime - (gg->timeout + gg->latency));
    if (lastdatatime < currentTime.dtime - (gg->timeout + gg->latency)) {
	lprintf(50, "%s: %s.%s: timeouted\n", PPREFIX(), gg->dd->name, gg->name);
	return(-1);
    }
    return(0);
}

static void pilotAddCurrentQuatOrientationFromSensorToSum(quat qqsum, vec4 weightsum, struct deviceDataData *gg) {
    struct pose			vv;
    quat			qq;
    int				r;
    vec4			ww;
    double			weightconf;
    
    if (pilotCheckDeviceForTimeout(gg)) return;
    
    r = regressionBufferEstimatePoseForTimeByLinearRegression(&gg->dataHistory, currentTime.dtime, qq);
    if (r) {
	lprintf(50, "%s: Warning: %s.%s pose estimation failed!\n", PPREFIX(), gg->dd->name, gg->name);
    }

    lprintf(PILOT_SENSOR_MERGE_DEBUG_LEVEL, "%s: add %s from %-20s %-20s: quat: weight %s, confidence %g\n", PPREFIX(), vec4ToString_st(qq), gg->dd->name, gg->name, vec4ToString_st(gg->weight), gg->confidence);

    vec4_scale(ww, gg->weight, gg->confidence);
    vec4_mul_elem(qq, qq, ww);
    vec4_add(qqsum, qqsum, qq);
    vec4_add(weightsum, weightsum, ww);
}

static void pilotAddCurrentRpyOrientationFromSensorToSum(vec3 sinsum, vec3 cossum, vec3 weightsum, struct deviceDataData *gg) {
    vec3			rpy;
    vec3			rpysin, rpycos;
    int				i, r;
    vec3			ww;
    double			weightconf;
    
    if (pilotCheckDeviceForTimeout(gg)) return;
    
    r = regressionBufferEstimatePoseForTimeByLinearRegression(&gg->dataHistory, currentTime.dtime, rpy);
    if (r) {
	lprintf(50, "%s: Warning: %s.%s pose estimation failed!\n", PPREFIX(), gg->dd->name, gg->name);
    }

    for(i=0; i<3; i++) {
	rpysin[i] = sin(rpy[i]);
	rpycos[i] = cos(rpy[i]);
    }
    
    lprintf(PILOT_SENSOR_MERGE_DEBUG_LEVEL, "%s: add %s from %-20s %-20s rpy: weight %s, confidence %g\n", PPREFIX(), vec3ToString_st(rpy), gg->dd->name, gg->name, vec3ToString_st(gg->weight), gg->confidence);

    vec3_scale(ww, gg->weight, gg->confidence);
    vec3_mul_elem(rpysin, rpysin, ww);
    vec3_mul_elem(rpycos, rpycos, ww);
    vec3_add(sinsum, sinsum, rpysin);
    vec3_add(cossum, cossum, rpycos);
    vec3_add(weightsum, weightsum, ww);
}

static void pilotAddCurrentPositionFromSensorToSum(vec3 ppsum, vec3 weightsum, struct deviceDataData *gg, quat droneOrientation) {
    struct pose		vv;
    double		*devicePos;
    vec3		mm, ww, dronepos, pbase;
    quat		ii;
    int			r;
    
    // take sensor's positions, use the orientation to translate it to the drone position
    // (based on sensor mount points) 

    // if (pilotCheckDeviceDataDataType(gg, DDT_VECTOR, 3)) return(-1);
    if (pilotCheckDeviceForTimeout(gg)) return;

    r = regressionBufferEstimatePoseForTimeByLinearRegression(&gg->dataHistory, currentTime.dtime, pbase);
    if (r) {
	lprintf(50, "%s: Warning: %s.%s pose estimation failed!\n", PPREFIX(), gg->dd->name, gg->name);
    }

    // translate from mount point to drone center
    quat_inverse(ii, droneOrientation);
    quat_mul_vec3(mm, ii, gg->dd->mount_position);
    vec3_add(dronepos, mm, pbase);
    //lprintf(PILOT_SENSOR_MERGE_DEBUG_LEVEL,"%s: --> %s\n", PPREFIX(), vecToString_st(dronepos));
    vec3_sub(dronepos, dronepos, gg->dd->mount_position);
    //lprintf(PILOT_SENSOR_MERGE_DEBUG_LEVEL,"%s: --> %s\n", PPREFIX(), vecToString_st(dronepos));
    
    if (gg->launchPoseSetFlag) {
	vec3_sub(dronepos, dronepos, gg->launchData);
    } else {
	memset(dronepos, 0, sizeof(dronepos));
    }
    
    lprintf(PILOT_SENSOR_MERGE_DEBUG_LEVEL,"%s: add %s from %-20s %-20s %s:  \tweights: %s, confidence %f\n", PPREFIX(), vecToString_st(dronepos), gg->dd->name, gg->name, vecToString_st(pbase), vecToString_st(gg->weight), gg->confidence);

    vec3_scale(ww, gg->weight, gg->confidence);
    vec3_mul_elem(dronepos, dronepos, ww);
    vec3_add(ppsum, ppsum, dronepos);
    vec3_add(weightsum, weightsum, ww);
    return;
}

static void pilotAddCurrentGroundDistanceFromSensorToSum(vec3 ppsum, vec3 weightsum, struct deviceDataData *gg, quat droneOrientation) {
    double			alt;
    int				r;
    
    // this is basically adding altitude from down radar

    // {static int cc = 0; if (cc++ % 300 != 0) return(-1);}
    
    if (pilotCheckDeviceForTimeout(gg)) return;

    r = regressionBufferEstimatePoseForTimeByLinearRegression(&gg->dataHistory, currentTime.dtime, &alt);

    // radar is getting vec1, which is [distance]
    if (alt < gg->min_range) return;
    if (alt > gg->max_range) return;
    
    // lprintf(0, "%s: Error: down radar not yet implemented\n", PPREFIX());

    // TODO: translate distance to altitude based on drone orientation
    if (gg->launchPoseSetFlag) {
	alt -= gg->launchData[0];
    } else {
	alt = 0;
    }
    
    ppsum[2] += alt * gg->weight[2];
    weightsum[2] += gg->weight[2];

    lprintf(PILOT_SENSOR_MERGE_DEBUG_LEVEL, "%s: add [0,0,%f] from %-20s %-20s: distance %f \tweight: %g, confidence %f\n", PPREFIX(), alt, gg->dd->name, gg->name,alt,  gg->weight[2], 1.0);
    
    return;
}

static void pilotAddCurrentAltitudeFromSensorToSum(vec3 ppsum, vec3 weightsum, struct deviceDataData *gg, quat droneOrientation) {
    double			alt;
    int				r;
    
    // this is basically adding altitude from down radar

    // {static int cc = 0; if (cc++ % 300 != 0) return(-1);}
    
    if (pilotCheckDeviceForTimeout(gg)) return;

    r = regressionBufferEstimatePoseForTimeByLinearRegression(&gg->dataHistory, currentTime.dtime, &alt);

    if (gg->launchPoseSetFlag) {
	alt -= gg->launchData[0];
    } else {
	alt = 0;
    }
    
    ppsum[2] += alt * gg->weight[2];
    weightsum[2] += gg->weight[2];

    lprintf(PILOT_SENSOR_MERGE_DEBUG_LEVEL, "%s: add [0,0,%f] from %-20s %-20s: altitude %f: \tweights: [0,0,%g], confidence %f\n", PPREFIX(), alt, gg->dd->name, gg->name, alt,  gg->weight[2], 1.0);
    
    return;
}


static void pilotAddQuatOrientation(quat qqsum, vec4 w, int datatype, void mapfun(quat, vec4, struct deviceDataData*)) {
    struct deviceDataData 	*ddl;

    assert(datatype > DT_NONE && datatype < DT_MAX);
    for(ddl=uu->deviceDataDataByType[datatype]; ddl!=NULL; ddl=ddl->nextWithSameType) {
	mapfun(qqsum, w, ddl);
    }
}

static void pilotAddRpyOrientation(vec3 rpysinsum, vec3 rpycossum, vec3 w, int datatype, void mapfun(vec3, vec3, vec3, struct deviceDataData*)) {
    struct deviceDataData 	*ddl;

    assert(datatype > DT_NONE && datatype < DT_MAX);
    for(ddl=uu->deviceDataDataByType[datatype]; ddl!=NULL; ddl=ddl->nextWithSameType) {
	mapfun(rpysinsum, rpycossum, w, ddl);
    }
}

static void pilotAddPosition(vec3 ppsum, vec3 weightsum, quat orientation, int datatype, void mapfun(vec3, vec3, struct deviceDataData *, quat)) {
    struct deviceDataData *ddl;
    
    assert(datatype > DT_NONE && datatype < DT_MAX);
    for(ddl=uu->deviceDataDataByType[datatype]; ddl!=NULL; ddl=ddl->nextWithSameType) {
	mapfun(ppsum, weightsum, ddl, orientation);
    }
}


static int pilotCombineCurrentOrientationFromSensors(quat orientation) {
    vec4			qq, qqweight;
    vec3			rpyweight;
    quat			qqsum;
    vec3			rpy, rpysinsum, rpycossum;
    quat			rpyq;
    int				i, j;
    int				qqwHasZero, rpywHasZero;
    double			rpyw, qqw;
    
    // We are using quanternion representation to compute the average of reported orientations

    memset(qqweight, 0, sizeof(qqweight));
    memset(rpyweight, 0, sizeof(rpyweight));
    memset(qqsum, 0, sizeof(qqsum));
    memset(rpysinsum, 0, sizeof(rpysinsum));
    memset(rpycossum, 0, sizeof(rpycossum));

    lprintf(PILOT_SENSOR_MERGE_DEBUG_LEVEL, "%s: Going to merge orientation from sensors\n", PPREFIX());

    // Go through all sensors contributing to the orientation, either RPY or Quaternion
    pilotAddQuatOrientation(qqsum, qqweight, DT_ORIENTATION_QUATERNION, pilotAddCurrentQuatOrientationFromSensorToSum);
    pilotAddRpyOrientation(rpysinsum, rpycossum, rpyweight, DT_ORIENTATION_RPY, pilotAddCurrentRpyOrientationFromSensorToSum);

    qqwHasZero = vec4_has_zero(qqweight);
    rpywHasZero = vec3_has_zero(rpyweight);

    // get average roll, pitch, yaw
    if (rpywHasZero) {
	memset(rpyq, 0, sizeof(rpyq));
	rpyw = 0;
    } else {
	for(i=0; i<3; i++) {
	    rpy[i] = atan2(rpysinsum[i]/rpyweight[i], rpycossum[i]/rpyweight[i]);
	}
	yprToQuat(rpy[2], rpy[1], rpy[0], rpyq);
	rpyw = vec3_len(rpyweight);
	vec4_scale(rpyq, rpyq, rpyw);	
    }

    // get average quaternion
    if (qqwHasZero) {
	memset(qq, 0, sizeof(qq));
	qqw = 0;
    } else {
	for(i=0; i<4; i++) {
	    qq[i] = qqsum[i] / qqweight[i];
	}
	quat_safe_norm(qq, qq);
	qqw = vec4_len(qqweight);
	vec4_scale(qq, qq, qqw);
    }

    // Now merge quat (in qq) and rpy (in rpyq) based on their weights
    memset(orientation, 0, sizeof(quat));
    //lprintf(PILOT_SENSOR_MERGE_DEBUG_LEVEL - 10, "%s: ----- Orientation from sensors: %s\n", PPREFIX(), quatToString_st(orientation));
    vec4_add(orientation, orientation, rpyq);
    //lprintf(PILOT_SENSOR_MERGE_DEBUG_LEVEL - 10, "%s: ----- Orientation from sensors: %s\n", PPREFIX(), quatToString_st(orientation));
    vec4_add(orientation, orientation, qq);
    //lprintf(PILOT_SENSOR_MERGE_DEBUG_LEVEL - 10, "%s: ----- Orientation from sensors: %s\n", PPREFIX(), quatToString_st(orientation));
    if (rpyw+qqw != 0) vec4_scale(orientation, orientation, 1.0/(rpyw+qqw));
    //lprintf(PILOT_SENSOR_MERGE_DEBUG_LEVEL - 10, "%s: ----- Orientation from sensors: %s\n", PPREFIX(), quatToString_st(orientation));
    quat_safe_norm(orientation, orientation);
    
    //lprintf(PILOT_SENSOR_MERGE_DEBUG_LEVEL - 10, "%s: ----- Orientation from sensors: %s\n", PPREFIX(), quatToString_st(orientation));
    return(0);
}

static int pilotCombineCurrentPositionFromSensors(vec3 position, quat orientation) {
    int				i,j;
    struct deviceData		*dd;
    vec3			ppsum, weightsum;

    // pilotComputeCurrentGyropose("gyropose", DT_POSITION_VECTOR, orientation);

    lprintf(PILOT_SENSOR_MERGE_DEBUG_LEVEL, "%s: Going to merge position from sensors\n", PPREFIX());

    memset(position, 0, sizeof(vec3));
    memset(ppsum, 0, sizeof(vec3));
    memset(weightsum, 0, sizeof(vec3));

    // Go through all sensors contributing to the position
    pilotAddPosition(ppsum, weightsum, orientation, DT_POSITION_VECTOR, pilotAddCurrentPositionFromSensorToSum);
    pilotAddPosition(ppsum, weightsum, orientation, DT_POSITION_NMEA, pilotAddCurrentPositionFromSensorToSum);
    pilotAddPosition(ppsum, weightsum, orientation, DT_GROUND_DISTANCE, pilotAddCurrentGroundDistanceFromSensorToSum);
    pilotAddPosition(ppsum, weightsum, orientation, DT_ALTITUDE, pilotAddCurrentAltitudeFromSensorToSum);

    if (vec3_has_zero(weightsum)) return(-1);
    for(i=0; i<3; i++) {
	position[i] = ppsum[i]/weightsum[i];
    }

    return(0);
}

static int pilotInternalDummyDevicesTick(int dataType, vec3 rpy) {
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
	    pilotUpdateZeropose(ddl, rpy);
	    break;
	default:
	    break;
	}
    }
    
    return(0);
}

////////////

static void pilotGetCurrentPositionAndOrientationFromSensors(vec3 position, vec3 rpy) {
    struct deviceDataData 	*gg;
    int				r;
    quat			orientation;
    double			yaw,pitch,roll;

    pilotInternalDummyDevicesTick(DT_ORIENTATION_QUATERNION, rpy);

    // first get and combine the orientation of the drone and use it to compute position
    r = pilotCombineCurrentOrientationFromSensors(orientation);
    if (r) {
	printf("%s: Error: No sensor providing orientation. Emergency landing!\n", PPREFIX());
	raspilotShutDownAndExit();
    }

    pilotInternalDummyDevicesTick(DT_POSITION_VECTOR, rpy);
    
    r = pilotCombineCurrentPositionFromSensors(position, orientation);
    if (r) {
	printf("%s: Error: No sensor providing position. Emergency landing!\n", PPREFIX());
	raspilotShutDownAndExit();
    }
    quatToYpr(orientation, &yaw, &pitch, &roll);
    rpy[0] = roll;
    rpy[1] = pitch;
    rpy[2] = yaw;
    
    lprintf(PILOT_SENSOR_MERGE_DEBUG_LEVEL - 10, "%s: Fused Position    from sensors: %s\n", PPREFIX(), vec3ToString_st(position));
    lprintf(PILOT_SENSOR_MERGE_DEBUG_LEVEL - 10, "%s: Fused Orientation from sensors: %s; quat: %s\n", PPREFIX(), vec3ToString_st(rpy), quatToString_st(orientation));
}

////////////

void pilotStoreLaunchPose(void *d) {
    int				i, j;
    double			meanTime;
    struct deviceData 		*dd;
    struct deviceDataData 	*ddd;

    lprintf(6, "%s: Storing launch poses.\n", PPREFIX());
    for(i=0; i<uu->deviceMax; i++) {
	dd = uu->device[i];
	for(j=0; j<dd->ddtMax; j++) {
	    ddd = dd->ddt[j];
	    regressionBufferGetMean(&ddd->dataHistory, &meanTime, ddd->launchData);
	    ddd->launchPoseSetFlag = 1;
	    // lprintf(23, "%s: %s.%s launch pose %s.\n", PPREFIX(), dd->name, ddd->name, vecToString_st(ddd->launchPose.pr));
	    lprintf(23, "%s: %s.%s stored data %s from %d values\n", PPREFIX(), dd->name, ddd->name, arrayWithDimToStr_st(ddd->launchData, ddd->dataHistory.vectorsize), MIN(ddd->dataHistory.size, ddd->dataHistory.n));
	}
    }
}

static void pilotGetDronePositionAndVelocityAndStoreInShortHistory() {
    vec3		smoothPosition;
    vec3		smoothRpy;
    vec3		sensorFusionPosition;
    vec3		sensorFusionRpy;
    vec3		velocity;
    vec3		rpyRotationSpeed;
    double		ddt, tdTick;

    
    // move average from sensors to get the current pose their report
    pilotGetCurrentPositionAndOrientationFromSensors(sensorFusionPosition, sensorFusionRpy);

    // add the pose from sensor to regression buffer and use regression to estimate smooth current position
    regressionBufferAddElem(&uu->shortHistoryPosition, currentTime.dtime, sensorFusionPosition);    
    regressionBufferAddElem(&uu->shortHistoryRpy, currentTime.dtime, sensorFusionRpy);
    regressionBufferEstimatePoseForTimeByLinearRegression(&uu->shortHistoryPosition, currentTime.dtime, smoothPosition);
    regressionBufferEstimatePoseForTimeByLinearRegression(&uu->shortHistoryRpy, currentTime.dtime, smoothRpy);

    // compute velocity from previous regressed pose and the current regressed pose
    tdTick = 1.0/uu->stabilization_loop_Hz;
    ddt = currentTime.dtime - uu->droneLastTickTime;
    if (ddt < tdTick) ddt = tdTick;
    
    vec3_sub(velocity, smoothPosition, uu->droneLastPosition);
    vec3_scale(velocity, velocity, 1.0/ddt);
    vec3_sub(rpyRotationSpeed, smoothRpy, uu->droneLastRpy);
    vec3_scale(rpyRotationSpeed, rpyRotationSpeed, 1.0/ddt);

    // TODO: Implement some realistic restriction so that velocity does not exceed "normal values"
    // and acceleration / deceleration constraints

    vec3_assign(uu->droneLastPosition, smoothPosition);
    vec3_assign(uu->droneLastRpy, smoothRpy);
    vec3_assign(uu->droneLastVelocity, velocity);
    vec3_assign(uu->droneLastRpyRotationSpeed, rpyRotationSpeed);
    uu->droneLastTickTime = currentTime.dtime;
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
#if 1
    for(i=0; i<uu->motor_number; i++) {
	t = uu->motor[i].thrust;
	if (uu->config.motor_bidirectional) {
	    nt = truncateToRange(t, -1, 1, NULL);
	} else {
	    nt = truncateToRange(t, 0, 1, NULL);
	}
	lprintf(PILOT_THRUST_NORMALIZATION_DEBUG_LEVEL, "%d: %g->%g;  ", i, t, nt);
	uu->motor[i].thrust = nt;
    }
#elif 0
    // This normalizes motor thrust to the range 0..1 in the way that the average thrust is
    // kept. I.e. drone shall hold its altitude but will be destabilized.
    if (uu->motor_bidirectional) {
	lprintf(0, "Not yet implemented !!!");
	exit(-1);
    }
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
    // but may loose altitude or climb too much.
    if (uu->motor_bidirectional) {
	lprintf(0, "Not yet implemented !!!");
	exit(-1);
    }
    if (tmax - tmin > 1.0) {
	lprintf(PILOT_THRUST_NORMALIZATION_DEBUG_LEVEL, "\n%s: Error: Thrust can not be normalized tmax - tmin == %f.\n", PPREFIX(), tmax-tmin);
    }
    shift = 0;
    if (tmax > 1) shift = 1-tmax;
    else if (tmin < 0) shift = -tmin;
    for(i=0; i<uu->motor_number; i++) {
	t = uu->motor[i].thrust;
	nt = t + shift;
	lprintf(PILOT_THRUST_NORMALIZATION_DEBUG_LEVEL, "%d: %g->%g;  ", i, t, nt);
	uu->motor[i].thrust = truncateToRange(nt, 0, 1);
    }
#endif
    lprintf(PILOT_THRUST_NORMALIZATION_DEBUG_LEVEL, "\n");
    return(0);
}

// This is the main function computing the thrust to be applied on each motor to reach/hold the waypoint
static void normalizeMotorThrustIfOutOfRange() {
    int 	i, r;
    double	t, tmin, tmax;
    double	motormin, motormax;
    
    static int 	normalizationFailedDuringLastSecondCounter;
    static int 	normalizationDuringLastSecondCounter;
    static int 	normalizationRequiredTimeSecond;

    motormax = 1;
    if (uu->config.motor_bidirectional) {
	motormin = -1;
    } else {
	motormin = 0;
    }
    
    tmin = DBL_MAX; tmax = DBL_MIN;
    for(i=0; i<uu->motor_number; i++) {
	t = uu->motor[i].thrust;
	if (t < tmin) tmin = t;
	if (t > tmax) tmax = t;
    }

    // Thrust of each motor shall be between 0 and 1. If it is out of bonds, then we have a problem
    if (normalizationDuringLastSecondCounter > 0 || tmin < motormin || tmax > motormax) {
	if (normalizationRequiredTimeSecond != currentTime.sec && normalizationDuringLastSecondCounter != 0) {
	    // This is a warning during normal flight, may be normal during landing
	    // TODO: Do not show it during landing phase
	    if (normalizationFailedDuringLastSecondCounter >= 5 || normalizationDuringLastSecondCounter >= 5) {
		lprintf(0, "%s: Error:   %3d thrust overflows during previous second. Maybe too aggressive drone setting!\n",
			PPREFIX(), normalizationDuringLastSecondCounter
		    );
	    }
	    if (normalizationFailedDuringLastSecondCounter >= 1) {
		lprintf(0, "%s: Error:   Thrust can't be normalized %d times during previous second!\n",
			PPREFIX(), normalizationFailedDuringLastSecondCounter
		    );
	    }
	    normalizationDuringLastSecondCounter = 0;
	    normalizationFailedDuringLastSecondCounter = 0;
	}

	if (tmin < motormin || tmax > motormax) {
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
	for(i=0; i<uu->motor_number; i++) uu->motor[i].thrust = truncateToRange(uu->motor[i].thrust, 0, 1, NULL);
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

static void maybeNormalizeMovingVelocity(vec3 movingVelocity) {
    static int 		r = 0;
    static time_t 	lrt = 0;
    
    // normalize separately XY speed and altitude. Altitude sensors are usually less precise.
    r += vec2TruncateToSize(movingVelocity, 9.9 * uu->config.drone_max_speed, 0, "movingVelocity");
    r += vec1TruncateToSize(&movingVelocity[2], 2.9 * uu->config.drone_max_speed, 0, "movingVerticalVelocity");
    if (currentTime.sec != lrt) {
	if (r >= 5) lprintf(5, "%s: Warning: %3d velocity truncations in last second. Maybe wrong position sensor?\n", PPREFIX(), r);
	lrt = currentTime.sec;
	r = 0;
    }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
// This is the main function computing thrust for motors to stabilize drone and
// move toward the next waypoint.
//

static void pilotComputeMotorThrustsForStabilizationAndWaypoint() {
    int 		i, r;
    vec3		targetPositionVector, targetPositionDroneFrame;
    double 		roll, pitch, yaw;
    double 		targetRoll, targetPitch, targetYaw;
    double		altitudeThrust, yawThrust, rollThrust, pitchThrust;
    double		tdTick, tdRpyFix;	// td stands for Time Delta
    vec3		posEstimatedAtHalfRpyFixTime;
    vec3		movingVelocity;
    vec3		movingVelocityDroneFrame;
    vec3		targetGroundVelocity, targetRelativeVelocity, targetVelocityDroneFrame, diffVelocityDroneFrame;
    vec3		cRpy;
    vec3		cveloRpy;
    double		t;
    double		yawRotationSpeed, rollRotationSpeed, pitchRotationSpeed;
    double		targetYawRotationSpeed, targetRollRotationSpeed, targetPitchRotationSpeed;
    double		verticalSpeed, targetVerticalSpeed;
    double		dmax, dspeed;
    
    // if not enough of data do nothing
    if (uu->shortHistoryPosition.n <= 2) return;
    if (uu->shortHistoryRpy.n <= 2) return;

    // Set some time quantums here
    // tdTick is the length of one pilot tick, it is used as PID controllers step.
    tdTick = 1.0/uu->stabilization_loop_Hz;
    // tdRpyFix is the timeframe in which we want to achieve targeted roll, pitch, yaw
    tdRpyFix = uu->config.pilot_reach_goal_orientation_time;
    
    // maximal distance on which we focus
    dmax = uu->config.drone_max_speed * uu->config.pilot_reach_goal_position_time;

    // get current drone pose and velocity
    regressionBufferEstimatePoseForTimeByLinearRegression(&uu->shortHistoryRpy, currentTime.dtime, cRpy);
    
    vec3_assign(movingVelocity, uu->droneLastVelocity);
    vec3_assign(cveloRpy, uu->droneLastRpyRotationSpeed);

    // We consider as starting position for our computation the estimated position/orientation at time in tdRpyFix/2.0 seconds,
    // i.e. when we suppose that the new orientation of the drone will start taking effect.
    regressionBufferEstimatePoseForTimeByLinearRegression(&uu->shortHistoryPosition, currentTime.dtime + tdRpyFix/2.0, posEstimatedAtHalfRpyFixTime);

    lprintf(30, "%s: Regression from History: Position     : %s\n", PPREFIX(), vec3ToString_st(posEstimatedAtHalfRpyFixTime));
    lprintf(30, "%s: Regression from History: Velocity     : %s\n", PPREFIX(), vec3ToString_st(movingVelocity));

    // Get current roll, pitch and yaw from current position pose.
    // pitch - negative == nose down;      positive == nose up
    // roll  - negative == left wing down; positive == left wing up
    // yaw   - positive == rotated counterclockwise (view from up)

    roll  = cRpy[0];
    pitch = cRpy[1];
    yaw   = cRpy[2];
    
    rollRotationSpeed  = cveloRpy[0];
    pitchRotationSpeed = cveloRpy[1];
    yawRotationSpeed   = cveloRpy[2];

    // small hack, suppose that we never travel much more than max_speed to avoid dirty values.
    maybeNormalizeMovingVelocity(movingVelocity);
    
    rollRotationSpeed = truncateToRange(rollRotationSpeed, -2.9 * uu->config.drone_max_rotation_speed, 2.9 * uu->config.drone_max_rotation_speed, "rollRotationSpeed");
    pitchRotationSpeed = truncateToRange(pitchRotationSpeed, -2.9 * uu->config.drone_max_rotation_speed, 2.9 * uu->config.drone_max_rotation_speed, "pitchRotationSpeed");
    yawRotationSpeed = truncateToRange(yawRotationSpeed, -2.9 * uu->config.drone_max_rotation_speed, 2.9 * uu->config.drone_max_rotation_speed, "yawRotationSpeed");

    // Compute the velocity to go. In our simple model, to move to a target point X, the drone first
    // rotates to the goal pitch&roll (during tdRyFix time), then it travels to the middle of
    // the distance while accelerating, then he rotates to "braking" pitch&roll (another tdRpyFix time)
    // and then slowing down to the final speed zero when reaching the point X.
    // TODO: The model is not very accurate. If we are nearly in good orientation it will not take as
    // much to get the orientation. Those times shall be infered from current orientation, rotation speed
    // and drone max rotation speed. For both acceleration and braking.
    
    vec3_sub(targetPositionVector, uu->currentWaypoint.position, posEstimatedAtHalfRpyFixTime);
    for(i=0; i<3; i++) {
	// Restrict targetPositionVector so that we do not need to go over max size in any direction
	// I think this is here to restrict vertical speed in particular, so that drone does not try to climb/descent
	// disproportionally and then eclipsing the necessity to hold X,Y position.
	targetPositionVector[i] = truncateToRange(targetPositionVector[i], -dmax, dmax, NULL);
	
	// Compute the max velocity we want to achieve toward the target point (which will be in the midway to the target).
	targetGroundVelocity[i] = 2.0 * targetPositionVector[i] / uu->config.pilot_reach_goal_position_time;
    }

    // Use PID controller to add "wind" velocity in order to get targetVelocity relative to the air.
    targetRelativeVelocity[0] = targetGroundVelocity[0] + pidControllerStep(&uu->pidX, targetGroundVelocity[0], movingVelocity[0], tdTick);
    targetRelativeVelocity[1] = targetGroundVelocity[1] + pidControllerStep(&uu->pidY, targetGroundVelocity[1], movingVelocity[1], tdTick);
    targetRelativeVelocity[2] = targetGroundVelocity[2];
    
    // lprintf(30, "%s: Info: target position vector %s, targetGroundVelocity: %s, targetRelativeVelocity: %s\n", PPREFIX(), vec3ToString_st(targetPositionVector), vec3ToString_st(targetGroundVelocity), vec3ToString_st(targetRelativeVelocity));
    
    // Translate everything neccessary to drone frame!
    // Attention: What we currently consider as drone frame is not rotated by drone pitch and roll!!!
    // We compute the target position relative to the drone's position, gravity and drone's yaw.
    // Other possibility would be to translate it completely to drone frame (including pitch, roll), but then what?

    vec3_sub(targetPositionDroneFrame, uu->currentWaypoint.position, posEstimatedAtHalfRpyFixTime);
    vec2Rotate(targetPositionDroneFrame, targetPositionDroneFrame, -yaw);
    vec3_assign(movingVelocityDroneFrame, movingVelocity);
    vec2Rotate(movingVelocityDroneFrame, movingVelocityDroneFrame, -yaw);
    vec3_assign(targetVelocityDroneFrame, targetRelativeVelocity);
    vec2Rotate(targetVelocityDroneFrame, targetVelocityDroneFrame, -yaw);

    lprintf(60, "%s: Info: target position w.r.t. drone: %s, velocity to target: %s\n", PPREFIX(), vec3ToString_st(targetPositionDroneFrame), vec3ToString_st(targetVelocityDroneFrame));

    // Truncate target velocity to hold user configured constraints.
    vec3TruncateToSize(targetVelocityDroneFrame, uu->config.drone_max_speed, 0, NULL);
    lprintf(60, "%s: Info: Target velocity normalized: %s\n", PPREFIX(), vec3ToString_st(targetVelocityDroneFrame));	

    // diffVelocityDroneFrame is the velocity we need to focus on.
    vec3_sub(diffVelocityDroneFrame, targetVelocityDroneFrame, movingVelocityDroneFrame);
    
    lprintf(30, "%s: XYZ Velocities:  Current: %s  --> Target: %s,  Difference: %s\n", PPREFIX(), vecToString_st(movingVelocityDroneFrame), vecToString_st(targetVelocityDroneFrame), vecToString_st(diffVelocityDroneFrame));

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
    targetPitch = truncateToRange(targetPitch, -uu->config.drone_max_inclination, uu->config.drone_max_inclination, NULL);
    targetRoll = truncateToRange(targetRoll, -uu->config.drone_max_inclination, uu->config.drone_max_inclination, NULL);

    // Compute rotation speed to get to the target roll, pitch, yaw
    // Also apply user constraints from the configuration
    targetPitchRotationSpeed = angleSubstract(targetPitch, pitch) / tdRpyFix;
    targetPitchRotationSpeed =  truncateToRange(targetPitchRotationSpeed, -uu->config.drone_max_rotation_speed, uu->config.drone_max_rotation_speed, NULL);
    targetRollRotationSpeed = angleSubstract(targetRoll, roll) / tdRpyFix;
    targetRollRotationSpeed =  truncateToRange(targetRollRotationSpeed, -uu->config.drone_max_rotation_speed, uu->config.drone_max_rotation_speed, NULL);
    targetYawRotationSpeed = angleSubstract(targetYaw, yaw) / tdRpyFix;
    targetYawRotationSpeed =  truncateToRange(targetYawRotationSpeed, -uu->config.drone_max_rotation_speed, uu->config.drone_max_rotation_speed, NULL);

    lprintf(27, "%s: RPY Orientation: Current: [%7.3f %7.3f %7.3f]  --> Target: [%7.3f %7.3f %7.3f]\n", PPREFIX(), roll, pitch, yaw, targetRoll, targetPitch, targetYaw);
    lprintf(22, "%s: RPY Speed:       Current: [%7.3f %7.3f %7.3f]  --> Target: [%7.3f %7.3f %7.3f]\n", PPREFIX(), rollRotationSpeed, pitchRotationSpeed, yawRotationSpeed, targetRollRotationSpeed, targetPitchRotationSpeed, targetYawRotationSpeed);

    // Use PID controllers to compute final motor thrusts to achieve target rotation speeds from current rotation speeds
    rollThrust = pidControllerStep(&uu->pidRoll, targetRollRotationSpeed, rollRotationSpeed, tdTick);
    pitchThrust = pidControllerStep(&uu->pidPitch, targetPitchRotationSpeed, pitchRotationSpeed, tdTick);
    yawThrust = pidControllerStep(&uu->pidYaw, targetYawRotationSpeed, yawRotationSpeed, tdTick);

    // finally get the Altitude thrust
    targetVerticalSpeed = targetVelocityDroneFrame[2];
    verticalSpeed = movingVelocityDroneFrame[2];
    altitudeThrust = pidControllerStep(&uu->pidAltitude, targetVerticalSpeed, verticalSpeed, tdTick);
    
    lprintf(22, "%s: PID Thrusts:    Altitude == %f;  RPY == %f %f %f\n", PPREFIX(), altitudeThrust, rollThrust, pitchThrust, yawThrust);

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

    // Security exception. Hard prevent the drone to fly much higher than drone_max_altitude.
    if (posEstimatedAtHalfRpyFixTime[2] > uu->config.drone_max_altitude + 1.0) pilotSetMinimalMotorThrust();

#if CONNECTION_JITTER_TEST
    {
	// This code was used to test latency and throughoutput of the Linux pipe to motors.
	static int nn = 0;
	for(i=0; i<uu->motor_number; i++) uu->motor[i].thrust = ((double)rand())/RAND_MAX;
	if (nn++ < 10) lprintf(0, "%s: Warning: Motors are running at random thrust !!!!!!!!!!!!!!!!!!!!!!!!!!!!! Do not fly !!!!!!!!\n", PPREFIX());
    }
#endif	

    lprintf(16, "%s: Thrust per motor: ", PPREFIX());
    for(i=0; i<uu->motor_number; i++) lprintf(16, "%5.3f ", uu->motor[i].thrust);
    lprintf(16, "\n");
}

// This is the main stabilization function executed each PILOT_STABILIZATION_TICK_MSEC
void pilotUpdatePositionHistoryAndRecomputeMotorThrust() {
    int 	r, c;
    
    pilotGetDronePositionAndVelocityAndStoreInShortHistory();
    pilotComputeMotorThrustsForStabilizationAndWaypoint();
}

static int64_t pilotScheduleNextTick(double frequency, void (*tickfunction)(void *arg), void *arg) {
    static int 		lts = 0;
    static int 		skewN=0;
    static double	skewSum=0;
    int64_t 		nextTickUsec, skew;
    
    setCurrentTime();
    nextTickUsec = currentTimeLineTimeUsec + 1000000LL / frequency;
    if (nextTickUsec <= currentTime.usec) {
	skew = (currentTime.usec - nextTickUsec);
	// lprintf(0, "%s: Warning: Some code took longer than %f ms. Time skewed by %f ms!\n", PPREFIX(), 1000/frequency, (currentTime.usec - nextTickUsec) / 1000.0);
	skewSum += skew / 1000.0;
	skewN ++;
	nextTickUsec = currentTime.usec + 1000000LL / frequency;
	if (lts != currentTime.sec) {
	    if (skewN != 0) lprintf(0, "%s: Warning: %d time skews in the last second. Average skew: %f ms. Total clock skew %f ms!\n", PPREFIX(), skewN, skewSum/skewN, skewSum);
	    skewN = 0;
	    skewSum = 0;
	    lts = currentTime.sec;
	}
    }
    timeLineRescheduleUniqueEvent(nextTickUsec, tickfunction, arg);
    return(nextTickUsec);
}

void pilotRegularStabilizationLoopTick(void *d) {
    // lprintf(30, "\n");
    lprintf(100, "%s: Stabilization tick\n", PPREFIX());
    nextStabilizationTickUsec = pilotScheduleNextTick(uu->stabilization_loop_Hz, pilotRegularStabilizationLoopTick, NULL);
    pilotUpdatePositionHistoryAndRecomputeMotorThrust();
    motorsThrustSend(NULL);
}

void pilotRegularPreLaunchTick(void *d) {
    // pre launch tick are running on low frequency because launching pipes, etc. takes time
    // but this makes that we have a smaller number of samples
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

void pilotSetMotors3dMode(void *d) {
    struct baio	*bb;

    // do not send anything in shutdown sequence (otherwise motors are awaken)
    if (shutDownInProgress) return;
    
    bb = baioFromMagic(uu->motorBaioMagic);
    if (bb == NULL) return;
    baioPrintfToBuffer(bb, "3d1\n");
    lprintf(10, "%s: Setting motors to 3d.\n", PPREFIX());
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
    vec3	cpos;
    vec3	crpy;

    if (uu->shortHistoryPosition.n != 0 && uu->shortHistoryRpy.n != 0) {
	regressionBufferEstimatePoseForTimeByLinearRegression(&uu->shortHistoryPosition, currentTime.dtime, cpos);
	regressionBufferEstimatePoseForTimeByLinearRegression(&uu->shortHistoryRpy, currentTime.dtime, crpy);
	trajectoryLogPrintf("%g %g %g  %g %g %g\n", cpos[0], cpos[1], cpos[2], crpy[0], crpy[1], crpy[2]);
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
		(dd->ddt[j]->mandatory && dd->ddt[j]->dataHistory.n == 0)
		||
		(uu->pilotLaunchTime + dd->warming_time > currentTime.dtime)
		) {
		// {
		// if (dd->connection.type ras!= DCT_INTERNAL && dd->lastActivityTime <= uu->pilotStartingTime) {
		// this sensor did not send data yet, continue waiting
		if (debugLevel > 0 && currentTime.msec > lastWaitingMsgMsec + 2000) {
		    if (lastWaitingMsgMsec != 0) {
			lprintf(0, "%s: Info: waiting for device: %s.", PPREFIX(), dd->name);
			if (uu->pilotLaunchTime + dd->warming_time > currentTime.dtime) {
			    lprintf(0, " Remains: %2d seconds.", (int)(uu->pilotLaunchTime + dd->warming_time - currentTime.dtime));
			}
			lprintf(0, "\n");
		    }
		    lastWaitingMsgMsec = currentTime.msec;
		}
		return(0);
	    }
	}
    }
    return(1);
}


