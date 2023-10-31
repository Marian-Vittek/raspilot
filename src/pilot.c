#include "common.h"


void motorSendThrustsToStream(struct deviceStreamData *ddl) {
    int 		i, j, r;
    struct baio		*bb;
    double		thrust;
    char		ttt[TMP_STRING_SIZE];
    
    // Hmm. I need to stop motors. do not send anything in shutdown sequence (otherwise motors are awaken)
    if (shutDownInProgress) return;
    if (ddl == NULL) return;
    
    lprintf(39, "%s: Motor thrust: ", PPREFIX());
    for(i=0; i<uu->motor_number; i++) lprintf(39, "%5.3f ", uu->motor[i].thrust);
    lprintf(39, "\n");

    for(i=0; i<uu->motor_number; i++) {
	// For statistics
	// Hmm. when flying too long, this will not add anything, because of the big difference.
	// For a long fly values will be biased.
	uu->motor[i].totalWork += uu->motor[i].lastSentThrust * (currentTime.dtime - uu->motorLastSendTime);
    }
    
    bb = baioFromMagic(ddl->dd->baioMagic);
    if (bb == NULL) return;

    j = 0;
    j += snprintf(ttt+j, TMP_STRING_SIZE-j, "%s", ddl->tag);
    for(i=0; i<uu->motor_number; i++) {
	uu->motor[i].lastSentThrust = uu->motor[i].thrust;
	j += snprintf(ttt+j, TMP_STRING_SIZE-j, " %d", (int)(uu->motor[i].thrust * MOTOR_STREAM_THRUST_FACTOR));
    }
    j += snprintf(ttt+j, TMP_STRING_SIZE-j, "\n");
    if (j >= TMP_STRING_SIZE) {
	lprintf(0, "%s: Error: Command to write to motors too long.\n", PPREFIX());
	return;
    }
    r = baioWriteToBuffer(bb, ttt, j);
    if (r != j) {
	lprintf(0, "%s: Error: Can't write to motor pipe. Probably buffer full.\n", PPREFIX());
	return;
    }
    uu->motorLastSendTime = currentTime.dtime;
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
    pilotSendThrusts(NULL);    
}

void motorsThrustSetAndSend(double thrust) {
    motorThrustSetAndSend(MOTORS_ALL, thrust);
    pilotSendThrusts(NULL);    
}

void motorsStop(void *d) {
    int i;
    motorsThrustSetAndSend(0);
}

void motorsStandby(void *d) {
    struct deviceStreamData 	*ddl;
    struct baio			*bb;

    // Hmm. Do this nicer by adding some special datatype instead of DT_THRUST
    for(ddl=uu->deviceStreamDataByType[DT_THRUST]; ddl!=NULL; ddl=ddl->nextWithSameType) {
	bb = baioFromMagic(ddl->dd->baioMagic);
	if (bb != NULL) {
	    baioPrintfToBuffer(bb, "stan\n");
	}
    }
}

void motorsExit(void *d) {
    struct deviceStreamData 	*ddl;
    struct baio			*bb;

    // Hmm. Do this nicer by adding some special datatype instead of DT_THRUST
    for(ddl=uu->deviceStreamDataByType[DT_THRUST]; ddl!=NULL; ddl=ddl->nextWithSameType) {
	bb = baioFromMagic(ddl->dd->baioMagic);
	if (bb != NULL) {
	    baioPrintfToBuffer(bb, "exit\n");
	}
    }
}

void motorsSendStreamThrustFactor(void *d) {
    struct deviceStreamData 	*ddl;
    struct baio			*bb;

    for(ddl=uu->deviceStreamDataByType[DT_THRUST]; ddl!=NULL; ddl=ddl->nextWithSameType) {
	bb = baioFromMagic(ddl->dd->baioMagic);
	if (bb != NULL) {
	    baioPrintfToBuffer(bb, "mfac %g\n", (double)MOTOR_STREAM_THRUST_FACTOR);
	}
    }
}



// This is an emergency function which does not use baio library
// and sends command directly to motor fd, it is used in from interrupt handler
// supposing raspilot task crashed and is in inconsistent state.
void motorsEmergencySendSpecialCommand(char *command) {
    struct deviceStreamData 	*ddl;
    struct baio			*bb;

    // Hmm. Do this nicer by adding some special datatype instead of DT_THRUST
    for(ddl=uu->deviceStreamDataByType[DT_THRUST]; ddl!=NULL; ddl=ddl->nextWithSameType) {
	bb = baioFromMagic(ddl->dd->baioMagic);
	if (bb != NULL && bb->wfd >= 0) {
	    writeToFd(bb->wfd, command, strlen(command));
	}
    }
    usleep(10000);
}

void motorsEmmergencyLand() {
    motorsEmergencySendSpecialCommand("\nland\n");
}

void motorsEmmergencyShutdown() {
    motorsEmergencySendSpecialCommand("\nstop\n");
}

//////////////////////////////////////////////////////////////////////////////////////


void pilotSendThrusts(void *d) {
    struct deviceStreamData 	*ddl;

    // do not send anything in shutdown sequence (otherwise motors are awaken)
    if (shutDownInProgress) return;
    
    for(ddl=uu->deviceStreamDataByType[DT_THRUST]; ddl!=NULL; ddl=ddl->nextWithSameType) {
	motorSendThrustsToStream(ddl);
    }
}

void pilotImmediateLanding() {
    vec3	cpose;
    
    deviceSendToAllDevices("info: landing\n");
    // go strait down on interactive down 
    regressionBufferEstimateForTime(&uu->longBufferPosition, currentTime.dtime, cpose);
    raspilotLand(cpose[0], cpose[1]);
    raspilotShutDownAndExit();
}

//////////////////////////////////////////////////////////////////////////////////////

// This function always return position 0,0,0
// It is used when we need to only stabilize the drone
static void pilotUpdateZeropose(struct deviceStreamData *gg) {
    vec3			vv;
    
    if (gg == NULL || gg->input == NULL) return;

    assert(gg->outputBuffer.vectorsize == 3);
    memset(&vv, 0, sizeof(vv));
    raspilotRingBufferAddElem(&gg->input->buffer, currentTime.dtime, vv);
    // give me a small confidence, not to interfere with real devices
    gg->input->confidence = 1e-50;
}

static int pilotCheckDeviceForTimeout(struct deviceStreamData *gg) {
    double lastdatatime;
    
    if (gg == NULL) return(-1);
    if (gg->input == NULL) {
	lprintf(50, "%s: %s.%s: no input buffer\n", PPREFIX(), gg->dd->name, gg->name);
	return(-1);
    }
    if (gg->input->buffer.n == 0) {
	lprintf(50, "%s: %s.%s: no history\n", PPREFIX(), gg->dd->name, gg->name);
	return(-1);
    }
    //raspilotRingBufferDump(&gg->input->buffer);
    lastdatatime = gg->input->buffer.a[gg->input->buffer.ailast * (gg->input->buffer.vectorsize+1)];
    // lprintf(0, "timeout check %f < %f %f %f == %f\n", lastdatatime, currentTime.dtime, gg->timeout, gg->latency, currentTime.dtime - (gg->timeout + gg->latency));
    if (lastdatatime < currentTime.dtime - (gg->timeout + gg->latency)) {
	lprintf(50, "%s: %s.%s: timeouted\n", PPREFIX(), gg->dd->name, gg->name);
	return(-1);
    }
    return(0);
}

#if 0
static void pilotAddCurrentQuatOrientationFromSensorToSum(quat qqsum, vec4 weightsum, struct deviceStreamData *gg) {
    quat			qq;
    int				r;
    vec4			ww;
    
    deviceTranslateInputToOutput(gg);
    if (pilotCheckDeviceForTimeout(gg)) return;
    
    r = regressionBufferEstimateForTime(&gg->outputBuffer, currentTime.dtime, qq);
    if (r) {
	lprintf(50, "%s: Warning: %s.%s pose estimation failed!\n", PPREFIX(), gg->dd->name, gg->name);
    }

    // TODO: yaw during launch shall be zero, so rotate the quaternion by launch yaw

    lprintf(PILOT_SENSOR_MERGE_DEBUG_LEVEL, "%s: add %s from %-20s %-20s: quat: weight %s, confidence %g\n", PPREFIX(), vec4ToString_st(qq), gg->dd->name, gg->name, vec4ToString_st(gg->weight), gg->confidence);

    vec4_scale(ww, gg->weight, gg->confidence);
    vec4_mul_elem(qq, qq, ww);
    vec4_add(qqsum, qqsum, qq);
    vec4_add(weightsum, weightsum, ww);
}
#endif

static void pilotAddCurrentRpyOrientationFromSensorToSum(vec3 sinsum, vec3 cossum, vec3 weightsum, struct deviceStreamData *gg) {
    vec3			rpy;
    vec3			rpysin, rpycos;
    int				i, r;
    vec3			ww;
    
    deviceTranslateInputToOutput(gg);
    if (pilotCheckDeviceForTimeout(gg)) return;
    
    r = regressionBufferEstimateForTime(&gg->outputBuffer, currentTime.dtime, rpy);
    if (r) {
	lprintf(50, "%s: Warning: %s.%s pose estimation failed!\n", PPREFIX(), gg->dd->name, gg->name);
    }

    if (! gg->launchPoseSetFlag) {
	memset(rpy, 0, sizeof(rpy));
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

void deviceTranslateSensorPositionToDronePositionFromQuaternion(vec3 sensorPosition, struct deviceData *dd, quat droneOrientation, vec3 resDronePosition) {
    quat 	ii;
    vec3	mm;
    
    // translate from mount point to drone center of gravity
    quat_inverse(ii, droneOrientation);
    quat_mul_vec3(mm, ii, dd->mount_position);
    vec3_add(resDronePosition, mm, sensorPosition);
    //lprintf(PILOT_SENSOR_MERGE_DEBUG_LEVEL,"%s: --> %s\n", PPREFIX(), vecToString_st(resDronePosition));
    vec3_sub(resDronePosition, resDronePosition, dd->mount_position);
    //lprintf(PILOT_SENSOR_MERGE_DEBUG_LEVEL,"%s: --> %s\n", PPREFIX(), vecToString_st(resDronePosition));
}

static void pilotAddPositionFromSensorToSum(vec3 ppsum, vec3 weightsum, struct deviceStreamData *gg) {
    vec3		ww, dronepos;
    int			r;

    // take sensor's positions, use the orientation to translate it to the drone position
    // (based on sensor mount points) 

    deviceTranslateInputToOutput(gg);
    if (pilotCheckDeviceForTimeout(gg)) return;

    r = regressionBufferEstimateForTime(&gg->outputBuffer, currentTime.dtime, dronepos);
    if (r) {
	lprintf(50, "%s: Warning: %s.%s pose estimation failed!\n", PPREFIX(), gg->dd->name, gg->name);
    }

    if (! gg->launchPoseSetFlag) {
	memset(dronepos, 0, sizeof(dronepos));
    }
    
    lprintf(PILOT_SENSOR_MERGE_DEBUG_LEVEL,"%s: add %s from %-20s %-20s \tweights: %s, confidence %f\n", PPREFIX(), vecToString_st(dronepos), gg->dd->name, gg->name, vec3ToString_st(gg->weight), gg->confidence);

    vec3_scale(ww, gg->weight, gg->confidence);
    vec3_mul_elem(dronepos, dronepos, ww);
    vec3_add(ppsum, ppsum, dronepos);
    vec3_add(weightsum, weightsum, ww);
    return;
}

static void pilotAddXYfromSensorToSum(vec3 ppsum, vec3 weightsum, struct deviceStreamData *gg) {
    double			xy[2];
    int				r;
    
    deviceTranslateInputToOutput(gg);
    if (pilotCheckDeviceForTimeout(gg)) return;

    r = regressionBufferEstimateForTime(&gg->outputBuffer, currentTime.dtime, xy);
    if (r) {
	lprintf(50, "%s: Warning: %s.%s flow pose estimation failed!\n", PPREFIX(), gg->dd->name, gg->name);
    }

    // TODO: Do not even call the position/orientation if launch pose is not set and device is timeouted: for ALL devices!!
    if (! gg->launchPoseSetFlag) {
	memset(xy, 0, sizeof(xy));
    }
    
    ppsum[0] += xy[0] * gg->confidence * gg->weight[0];
    ppsum[1] += xy[1] * gg->confidence * gg->weight[1];
    weightsum[0] += gg->confidence * gg->weight[0];
    weightsum[1] += gg->confidence * gg->weight[1];

    lprintf(PILOT_SENSOR_MERGE_DEBUG_LEVEL, "%s: add [%f,%f,0] from %-20s %-20s: weight: %s, confidence %f\n", PPREFIX(), xy[0], xy[1], gg->dd->name, gg->name, vec2ToString_st(gg->weight), 1.0);
    
    return;
}

static void pilotAddAltitudeFromSensorToSum(vec3 ppsum, vec3 weightsum, struct deviceStreamData *gg) {
    double			alt;
    int				r;
    
    deviceTranslateInputToOutput(gg);
    if (pilotCheckDeviceForTimeout(gg)) return;

    r = regressionBufferEstimateForTime(&gg->outputBuffer, currentTime.dtime, &alt);

    if (! gg->launchPoseSetFlag) {
	alt = 0;
    }
    
    ppsum[2] += alt * gg->weight[2];
    weightsum[2] += gg->weight[2];

    lprintf(PILOT_SENSOR_MERGE_DEBUG_LEVEL, "%s: add [0,0,%f] from %-20s %-20s: altitude %f: \tweights: [0,0,%g], confidence %f\n", PPREFIX(), alt, gg->dd->name, gg->name, alt,  gg->weight[2], 1.0);
    
    return;
}

#if 0
static void pilotAddQuatOrientation(quat qqsum, vec4 w, int datatype, void mapfun(quat, vec4, struct deviceStreamData*)) {
    struct deviceStreamData 	*ddl;

    assert(datatype > DT_NONE && datatype < DT_MAX);
    for(ddl=uu->deviceStreamDataByType[datatype]; ddl!=NULL; ddl=ddl->nextWithSameType) {
	mapfun(qqsum, w, ddl);
    }
}
#endif

static void pilotAddRpyOrientation(vec3 rpysinsum, vec3 rpycossum, vec3 w, int datatype, void mapfun(vec3, vec3, vec3, struct deviceStreamData*)) {
    struct deviceStreamData 	*ddl;

    assert(datatype > DT_NONE && datatype < DT_MAX);
    for(ddl=uu->deviceStreamDataByType[datatype]; ddl!=NULL; ddl=ddl->nextWithSameType) {
	mapfun(rpysinsum, rpycossum, w, ddl);
    }
}

static void pilotAddPosition(vec3 ppsum, vec3 weightsum, int datatype, void mapfun(vec3, vec3, struct deviceStreamData *)) {
    struct deviceStreamData *ddl;
    
    assert(datatype > DT_NONE && datatype < DT_MAX);
    for(ddl=uu->deviceStreamDataByType[datatype]; ddl!=NULL; ddl=ddl->nextWithSameType) {
	mapfun(ppsum, weightsum, ddl);
    }
}


static int pilotCombineCurrentOrientationFromSensors(vec3 rpy) {
    //vec4			qq, qqweight;
    vec3			rpyweight;
    quat			qqsum;
    vec3			rpysinsum, rpycossum;
    //quat			rpyq;
    int				i;
    int				qqwHasZero, rpywHasZero;
    
    // We are using quanternion representation to compute the average of reported orientations

    //memset(qqweight, 0, sizeof(qqweight));
    memset(rpyweight, 0, sizeof(rpyweight));
    //memset(qqsum, 0, sizeof(qqsum));
    memset(rpysinsum, 0, sizeof(rpysinsum));
    memset(rpycossum, 0, sizeof(rpycossum));

    lprintf(PILOT_SENSOR_MERGE_DEBUG_LEVEL, "%s: Going to merge orientation from sensors\n", PPREFIX());

    // Go through all sensors contributing to the orientation, either RPY or Quaternion
    // TODO: Probably completely remove quaternions for the sake of simplicity
    // pilotAddQuatOrientation(qqsum, qqweight, DT_ORIENTATION_QUATERNION, pilotAddCurrentQuatOrientationFromSensorToSum);
    pilotAddRpyOrientation(rpysinsum, rpycossum, rpyweight, DT_ORIENTATION_RPY_SHM, pilotAddCurrentRpyOrientationFromSensorToSum);
    pilotAddRpyOrientation(rpysinsum, rpycossum, rpyweight, DT_ORIENTATION_RPY, pilotAddCurrentRpyOrientationFromSensorToSum);

    //qqwHasZero = vec4_has_zero(qqweight);
    rpywHasZero = vec3_has_zero(rpyweight);

    // get average roll, pitch, yaw
    for(i=0; i<3; i++) {
	// avoid division by zero
	if (rpyweight[i] == 0) {
	    rpy[i] = 0;
	} else {
	    rpy[i] = atan2(rpysinsum[i]/rpyweight[i], rpycossum[i]/rpyweight[i]);
	}
    }

    return(0);
}

static int pilotCombineCurrentPositionFromSensors(vec3 position) {
    int				i;
    vec3			ppsum, weightsum;

    // pilotComputeCurrentGyropose("gyropose", DT_POSITION_VECTOR, orientation);

    lprintf(PILOT_SENSOR_MERGE_DEBUG_LEVEL, "%s: Going to merge position from sensors\n", PPREFIX());

    memset(position, 0, sizeof(vec3));
    memset(ppsum, 0, sizeof(vec3));
    memset(weightsum, 0, sizeof(vec3));
    
    // Go through all sensors contributing to the position
    pilotAddPosition(ppsum, weightsum, DT_POSITION_SHM, pilotAddPositionFromSensorToSum);
    pilotAddPosition(ppsum, weightsum, DT_POSITION_VECTOR, pilotAddPositionFromSensorToSum);
    pilotAddPosition(ppsum, weightsum, DT_POSITION_NMEA, pilotAddPositionFromSensorToSum);
    pilotAddPosition(ppsum, weightsum, DT_FLOW_XY, pilotAddXYfromSensorToSum);
    pilotAddPosition(ppsum, weightsum, DT_BOTTOM_RANGE, pilotAddAltitudeFromSensorToSum);
    pilotAddPosition(ppsum, weightsum, DT_ALTITUDE, pilotAddAltitudeFromSensorToSum);

    if (vec3_has_zero(weightsum)) return(-1);
    for(i=0; i<3; i++) {
	position[i] = ppsum[i]/weightsum[i];
    }

    return(0);
}

static int pilotInternalDummyDevicesTick(int dataType) {
    struct deviceData		*dd;
    struct deviceStreamData	*ddl;
    
    lprintf(60, "%s: Updating internal position devices\n", PPREFIX());

    // TODO: precompute a list of all internal devices
    for(ddl=uu->deviceStreamDataByType[dataType]; ddl!=NULL; ddl=ddl->nextWithSameType) {
	dd = ddl->dd;
	switch (dd->connection.type) {
	case DCT_INTERNAL_ZEROPOSE:
	    pilotUpdateZeropose(ddl);
	    break;
	default:
	    break;
	}
    }
    
    return(0);
}

////////////

static void pilotGetCurrentPositionAndOrientationFromSensors(vec3 position, vec3 rpy) {
    int				r;

    memset(rpy, 0, sizeof(vec3));
    
    // In the following call rpy is for nothing, not used nor set.
    pilotInternalDummyDevicesTick(DT_ORIENTATION_RPY);

    // first get and combine the orientation of the drone and use it to compute position
    r = pilotCombineCurrentOrientationFromSensors(rpy);
    if (r) {
	printf("%s: Error: No sensor providing orientation. Emergency landing!\n", PPREFIX());
	raspilotShutDownAndExit();
    }

    pilotInternalDummyDevicesTick(DT_POSITION_VECTOR);
    
    r = pilotCombineCurrentPositionFromSensors(position);
    if (r) {
	printf("%s: Error: No sensor providing position. Emergency landing!\n", PPREFIX());
	raspilotShutDownAndExit();
    }
    
    lprintf(PILOT_SENSOR_MERGE_DEBUG_LEVEL - 10, "%s: Fused Position    from sensors: %s\n", PPREFIX(), vec3ToString_st(position));
    lprintf(PILOT_SENSOR_MERGE_DEBUG_LEVEL - 10, "%s: Fused Orientation from sensors: %s\n", PPREFIX(), vec3ToString_st(rpy));
}

////////////

void pilotStoreLaunchPose(void *d) {
    int				i, j;
    double			meanTime;
    struct deviceData 		*dd;
    struct deviceStreamData 	*ddd;

    lprintf(20, "\n\n");
    lprintf(6, "%s: Storing launch poses.\n", PPREFIX());
    for(i=0; i<uu->deviceMax; i++) {
	dd = uu->device[i];
	for(j=0; j<dd->ddtMax; j++) {
	    ddd = dd->ddt[j];
	    regressionBufferGetMean(&ddd->outputBuffer, &meanTime, ddd->launchData);
	    ddd->launchPoseSetFlag = 1;
	    // lprintf(23, "%s: %s.%s launch pose %s.\n", PPREFIX(), dd->name, ddd->name, vecToString_st(ddd->launchPose.pr));
	    lprintf(6, "%s: %s.%s stored data %s from %d values\n", PPREFIX(), dd->name, ddd->name, arrayWithDimToStr_st(ddd->launchData, ddd->outputBuffer.vectorsize), MIN(ddd->outputBuffer.size, ddd->outputBuffer.n));
	}
    }
}

static void maybeNormalizeMovingVelocity(vec3 movingVelocity) {
    static int 		r = 0;
    static time_t 	lrt = 0;
    
    // normalize separately XY speed and altitude. Altitude sensors are usually less precise.
    r += vec2TruncateToSize(movingVelocity, 9.9 * uu->config.drone_max_speed, 0, "movingVelocity");
    r += vec1TruncateToSize(&movingVelocity[2], 2.9 * uu->config.drone_max_speed, 0, "movingVerticalVelocity");
    if (currentTime.sec != lrt) {
	if (uu->flyStage == FS_FLY && r >= 5) lprintf(5, "%s: Warning: %3d velocity truncations in last second. Maybe wrong position sensor?\n", PPREFIX(), r);
	lrt = currentTime.sec;
	r = 0;
    }
}

static void pilotGetDronePositionAndVelocityAndStoreInBuffers() {
    vec3		smoothPosition;
    vec3		smoothRpy, smoothRpyLong;
    vec3		sensorFusionPosition;
    vec3		sensorFusionRpy;
    vec3		velocity;
    vec3		rpyRotationSpeed;
    double		historyPose[6];
    double		ddt, mddt, tdTick;
    int			i;
    vec3		rpyAcceleration;
    vec3		acceleration;
    double 		maxRotationAcceleration, maxAcceleration;
    
    // move average from sensors to get the current pose their report
    pilotGetCurrentPositionAndOrientationFromSensors(sensorFusionPosition, sensorFusionRpy);

    // add the pose from sensor to regression buffer and use regression to estimate smooth current position
    regressionBufferAddElem(&uu->longBufferPosition, currentTime.dtime, sensorFusionPosition);    
    regressionBufferAddElem(&uu->longBufferRpy, currentTime.dtime, sensorFusionRpy);
    regressionBufferAddElem(&uu->shortBufferPosition, currentTime.dtime, sensorFusionPosition);    
    regressionBufferAddElem(&uu->shortBufferRpy, currentTime.dtime, sensorFusionRpy);

    // position and yaw is taken from long buffer, rp from short
    regressionBufferEstimateForTime(&uu->longBufferPosition, currentTime.dtime, smoothPosition);
    regressionBufferEstimateForTime(&uu->longBufferRpy, currentTime.dtime, smoothRpyLong);    
    regressionBufferEstimateForTime(&uu->shortBufferRpy, currentTime.dtime, smoothRpy);
    smoothRpy[2] = smoothRpyLong[2];
    
    
    // compute velocity from previous regressed pose and the current regressed pose
    tdTick = 1.0/uu->stabilization_loop_Hz;
    ddt = currentTime.dtime - uu->droneLastTickTime;
    // if time tick is abnormal, try to put it to a range
    mddt = tdTick / 2;
    if (ddt < mddt) {
	if (uu->flyStage == FS_FLY) lprintf(0, "%s: Warning: update time normalized from %g to %g for buffering.\n", PPREFIX(), ddt, mddt);
	ddt = mddt;
    }
    mddt = 10.0 * tdTick;
    if (ddt > mddt) {
	if (uu->flyStage == FS_FLY) lprintf(0, "%s: Warning: update time normalized from %g to %g for buffering.\n", PPREFIX(), ddt, mddt);
	ddt = mddt;
    }
    
    vec3_sub(velocity, smoothPosition, uu->droneLastPosition);
    vec3_scale(velocity, velocity, 1.0/ddt);
    vec3_sub(rpyRotationSpeed, smoothRpy, uu->droneLastRpy);
    vec3_scale(rpyRotationSpeed, rpyRotationSpeed, 1.0/ddt);


#if 1
//#define RPY_MAX_SPEED (M_PI)
#define RPY_MAX_SPEED (4.0 * uu->config.drone_max_rotation_speed)
    // Filter rotation speed to reasonable values
    for(i=0; i<3; i++) rpyRotationSpeed[i] = truncateToRange(rpyRotationSpeed[i], - RPY_MAX_SPEED, RPY_MAX_SPEED, "rpySpeed");
    // The same for velocity
    maybeNormalizeMovingVelocity(velocity);

#endif

#if 1
    // Filter rotation acceleration / deceleration to reasonable values
    maxRotationAcceleration = uu->config.drone_max_rotation_speed * 5;
    vec3_sub(rpyAcceleration, rpyRotationSpeed, uu->droneLastRpyRotationSpeed);
    for(i=0; i<3; i++) rpyAcceleration[i] = truncateToRange(rpyAcceleration[i], -maxRotationAcceleration, maxRotationAcceleration, "rpyAcceleration");
    vec3_add(rpyRotationSpeed, uu->droneLastRpyRotationSpeed, rpyAcceleration);

    // The same for movement acceleration / deceleration
    maxAcceleration = uu->config.drone_max_speed * 5;
    vec3_sub(acceleration, velocity, uu->droneLastVelocity);
    for(i=0; i<3; i++) acceleration[i] = truncateToRange(acceleration[i], -maxAcceleration, maxAcceleration, "acceleration");
    vec3_add(velocity, uu->droneLastVelocity, acceleration);
#endif
    
    vec3_assign(uu->droneLastPosition, smoothPosition);
    vec3_assign(uu->droneLastRpy, smoothRpy);
    vec3_assign(uu->droneLastVelocity, velocity);
    vec3_assign(uu->droneLastRpyRotationSpeed, rpyRotationSpeed);

    // add current position and orientation to long time history buffer
    vec3_assign(historyPose, smoothPosition);
    vec3_assign(historyPose+3, smoothRpy);
    raspilotRingBufferAddElem(uu->historyPose, currentTime.dtime, historyPose);

    // we are done. Values updated for currenbt tick.
    uu->droneLastTickTime = currentTime.dtime;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Normalization is called when pilot requires a thrust out of bonds 0..1
// It means that either we need to rotate in opposite direction or more thrust than motor is able to provide.
// Anyway it means a problem which can not be resolved in a 100% safe way. So, in order not to crash severly,
// we try to put thrust into the range 0..1 in some way

static int pilotNormalizeMotorThrustByRelaxingYaw() {
    int 	i, res;
    double 	requiredOffset, factor, t, requiredFactor, reducedFactor, reduction, maxReduction, nt;

    res = 0;
    requiredFactor = 0;
    for(i=0; i<uu->motor_number; i++) {
	t = uu->motor[i].thrust;
	requiredOffset = 0;
	if (t < 0.0) {
	    requiredOffset = -t;
	} else if (t > 1.0) {
	    requiredOffset = 1.0 - t;
	}
	factor = requiredOffset / uu->motor_yaw_forces[i];
	if (factor < 0) {
	    if (requiredFactor > 0) return(-1);
	    if (factor < requiredFactor) requiredFactor = factor;
	} else if (factor > 0) {
	    if (requiredFactor < 0) return(-1);
	    if (factor > requiredFactor) requiredFactor = factor;
	}
    }

    factor = reducedFactor = requiredFactor;

    /*
    for(i=0; i<uu->motor_number; i++) {
	t = uu->motor[i].thrust;
	nt = t + factor * uu->motor_yaw_forces[i];
	printf("%d: %g -> %g, ", i, t, nt);
    }
    printf("\n");
    */
    
    if (1) {
	maxReduction = 0;
	for(i=0; i<uu->motor_number; i++) {
	    t = uu->motor[i].thrust;
	    nt = t + factor * uu->motor_yaw_forces[i];
	    reduction = 0;
	    if (nt < 0) {
		reduction = -nt * uu->motor_yaw_forces[i];
	    } else if (nt > 1.0) {
		reduction = (1.0-nt) * uu->motor_yaw_forces[i];
	    }
	    //printf("%d: reduction %g\n", i, reduction);

	    if (maxReduction == 0) {
		maxReduction = reduction;
	    } else if (maxReduction > 0) {
		if (reduction > maxReduction) maxReduction = reduction;
	    } else if (maxReduction < 0) {
		if (reduction < maxReduction) maxReduction = reduction;
	    }
	}
	//printf("\n");
    }

    if (maxReduction != 0) {
	// We can not normalize thrust entirely. Do it at least partially.
	factor += maxReduction;
	res = 1;
    }
    
    lprintf(PILOT_THRUST_NORMALIZATION_DEBUG_LEVEL, "%s: Relaxing Yaw: ", PPREFIX());
    for(i=0; i<uu->motor_number; i++) {
	t = uu->motor[i].thrust;
	nt = t + factor * uu->motor_yaw_forces[i];
	lprintf(PILOT_THRUST_NORMALIZATION_DEBUG_LEVEL, "%d: %g->%g;  ", i, t, nt);
	uu->motor[i].thrust = nt;
    }
    lprintf(PILOT_THRUST_NORMALIZATION_DEBUG_LEVEL, "\n");
    return(res);
}

static int pilotNormalizeMotorThrustByRelaxingAltitude(double tmin, double tmax) {
    int		i;
    double 	t, nt, shift;
    
    
    if (tmax - tmin > 1.0) {
	// lprintf(PILOT_THRUST_NORMALIZATION_DEBUG_LEVEL, "\n%s: Error: Thrust can not be normalized tmax - tmin == %f.\n", PPREFIX(), tmax-tmin);
	return(-1);
    }
    
    // We can not completely relax desired altitude thrust. So do it only for smaller excesses.
    if (tmax >  1.3) return(-1);
    if (tmin < -0.3) return(-1);
    
    shift = 0;
    if (tmax > 1) shift = 1-tmax;
    else if (tmin < 0) shift = -tmin;
    lprintf(PILOT_THRUST_NORMALIZATION_DEBUG_LEVEL, "%s: Relaxing altitude: ", PPREFIX());
    for(i=0; i<uu->motor_number; i++) {
	t = uu->motor[i].thrust;
	nt = t + shift;
	lprintf(PILOT_THRUST_NORMALIZATION_DEBUG_LEVEL, "%d: %g->%g;  ", i, t, nt);
	uu->motor[i].thrust = truncateToRange(nt, 0, 1, NULL);
    }
 
    lprintf(PILOT_THRUST_NORMALIZATION_DEBUG_LEVEL, "\n");
    return(0);
}

static int pilotNormalizeMotorThrust() {
    int 	i, r;
    double 	t, nt, tmin, tmax, tsum;

    tmin = DBL_MAX; tmax = -DBL_MAX; tsum = 0;
    for(i=0; i<uu->motor_number; i++) {
	t = uu->motor[i].thrust;
	tsum += t;
	if (t < tmin) tmin = t;
	if (t > tmax) tmax = t;
    }
    lprintf(PILOT_THRUST_NORMALIZATION_DEBUG_LEVEL, "%s: Normalizing motor thrusts\n", PPREFIX());

    // First try to normalize by relaxing yaw stability
    r = pilotNormalizeMotorThrustByRelaxingYaw();
    if (r == 0) return(0);
    
    // Then try to normalize by relaxing altitude stability
    r = pilotNormalizeMotorThrustByRelaxingAltitude(tmin, tmax);
    if (r == 0) return(0);
    
    // still not normalized simply truncate thrusts
    lprintf(PILOT_THRUST_NORMALIZATION_DEBUG_LEVEL, "%s: Relaxing stability: ", PPREFIX());
    for(i=0; i<uu->motor_number; i++) {
	t = uu->motor[i].thrust;
	nt = truncateToRange(t, 0, 1, NULL);
	lprintf(PILOT_THRUST_NORMALIZATION_DEBUG_LEVEL, "%d: %g->%g;  ", i, t, nt);
	uu->motor[i].thrust = nt;
    }
    // TODO: Maybe try to normalize in the way which will hold roll, pitch and altitude, but may loose yaw stabilization.
    // Yaw stabilization is not as important as others are. Moreover may be subject of error jitters.
    lprintf(PILOT_THRUST_NORMALIZATION_DEBUG_LEVEL, "\n");
    return(1);
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
    motormin = 0;
    
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
	    if (normalizationDuringLastSecondCounter >= uu->autopilot_loop_Hz/10) {
		lprintf(0, "%s: Warning:   %3d thrust overflows during previous second. Maybe too aggressive drone setting!\n",
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

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////
// This is the main function computing roll, pitch and yaw to go to to the next waypoint.
//

static void pilotComputeTargetRollPitchYawForWaypoint() {
    int 		i, r;
    vec3		targetPositionVector, targetPositionDroneFrame;
    double 		roll, pitch, yaw;
    double		tdTick, tdRpyFix;	// td stands for Time Delta
    vec3		posEstimatedAtHalfRpyFixTime;
    vec3		movingVelocity;
    vec3		movingVelocityDroneFrame;
    vec3		targetGroundVelocity, targetRelativeVelocity, targetVelocityDroneFrame, diffVelocityDroneFrame;
    vec3		cRpy;
    double		verticalSpeed, targetVerticalSpeed;
    double		dmax, dspeed;

    // if not enough of data do nothing
    if (uu->longBufferPosition.n <= 2) return;
    if (uu->longBufferRpy.n <= 2) return;
    if (uu->shortBufferPosition.n <= 2) return;
    if (uu->shortBufferRpy.n <= 2) return;

    // Set some time quantums here
    // tdTick is the length of one pilot tick, it is used as PID controllers step.
    tdTick = 1.0/uu->autopilot_loop_Hz;
    
    // tdRpyFix is the timeframe in which we want to achieve targeted roll, pitch, yaw
    // TODO: Determine this dynamically someway as well as pilot_reach_goal_position_time depending
    // on the actual difference between target and current state.
    tdRpyFix = uu->config.pilot_reach_goal_orientation_time;
    
    // maximal distance on which we focus
    dmax = uu->config.drone_max_speed * uu->config.pilot_reach_goal_position_time;

    // get current drone pose and velocity
    regressionBufferEstimateForTime(&uu->longBufferRpy, currentTime.dtime, cRpy);
    
    vec3_assign(movingVelocity, uu->droneLastVelocity);

    // We consider as starting position for our computation the estimated position/orientation at time in tdRpyFix/2.0 seconds,
    // i.e. when we suppose that the new orientation of the drone will start taking effect.
    regressionBufferEstimateForTime(&uu->longBufferPosition, currentTime.dtime + tdRpyFix/2.0, posEstimatedAtHalfRpyFixTime);

    lprintf(30, "%s: Regression from buffer: Position     : %s\n", PPREFIX(), vec3ToString_st(posEstimatedAtHalfRpyFixTime));
    lprintf(30, "%s: Regression from buffer: Velocity     : %s\n", PPREFIX(), vec3ToString_st(movingVelocity));

    // Get current roll, pitch and yaw from current position pose.
    // pitch - negative == nose down;      positive == nose up
    // roll  - negative == left wing down; positive == left wing up
    // yaw   - positive == rotated counterclockwise (view from up)

    roll  = cRpy[0];
    pitch = cRpy[1];
    yaw   = cRpy[2];
    
    // some very basic self bug check
    if (vec3_has_nan(cRpy) || vec3_has_nan(posEstimatedAtHalfRpyFixTime) || vec3_has_nan(movingVelocity)) {
	// bug, bug, bug, panic.
	lprintf(0, "%s: Some key value has wrong value. Panic!\n", PPREFIX());
	mainStandardShutdown(NULL);
	return;
    }
    // basic panic check
    if (fabs(roll) >= uu->config.drone_panic_inclination || fabs(pitch) >= uu->config.drone_panic_inclination) {
	// inclinarion panic.
	lprintf(0, "%s: roll, pitch == %g %g, 'drone_panic_inclination' reached! Shutting down!\n", PPREFIX(), roll, pitch);
	mainStandardShutdown(NULL);
	return;
    }

    // small hack, suppose that we never travel much more than max_speed to avoid dirty values.
    // maybeNormalizeMovingVelocity(movingVelocity);
    
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

    // Truncate target velocity to hold user configured constraints.
    vec3TruncateToSize(targetGroundVelocity, uu->config.drone_max_speed, 0, NULL);
    // lprintf(30, "%s: Info: Target ground velocity normalized: %s\n", PPREFIX(), vec3ToString_st(targetGroundVelocity));	

    // Use PID controller to add "wind" velocity in order to get targetVelocity relative to the air.
    targetRelativeVelocity[0] = movingVelocity[0] + pidControllerStep(&uu->pidX, targetGroundVelocity[0], movingVelocity[0], tdTick);
    targetRelativeVelocity[1] = movingVelocity[1] + pidControllerStep(&uu->pidY, targetGroundVelocity[1], movingVelocity[1], tdTick);
    //targetRelativeVelocity[0] = pidControllerStep(&uu->pidX, targetGroundVelocity[0], movingVelocity[0], tdTick);
    //targetRelativeVelocity[1] = pidControllerStep(&uu->pidY, targetGroundVelocity[1], movingVelocity[1], tdTick);
    targetRelativeVelocity[2] = targetGroundVelocity[2];
    
    lprintf(30, "%s: Info: target position vector %s, targetGroundVelocity: %s, targetRelativeVelocity: %s\n", PPREFIX(), vec3ToString_st(targetPositionVector), vec3ToString_st(targetGroundVelocity), vec3ToString_st(targetRelativeVelocity));
    
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
    uu->targetPitch = - atan2(diffVelocityDroneFrame[0], dspeed);
    uu->targetRoll = - atan2(diffVelocityDroneFrame[1], dspeed);	
    uu->targetYaw = uu->currentWaypoint.yaw;

    // Normalize angles to <-Pi, Pi> range and apply user constraints from the configuration
    uu->targetPitch = normalizeToRange(uu->targetPitch, -M_PI, M_PI);
    uu->targetRoll = normalizeToRange(uu->targetRoll, -M_PI, M_PI);
    uu->targetPitch = truncateToRange(uu->targetPitch, -uu->config.drone_max_inclination, uu->config.drone_max_inclination, "target pitch");
    uu->targetRoll = truncateToRange(uu->targetRoll, -uu->config.drone_max_inclination, uu->config.drone_max_inclination, "target roll");

    // Compute rotation speed to get to the target roll, pitch, yaw
    // Also apply user constraints from the configuration
    uu->targetPitchRotationSpeed = angleSubstract(uu->targetPitch, pitch) / tdRpyFix;
    uu->targetPitchRotationSpeed =  truncateToRange(uu->targetPitchRotationSpeed, -uu->config.drone_max_rotation_speed, uu->config.drone_max_rotation_speed, "targetPitchRotationSpeed");
    uu->targetRollRotationSpeed = angleSubstract(uu->targetRoll, roll) / tdRpyFix;
    uu->targetRollRotationSpeed =  truncateToRange(uu->targetRollRotationSpeed, -uu->config.drone_max_rotation_speed, uu->config.drone_max_rotation_speed, "targetRollRotationSpeed");
    uu->targetYawRotationSpeed = angleSubstract(uu->targetYaw, yaw) / tdRpyFix;
    uu->targetYawRotationSpeed =  truncateToRange(uu->targetYawRotationSpeed, -uu->config.drone_max_rotation_speed, uu->config.drone_max_rotation_speed, NULL);
    // lprintf(27, "%s: tdRpyFix == %g\n", PPREFIX(), tdRpyFix);
    lprintf(27, "%s: RPY Orientation: Current: [%7.3f %7.3f %7.3f]  --> Target: [%7.3f %7.3f %7.3f]\n", PPREFIX(), roll, pitch, yaw, uu->targetRoll, uu->targetPitch, uu->targetYaw);

    // get the Altitude thrust
    targetVerticalSpeed = targetVelocityDroneFrame[2];
    verticalSpeed = movingVelocityDroneFrame[2];
    uu->altitudeThrust = pidControllerStep(&uu->pidAltitude, targetVerticalSpeed, verticalSpeed, tdTick);

}

///////////////////////////////////////////////////////////////////////////////////////////////////////
// This is the main function computing thrust for motors to stabilize drone and get target roll, pitch, yaw.
//

static void pilotComputeMotorThrustsForStabilization() {
    int 		i, r;
    double 		roll, pitch, yaw;
    double		yawThrust, rollThrust, pitchThrust;
    double		tdTick;	// td stands for Time Delta
    vec3		cRpy;
    double		t;
    double		yawRotationSpeed, rollRotationSpeed, pitchRotationSpeed;

    // if not enough of data do nothing
    if (uu->longBufferPosition.n <= 2) return;
    if (uu->longBufferRpy.n <= 2) return;
    if (uu->shortBufferPosition.n <= 2) return;
    if (uu->shortBufferRpy.n <= 2) return;

    // Set some time quantums here
    // tdTick is the length of one pilot tick, it is used as PID controllers step.
    tdTick = 1.0/uu->stabilization_loop_Hz;
    
    // get current drone pose and velocity
    regressionBufferEstimateForTime(&uu->longBufferRpy, currentTime.dtime, cRpy);
    
    // Get current roll, pitch and yaw from current position pose.
    // pitch - negative == nose down;      positive == nose up
    // roll  - negative == left wing down; positive == left wing up
    // yaw   - positive == rotated counterclockwise (view from up)

    roll  = cRpy[0];
    pitch = cRpy[1];
    yaw   = cRpy[2];
    
    rollRotationSpeed  = uu->droneLastRpyRotationSpeed[0];
    pitchRotationSpeed = uu->droneLastRpyRotationSpeed[1];
    yawRotationSpeed   = uu->droneLastRpyRotationSpeed[2];

    // some very basic self bug check
    if (vec3_has_nan(cRpy) || vec3_has_nan(uu->droneLastRpyRotationSpeed)) {
	// bug, bug, bug, panic.
	lprintf(0, "%s: Some key value has wrong value. Panic!\n", PPREFIX());
	mainStandardShutdown(NULL);
	return;
    }

    lprintf(22, "%s: RPY Speed:       Current: [%7.3f %7.3f %7.3f]  --> Target: [%7.3f %7.3f %7.3f]\n", PPREFIX(), rollRotationSpeed, pitchRotationSpeed, yawRotationSpeed, uu->targetRollRotationSpeed, uu->targetPitchRotationSpeed, uu->targetYawRotationSpeed);
#define USE_PID_WITH_SPEED 1
    // TODO: Add here an option for just P controller which will take into account uu->config.pilot_reach_goal_orientation_time
#if USE_PID_WITH_SPEED
    // Use PID controllers to compute final motor thrusts to achieve target rotation speeds from current rotation speeds
    // In this case a P controller shall do the job!
    rollThrust = pidControllerStep(&uu->pidRoll, uu->targetRollRotationSpeed, rollRotationSpeed, tdTick);
    pitchThrust = pidControllerStep(&uu->pidPitch, uu->targetPitchRotationSpeed, pitchRotationSpeed, tdTick);
    yawThrust = pidControllerStep(&uu->pidYaw, uu->targetYawRotationSpeed, yawRotationSpeed, tdTick);
#else
    rollThrust = pidControllerStep(&uu->pidRoll, uu->targetRoll, roll, tdTick);
    pitchThrust = pidControllerStep(&uu->pidPitch, uu->targetPitch, pitch, tdTick);
    yawThrust = pidControllerStep(&uu->pidYaw, uu->targetYaw, yaw, tdTick);    
#endif
    
    lprintf(22, "%s: PID Thrusts:    Altitude == %f;  RPY == %f %f %f\n", PPREFIX(), uu->altitudeThrust, rollThrust, pitchThrust, yawThrust);

    // Combine everything into thrusts for each motor
    for(i=0; i<uu->motor_number; i++) {
	t = 0;
	t += uu->altitudeThrust;
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
void pilotUpdateBufferAndRecomputeMotorThrust() {
    pilotGetDronePositionAndVelocityAndStoreInBuffers();
    pilotComputeTargetRollPitchYawForWaypoint();
    pilotComputeMotorThrustsForStabilization();
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
	    if (skewN != 0) {
		if (uu->flyStage == FS_FLY) lprintf(0, "%s: Warning: %d time skews in the last second. Average skew: %f ms.\n", PPREFIX(), skewN, skewSum/skewN);
		// lprintf(40, " Total clock skew %f ms!", skewSum);
		// lprintf(0, "\n");
	    }
	    skewN = 0;
	    skewSum = 0;
	    lts = currentTime.sec;
	}
    }
    timeLineRescheduleUniqueEvent(nextTickUsec, tickfunction, arg);
    return(nextTickUsec);
}

static void pilotTickSetDebugLevel() {
    // With -l10 option, log only 10 ticks per second, no matter the rate
    // This reduces the size of logs --> less pressure to the system and logs are easier to read.
    if (log10TicksPerSecond == 0 || tickCounter % (int)(1+uu->autopilot_loop_Hz/10) == 0) {
	logLevel = baseLogLevel;
    } else {
	logLevel = debugLevel+2;
    }
}

void pilotAutopilotLoopTick(void *d) {
    // This is the main function called at each tick of standard rapilot.
    // lprintf(30, "\n");
    lprintf(100, "%s: Stabilization tick\n", PPREFIX());
    nextStabilizationTickUsec = pilotScheduleNextTick(uu->autopilot_loop_Hz, pilotAutopilotLoopTick, NULL);
    pilotTickSetDebugLevel();
    if (uu->flyStage >= FS_PRE_FLY) {
	// pilotGetDronePositionAndVelocityAndStoreInBuffer();
    }
    if (uu->flyStage >= FS_FLY) {
	pilotComputeTargetRollPitchYawForWaypoint();
    }
}

void pilotRegularStabilizationTick(void *d) {
    // This is the main function called at each tick of standard rapilot.
    // lprintf(30, "\n");
    lprintf(100, "%s: PID tick\n", PPREFIX());
    tickCounter ++;
    nextPidTickUsec = pilotScheduleNextTick(uu->stabilization_loop_Hz, pilotRegularStabilizationTick, NULL);
    pilotTickSetDebugLevel();
    if (uu->flyStage >= FS_PRE_FLY) {
	pilotGetDronePositionAndVelocityAndStoreInBuffers();
    }
    if (uu->flyStage >= FS_FLY) {
	pilotComputeMotorThrustsForStabilization();
    }
    if (uu->flyStage >= FS_PRE_FLY) {
	pilotSendThrusts(NULL);
    }
}

void pilotRegularSpecialModeTick(void *d) {
    // Ticks used in other modes than flying like ESC calibration, motor testing, etc.
    tickCounter ++;
    pilotScheduleNextTick(PILOT_PRELAUNCH_FREQUENCY_HZ, pilotRegularSpecialModeTick, NULL);
    pilotSendThrusts(NULL);
}

void pilotRegularStabilizationLoopRescheduleToSoon() {
    int		r;
    uint64_t	tt;

    tt = currentTime.usec + 500;
    if (nextPidTickUsec > tt) {
	r = timeLineRescheduleUniqueEventIfExisted(tt, pilotRegularStabilizationTick, NULL);
	if (r == 0) nextPidTickUsec = tt;
    }
}

void pilotRegularSendPings(void *d) {
    struct baio			*bb;
    struct deviceStreamData 	*ddl;

    // do not send anything in shutdown sequence (otherwise motors are awaken)
    if (shutDownInProgress) return;
    
    for(ddl=uu->deviceStreamDataByType[DT_PING]; ddl!=NULL; ddl=ddl->nextWithSameType) {
	bb = baioFromMagic(ddl->dd->baioMagic);
	if (bb == NULL) return;
	baioPrintfToBuffer(bb, "ping%"PRId64"\n", currentTime.usec);
    }

    lprintf(100, "%s: Sending pings\n", PPREFIX());
    timeLineInsertEvent(UTIME_AFTER_MSEC(2022), pilotRegularSendPings, NULL);
}

void pilotRegularSaveTrajectory(void *d) {
    vec3	cpos;
    vec3	crpy;

    if (uu->longBufferPosition.n != 0 && uu->longBufferRpy.n != 0) {
	regressionBufferEstimateForTime(&uu->longBufferPosition, currentTime.dtime, cpos);
	regressionBufferEstimateForTime(&uu->longBufferRpy, currentTime.dtime, crpy);
	trajectoryLogPrintf("%g %g %g  %g %g %g\n", cpos[0], cpos[1], cpos[2], crpy[0], crpy[1], crpy[2]);
    }
    timeLineInsertEvent(TLINE_UTIME_AFTER_MSEC(100), pilotRegularSaveTrajectory, d);
}

void pilotInteractiveInputRegularCheck(void *d) {
    int		c;

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
		(dd->ddt[j]->mandatory && (dd->ddt[j]->input == NULL || dd->ddt[j]->input->buffer.n == 0))
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


