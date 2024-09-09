#include "common.h"


void motorSendThrustsToStream(struct deviceStreamData *ddl) {
    int 		i, j, r;
    struct baio		*bb;
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
    motorsThrustSetAndSend(0);
}

void motorsSendMessage(char *fmt, ...) {
    struct deviceStreamData 	*ddl;
    struct baio			*bb;
    va_list			ap, aa;

    // Hmm. Do this nicer by adding some special datatype instead of DT_THRUST
    va_start(ap, fmt);
    for(ddl=uu->deviceStreamDataByType[DT_THRUST]; ddl!=NULL; ddl=ddl->nextWithSameType) {
	bb = baioFromMagic(ddl->dd->baioMagic);
	if (bb != NULL) {
	    va_copy(aa, ap);
	    baioVprintfToBuffer(bb, fmt, aa);
	    va_end(aa);
	}
    }
    va_end(ap);
}

void motorsStandby(void *d) {
    motorsSendMessage("stan\n");
}

void motorsExit(void *d) {
    motorsSendMessage("exit\n");
}

void motorsSendStreamThrustFactor(void *d) {
    motorsSendMessage("mfac %g\n", (double)MOTOR_STREAM_THRUST_FACTOR);
}

void motorsBeep(void *d) {
    motorsSendMessage("beep\n");
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
    
    // deviceSendToAllDevices("info: landing\n");
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
	lprintf(10, "%s: %s.%s: no input buffer buffer\n", PPREFIX(), gg->dd->name, gg->name);
	return(-1);
    }
    if (gg->outputBuffer.n == 0) {
	lprintf(10, "%s: %s.%s: no history\n", PPREFIX(), gg->dd->name, gg->name);
	return(-1);
    }
    lastdatatime = gg->outputBuffer.time[gg->outputBuffer.ailast];
    // lprintf(0, "timeout check %f < %f %f %f == %f\n", lastdatatime, currentTime.dtime, gg->timeout, gg->latency, currentTime.dtime - (gg->timeout + gg->latency));
    if (lastdatatime < currentTime.dtime - gg->timeout - gg->latency) {
	lprintf(20, "%s: %s.%s: timeouted. Last data from %s\n", PPREFIX(), gg->dd->name, gg->name, sprintSecTime_st(lastdatatime*1000000.0));
	return(-1);
    }
    return(0);
}

#if 0
static void pilotAddCurrentQuatOrientationFromSensorToSum(vec3 sinsum, vec3 cossum, vec3 weightsum, struct deviceStreamData *gg) {
    quat			qq;
    int				r;
    vec4			ww;
    
    deviceTranslateInputToOutput(gg);
    if (pilotCheckDeviceForTimeout(gg)) return;
    
    r = regressionBufferEstimateForTime(&gg->outputBuffer, currentTime.dtime, qq);
    if (r) {
	lprintf(50, "%s: Warning: %s.%s pose estimation failed!\n", PPREFIX(), gg->dd->name, gg->name);
    }

    // TODO: translate quaternion qq to roll, pitch, yaw and add it to sinsum, cossum and weightsum

    /*
      // This is old way when we worked with quaternions directly
    lprintf(PILOT_SENSOR_MERGE_DEBUG_LEVEL, "%s: add %s from %-20s %-20s: quat: weight %s, confidence %g\n", PPREFIX(), vec4ToString_st(qq), gg->dd->name, gg->name, vec4ToString_st(gg->weight), gg->confidence);

    vec4_scale(ww, gg->weight, gg->confidence);
    vec4_mul_elem(qq, qq, ww);
    vec4_add(qqsum, qqsum, qq);
    vec4_add(weightsum, weightsum, ww);
    */
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

#if 0
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
#endif

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

    // The device provide a vector of size 1, so the weight is also vector of size 1. weight[0] is the actual weight.
    ppsum[2] += alt * gg->weight[0];
    weightsum[2] += gg->weight[0];

    lprintf(PILOT_SENSOR_MERGE_DEBUG_LEVEL, "%s: add [0,0,%f] from %-20s %-20s: altitude %f: \tweights: [0,0,%g], confidence %f\n", PPREFIX(), alt, gg->dd->name, gg->name, alt,  gg->weight[0], 1.0);
    
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
    vec3			rpyweight;
    vec3			rpysinsum, rpycossum;
    int				i;
    int				vecHasZero;
    
    memset(rpyweight, 0, sizeof(rpyweight));
    memset(rpysinsum, 0, sizeof(rpysinsum));
    memset(rpycossum, 0, sizeof(rpycossum));

    lprintf(PILOT_SENSOR_MERGE_DEBUG_LEVEL, "%s: Going to merge orientation from sensors\n", PPREFIX());

    // TODO: Probably completely remove quaternions for the sake of simplicity
    // pilotAddQuatOrientation(qqsum, qqweight, DT_ORIENTATION_QUATERNION, pilotAddCurrentQuatOrientationFromSensorToSum);
    pilotAddRpyOrientation(rpysinsum, rpycossum, rpyweight, DT_ORIENTATION_RPY_SHM, pilotAddCurrentRpyOrientationFromSensorToSum);
    pilotAddRpyOrientation(rpysinsum, rpycossum, rpyweight, DT_ORIENTATION_RPY, pilotAddCurrentRpyOrientationFromSensorToSum);

    vecHasZero = vec3_has_zero(rpyweight);

    // get average roll, pitch, yaw
    for(i=0; i<3; i++) {
	// avoid division by zero
	if (rpyweight[i] == 0) {
	    rpy[i] = 0;
	} else {
	    rpy[i] = atan2(rpysinsum[i]/rpyweight[i], rpycossum[i]/rpyweight[i]);
	}
    }

    return(vecHasZero);
}

static int pilotCombineCurrentPositionFromSensors(vec3 position) {
    int				i;
    vec3			ppsum, weightsum;
    int				vecHasZero;

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

    vecHasZero = vec3_has_zero(weightsum);

    for(i=0; i<3; i++) {
	if (vecHasZero) {
	    position[i] = 0;
	} else {
	    position[i] = ppsum[i]/weightsum[i];
	}
    }

    return(vecHasZero);
}

static void pilotAddAccelerationFromSensorToSum(vec3 ppsum, vec3 weightsum, struct deviceStreamData *gg) {
    vec3		ww, droneacc;
    double		meantime;
    int			r;

    deviceTranslateInputToOutput(gg);
    if (pilotCheckDeviceForTimeout(gg)) return;

    // We take mean for acceleration. I do not know why but I feel it better
    // r = regressionBufferEstimateForTime(&gg->outputBuffer, currentTime.dtime, droneacc);
    regressionBufferGetMean(&gg->outputBuffer, &meantime, droneacc);

    if (! gg->launchPoseSetFlag) {
	memset(droneacc, 0, sizeof(droneacc));
    }
    
    lprintf(PILOT_SENSOR_MERGE_DEBUG_LEVEL,"%s: add %s from %-20s %-20s \tweights: %s, confidence %f\n", PPREFIX(), vecToString_st(droneacc), gg->dd->name, gg->name, vec3ToString_st(gg->weight), gg->confidence);

    vec3_scale(ww, gg->weight, gg->confidence);
    vec3_mul_elem(droneacc, droneacc, ww);
    vec3_add(ppsum, ppsum, droneacc);
    vec3_add(weightsum, weightsum, ww);
    return;
}

static void pilotAddAcceleration(vec3 ppsum, vec3 weightsum, int datatype, void mapfun(vec3, vec3, struct deviceStreamData *)) {
    struct deviceStreamData *ddl;
    
    assert(datatype > DT_NONE && datatype < DT_MAX);
    for(ddl=uu->deviceStreamDataByType[datatype]; ddl!=NULL; ddl=ddl->nextWithSameType) {
	mapfun(ppsum, weightsum, ddl);
    }
}


static int pilotCombineCurrentAccelerationFromSensors(vec3 acceleration) {
    int				i;
    vec3			ppsum, weightsum;
    int				vecHasZero;

    // pilotComputeCurrentGyropose("gyropose", DT_POSITION_VECTOR, orientation);

    lprintf(PILOT_SENSOR_MERGE_DEBUG_LEVEL, "%s: Going to merge acceleration from sensors\n", PPREFIX());

    memset(acceleration, 0, sizeof(vec3));
    memset(ppsum, 0, sizeof(vec3));
    memset(weightsum, 0, sizeof(vec3));
    
    // Go through all sensors contributing to the position
    pilotAddAcceleration(ppsum, weightsum, DT_EARTH_ACCELERATION, pilotAddAccelerationFromSensorToSum);
    pilotAddAcceleration(ppsum, weightsum, DT_EARTH_ACCELERATION_SHM, pilotAddAccelerationFromSensorToSum);

    vecHasZero = vec3_has_zero(weightsum);
    for(i=0; i<3; i++) {
	if (vecHasZero) {
	    acceleration[i] = 0;
	} else {
	    acceleration[i] = ppsum[i]/weightsum[i];
	}
    }

    return(vecHasZero);
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

static void pilotGetCurrentAccelerationPositionAndOrientationFromSensors(vec3 acceleration, vec3 position, vec3 rpy) {
    int				r;

    memset(rpy, 0, sizeof(vec3));
    
    // In the following call rpy is for nothing, not used nor set.
    pilotInternalDummyDevicesTick(DT_ORIENTATION_RPY);

    pilotCombineCurrentAccelerationFromSensors(acceleration);
    
    // first get and combine the orientation of the drone and use it to compute position
    r = pilotCombineCurrentOrientationFromSensors(rpy);
    if (r && uu->flyStage > FS_PRE_FLY) {
	printf("%s: Error: No sensor providing orientation. Emergency landing!\n", PPREFIX());
	raspilotShutDownAndExit();
    }

    pilotInternalDummyDevicesTick(DT_POSITION_VECTOR);
    
    r = pilotCombineCurrentPositionFromSensors(position);
    if (r && uu->flyStage > FS_PRE_FLY) {
	printf("%s: Error: No sensor providing position. Emergency landing!\n", PPREFIX());
	raspilotShutDownAndExit();
    }
    
    lprintf(PILOT_SENSOR_MERGE_DEBUG_LEVEL - 10, "%s: Fused Position    from sensors: %s\n", PPREFIX(), vec3ToString_st(position));
    lprintf(PILOT_SENSOR_MERGE_DEBUG_LEVEL - 10, "%s: Fused Orientation from sensors: %s\n", PPREFIX(), vec3ToString_st(rpy));
}

////////////

void pilotLaunchPoseSet(void *d) {
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
	    regressionBufferReset(&ddd->outputBuffer);
	    memset(ddd->drift_offset_per_second, 0, deviceDataStreamVectorLength[ddd->type]*sizeof(double));
	}
    }
}

void pilotLaunchPoseClear(void *d) {
    int				i,j;
    struct deviceData 		*dd;
    struct deviceStreamData 	*ddd;
    
    lprintf(20, "\n\n");
    lprintf(6, "%s: Clearing launch poses.\n", PPREFIX());
    for(i=0; i<uu->deviceMax; i++) {
	dd = uu->device[i];
	for(j=0; j<dd->ddtMax; j++) {
	    ddd = dd->ddt[j];
	    ddd->launchPoseSetFlag = 0;
	    assert(deviceDataStreamVectorLength[ddd->type] == ddd->outputBuffer.vectorsize);
	    memset(ddd->launchData, 0, ddd->outputBuffer.vectorsize*sizeof(double));
	    regressionBufferReset(&ddd->outputBuffer);
	    memset(ddd->drift_offset_per_second, 0, deviceDataStreamVectorLength[ddd->type]*sizeof(double));
	}
    }
}

// TODO, compute this once per tick and store into struct universe
double pilotGetNormalizedLastStabilizationTickLength() {
    double		ddt, mddt, tdTick;

    // compute velocity from previous regressed pose and the current regressed pose
    tdTick = 1.0/uu->stabilization_loop_Hz;
    ddt = currentTime.dtime - uu->droneLastTickTime;
    // if time tick is abnormal, try to put it around tdTick
    mddt = tdTick / 2;
    if (ddt < mddt) {
	if (uu->flyStage == FS_FLY) lprintf(0, "%s: Warning: update time normalized from %g to %g for filtering.\n", PPREFIX(), ddt, mddt);
	ddt = mddt;
    }
    mddt = 10.0 * tdTick;
    if (ddt > mddt) {
	if (uu->flyStage == FS_FLY) lprintf(0, "%s: Warning: update time normalized from %g to %g for filtering.\n", PPREFIX(), ddt, mddt);
	ddt = mddt;
    }
    return(ddt);
}

#if 0
// This applies some common sense constraint on position and orientation vales.
// But it seems that it works better without it.
static void pilotFilterCurrentAccelerationPositionAndOrientationValues(vec3 acceleration, vec3 position, vec3 rpy) {
    vec3		velocity;
    vec3		rpyRotationSpeed;
    double		ddt;
    int			i;
    vec3		rpyAcceleration;
    vec3		accAsVelocityDifference;
    double 		maxRotationAcceleration, maxAcceleration;
    double		rpyMaxSpeed;
    double		accSensorWeight, accVelocityWeight;
    vec3 		fusedacceleration;
    double		xx;
    
    ddt = uu->droneLastStabilizationTickLength;

    ///////////
    // filter speeds of rotation to reasonable values when some sensor becomes buggy

    // get rpy rotation speeds
    vec3_sub(rpyRotationSpeed, rpy, uu->droneLastRpy);
    vec3_scale(rpyRotationSpeed, rpyRotationSpeed, 1.0/ddt);

    // acceleration / deceleration of rotation
    // large values may still be reasonable, It seem the drone can start rotate very quickly
    // Maybe I shall remove this filter completely
    maxRotationAcceleration = M_PI * 2000;
    vec3_sub(rpyAcceleration, rpyRotationSpeed, uu->droneLastRpyRotationSpeed);
    vec3_scale(rpyAcceleration, rpyAcceleration, 1/ddt);
    // lprintf(22, "rpyRotationSpeed %s; uu->droneLastRpyRotationSpeed %s; rpyAcceleration %s\n", vec3ToString_st(rpyRotationSpeed), vec3ToString_st(uu->droneLastRpyRotationSpeed), vec3ToString_st(rpyAcceleration));
    for(i=0; i<3; i++) rpyAcceleration[i] = truncateToRange(rpyAcceleration[i], -maxRotationAcceleration, maxRotationAcceleration, "rpyAcceleration", i);
    vec3_scale(rpyAcceleration, rpyAcceleration, ddt);
    vec3_add(rpyRotationSpeed, uu->droneLastRpyRotationSpeed, rpyAcceleration);

    // If still unreasonably high, filter rotation speed directly
    rpyMaxSpeed = 4.0 * uu->config.drone_max_rotation_speed;
    for(i=0; i<3; i++) rpyRotationSpeed[i] = truncateToRange(rpyRotationSpeed[i], - rpyMaxSpeed, rpyMaxSpeed, "rpySpeed", i);

    // rpy is last rpy + rpyRotationSpeed * time
    vec3_scale(rpy, rpyRotationSpeed, ddt);
    vec3_add(rpy, rpy, uu->droneLastRpy);

    
    ///////
    // the same for drone movement: acceleration, velocity, position
    
    // Limit acceleration to a reasonable values
    maxAcceleration = uu->config.drone_max_speed * 5;
    // Also consider the maxacceleration can not be higher than physical capabilities
    // I suppose motors do not give more acceleration than 3G.
    if (maxAcceleration >= 1 * GRAVITY_ACCELERATION) maxAcceleration = 1 * GRAVITY_ACCELERATION;

    // It's probably better to restrict each part of vector separately than the whole vector,
    // so that for example, if only altimeter is buggy we do not restrict xy acceleration.
    // vec3TruncateToSize(acceleration, maxAcceleration, 1, "acceleration");
    vecTruncateInnerToRange(acceleration, 3, -maxAcceleration, maxAcceleration, "acceleration");

    vec3_sub(velocity, position, uu->droneLastPosition);
    vec3_scale(velocity, velocity, 1.0/ddt);
    
    vec3_sub(accAsVelocityDifference, velocity, uu->droneLastVelocity);
    vec3_scale(accAsVelocityDifference, accAsVelocityDifference, 1.0/ddt);
    // vec3TruncateToSize(accAsVelocityDifference, maxAcceleration, 1, "accAsVelocityDifference");
    vecTruncateInnerToRange(accAsVelocityDifference, 3, -maxAcceleration, maxAcceleration, "accAsVelocityDifference");

    // make acceleration as weighted average of direct meassured acceleration and velocity difference
    // Hmm. does this actually help?
    accSensorWeight = 0.0;
    accVelocityWeight = 1.0 - accSensorWeight;
    vec3_scale(fusedacceleration, acceleration, accSensorWeight);
    vec3_scale(accAsVelocityDifference, accAsVelocityDifference, accVelocityWeight);
    vec3_add(fusedacceleration, fusedacceleration, accAsVelocityDifference);
    
    // velocity is previous velocity plus acceleration * time
    vec3_scale(velocity, fusedacceleration, ddt);
    vec3_add(velocity, velocity, uu->droneLastVelocity);
    // maybe reset to reasonble values if not good yet
    maybeNormalizeMovingVelocity(velocity);

    // position is last position + velocity * time
    vec3_scale(position, velocity, ddt);
    vec3_add(position, position, uu->droneLastPosition);
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

#endif


// TODO. SPlit this into two functions, one for getting oriantation RPY and another for position
// Orientation will be called in stabilization loop, position in mission autopilot.
static void pilotGetDronePositionAndVelocityAndStoreInBuffers() {
    vec3		smoothPosition;
    vec3		smoothAcceleration, smoothRpy, smoothRpyLong;
    vec3		sensorFusionAcceleration;
    vec3		sensorFusionPosition;
    vec3		sensorFusionRpy;
    vec3		acceleration, velocity;
    vec3		rpyRotationSpeed;
    double		historyPose[6];
    double		ddt;
    vec3 		ba, bp, br;

    ddt = uu->droneLastStabilizationTickLength;
    
    // get the pose reported by sensor buffers
    pilotGetCurrentAccelerationPositionAndOrientationFromSensors(sensorFusionAcceleration, sensorFusionPosition, sensorFusionRpy);

#if 0    
    // We can not filter here. It'd make strange effect in combination with regression buffers making
    // that position is continuously wrong due to acceleration constraints
    // pilotFilterCurrentAccelerationPositionAndOrientationValues(sensorFusionAcceleration, sensorFusionPosition, sensorFusionRpy);
#endif
    
    // add the pose from sensor to regression buffer and use regression to estimate smooth current position
    regressionBufferAddElem(&uu->longBufferPosition, currentTime.dtime, sensorFusionPosition);    
    regressionBufferAddElem(&uu->longBufferRpy, currentTime.dtime, sensorFusionRpy);
    regressionBufferAddElem(&uu->shortBufferAcceleration, currentTime.dtime, sensorFusionAcceleration);    
    regressionBufferAddElem(&uu->shortBufferPosition, currentTime.dtime, sensorFusionPosition);    
    regressionBufferAddElem(&uu->shortBufferRpy, currentTime.dtime, sensorFusionRpy);

#if 1
    // Hmm. is this "double regression" useful? I keep it as it may be useful in the future.
    // I can always configure the size of the bufer to 1 or 2 if I don't want it.
    // position and yaw is taken from long buffer, roll and pitch from short
    regressionBufferEstimateForTime(&uu->longBufferPosition, currentTime.dtime, smoothPosition);
    regressionBufferEstimateForTime(&uu->longBufferRpy, currentTime.dtime, smoothRpyLong);    
    regressionBufferEstimateForTime(&uu->shortBufferRpy, currentTime.dtime, smoothRpy);
    regressionBufferEstimateForTime(&uu->shortBufferAcceleration, currentTime.dtime, smoothAcceleration);
    smoothRpy[2] = smoothRpyLong[2];
#else
    vec3_assign(smoothPosition, sensorFusionPosition);
    vec3_assign(smoothAcceleration, sensorFusionAcceleration);
    vec3_assign(smoothRpy, sensorFusionRpy);
#endif    

    // The filter may be useless here, it is a kind of security guard only.
    // I put it out, it flies better without it. Instead try to add accelerometer data to velocity -> position
    // pilotFilterCurrentAccelerationPositionAndOrientationValues(smoothAcceleration, smoothPosition, smoothRpy);
    
    vec3_sub(rpyRotationSpeed, smoothRpy, uu->droneLastRpy);
    vec3_scale(rpyRotationSpeed, rpyRotationSpeed, 1.0/ddt);
    vec3_sub(velocity, smoothPosition, uu->droneLastPosition);
    vec3_scale(velocity, velocity, 1.0/ddt);
    
    // vec3_sub(acceleration, velocity, uu->droneLastVelocity);
    // vec3_scale(acceleration, acceleration, 1.0/ddt);    
    
    vec3_assign(uu->droneLastPosition, smoothPosition);
    vec3_assign(uu->droneLastRpy, smoothRpy);
    vec3_assign(uu->droneLastVelocity, velocity);
    vec3_assign(uu->droneLastRpyRotationSpeed, rpyRotationSpeed);
    vec3_assign(uu->droneLastAcceleration, smoothAcceleration);

    // add current position and orientation to long time history buffer
    vec3_assign(historyPose, smoothPosition);
    vec3_assign(historyPose+3, smoothRpy);
    raspilotRingBufferAddElem(uu->historyPose, currentTime.dtime, historyPose);

}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
// What we call "normalization" is the the way how motor thrust is adjusted into the range 0..1. It may happen
// that pilot physics compute the required thrust out of bonds 0..1. It means that either we need
// to rotate propellers in the opposite direction or we need more thrust than motors are able to provide.
// Anyway it means a problem which can not be resolved in a 100% safe way. So, in order not to loose stability 
// (and crash severly),  we first try to put thrust into the range 0..1 by relaxing to yaw constraints or
// maybe loose or get more altitude.

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

static int pilotNormalizeMotorThrustByRelaxingAltitude() {
    int		i;
    double 	tmin, tmax, t, nt, shift;
    
    tmin = DBL_MAX; tmax = -DBL_MAX; 
    for(i=0; i<uu->motor_number; i++) {
	t = uu->motor[i].thrust;
	if (t < tmin) tmin = t;
	if (t > tmax) tmax = t;
    }
    
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
	uu->motor[i].thrust = truncateToRange(nt, 0, 1, NULL, INDEX_NAN);
    }
 
    lprintf(PILOT_THRUST_NORMALIZATION_DEBUG_LEVEL, "\n");
    return(0);
}

static int pilotNormalizeMotorThrust() {
    int 	i, r;
    double 	t, nt;

    lprintf(PILOT_THRUST_NORMALIZATION_DEBUG_LEVEL, "%s: Normalizing motor thrusts\n", PPREFIX());
    
    // First try to normalize by relaxing yaw stability
    r = pilotNormalizeMotorThrustByRelaxingYaw();
    if (r == 0) return(0);
    
    // Then try to normalize by relaxing altitude stability
    r = pilotNormalizeMotorThrustByRelaxingAltitude();
    if (r == 0) return(0);
    
    // still not normalized simply truncate thrusts
    lprintf(PILOT_THRUST_NORMALIZATION_DEBUG_LEVEL, "%s: Relaxing stability: ", PPREFIX());
    for(i=0; i<uu->motor_number; i++) {
	t = uu->motor[i].thrust;
	nt = truncateToRange(t, 0, 1, NULL, INDEX_NAN);
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
	    if (uu->flyStage == FS_FLY) {
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

///////////////////////////////////////////////////////////////////////////////////////////////////////
//
static void pilotCombineAllMotorThrusts(double rollThrust, double pitchThrust, double yawThrust, double altitudeThrust) {
    double t;
    int i;
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

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////
// This is the main function computing roll, pitch and yaw to go to to the next waypoint.
//


// TODO: Removing everything around altitude from this function
// TODO: Split it to roll+pitch and yaw
static void pilotComputeTargetRollPitchYawForWaypoint() {
    int 		i;
    vec2		targetPositionVector, targetPositionDroneFrame;
    double 		roll, pitch, yaw;
    double		tdTick, tdRpyFix;	// td stands for Time Delta
    vec3		posEstimatedInRpyFixTime;
    vec2		movingVelocity;
    vec2		movingVelocityDroneFrame;
    vec2		targetGroundVelocity, targetVelocityDroneFrame, diffVelocityDroneFrame, targetVelocityThrustDroneFrame;
    vec3		cRpy;
    double		dmax, dspeed;
    
    // if not enough of data do nothing
    if (uu->longBufferPosition.n <= 2) return;
    if (uu->longBufferRpy.n <= 2) return;
    if (uu->shortBufferPosition.n <= 2) return;
    if (uu->shortBufferRpy.n <= 2) return;

    // Set some time quantums here
    // tdTick is the length of one pilot tick, it is used as PID controllers step.
    tdTick = 1.0 / uu->autopilot_loop_Hz;
    
    // tdRpyFix is the timeframe in which we want to achieve targeted roll, pitch, yaw
    // TODO: Determine this dynamically someway as well as pilot_reach_goal_position_time depending
    // on the actual difference between target and current state.
    tdRpyFix = uu->config.pilot_reach_goal_orientation_time;
    
    // maximal distance on which we focus
    dmax = uu->config.drone_max_speed * uu->config.pilot_reach_goal_position_time;

    // get current drone pose and velocity (??? + tdRpyFix)
    regressionBufferEstimateForTime(&uu->longBufferRpy, currentTime.dtime, cRpy);
    
    vec2_assign(movingVelocity, uu->droneLastVelocity);

    // We consider as starting position for our computation the estimated position/orientation at time in tdRpyFix/2.0 seconds,
    // i.e. when we suppose that the new orientation of the drone will start taking effect.
    regressionBufferEstimateForTime(&uu->longBufferPosition, currentTime.dtime + tdRpyFix/2, posEstimatedInRpyFixTime);

    lprintf(30, "%s: Regression from buffer: Position     : %s\n", PPREFIX(), vec2ToString_st(posEstimatedInRpyFixTime));
    lprintf(30, "%s: Regression from buffer: Velocity     : %s\n", PPREFIX(), vec2ToString_st(movingVelocity));

    // Get current roll, pitch and yaw from current position pose.
    // pitch - negative == nose down;      positive == nose up
    // roll  - negative == left wing down; positive == left wing up
    // yaw   - positive == rotated counterclockwise (view from up)

    roll  = cRpy[0];
    pitch = cRpy[1];
    yaw   = cRpy[2];
    
    // some very basic self bug check
    if (vec3_has_nan(cRpy) || vec3_has_nan(posEstimatedInRpyFixTime) || vec2_has_nan(movingVelocity)) {
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
    // TODO: Apply some better model
    
    vec2_sub(targetPositionVector, uu->currentWaypoint.position, posEstimatedInRpyFixTime);
    for(i=0; i<2; i++) {
	// Restrict targetPositionVector so that we do not need to go over max size in any direction
	// I think this is here to restrict vertical speed in particular, so that drone does not try to climb/descent
	// disproportionally and then eclipsing the necessity to hold X,Y position.
	targetPositionVector[i] = truncateToRange(targetPositionVector[i], -dmax, dmax, NULL, INDEX_NAN);

	// Compute the max velocity we want to achieve toward the target point (which will be in the midway to the target).
	targetGroundVelocity[i] = 2.0 * targetPositionVector[i] / uu->config.pilot_reach_goal_position_time;
    }

    // Truncate target velocity to hold user configured constraints.
    vec2TruncateToSize(targetGroundVelocity, uu->config.drone_max_speed, 0, NULL);
    // lprintf(30, "%s: Info: Target ground velocity normalized: %s\n", PPREFIX(), vec3ToString_st(targetGroundVelocity));	

    lprintf(30, "%s: Info: target position vector %s, targetGroundVelocity: %s\n", PPREFIX(), vec2ToString_st(targetPositionVector), vec2ToString_st(targetGroundVelocity));
    
    // TODO: Maybe do PID in global frame and then translate to drone frame. Wind is in global frame, not drone.
    // Translate everything neccessary to drone frame!
    // Attention: What we currently consider as drone frame is not rotated by drone pitch and roll!!!
    // We compute the target position relative to the drone's position, gravity and drone's yaw.

    vec2_sub(targetPositionDroneFrame, uu->currentWaypoint.position, posEstimatedInRpyFixTime);
    vec2Rotate(targetPositionDroneFrame, targetPositionDroneFrame, -yaw);
    vec2_assign(movingVelocityDroneFrame, movingVelocity);
    vec2Rotate(movingVelocityDroneFrame, movingVelocityDroneFrame, -yaw);
    vec2_assign(targetVelocityDroneFrame, targetGroundVelocity);
    vec2Rotate(targetVelocityDroneFrame, targetVelocityDroneFrame, -yaw);

    lprintf(60, "%s: Info: target position w.r.t. drone: %s, velocity to target: %s\n", PPREFIX(), vec2ToString_st(targetPositionDroneFrame), vec2ToString_st(targetVelocityDroneFrame));

    // This is computed just to see the effect of PID controller in debug output.
    vec2_sub(diffVelocityDroneFrame, targetVelocityDroneFrame, movingVelocityDroneFrame);

    // diffVelocityDroneFrame is the velocity we need to focus on. It may be inreased/decreased by PID controller
    // to accomodate drone construction irregularities and wind.
    targetVelocityThrustDroneFrame[0] = pidControllerStep(&uu->pidX, targetVelocityDroneFrame[0], movingVelocityDroneFrame[0], tdTick);
    targetVelocityThrustDroneFrame[1] = pidControllerStep(&uu->pidY, targetVelocityDroneFrame[1], movingVelocityDroneFrame[1], tdTick);

    lprintf(30, "%s: XY Velocities:  Current: %s  --> Target: %s,  Difference: %s. Target thrust: %s\n", PPREFIX(), vecToString_st(movingVelocityDroneFrame), vecToString_st(targetVelocityDroneFrame), vecToString_st(targetVelocityThrustDroneFrame), vecToString_st(targetVelocityThrustDroneFrame));

    // Infer target pitch and roll from the targetVelocityThrusty. It corresponds to the horizontal velocity
    // we need to achieve in tdPositionFix/2.
    // We suppose that the average vertical thrust made by propellers equals to the gravity acceleration (9.8).
    // In normal conditions the drone is holding altitude which is ensured by the altitude PID controller (for the moment, we do not
    // consider cases where we are climbing or descending very fast). So, first compute the imaginative vertical speed
    // that the motor thrust is producing. I.e. the speed at the moment the drone shall have the maximal horizontal
    // velocity (diffVelocity) to the point X.
    // That imaginative speed is proportional to the gravity acceleration:
    dspeed = GRAVITY_ACCELERATION * 0.5 * uu->config.pilot_reach_goal_position_time;
    
    // Target pitch and roll is the angle between "gravity vertical speed" and drone "target horizontal speed".
    // Strictly speaking it is probably vector between accelerations, but it gives the same result.
    uu->targetPitch = - atan2(targetVelocityThrustDroneFrame[0], dspeed);
    uu->targetRoll = - atan2(targetVelocityThrustDroneFrame[1], dspeed);	
    uu->targetYaw = uu->currentWaypoint.yaw;

    // Normalize angles to <-Pi, Pi> range and apply user constraints from the configuration
    uu->targetPitch = normalizeToRange(uu->targetPitch, -M_PI, M_PI);
    uu->targetRoll = normalizeToRange(uu->targetRoll, -M_PI, M_PI);
    uu->targetPitch = truncateToRange(uu->targetPitch, -uu->config.drone_max_inclination, uu->config.drone_max_inclination, "target pitch", INDEX_NAN);
    uu->targetRoll = truncateToRange(uu->targetRoll, -uu->config.drone_max_inclination, uu->config.drone_max_inclination, "target roll", INDEX_NAN);

    lprintf(30, "%s: Pilot Target RPY: [%g, %g, %g]\n", PPREFIX(), uu->targetRoll, uu->targetPitch, uu->targetYaw);
}


static void pilotAltitudeThrustAverageAndBatteryStatus(double altitudeThrust) {
    static time_t 	currentSumSecond;
    static double 	lastSecondSum = 0;
    static int 		lastSecondNumOfSample = 0;

    // TODO: Maybe put a regression buffer here as well to get the mean value
    if (currentSumSecond == currentTime.sec) {
	lastSecondSum += altitudeThrust;
	lastSecondNumOfSample ++;
    } else {
	// avoid division by zero
	if (lastSecondNumOfSample != 0) {
	    uu->averageAltitudeThrust = lastSecondSum / lastSecondNumOfSample;
	    // for the moment the battery status factor is simply the average of altitudeThrusts for the last second
	    uu->batteryStatusRpyFactor = uu->averageAltitudeThrust;
	    // Hmm. how to infer percentage in some way from this?
	    // uu->config.motor_altitude_thrust_max is probably considered as zero percent
	    // what shall be considered as 100% ???? probably hold
	    uu->batteryStatusPerc = (uu->averageAltitudeThrust - uu->config.motor_altitude_thrust_hold) / (uu->config.motor_altitude_thrust_max - uu->config.motor_altitude_thrust_hold) * 100.0;
	}
	lastSecondSum = 0;
	lastSecondNumOfSample = 0;
	currentSumSecond = currentTime.sec;
    }
    
}

// This is the main function computing main thrust to reach altitude goal
static double pilotComputeAutoThrustForAltitudeHold(double targetAltitude) {
    double 	thrust;
    double 	targetAltitudeSpeed, altitudeSpeed, altitude;
    double 	tdTick;
    
    // if not enough of data do nothing
    if (uu->longBufferPosition.n <= 2) return(-1);
    if (uu->longBufferRpy.n <= 2) return(-1);

    // If we run out of battery land!
    if (uu->averageAltitudeThrust >= uu->config.motor_altitude_thrust_max && uu->flyStage == FS_FLY) {
	lprintf(1, "%s: Info: Battery low. Landing!\n", PPREFIX());
	uu->flyStage = FS_EMERGENCY_LANDING;
    }

    if (uu->flyStage == FS_EMERGENCY_LANDING) targetAltitude = -0.1;
    
    tdTick = pilotGetNormalizedLastStabilizationTickLength();

    altitude = uu->droneLastPosition[2];
    altitudeSpeed = uu->droneLastVelocity[2];

    targetAltitudeSpeed = (targetAltitude - altitude) / uu->config.pilot_reach_goal_position_time;
    // In the old implementatio this was multiplied by 2 supposing we have to reach target speed in middle time?
    // Don't think it makes any difference
    // targetAltitudeSpeed = 2.0 * (targetAltitude - altitude) / uu->config.pilot_reach_goal_position_time;

    vec1TruncateToSize(&targetAltitudeSpeed, uu->config.drone_max_speed, 0, "target altitude speed");

    lprintf(30, "%s: Info: Altitude target: %g current: %g. Altitude speed target: %g current: %g\n", PPREFIX(), targetAltitude, altitude, targetAltitudeSpeed, altitudeSpeed);
    
    // get the Altitude thrust
    thrust = pidControllerStep(&uu->pidAltitude, targetAltitudeSpeed, altitudeSpeed, tdTick);

#if 1
    // This may improve altitude stability
    // TODO: Maybe acceleration data shall be used in the D part of the PID above, or be added to altitude speed?
    thrust += pidControllerStep(&uu->pidAccAltitude, 0, uu->droneLastAcceleration[2], tdTick);
#endif
    
    return(thrust);
}

static double pilotComputeAssistedThrustForAltitude() {
    double 			thrust;
    double 			mtime;
    vec3			acceleration;
    double			accZ;

    if (uu->flyStage == FS_EMERGENCY_LANDING) {
	thrust = uu->config.motor_thrust_min_spin;
    } else {
	regressionBufferGetMean(&uu->shortBufferAcceleration, &mtime, acceleration);
	accZ = acceleration[2];
	thrust = uu->rc.altitude.value;
	// the idea of PID assistance is that we keep climbing speed constant, hence acceleration zero
	// Hmm. how to incorporate barometer and some kind of altitude hold here ?
	thrust += pidControllerStep(&uu->pidAccAltitude, 0, accZ, 1.0/uu->stabilization_loop_Hz);
    }
    return(thrust);
}

static int pilotComputeTargetAltitudeThrust(double *altitudeThrust) {
    double 		thrust, rpFactor;

    if (uu->config.pilot_main_mode == MODE_MANUAL_RC) {
	if (uu->config.manual_rc_altitude.mode == RCM_PASSTHROUGH) {
	    if (uu->flyStage == FS_EMERGENCY_LANDING) {
		thrust = uu->config.motor_thrust_min_spin;
	    } else {
		thrust = uu->rc.altitude.value;
	    }
	} else if (uu->config.manual_rc_altitude.mode == RCM_ACRO) {
	    thrust = pilotComputeAssistedThrustForAltitude();
	} else {
	    thrust = pilotComputeAutoThrustForAltitudeHold(uu->rc.altitude.value);
	}
    } else {
	thrust = pilotComputeAutoThrustForAltitudeHold(uu->currentWaypoint.position[2]);
    }
    if (thrust < 0) return(-1);

    
    thrust += uu->config.motor_altitude_thrust_hold;
    
    // use the new thrust to estimate battery status factor for roll, pitch, yaw control
    pilotAltitudeThrustAverageAndBatteryStatus(thrust);

#if ALTITUDE_THRUST_CORRECTION_FOR_ROLL_PITCH
    // Make a correction to altitude thrust due to current roll, pitch
    rpFactor = fabs(cos(uu->droneLastRpy[0]) * cos(uu->droneLastRpy[0]));
    // avoid excessing thrust from division by small values and/or zero.
    if (rpFactor <= 0.25) rpFactor = 0.25;
    thrust = thrust / rpFactor;
#endif
    
    *altitudeThrust = thrust;
    
    return(0);
}



///////////////////////////////////////////////////////////////////////////////////////////////////////
// This is the main function computing thrust for motors to stabilize drone and get target roll, pitch, yaw.
//

static int pilotComputeTargetRpyRotationVelocity() {
    double 		roll, pitch, yaw;
    double		tdRpyFix;	// td stands for Time Delta
    vec3		cRpy;

    // if not enough of data do nothing
    if (uu->longBufferPosition.n <= 2) return(-1);
    if (uu->longBufferRpy.n <= 2) return(-1);
    if (uu->shortBufferPosition.n <= 2) return(-1);
    if (uu->shortBufferRpy.n <= 2) return(-1);

    // tdRpyFix is the timeframe in which we want to achieve targeted roll, pitch, yaw
    // TODO: Determine this dynamically someway as well as pilot_reach_goal_position_time depending
    // on the actual difference between target and current state.
    tdRpyFix = uu->config.pilot_reach_goal_orientation_time;

#if 1
    
    // get pose at the time of interest
    regressionBufferEstimateForTime(&uu->shortBufferRpy, currentTime.dtime+tdRpyFix/2, cRpy);
    // regressionBufferEstimateForTime(&uu->shortBufferRpy, currentTime.dtime, cRpy);
    
    // some very basic self bug check
    if (vec3_has_nan(cRpy)) {
	// bug, bug, bug, panic.
	lprintf(0, "%s: Some key value is wrong. Panic!\n", PPREFIX());
	mainStandardShutdown(NULL);
	return(-1);
    }

    // Get current roll, pitch and yaw from current position pose.
    // pitch - negative == nose down;      positive == nose up
    // roll  - negative == left wing down; positive == left wing up
    // yaw   - positive == rotated counterclockwise (view from up)

    roll  = cRpy[0];
    pitch = cRpy[1];
    yaw   = cRpy[2];
    
#else
    
    // get current pose 
    if (vec3_has_nan(uu->droneLastRpy)) {
	// bug, bug, bug, panic.
	lprintf(0, "%s: Some key value is wrong. Panic!\n", PPREFIX());
	mainStandardShutdown(NULL);
	return(-1);
    }
    roll = uu->droneLastRpy[0];
    pitch = uu->droneLastRpy[1];
    yaw = uu->droneLastRpy[2];
    
#endif    

    // Compute rotation speed to get to the target roll, pitch, yaw
    // And apply limit constraints from the configuration
    // Actually we shall probably divide by tdRpyFix/2 as we want to accelerate rotation half way and then decelerate
    uu->targetPitchRotationSpeed = angleSubstract(uu->targetPitch, pitch) / tdRpyFix;
    uu->targetPitchRotationSpeed =  truncateToRange(uu->targetPitchRotationSpeed, -uu->config.drone_max_rotation_speed, uu->config.drone_max_rotation_speed, "targetPitchRotationSpeed", INDEX_NAN);
    uu->targetRollRotationSpeed = angleSubstract(uu->targetRoll, roll) / tdRpyFix;
    uu->targetRollRotationSpeed =  truncateToRange(uu->targetRollRotationSpeed, -uu->config.drone_max_rotation_speed, uu->config.drone_max_rotation_speed, "targetRollRotationSpeed", INDEX_NAN);
    uu->targetYawRotationSpeed = angleSubstract(uu->targetYaw, yaw) / tdRpyFix;
    uu->targetYawRotationSpeed =  truncateToRange(uu->targetYawRotationSpeed, -uu->config.drone_max_rotation_speed, uu->config.drone_max_rotation_speed, NULL, INDEX_NAN);
    // lprintf(27, "%s: tdRpyFix == %g\n", PPREFIX(), tdRpyFix);
    lprintf(27, "%s: RPY Orientation: Current: [%7.3f %7.3f %7.3f]  --> Target: [%7.3f %7.3f %7.3f]\n", PPREFIX(), roll, pitch, yaw, uu->targetRoll, uu->targetPitch, uu->targetYaw);

    return(0);
}

static int pilotComputeTargetRpyThrustForRpyVelocity(vec3 rpyThrusts) {
    double		yawThrust, rollThrust, pitchThrust;
    double		tdTick;	// td stands for Time Delta
    double		yawRotationSpeed, rollRotationSpeed, pitchRotationSpeed;
    
    // if not enough of data do nothing
    if (uu->longBufferPosition.n <= 2) return(-1);
    if (uu->longBufferRpy.n <= 2) return(-1);
    if (uu->shortBufferPosition.n <= 2) return(-1);
    if (uu->shortBufferRpy.n <= 2) return(-1);

    // tdTick is the length of one pilot tick, it is used as PID controllers step.
    tdTick = uu->droneLastStabilizationTickLength;
    
    rollRotationSpeed  = uu->droneLastRpyRotationSpeed[0];
    pitchRotationSpeed = uu->droneLastRpyRotationSpeed[1];
    yawRotationSpeed   = uu->droneLastRpyRotationSpeed[2];

    // some very basic self bug check
    if (vec3_has_nan(uu->droneLastRpyRotationSpeed)) {
	// bug, bug, bug, panic.
	lprintf(0, "%s: Some key value has wrong value. Panic!\n", PPREFIX());
	mainStandardShutdown(NULL);
	return(-1);
    }

    lprintf(22, "%s: RPY Speed:       Current: [%7.3f %7.3f %7.3f]  --> Target: [%7.3f %7.3f %7.3f]\n", PPREFIX(), rollRotationSpeed, pitchRotationSpeed, yawRotationSpeed, uu->targetRollRotationSpeed, uu->targetPitchRotationSpeed, uu->targetYawRotationSpeed);

    // Use PID controllers to compute final roll, pitch yaw thrusts to achieve target rotation speeds from current rotation speeds
    // TODO Reconsider: Is voltage sent from ESC proportional to the final speed of the motor+propeller or is it giving them constant acceleration?
    // It is not clear for me how this physics works and if it changes something in the code.
    rollThrust = pidControllerStep(&uu->pidRoll, uu->targetRollRotationSpeed, rollRotationSpeed, tdTick);
    pitchThrust = pidControllerStep(&uu->pidPitch, uu->targetPitchRotationSpeed, pitchRotationSpeed, tdTick);
    yawThrust = pidControllerStep(&uu->pidYaw, uu->targetYawRotationSpeed, yawRotationSpeed, tdTick);

    rpyThrusts[0] = rollThrust;
    rpyThrusts[1] = pitchThrust;
    rpyThrusts[2] = yawThrust;

#if 1
    // roll,pitch yaw thrust shall be multiplied by battery status to accomodate current strength/weakness of the battery
    if (uu->batteryStatusRpyFactor > 0 && uu->batteryStatusRpyFactor <= 1.0) {
	rpyThrusts[0] = rollThrust  * uu->batteryStatusRpyFactor ;
	rpyThrusts[1] = pitchThrust * uu->batteryStatusRpyFactor ;
	rpyThrusts[2] = yawThrust * uu->batteryStatusRpyFactor ;
    } else {
	lprintf(10, "%s: Internal Error: uu->batteryStatusRpyFactor %g out of range 0-1\n", PPREFIX(), uu->batteryStatusRpyFactor);
    }
#endif    

    return(0);
}

static int pilotComputeTargetRpyThrust(vec3 rpyThrusts) {
    int		r;

    if (uu->config.pilot_main_mode == MODE_MANUAL_RC) {
	if (uu->config.manual_rc_roll.mode == RCM_TARGET) uu->targetRoll = uu->rc.roll.value;
	if (uu->config.manual_rc_pitch.mode == RCM_TARGET) uu->targetPitch = uu->rc.pitch.value;
	if (uu->config.manual_rc_yaw.mode == RCM_TARGET) uu->targetYaw = uu->rc.yaw.value;
    }
    
    pilotComputeTargetRpyRotationVelocity();
    
    if (uu->config.pilot_main_mode == MODE_MANUAL_RC) {
	if (uu->config.manual_rc_roll.mode == RCM_ACRO) uu->targetRollRotationSpeed = uu->rc.roll.value;
	if (uu->config.manual_rc_pitch.mode == RCM_ACRO) uu->targetPitchRotationSpeed = uu->rc.pitch.value;
	if (uu->config.manual_rc_yaw.mode == RCM_ACRO) uu->targetYawRotationSpeed = uu->rc.yaw.value;
	
	r = pilotComputeTargetRpyThrustForRpyVelocity(rpyThrusts);
	
	if (uu->config.manual_rc_roll.mode == RCM_PASSTHROUGH) rpyThrusts[0] = uu->rc.roll.value;
	if (uu->config.manual_rc_pitch.mode == RCM_PASSTHROUGH) rpyThrusts[1] = uu->rc.pitch.value;
	if (uu->config.manual_rc_yaw.mode == RCM_PASSTHROUGH) rpyThrusts[2] = uu->rc.yaw.value;	    
    } else {
	r = pilotComputeTargetRpyThrustForRpyVelocity(rpyThrusts);
    }
    return(r);
}

// This is the main stabilization function executed each PILOT_STABILIZATION_TICK_MSEC
void pilotInitiatePids() {
    vec3 	rpyThrust;
    double	altitudeThrust;
    
    uu->droneLastStabilizationTickLength =  1.0/uu->autopilot_loop_Hz;
    pilotGetDronePositionAndVelocityAndStoreInBuffers();
    pilotComputeTargetRollPitchYawForWaypoint();
    
    pilotComputeTargetAltitudeThrust(&altitudeThrust);
    pilotComputeTargetRpyThrust(rpyThrust);
    uu->droneLastTickTime = currentTime.dtime;
}

int64_t pilotScheduleNextTick(double frequency, void (*tickfunction)(void *arg), void *arg) {
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

static void pilotSetMotorThrust() {
    double 	altitudeThrust;
    vec3 	rpyThrust;
    int 	r;

    r = 0;
    r |= pilotComputeTargetRpyThrust(rpyThrust);
    r |= pilotComputeTargetAltitudeThrust(&altitudeThrust);
    if (r == 0) {
	pilotCombineAllMotorThrusts(rpyThrust[0], rpyThrust[1], rpyThrust[2], altitudeThrust);
    } else {
	// We have a problem.
	motorsThrustSet(0);
    }
}

void pilotRegularStabilisationTick(void *d) {
    // This is the main function called at each stabilization tick of standard raspilot
    // lprintf(30, "\n");
    lprintf(100, "%s: Stabilization tick\n", PPREFIX());
    nextPidTickUsec = pilotScheduleNextTick(uu->stabilization_loop_Hz, pilotRegularStabilisationTick, NULL);
    uu->droneLastStabilizationTickLength = pilotGetNormalizedLastStabilizationTickLength();    
    pilotGetDronePositionAndVelocityAndStoreInBuffers();
    if (uu->flyStage >= FS_PRE_FLY) {
	if (uu->flyStage >= FS_FLY) {
	    pilotSetMotorThrust();
	}
	pilotSendThrusts(NULL);
    }
    // we are done. Values updated for currenbt tick.
    uu->droneLastTickTime = currentTime.dtime;
}

void pilotRegularMissionModeLoopTick(void *d) {
    // This is the main function called at each tick of mission raspilot.
    // lprintf(30, "\n");
    lprintf(100, "%s: Mission tick\n", PPREFIX());
    nextStabilizationTickUsec = pilotScheduleNextTick(uu->autopilot_loop_Hz, pilotRegularMissionModeLoopTick, NULL);
    if (uu->flyStage >= FS_FLY) {
	pilotComputeTargetRollPitchYawForWaypoint();
    }
}

static void pilotGimbalSendAxis(int streamType, int value) {
    struct deviceStreamData 	*ddl;
    struct baio 		*bb;
    
    for(ddl=uu->deviceStreamDataByType[streamType]; ddl!=NULL; ddl=ddl->nextWithSameType) {
	bb = baioFromMagic(ddl->dd->baioMagic);
	if (bb != NULL) {
	    // lprintf(0, "%s: sending gimbal: %s %d\n", PPREFIX(), ddl->tag, value);
	    baioPrintfToBuffer(bb, "%s %d\n", ddl->tag, value);
	}
    }
}

void pilotRegularSendGimbalPwm(void *d) {
    // TODO: !!!
    static int 			lastx = 5000;
    static int 			lasty = 5000;
    char			ttt[TMP_STRING_SIZE];
    struct baio			*bb;
    struct deviceStreamData 	*ddl;
    double			gx, gy;
    int				x, y;

    if (shutDownInProgress) return;

    pilotScheduleNextTick(200, pilotRegularSendGimbalPwm, NULL);

    gx = uu->targetGimbalX + uu->droneLastRpy[0];
    gy = uu->targetGimbalY - uu->droneLastRpy[1];
    
    // translate -Pi/2 Pi/2 to 0 - 1000
    x = 5000 + gx / M_PI * 10000;
    y = 5000 + gy / M_PI * 10000;

    // lprintf(0, "%s: gimbal x,y == %d, %d\n", PPREFIX(), x, y);

    if (x < 0) x = 0;
    if (x >= 10000) x = 9999;
    if (y < 0) y = 0;
    if (y >= 10000) y = 9999;

    if (x != lastx) {
	lastx = x;
	pilotGimbalSendAxis(DT_GIMBAL_X, x);
    }
    if (y != lasty) {
	lasty = y;
	pilotGimbalSendAxis(DT_GIMBAL_Y, y);
    }
    
}

void pilotRegularMotorTestModeTick(void *d) {
    // Ticks used in other modes than flying like ESC calibration, motor testing, etc.
    pilotScheduleNextTick(PILOT_PRELAUNCH_FREQUENCY_HZ, pilotRegularMotorTestModeTick, NULL);
    pilotSendThrusts(NULL);
}

void pilotRegularStabilizationLoopRescheduleToSoon() {
    int		r;
    uint64_t	tt;

    tt = currentTime.usec + 500;
    if (nextPidTickUsec > tt) {
	r = timeLineRescheduleUniqueEventIfExisted(tt, pilotRegularStabilisationTick, NULL);
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
		dd->ddt[j]->mandatory &&
		(
		    dd->ddt[j]->input == NULL ||
		    dd->ddt[j]->input->buffer.n < dd->ddt[j]->input->buffer.size ||
		    uu->pilotLaunchTime + dd->warming_time > currentTime.dtime
		    )
		) {
		// {
		// if (dd->connection.type ras!= DCT_INTERNAL && dd->lastActivityTime <= uu->pilotStartingTime) {
		// this sensor did not send data yet, continue waiting
		if (debugLevel > 0 && currentTime.msec > lastWaitingMsgMsec + 3000) {
		    if (lastWaitingMsgMsec != 0) {
			lprintf(0, "%s: Info: waiting for device: %s.", PPREFIX(), dd->name);
			mavlinkPrintfStatusTextToListeners("waiting for device: %s.", dd->name);
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


