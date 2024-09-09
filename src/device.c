#include "common.h"


int deviceIsSharedMemoryDataStream(struct deviceStreamData *ddl) {
    int shmFlag;
    switch (ddl->type) {
    case DT_POSITION_SHM:
    case DT_ORIENTATION_RPY_SHM:
    case DT_EARTH_ACCELERATION_SHM:
	shmFlag = 1;
	break;
    default:
	shmFlag = 0;
	break;
    }
    return(shmFlag);
}

//////////////////////////////////////////////////////////////////////////////////////////////////
// parsing output from devices

static int parsePong(char *tag, char *p, struct deviceData *dd, struct deviceStreamData *ddd) {
    double tstamp, lat;
    
    tstamp = atof(p) / 1000000.0;
    lat = currentTime.dtime-tstamp;
    lprintf(100 - ddd->debug_level, "%s: Info: %s: latency: %g ms.\n", PPREFIX(), dd->name, lat*1000);
    // for statistics
    ddd->pongTotalTimeForStatistics += lat;
    return(0);
}

static int parseDeviceDebugPrint(char *tag, char *p, struct deviceData *dd, struct deviceStreamData *ddd) {
    lprintf(100 - ddd->debug_level, "%s: Info: %s: Debug: %s\n", PPREFIX(), dd->name, p);
    return(0);
}

static int parseVector(double *rr, int length, char *tag, char *s, struct deviceData *dd, struct deviceStreamData *ddd) {
    int		i;
    char 	*p, *pe;
    double 	*v;

    p = s;
    lprintf( 50, "%s: parsing vector%d %s\n", PPREFIX(), length, p);

    assert(rr != NULL);
    for(i=0; i<length; i++) {
	rr[i] = strtod(p, &pe);
#if TEST1	
	rr[i] = strtodn(p, &pe);
#endif	
	if (pe == p) return(-1);
	p = pe;    
    }
    ddd->input->confidence = 1.0;
    lprintf(100 - ddd->debug_level, "%s: %s: %s: parsed %s --> %s\n", PPREFIX(), dd->name, tag, s, arrayWithDimToStr_st(rr, length));
    return(0);
}

static double parseNmeaLatitudeOrLongitude(char *p, int di) {
    int 	i;
    double 	deg, min;
	
    SKIP_SPACE(p);
    deg = 0;
    for(i=0; i<di && p[i]; i++) deg = deg * 10 + (p[i] - '0');
    min = strtod(p+i, NULL);
    return(deg + min / 60.0);
}

int parseNmeaPosition(double *rr, char *tag, char *s, struct deviceData *dd, struct deviceStreamData *ddd) {
    char				*p;
    int					r, unit;
    double				latitude, longitude, altitude;
    double				x,y,z;
    int					latdir, longdir, fixtype;
    double 				lat_distance;
    double 				lng_distance;

    p = s;
    SKIP_SPACE(p);

    // if not a coordinate message, ignore
    if (strlen(tag) != 6) return(-1);
    if (strcmp(tag+3, "GGA") != 0) return(-1);

    assert(rr != NULL);
    
    // TODO: check checksum!
    NMEA_NEXT_FIELD(p);
    NMEA_NEXT_FIELD(p);
    latitude = parseNmeaLatitudeOrLongitude(p, 2);
    NMEA_NEXT_FIELD(p);
    SKIP_SPACE(p);
    latdir = *p;
    NMEA_NEXT_FIELD(p);
    longitude = parseNmeaLatitudeOrLongitude(p, 3);
    NMEA_NEXT_FIELD(p);
    SKIP_SPACE(p);
    longdir = *p;
    NMEA_NEXT_FIELD(p);
    SKIP_SPACE(p);
    fixtype = *p;
    NMEA_NEXT_FIELD(p);
    NMEA_NEXT_FIELD(p);
    NMEA_NEXT_FIELD(p);
    altitude = strtod(p, NULL);
    // lprintf("%s: NMEA got %g %c ; %g %c\n", PPREFIX(), latitude, latdir, longitude, longdir);

    if (latdir == 'S' || latdir == 's') latitude = -latitude;
    if (longdir == 'W' || longdir == 'w') longitude = -longitude;

    switch (fixtype) {
    case '0':
	// Invalid
	ddd->input->confidence = 0.0;
	break;
    case '1':
	// Autonomous GPS fix, no correction data used.
	ddd->input->confidence = 0.7;
	break;
    case '2':
	// DGPS fix, using a local DGPS base station or correction service such as WAAS or EGNOS.
	ddd->input->confidence = 0.8;
	break;
    case '3':
	// PPS fix ???
	ddd->input->confidence = 0.6;
	break;
    case '4':
	// RTK fix, high accuracy Real Time Kinematic.
	ddd->input->confidence = 1.0;
	break;
    case '5':
	// RTK Float, better than DGPS, but not quite RTK.
	ddd->input->confidence = 0.9;
	break;
    case '6':
	// Estimated fix (dead reckoning).
	ddd->input->confidence = 0.5;
	break;
    default:
	// We do not know what
	ddd->input->confidence = 0.3;
	break;
    }

    // TODO: Figure out something more precise.
    lat_distance = 111000.0;
    lng_distance = 111321.0 * cos(latitude * M_PI/180);

    x = lng_distance * longitude;
    y = lat_distance * latitude;
    z = altitude;

    // TODO: We actually need to translate z,y,z global coordinates to GBASE coordinates.
    // I.e. to the coordinates relative to the drone starting point and to drone starting orientation!
    
    rr[0] = x;
    rr[1] = y;
    rr[2] = z;

    // TODO: confidence based on number of satelites?
    lprintf(100 - ddd->debug_level, "%s: %s: %s: parsed %s --> %s\n", PPREFIX(), dd->name, tag, s, arrayWithDimToStr_st(rr, 3));
    return(0);
}

struct deviceData *deviceFindByName(char *name) {
    int i;
    for(i=0; i<uu->deviceMax; i++) {
	if (strcmp(uu->device[i]->name, name) == 0) return(uu->device[i]);
    }
    return(NULL);
}

struct deviceStreamData *deviceFindStreamByName(struct deviceData *dd, char *name) {
    int i;
    for(i=0; i<dd->ddtMax; i++) {
	if (strcmp(dd->ddt[i]->name, name) == 0) return(dd->ddt[i]);
    }
    return(NULL);
}

struct deviceStreamData *deviceFindStreamByType(struct deviceData *dd, int type) {
    int i;
    for(i=0; i<dd->ddtMax; i++) {
	if (dd->ddt[i]->type == type) return(dd->ddt[i]);
    }
    return(NULL);
}

void manualPilotSetControl(struct manualControlState *cc, double rc_value, struct manual_rc *ss, char *controlName, int loglevel) {
    double newvalue, newbase;
    
    InternalCheck(rc_value >= 0 || rc_value <= 1);

    cc->rc_value = rc_value;
    newbase = cc->base;
    if (rc_value < ss->scroll_zone) {
	// scroll 'down'
	newbase = cc->base - (currentTime.dtime - cc->lastUpdateDtime) * ss->scroll_speed;
	if (newbase < ss->min + 0.5 * ss->sensitivity) newbase = ss->min + 0.5 * ss->sensitivity;
    } else if (rc_value > 1.0 - ss->scroll_zone) {
	// scroll 'up'
	newbase = cc->base + (currentTime.dtime - cc->lastUpdateDtime) * ss->scroll_speed;
	if (newbase > ss->max - 0.5 * ss->sensitivity) newbase = ss->max - 0.5 * ss->sensitivity;
    }
    
    cc->base = newbase;
    
    //printf("%s: rcvalue: %g;   base: %g --> %g; min_zone: %g\n", controlName, rc_value, oldbase, cc->base, ss->min_zone);

    if (rc_value > 0.5 - ss->middle_neutral_zone/2 && rc_value < 0.5 + ss->middle_neutral_zone/2) {
	newvalue = cc->base;
	//printf("%s: case 0: value: %g --> %g;\n", controlName, cc->value, newvalue);
    } else if (rc_value < 0.5) {
	newvalue = (rc_value - 0.5 + ss->middle_neutral_zone/2) * ss->sensitivity + cc->base;
	//printf("%s: case 1: value: %g --> %g;\n", controlName, cc->value, newvalue);
    } else {
	newvalue = (rc_value - 0.5 - ss->middle_neutral_zone/2) * ss->sensitivity + cc->base;
	//printf("%s: case 2: value: %g --> %g;\n", controlName, cc->value, newvalue);
    }

    // normally useless, but be sure anyway
    if (newvalue < ss->min) newvalue = ss->min;
    if (newvalue > ss->max) newvalue = ss->max;
    
    //printf("%s: value: %g --> %g;\n", controlName, cc->value, newvalue);

    cc->value = newvalue;
    cc->lastUpdateDtime = currentTime.dtime;

    // Hmm. Make this step configurable
    // lprintf(0, "%s: Warning: manual control %s: raw %g --> %g\n", PPREFIX(), controlName, rc_value, cc->value);
    if (fabs(cc->lastReportedValue - cc->value) >= 0.01) {
	lprintf(loglevel, "%s: manual control %s: raw %g --> %g\n", PPREFIX(), controlName, rc_value, cc->value);
	// lprintf(0, "%s: Warning: manual control %s: raw %g --> %g\n", PPREFIX(), controlName, rc_value, cc->value);
	// printf("%s: manual control %s: raw %g --> %g\n", PPREFIX(), controlName, rc_value, cc->value);
	if (loglevel < 5) {
	    mavlinkPrintfStatusTextToListeners("rc: %s: %g", controlName, cc->value);
	}
	cc->lastReportedValue = cc->value;
    }
}

void manualPilotSetRoll(double vv) {
    manualPilotSetControl(&uu->rc.roll, vv, &uu->config.manual_rc_roll, "roll", 20);
}
void manualPilotSetPitch(double vv) {
    manualPilotSetControl(&uu->rc.pitch, vv, &uu->config.manual_rc_pitch, "pitch", 20);
}
void manualPilotSetYaw(double vv) {
    manualPilotSetControl(&uu->rc.yaw, vv, &uu->config.manual_rc_yaw, "yaw", 20);
}
void manualPilotSetAltitude(double vv) {
    int loglevel;
    
    if (uu->flyStage <= FS_PRE_FLY) {
	loglevel = 2;
    } else {
	loglevel = 20;
    }
    manualPilotSetControl(&uu->rc.altitude, vv, &uu->config.manual_rc_altitude, "altitude", loglevel);
}

void manualControlInit(struct manualControlState *ss, struct manual_rc *mm) {
    ss->lastReportedValue = 0;
    ss->rc_value = 0.5;
    ss->lastUpdateDtime = currentTime.dtime;
    ss->base = mm->initial_scroll_middle;
    manualPilotSetControl(ss, 0.5, mm, "Init", 99);
}

void manualControlRegularCheck(void *d) {
    pilotScheduleNextTick(2, manualControlRegularCheck, NULL);

    manualPilotSetControl(&uu->rc.roll, uu->rc.roll.rc_value, &uu->config.manual_rc_roll, "roll", 20);
    manualPilotSetControl(&uu->rc.pitch, uu->rc.pitch.rc_value,  &uu->config.manual_rc_pitch, "pitch", 20);
    manualPilotSetControl(&uu->rc.yaw, uu->rc.yaw.rc_value, &uu->config.manual_rc_yaw, "yaw", 20);
    manualPilotSetControl(&uu->rc.altitude, uu->rc.altitude.rc_value, &uu->config.manual_rc_altitude, "altitude", 20);

#if 0
    uu->targetGimbalX += uu->manual.gimbalXIncrementPerSecond / uu->autopilot_loop_Hz;
    uu->targetGimbalY += uu->manual.gimbalYIncrementPerSecond / uu->autopilot_loop_Hz;

    uu->targetGimbalX = truncateToRange(uu->targetGimbalX, -M_PI/2, M_PI/2, NULL, INDEX_NAN);
    uu->targetGimbalY = truncateToRange(uu->targetGimbalY, -M_PI/2, M_PI/2, NULL, INDEX_NAN);
#endif    
}

// Jstest Joystick lines look like: Event: type 2, time 8695440, number 0, value 6808
// TODO: Rewrite this to use functions above
int parseJstestJoystickFlighControl(double *rr, char *tag, char *s, struct deviceData *dd, struct deviceStreamData *ddd) {
    char 			*stype;
    char 			*snumber;
    char 			*svalue;
    int 			type, number;
    int				ivalue;
    int				loglevel;
    double			dvalue, alt;
    struct deviceData 		*gyro;
    struct deviceStreamData 	*ggg;
    
    stype = strstr(s, "type ");
    if (stype == NULL) return(-1);
    snumber = strstr(s, "number ");
    if (snumber == NULL) return(-1);
    svalue = strstr(s, "value ");
    if (svalue == NULL) return(-1);

    if (uu->pilotLaunchTime + dd->warming_time > currentTime.dtime) return(0);

    
    type = atoi(stype + strlen("type "));
    number = atoi(snumber + strlen("number "));
    ivalue = atoi(svalue + strlen("value "));
    // normalize to range 0-1
    dvalue = ivalue / 65536.0 + 0.5;

    if (type == 1) {
	
	// buttton
	if (1 || uu->flyStage >= FS_PRE_FLY) {
	    lprintf(0, "%s: Unknown joystick button pressed. Emergency landing!\n", PPREFIX());
	    uu->flyStage = FS_EMERGENCY_LANDING;
	}
	
    } else if (type == 2) {
	
	// joystick control
	switch (number) {
	case 0:
	    // In my joystick, ax 0 is roll
	    manualPilotSetRoll(dvalue);
	    break;
	case 1:
	    // In my joystick, ax 1 is pitch
	    manualPilotSetPitch(dvalue);
	    break;
	case 2:
	    manualPilotSetYaw(dvalue);
	case 3:
	    manualPilotSetAltitude(1.0-dvalue);
	    break;
#if 1
	case 4:
	    // gimbal X
	    uu->rc.gimbalXIncrementPerSecond = 2 * signd(dvalue) * M_PI / 2;
	    break;
	case 5:
	    // gimbal Y
	    uu->rc.gimbalYIncrementPerSecond = 2 * - signd(dvalue) * M_PI / 2;
	    break;
#endif	    
	default:
	    break;
	}
	
    }
    return(0);
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// translate vector read from device to position and/or orientation of drone

#if 0
static void deviceDeduceMountOrientationFromQuaternion(struct deviceData *dd, double *vv) {
    double y,p,r;
    // Add mount RPY.
    // Hmm. Maybe I shall just parse and store here and deduce mount_rpy during sensor fusion
    // TODO: maybe I shall convert mount rpy to quaternion and multipy quats here!
    quatToRpy(vv, &r, &p, &y);
    // lprintf(1, "%s: debug got orientation rpy %f %f %f\n", PPREFIX(), r, p, y);
    // apply mount correction
    r -= dd->mount_rpy[0];
    p -= dd->mount_rpy[1];
    y -= dd->mount_rpy[2];
    // lprintf(1, "%s: debug after sub mount %s\n", PPREFIX(), vec3ToString_st(&pp.pr[7]));
    rpyToQuat(r, p, y, vv);
}
#endif

static void deviceSensorPositionToDronePosition(vec3 resDronePosition, vec3 sensorPosition, struct deviceData *dd, double time) {
    quat 	ii,droneOrientation;
    vec3	mm, ww;
    double	*pose;
    double 	r,p,y;
    
    // translate from mount point to drone center of gravity
    raspilotRingBufferFindRecordForTime(uu->historyPose, time, NULL, &pose);
    if (pose == NULL) {
	// no info about orientation, suppose we are on level
	r = p = y = 0;
    } else {
	r = pose[3];
	p = pose[4];
	y = pose[5];
    }
    // TODO, do the rotation by r,p,y directly here
    rpyToQuat(r, p, y, droneOrientation);
    quat_inverse(ii, droneOrientation);
    quat_mul_vec3(mm, ii, dd->mount_position);
    vec3_add(resDronePosition, mm, sensorPosition);
    //lprintf(PILOT_SENSOR_MERGE_DEBUG_LEVEL,"%s: --> %s\n", PPREFIX(), vecToString_st(resDronePosition));
    vec3_sub(resDronePosition, resDronePosition, dd->mount_position);
    //lprintf(PILOT_SENSOR_MERGE_DEBUG_LEVEL,"%s: --> %s\n", PPREFIX(), vecToString_st(resDronePosition));
}


static void deviceTranslateBottomRangeToAltitude(struct deviceData *dd, struct deviceStreamData *ddd, double sampleTime, double *ran, double *altitude) {
    double range, alt;
    double *pose;
    
    range = *ran;
    ddd->input->confidence = 1.0;
    if (range < ddd->min_range) ddd->input->confidence = 0;
    if (range > ddd->max_range) ddd->input->confidence = 0;
    // we suppose that at the launch time the rangefinder looks downward!!!
    // translate range to altitude
    if (! ddd->launchPoseSetFlag) {
	alt = range;
    } else {
	raspilotRingBufferFindRecordForTime(uu->historyPose, sampleTime, NULL, &pose);
	alt = range * fabs(cos(pose[3]) * cos(pose[4]));
	// lprintf(0, "range, r,p alt == %8f, %8f, %8f --> %8f\n", range, pose[3], pose[4], alt);
	alt -= ddd->launchData[0];
    }
    *altitude = alt;
}

static void deviceTranslateFlowXYtoDronePositionXY(struct deviceData *dd, struct deviceStreamData *ddd, double previousSampleTime, double sampleTime, double *angleFlow, double *dronePosXY) {
    double			x,y,alt;
    double			d[3], p[3], dp[3];
    double			*pose, *oldpose;
    double			roll, pitch, yaw;
    double 			oldroll, oldpitch;

    dronePosXY[0] = dronePosXY[1] = 0;

    if (ddd->input == NULL) {
	lprintf(0, "%s: Error: no input buffer for %s.%s.\n", PPREFIX(), dd->name, ddd->name);
	ddd->input->confidence = 0;
	return;
    }
    
    if (previousSampleTime >= sampleTime) {
	lprintf(0, "%s: Error: time inversion. Probably too small input buffer %s.\n", PPREFIX(), ddd->input->buffer.name);
	ddd->input->confidence = 0;
	return;
    }

    raspilotRingBufferFindRecordForTime(uu->historyPose, previousSampleTime, NULL, &oldpose);
    raspilotRingBufferFindRecordForTime(uu->historyPose, sampleTime, NULL, &pose);

    if (pose == NULL || oldpose == NULL) return;
    
    // 0.05 is a hack, the alt of sensor when on ground
    // TODO: Compute this automatically from launchPose and mount point
    alt = pose[2] + 0.05;

    if (alt <= 0.02 || uu->historyPose == NULL || uu->historyPose->n == 0 || ddd->input == NULL || ddd->input->buffer.n == 0) {
	ddd->input->confidence = 0;
	return;
    }

    oldroll = oldpose[3];
    oldpitch = oldpose[4];
    roll = uu->droneLastRpy[0];
    pitch = uu->droneLastRpy[1];
    yaw = uu->droneLastRpy[2];

    // If we are too much inclined ignore this sensor
    if (roll <= - M_PI/4 || roll >= M_PI/4
	|| pitch <= - M_PI/4 || pitch >= M_PI/4
	|| oldroll <= - M_PI/4 || oldroll >= M_PI/4
	|| oldpitch <= - M_PI/4 || oldpitch >= M_PI/4) {
	ddd->input->confidence = 0;
	return;
    }
    
    // lprintf(100 - ddd->debug_level, "imaginary flow due to rotation: %f, %f\n", imaginaryincrx, imaginaryincry);

    // The sensor sends angles of the optical shift/rotation in degrees.
    // Translate it to distance while deducing the difference in roll,pitch.
    // Not sure which formula is better. Seems that the first one is a bit more stable.


#if 1
    d[0] = alt * (tan(angleFlow[0]) - tan(pitch) + tan(oldpitch));
    d[1] = alt * (tan(angleFlow[1]) - tan(roll) + tan(oldroll));
#elif 0
    d[0] = alt * (sin(angleFlow[0]) - tan(pitch) + tan(oldpitch));
    d[1] = alt * (sin(angleFlow[1]) - tan(roll) + tan(oldroll));
#else
    d[0] = alt * (tan(oldpitch+angleFlow[0]) - tan(pitch));
    d[1] = alt * (tan(oldroll+angleFlow[1]) - tan(roll));
#endif

    // rotate by yaw
    vec2Rotate(d, d, yaw);
    
    p[0] = oldpose[0] + d[0];
    p[1] = oldpose[1] + d[1];
    p[2] = 0;
    
    // lprintf(100 - ddd->debug_level, "oldx,y == %f, %f; flow increment %f, %f: new computed x, y:  %f, %f\n", oldpose[0], oldpose[1], angleFlow[0], angleFlow[1], x, y);
    // lprintf(0, "mmm-motion %f %f\n", angleFlow[0], angleFlow[1]);
    // lprintf(0, "mmm-motion %f %f\n", x, y);
    
    // translate from sensor to drone position
    deviceSensorPositionToDronePosition(dp, p, dd, sampleTime);
	
    dronePosXY[0] = dp[0];
    dronePosXY[1] = dp[1];
    return;
}

// This is the main function processing line read from a device. It parses the input line,
// and add it to the inputBuffer together with its timestamp.
void deviceParseInputStreamLineToInputBuffer(struct deviceData *dd, char *s, int n) {	
    struct deviceStreamData 		*ddd;
    double				*inputVector;
    double				sampletime;
    char				*p, *t, *tag;
    int					i, taglen, r, tagFoundFlag;

    lprintf( 66, "%s: %s: Parsing line: %*.*s\n", PPREFIX(), dd->name, n, n, s);

    p = s;
    SKIP_SPACE(p);

    tagFoundFlag = 0;
    // There are maybe two, three streams per device. Check one after another.
    for(i=0; i<dd->ddtMax; i++) {
	ddd = dd->ddt[i];
	assert(ddd != NULL);
	tag = ddd->tag;
	taglen = strlen(tag);
	if (strncmp(p, tag, taglen) == 0) {
	    tagFoundFlag = 1;
	    t = p + taglen;
	    if (deviceDataStreamVectorLength[ddd->type] == 0 || ddd->input == NULL) {
		inputVector = NULL;
	    } else {
		inputVector = raspilotRingBufferGetFirstFreeVector(&ddd->input->buffer);
		memset(inputVector, 0, deviceDataStreamVectorLength[ddd->type] * sizeof(double));
	    }
	    sampletime = currentTime.dtime ;
	    switch(ddd->type) {
	    case DT_VOID:
		assert(deviceDataStreamVectorLength[ddd->type] == 0);
		r = 0;
		break;
	    case DT_DEBUG:
		assert(deviceDataStreamVectorLength[ddd->type] == 0);
		r = parseDeviceDebugPrint(tag, t, dd, ddd);
		break;
	    case DT_PONG:
		assert(deviceDataStreamVectorLength[ddd->type] == 0);
		r = parsePong(tag, t, dd, ddd);
		break;
	    case DT_ORIENTATION_RPY:
		assert(deviceDataStreamVectorLength[ddd->type] == 3);
		r = parseVector(inputVector, deviceDataStreamVectorLength[ddd->type], tag, t, dd, ddd);
		break;
	    case DT_EARTH_ACCELERATION:
		assert(deviceDataStreamVectorLength[ddd->type] == 3);
		r = parseVector(inputVector, deviceDataStreamVectorLength[ddd->type], tag, t, dd, ddd);
		break;
		/*
	    case DT_ORIENTATION_QUATERNION:
		assert(deviceDataStreamVectorLength[ddd->type] == 4);
		r = parseVector(inputVector, deviceDataStreamVectorLength[ddd->type], tag, t, dd, ddd);
		break;
		*/
	    case DT_POSITION_NMEA:
		r = parseNmeaPosition(inputVector, tag, t, dd, ddd);
		break;
	    case DT_MAGNETIC_HEADING_NMEA:
		lprintf(0, "%s: Not yet implemented\n", PPREFIX());
		r = 0;
		break;
	    case DT_JSTEST:
		r = parseJstestJoystickFlighControl(inputVector, tag, t, dd, ddd);
		break;
	    case DT_POSITION_VECTOR:
		assert(deviceDataStreamVectorLength[ddd->type] == 3);
		r = parseVector(inputVector, deviceDataStreamVectorLength[ddd->type], tag, t, dd, ddd);
		break;
	    case DT_BOTTOM_RANGE:
		assert(deviceDataStreamVectorLength[ddd->type] == 1);
		r = parseVector(inputVector, deviceDataStreamVectorLength[ddd->type], tag, t, dd, ddd);
		break;
	    case DT_FLOW_XY:
		assert(deviceDataStreamVectorLength[ddd->type] == 2);
		r = parseVector(inputVector, deviceDataStreamVectorLength[ddd->type], tag, t, dd, ddd);
		break;
	    case DT_ALTITUDE:
		assert(deviceDataStreamVectorLength[ddd->type] == 1);
		r = parseVector(inputVector, deviceDataStreamVectorLength[ddd->type], tag, t, dd, ddd);
		break;
	    case DT_TEMPERATURE:
		assert(deviceDataStreamVectorLength[ddd->type] == 1);
		r = parseVector(inputVector, deviceDataStreamVectorLength[ddd->type], tag, t, dd, ddd);
		break;
	    case DT_MAGNETIC_HEADING:
		assert(deviceDataStreamVectorLength[ddd->type] == 3);
		r = parseVector(inputVector, deviceDataStreamVectorLength[ddd->type], tag, t, dd, ddd);
		break;
	    default:
		if (! dd->data_ignore_unknown_tags) printf("%s: %s: Error: Tag %s not implemented!\n", PPREFIX(), dd->name, tag);
		r = -1;
	    }
	    if (r == 0) {
		ddd->totalNumberOfRecordsReceivedForStatistics ++;
		if (deviceDataStreamVectorLength[ddd->type] > 0) {
		    assert(inputVector != NULL);
		    assert(ddd->input != NULL);
		    raspilotRingBufferAddElem(&ddd->input->buffer, sampletime, inputVector);
		    ddd->input->confidence = 1.0;
		    lprintf(90 - ddd->debug_level, "%s: %s: %s: time: %lld: Parsed %s\n", PPREFIX(), dd->name, tag, (long long)sampletime, arrayWithDimToStr_st(inputVector, ddd->input->buffer.vectorsize));
		}
	    } else {
		printf("%s:  %s: Error %d when parsing line: %*.*s\n", PPREFIX(), dd->name, r, n, n, s);
	    }
	}
    }
    if (tagFoundFlag == 0 && ! dd->data_ignore_unknown_tags) lprintf( 22, "%s:  %s: Warning: No data tag found in line: %*.*s\n", PPREFIX(), dd->name, n, n, s);
    return;
}

static void deviceMaybeAddDriftToOutputVector(struct deviceData *dd, struct deviceStreamData *ddd, double outputVector[DEVICE_DATA_VECTOR_MAX]) {
    int 	i;
    double 	td;
    
    if (ddd->driftOffsetLastIncrementTime != 0) {
	td = currentTime.dtime - ddd->driftOffsetLastIncrementTime;
	if (td < 0 || td > 10.0) {
	    lprintf(0, "%s: %s: %s: Drift offset time delta %g out of range.\n", PPREFIX(), dd->name, ddd->name, td);
	} else {
	    for(i=0; i<deviceDataStreamVectorLength[ddd->type]; i++) {
		ddd->driftOffset[i] += td * ddd->drift_offset_per_second[i];
		outputVector[i] = outputVector[i] + ddd->driftOffset[i];
	    }
	    ddd->driftOffsetLastIncrementTime = currentTime.dtime;
	}
    }
}

static void deviceRegularAutoAdjustementOfDrifts(void *aaa) {
    struct deviceStreamDataDriftUpdateStr 	*a;
    struct deviceData 				*dd;
    struct deviceStreamData 			*ddd;
    int						i;
    double					vv[DEVICE_DATA_VECTOR_MAX];
    double					actualValue, deviceValue;
    double					drift;
    double					driftFixTime;
    
    a = (struct deviceStreamDataDriftUpdateStr *) aaa;
    ddd = a->ddd;
    i = a->i;
    dd = ddd->dd;

    lprintf(60, "%s: driftAutoUpdate: %s:%s:%d\n", PPREFIX(), dd->name, ddd->name, i);

    // if somebody set drift_auto_fix_period to zero do nothing more.
    driftFixTime = ddd->drift_auto_fix_period[i] * 2;
    if (driftFixTime == 0) {
	lprintf(0, "%s: Info: Removing stream from drift auto updates.\n",
		PPREFIX(), deviceDataTypeNames[ddd->type], dd->name, ddd->name
	    );
	return;
    }
    
    // schedule next update
    timeLineInsertEvent(currentTime.usec+ddd->drift_auto_fix_period[i]*1000000, deviceRegularAutoAdjustementOfDrifts, a);

    // Fix drift also during waiting for sensors, for now.
    if (uu->flyStage < FS_PRE_FLY) return;

    regressionBufferEstimateForTime(&ddd->outputBuffer, currentTime.dtime, vv);
    deviceValue = vv[i];

    //double			drift_auto_fix_period[DEVICE_DATA_VECTOR_MAX];
    //double   			drift_offset_per_second[DEVICE_DATA_VECTOR_MAX];
    //double			driftOffset[DEVICE_DATA_VECTOR_MAX];
    //double			driftOffsetLastIncrementTime;

    switch (ddd->type) {
    case DT_ORIENTATION_RPY:
    case DT_ORIENTATION_RPY_SHM:
	// for now we are only fixing drifting in yaw
	if (i != 2) {
	    lprintf(0, "%s: Error: driftAutoUpdate %s: %s:%s. Only yaw drift is implemented at the moment.\n",
		    PPREFIX(), deviceDataTypeNames[ddd->type], dd->name, ddd->name
		);
	    return;
	}
	actualValue = uu->droneLastRpy[2];
	drift = actualValue - deviceValue;
	// we are supposed to fix the drift in drift_auto_fix_period * 2;
	ddd->drift_offset_per_second[i] = drift / driftFixTime;
	lprintf(20, "%s: Info: driftAutoUpdate %s: %s:%s. Yaw drift set to %g.\n",
		PPREFIX(), deviceDataTypeNames[ddd->type], dd->name, ddd->name, ddd->drift_offset_per_second[i]
	    );
	ddd->driftOffsetLastIncrementTime = currentTime.dtime;
	break;
    default:
	lprintf(0, "%s: Error: driftAutoUpdate for type %s: %s:%s not yet implemented\n", PPREFIX(), deviceDataTypeNames[ddd->type], dd->name, ddd->name);
	break;
    }    
    
}

static void deviceInitiateRegularAutoAdjustementOfDrifts(struct deviceData *dd) {
    int 					j, i;
    struct deviceStreamData 			*ddd;
    struct deviceStreamDataDriftUpdateStr 	*a;

    for(j=0; j<dd->ddtMax; j++) {
	ddd = dd->ddt[j];
	for(i=0; i<deviceDataStreamVectorLength[ddd->type]; i++) {
	    if (ddd->drift_auto_fix_period[i] != 0) {
		ALLOC(a, struct deviceStreamDataDriftUpdateStr);
		a->ddd = ddd;
		a->i = i;
		timeLineInsertEvent(UTIME_AFTER_MSEC(100), deviceRegularAutoAdjustementOfDrifts, a);
	    }
	}
    }
}


// This is the main function translating inputBuffer to outputBuffer, i.e. translating raw data read from the device
// to the drone position and/or orientation to be fused by pilot.
void deviceTranslateInputToOutput(struct deviceStreamData *ddd) {
    struct deviceData			*dd;
    double				previousSampleTime, sampletime, td;
    double				*inputVector;
    double				outputVector[DEVICE_DATA_VECTOR_MAX];
    int					i, j, imax, count, li, ii;
    
    dd = ddd->dd;

    lprintf( 66, "%s: %s.%s: Translating input to output.\n", PPREFIX(), dd->name, ddd->name);
    if (ddd->input == NULL) return;

    if (ddd->input->status <= RIBS_NONE || ddd->input->status >= RIBS_MAX) {
	lprintf(1, "%s: %s.%s: Wrong status %d.\n", PPREFIX(), dd->name, ddd->name, ddd->input->status);
	return;
    }
    // If shared and not in ok state do nothing
    lprintf( 66, "%s: %s.%s: Shared memory: status %d, n %d.\n", PPREFIX(), dd->name, ddd->name, ddd->input->status, ddd->input->buffer.n);
    if (ddd->input->status == RIBS_SHARED_INITIALIZE || ddd->input->status == RIBS_SHARED_FINALIZE) return;
    if (ddd->input->status == RIBS_SHARED_OK) {
	// shared memory input, get mutex
	//lprintf(0, "[");
	pthread_mutex_lock(&ddd->input->mutex);
	__sync_synchronize();
    }
    // I did not process n last values
    count = ddd->input->buffer.n - ddd->inputToOutputN;
    // do not translate more then outputbuffer size.
    if (count > ddd->outputBuffer.size) count = ddd->outputBuffer.size;
    // so we have to process last n values
    ii = ddd->input->buffer.ai+ddd->input->buffer.size-count;
    imax = ddd->input->buffer.ai+ddd->input->buffer.size;
    // ok. mark them as tramslated
    ddd->inputToOutputN = ddd->input->buffer.n;
    // This loop usually goes 0 times for slow devices, sometime 1 time when there are new data
    for(; ii<imax; ii++) {
    	i = ii % ddd->input->buffer.size;
	sampletime = ddd->input->buffer.a[i * (ddd->input->buffer.vectorsize+1)] - ddd->latency;

	// Some basic chect to detect wrong time on sender side
	// lprintf(1, "%s: %s.%s: checking time %g.\n", PPREFIX(), dd->name, ddd->name, sampletime);
	if (sampletime < currentTime.dtime - 7*24*60*60 || sampletime > currentTime.dtime + 7*24*60*60) {
	    static time_t msgtime = 0;
	    if (time(NULL) != msgtime) {
		lprintf(1, "%s: Error: %s.%s: wrong timestamp: %g.\n", PPREFIX(), dd->name, ddd->name, sampletime);
		msgtime = time(NULL);
	    }
	}
	inputVector = &ddd->input->buffer.a[i * (ddd->input->buffer.vectorsize+1)+1];
	switch(ddd->type) {
	case DT_VOID:
	case DT_DEBUG:
	case DT_PONG:
	    break;
	case DT_ORIENTATION_RPY:
	case DT_ORIENTATION_RPY_SHM:
	    // apply mount corrections
	    for(j=0; j<deviceDataStreamVectorLength[ddd->type]; j++) {
		outputVector[j] = inputVector[dd->mount_rpy_order[j]] * dd->mount_rpy_scale[j] - dd->mount_rpy[j];
	    }
	    // vec3_mul_elem(outputVector, outputVector, dd->mount_rpy_scale);
	    // vec3_sub(outputVector, outputVector, dd->mount_rpy);
	    if (ddd->launchPoseSetFlag) {
		// Originally I though that Yaw during launch shall be zero, so deduce launch yaw reported by the sensor
		// but maybe we will have yaw directly the one reported by some sensors (Magnetometer for example).
		outputVector[2] -= ddd->launchData[2];
	    }
	    break;
	case DT_EARTH_ACCELERATION:
	case DT_EARTH_ACCELERATION_SHM:
	    vec3_assign(outputVector, inputVector);
	    break;
	    /*
	case DT_ORIENTATION_QUATERNION:
	    memcpy(outputVector, inputVector, 4 * sizeof(double));
	    deviceDeduceMountOrientationFromQuaternion(dd, outputVector);
	    // TODO: Yaw during launch shall be zero, so deduce launch yaw reported by the sensor
	    break;
	    */
	case DT_POSITION_NMEA:
	    // TODO: deduce launching point coordinates and mount point!!!
	    break;
	case DT_MAGNETIC_HEADING_NMEA:
	    lprintf(0, "%s: Not yet implemented\n", PPREFIX());
	    break;
	case DT_JSTEST:
	    lprintf(0, "%s: Not yet implemented\n", PPREFIX());
	    break;
	case DT_POSITION_VECTOR:
	case DT_POSITION_SHM:
	    assert(deviceDataStreamVectorLength[ddd->type] == 3);
	    deviceSensorPositionToDronePosition(outputVector, inputVector, dd, sampletime);
	    if (ddd->launchPoseSetFlag) {
		vec3_sub(outputVector, outputVector, ddd->launchData);
	    }
	    break;
	case DT_BOTTOM_RANGE:
	    assert(deviceDataStreamVectorLength[ddd->type] == 1);
	    outputVector[0] = inputVector[0];
	    deviceTranslateBottomRangeToAltitude(dd, ddd, sampletime, inputVector, outputVector);
	    break;
	case DT_FLOW_XY:
	    assert(deviceDataStreamVectorLength[ddd->type] == 2);
	    if (ddd->input->buffer.n >= 2) {
		li = (i + ddd->input->buffer.size - 1) % ddd->input->buffer.size;
		previousSampleTime = ddd->input->buffer.a[li*(ddd->input->buffer.vectorsize+1)];
		deviceTranslateFlowXYtoDronePositionXY(dd, ddd, previousSampleTime, sampletime, inputVector, outputVector);
	    } else {
		vec2_assign(outputVector, inputVector);
	    }
	    break;
	case DT_ALTITUDE:
	    assert(deviceDataStreamVectorLength[ddd->type] == 1);
	    // Deduce launch altitude
	    if (ddd->launchPoseSetFlag) {
		outputVector[0] = inputVector[0] - ddd->launchData[0];
	    } else {
		outputVector[0] = inputVector[0];
	    }
	    break;
	case DT_TEMPERATURE:
	    assert(deviceDataStreamVectorLength[ddd->type] == 1);
	    outputVector[0] = inputVector[0];
	    break;
	case DT_MAGNETIC_HEADING:
	    assert(deviceDataStreamVectorLength[ddd->type] == 3);
	    lprintf(0, "%s: Magnetic heading: not yet implemented\n", PPREFIX());
	    break;
	default:
	    break;
	}
	if (ddd->outputBuffer.vectorsize > 0) {
	    // For drifting devices add the drift
	    deviceMaybeAddDriftToOutputVector(dd, ddd, outputVector);
	    regressionBufferAddElem(&ddd->outputBuffer, sampletime, outputVector);
	    ddd->confidence = ddd->input->confidence;
	    lprintf(100 - ddd->debug_level, "%s: %s: %s: Translating %16.6f: %s --> %s\n", PPREFIX(), dd->name, ddd->name, sampletime, arrayWithDimToStr_st(inputVector, ddd->input->buffer.vectorsize), arrayWithDimToStr_st(outputVector, ddd->outputBuffer.vectorsize));
	}
    }
    
    if (ddd->input->status == RIBS_SHARED_OK) {
	// shared memory input, free mutex
	pthread_mutex_unlock(&ddd->input->mutex);
	//lprintf(0, "]");
    }
    return;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
// devices communication

static int baioLineInputDevCallBackOnRead(struct deviceData *dd, struct baio *bb, int fromj, int num) {
    int				i;
    struct baioBuffer		*b;

    b = &bb->readBuffer;
	
    // we have n available chars starting at s[i]
    assert(b->i >= 0 && b->j >= 0 && b->i < b->size && b->j < b->size && b->i < b->j && b->size > 0);

    i = fromj;
    for(;;) {

	// TODO: Add parsing of binary messages in form 'BN[1-byte-length]<tag><bin-data-vector>'
	if (b->j - i >= 3 && strncmp(b->b+i, "BN", 2) == 0) {
	    printf("%s: Error: Parsing of binary messages not yet implemented. In the input of %s!\n", PPREFIX(), dd->name);
	    b->i = b->j;
	    // return(1);
	} else {
	    // TODO: Text line message must start with "LN".
	    
	    // did we read newline?
	    while (i < b->j && b->b[i] != '\n') i++;
	
	    if (i >= b->j) {
		// the newline was not read
		if (b->i == 0 && i >= b->size - bb->minFreeSizeAtEndOfReadBuffer - 1) {
		    // if the buffer is full, flush it to be able to read next possible lines
		    printf("%s: Warning: Buffer full, skipping %d bytes in %s!\n", PPREFIX(), i, dd->name);
		    b->i = b->j;
		}
		return(1);
	    }

	    // we've got a line
	    if (dd->enabled) {
		b->b[i] = 0;
		if (isspaceString(&b->b[b->i])) {
		    lprintf( 55, "%s: Info: skipping empty line from %s\n", PPREFIX(), dd->name);
		} else {
		    // only call the callback if the line is not empty
		    if (! shutDownInProgress) deviceParseInputStreamLineToInputBuffer(dd, &b->b[b->i], i - b->i);
		}
		b->b[i] = '\n';
	    }
	    i = b->i = i+1;
	}
    }
    
    //lprintf(0, "%s:%d\n", __FILE__, __LINE__);
    return(0);
}

static int baioMavlinkInputDevCallBackOnRead(struct deviceData *dd, struct baio *bb, int fromj, int num) {
    int				i, ii;
    struct baioBuffer		*b;

    b = &bb->readBuffer;
	
    lprintf(50, "%s: READ %d bytes (fromj==%d): %.*s", PPREFIX(), num, fromj, num, bb->readBuffer.b+bb->readBuffer.j-num);
    lprintf(50, "%s: READ %d bytes\n", PPREFIX(), num);

    mavlinkParseInput(dd, bb, fromj, num);
    return(0);
}

static int baioDeviceCallBackOnDelete(struct baio *b) {
    int			i;
    struct deviceData 	*dd;

    if (shutDownInProgress) return(0);
	
    if (b->baioType == BAIO_TYPE_FD) baioCloseFd(b);
    i = b->userParam[0].i;
    assert(i>=0 && i<uu->deviceMax);
    dd = uu->device[i];
    assert(dd != NULL);
    // print both to stdout and log. We do not know which of them is currently usable
    printf("%s: Error: connection to %s lost\n", PPREFIX(), dd->name);
    lprintf(0, "%s: Error: connection to %s lost\n", PPREFIX(), dd->name);

    // if (i != 0) {assert(0); exit(-1);}
    /*
    if (dd->autoReconnectFlag) {
	timeLineInsertUniqEventIfNotYetInserted(UTIME_AFTER_MSEC(100), (timelineEventFunType*)baioLineIOConnectInternal, (void*)dd);
    }
    */
    return(0);
}

static int baioDeviceCallBackOnError(struct baio *b) {
    int			i;
    struct deviceData 	*dd;
	
    i = b->userParam[0].i;
    assert(i>=0 && i<uu->deviceMax);
    dd = uu->device[i];
    assert(dd != NULL);

    // print both to stdout and log. We do not know which of them is currently usable
    printf("%s: Error: problem in connection to %s, closing it.\n", PPREFIX(), dd->name);
    lprintf(0, "%s: Error: problem in connection to %s, closing it.\n", PPREFIX(), dd->name);
    baioClose(b);
    return(0);
}

static int baioDeviceCallBackOnRead(struct baio *bb, int fromj, int num) {
    int				ii, res;
    struct deviceData 		*dd;

    lprintf( 99, "%s: READ %d bytes (fromj==%d): %.*s", PPREFIX(), num, fromj, num, bb->readBuffer.b+bb->readBuffer.j-num);
    lprintf( 35, "%s: READ %d bytes\n", PPREFIX(), num);
	
    ii = bb->userParam[0].i;
    assert(ii >= 0 && ii < uu->deviceMax);
    dd = uu->device[ii];
    assert(dd != NULL);

    dd->lastActivityTime = currentTime.dtime;

    switch (dd->connection.type) {
    case DCT_MAVLINK_PTTY:
	res = baioMavlinkInputDevCallBackOnRead(dd, bb, fromj, num);
	break;
    default:
	res = baioLineInputDevCallBackOnRead(dd, bb, fromj, num);
	break;
    }

    return(res);
}

void deviceInitiate(int i) {
    struct deviceData 	*dd;
    struct baio 	*bb;
    
    dd = uu->device[i];
    dd->enabled = 0;
    dd->baioMagic = 0;

    switch (dd->connection.type) {
    case DCT_INTERNAL_ZEROPOSE:
	bb = NULL;
	dd->enabled = 1;
	return;
	break;
    case DCT_COMMAND_EXEC:
	bb = baioNewPipedCommand(dd->connection.u.command, BAIO_IO_DIRECTION_RW, 0, 0);
	break;
    case DCT_COMMAND_BASH:
	bb = baioNewPipedCommand(dd->connection.u.command, BAIO_IO_DIRECTION_RW, 1, 0);
	break;
    case DCT_NAMED_PIPES:
	bb = baioNewNamedPipes(dd->connection.u.namedPipes.read_pipe, dd->connection.u.namedPipes.write_pipe, 0);
	break;
    case DCT_MAVLINK_PTTY:
	bb = baioNewPseudoTerminal(dd->connection.u.mavlink.link, 1000000, 0);
	mavlinkInitiate(dd, bb);
	break;
    default:
	printf("%s: Internal error: wrong connection type %d for device %s.\n", PPREFIX(), dd->connection.type, dd->name);
	lprintf(0, "%s: Internal error: wrong connection type %d for device %s.\n", PPREFIX(), dd->connection.type, dd->name);
	bb = NULL;
	break;
    }
    if (bb == NULL) {
	printf("%s: Error: can't create connection to device %s. Ignoring device.\n", PPREFIX(), dd->name);
	lprintf(0, "%s: Error: can't create connection to device %s. Ignoring device.\n", PPREFIX(), dd->name);
	return;
    } 
    dd->baioMagic = bb->baioMagic;
    dd->lastActivityTime = currentTime.dtime;
    dd->enabled = 1;
    // make read buffer larger (there are devices like rotational lidar ...)
    // this overwrites defaulf buffer sizes of baio
    bb->initialReadBufferSize = (1<<16);
    bb->initialWriteBufferSize = (1<<10);
    callBackAddToHook(&bb->callBackOnRead, (callBackHookFunArgType) baioDeviceCallBackOnRead);
    callBackAddToHook(&bb->callBackOnError, (callBackHookFunArgType) baioDeviceCallBackOnError);
    callBackAddToHook(&bb->callBackOnDelete, (callBackHookFunArgType) baioDeviceCallBackOnDelete);
    bb->userParam[0].i = i;

    deviceInitiateRegularAutoAdjustementOfDrifts(dd);
}

void deviceFinalize(int i) {
    struct deviceData 		*dd;
    struct deviceStreamData 	*ddd;
    struct baio 		*bb;
    int				j;
    
    dd = uu->device[i];
    dd->enabled = 0;
    
    for(j=0; j<dd->ddtMax; j++) {
	ddd = dd->ddt[j];
	if (ddd != NULL) {
	    if (deviceIsSharedMemoryDataStream(ddd) && ddd->input != NULL) {
		ddd->input->status = RIBS_SHARED_FINALIZE;
	    }
	}
    }
    baioCloseMagic(dd->baioMagic);
}

// Be carefull with this. Some devices are text some are mavlink, ...
void deviceSendToAllDevices(char *fmt, ...) {
    int		i;
    va_list     arg_ptr, ap;
    struct baio	*bb;

    va_start(ap, fmt);

    for(i=0; i<uu->deviceMax; i++) {
	bb = baioFromMagic(uu->device[i]->baioMagic);
	if (bb != NULL) {
	    va_copy(arg_ptr, ap);
	    baioVprintfToBuffer(bb, fmt, arg_ptr);
	    va_end(arg_ptr);
	}
    }
    va_end(ap);
}


