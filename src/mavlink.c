#include "common.h"

/*
  The MAVLink protocol code generator does its own alignment, so
  alignment cast warnings can be ignored
 */

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wcast-align"

#if defined(__GNUC__) && __GNUC__ >= 9
#pragma GCC diagnostic ignored "-Waddress-of-packed-member"
#endif

#define MAVLINK_USE_MESSAGE_INFO 1

#include "mavlink/common/mavlink.h"

static double lastReceivedMavlinkHeartbeatTime = 0;

// TODO: Move this into dd
mavlink_status_t status;
int chan = MAVLINK_COMM_0;

// TODO: Generate all messages used by ~/usr/openhd/QOpenHD-2.5-evo/app/telemetry/models/fcmavlinksystem.cpp
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// sending

#if 0
enum MAV_MODE {
// This is mavlink mode definition
   MAV_MODE_PREFLIGHT=0, /* System is not ready to fly, booting, calibrating, etc. No flag is set. | */
   MAV_MODE_MANUAL_DISARMED=64, /* System is allowed to be active, under manual (RC) control, no stabilization | */
   MAV_MODE_TEST_DISARMED=66, /* UNDEFINED mode. This solely depends on the autopilot - use with caution, intended for developers only. | */
   MAV_MODE_STABILIZE_DISARMED=80, /* System is allowed to be active, under assisted RC control. | */
   MAV_MODE_GUIDED_DISARMED=88, /* System is allowed to be active, under autonomous control, manual setpoint | */
   MAV_MODE_AUTO_DISARMED=92, /* System is allowed to be active, under autonomous control and navigation (the trajectory is decided onboard and not pre-programmed by waypoints) | */
   MAV_MODE_MANUAL_ARMED=192, /* System is allowed to be active, under manual (RC) control, no stabilization | */
   MAV_MODE_TEST_ARMED=194, /* UNDEFINED mode. This solely depends on the autopilot - use with caution, intended for developers only. | */
   MAV_MODE_STABILIZE_ARMED=208, /* System is allowed to be active, under assisted RC control. | */
   MAV_MODE_GUIDED_ARMED=216, /* System is allowed to be active, under autonomous control, manual setpoint | */
   MAV_MODE_AUTO_ARMED=220, /* System is allowed to be active, under autonomous control and navigation (the trajectory is decided onboard and not pre-programmed by waypoints) | */
};
#endif

static int getMavlinkMode() {
    // Translate raspilot mode into mavlink mode
    if (uu->flyStage < FS_STANDBY) {
	return(MAV_MODE_PREFLIGHT);
    } else if (1 /* always disarmed, otherwise openhd stops wifi */ || uu->flyStage < FS_PRE_FLY) {
	// Hmm. we can control mode by each of component roll,pitch,yaw,altitude
	// I shall combine that into single mode for mavlink, let's take roll only
	if (uu->config.manual_rc_roll.mode == RCM_PASSTHROUGH) return(MAV_MODE_MANUAL_DISARMED);
	if (uu->config.manual_rc_roll.mode == RCM_ACRO) return(MAV_MODE_STABILIZE_DISARMED);
	if (uu->config.manual_rc_roll.mode == RCM_TARGET) return(MAV_MODE_GUIDED_DISARMED);
	if (uu->config.manual_rc_roll.mode == RCM_AUTO) return(MAV_MODE_AUTO_DISARMED);
    } else {
	if (uu->config.manual_rc_roll.mode == RCM_PASSTHROUGH) return(MAV_MODE_MANUAL_ARMED);
	if (uu->config.manual_rc_roll.mode == RCM_ACRO) return(MAV_MODE_STABILIZE_ARMED);
	if (uu->config.manual_rc_roll.mode == RCM_TARGET) return(MAV_MODE_GUIDED_ARMED);
	if (uu->config.manual_rc_roll.mode == RCM_AUTO) return(MAV_MODE_AUTO_ARMED);	
    }
    return(MAV_MODE_PREFLIGHT);
}

#if 0
enum MAV_STATE
{
   MAV_STATE_UNINIT=0, /* Uninitialized system, state is unknown. | */
   MAV_STATE_BOOT=1, /* System is booting up. | */
   MAV_STATE_CALIBRATING=2, /* System is calibrating and not flight-ready. | */
   MAV_STATE_STANDBY=3, /* System is grounded and on standby. It can be launched any time. | */
   MAV_STATE_ACTIVE=4, /* System is active and might be already airborne. Motors are engaged. | */
   MAV_STATE_CRITICAL=5, /* System is in a non-normal flight mode (failsafe). It can however still navigate. | */
   MAV_STATE_EMERGENCY=6, /* System is in a non-normal flight mode (failsafe). It lost control over parts or over the whole airframe. It is in mayday and going down. | */
   MAV_STATE_POWEROFF=7, /* System just initialized its power-down sequence, will shut down now. | */
   MAV_STATE_FLIGHT_TERMINATION=8, /* System is terminating itself (failsafe or commanded). | */
   MAV_STATE_ENUM_END=9, /*  | */
};
#endif

static int getMavlinkState() {
    if (uu->flyStage < FS_STANDBY) return(MAV_STATE_CALIBRATING);
    if (uu->flyStage == FS_STANDBY) return(MAV_STATE_CALIBRATING); // return(MAV_STATE_STANDBY);
    if (uu->flyStage == FS_EMERGENCY_LANDING) return(MAV_STATE_EMERGENCY);
    if (uu->flyStage == FS_SHUTDOWN) return(MAV_STATE_FLIGHT_TERMINATION);
    return(MAV_STATE_ACTIVE);
}

void mavlinkSendHeartbeatRegular(void *d) {
    int 		baioMagic, dindex, len;
    struct baio		*bb;
    struct deviceData 	*dd;
    mavlink_message_t 	msg;
    uint8_t 		buf[MAVLINK_MAX_PACKET_LEN];

    baioMagic = (intptr_t) d;
    bb = baioFromMagic(baioMagic);
    if (bb == NULL) return;

    dindex = bb->userParam[0].i;
    dd = uu->device[dindex];

    mavlink_msg_heartbeat_pack(dd->connection.u.mavlink.system_id,  dd->connection.u.mavlink.component_id, &msg,
			       MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_ARDUPILOTMEGA /* don't use GENERIC, does not answer*/,
			       getMavlinkMode(), 0, getMavlinkState()
	);


    len = mavlink_msg_to_send_buffer(buf, &msg);
    baioWriteToBuffer(bb, buf, len);
    
    timeLineInsertEvent(UTIME_AFTER_MSEC(300), mavlinkSendHeartbeatRegular, d);
}

void mavlinkSendAttitudeRegular(void *d) {
    int 		baioMagic, dindex, len;
    struct baio		*bb;
    struct deviceData 	*dd;
    mavlink_message_t 	msg;
    uint8_t 		buf[MAVLINK_MAX_PACKET_LEN];

    baioMagic = (intptr_t) d;
    bb = baioFromMagic(baioMagic);
    if (bb == NULL) return;

    dindex = bb->userParam[0].i;
    dd = uu->device[dindex];

    
    mavlink_msg_attitude_pack(dd->connection.u.mavlink.system_id,  dd->connection.u.mavlink.component_id, &msg,
			      currentTime.msec - uu->pilotStartingTime*1000,
			      uu->droneLastRpy[0], uu->droneLastRpy[1], uu->droneLastRpy[2],
			      uu->droneLastRpyRotationSpeed[0], uu->droneLastRpyRotationSpeed[1], uu->droneLastRpyRotationSpeed[2]
	);
    
    len = mavlink_msg_to_send_buffer(buf, &msg);
    baioWriteToBuffer(bb, buf, len);
    
    timeLineInsertEvent(UTIME_AFTER_MSEC(50), mavlinkSendAttitudeRegular, d);
}

void mavlinkSendBatteryStatusRegular(void *d) {
    int 		i, baioMagic, dindex, len;
    struct baio		*bb;
    struct deviceData 	*dd;
    mavlink_message_t 	msg;
    uint8_t 		buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t		voltage[10];
    uint16_t		voltageExt[4];
    int			remainingPercentage;
    
    baioMagic = (intptr_t) d;
    bb = baioFromMagic(baioMagic);
    if (bb == NULL) return;

    dindex = bb->userParam[0].i;
    dd = uu->device[dindex];

    for(i=0; i<10; i++) voltage[i] = INT16_MAX-1;
    for(i=0; i<4; i++) voltageExt[i] = 0;

    remainingPercentage = uu->batteryStatusPerc;
    
    mavlink_msg_battery_status_pack(dd->connection.u.mavlink.system_id,  dd->connection.u.mavlink.component_id, &msg,
				    0, MAV_BATTERY_FUNCTION_UNKNOWN, MAV_BATTERY_TYPE_UNKNOWN, INT16_MAX,
				    voltage, -1, -1, -1, remainingPercentage, 0,
				    (remainingPercentage < 10 ? MAV_BATTERY_CHARGE_STATE_LOW : MAV_BATTERY_CHARGE_STATE_OK),
				    voltageExt, MAV_BATTERY_MODE_UNKNOWN, 0
	);
    
    len = mavlink_msg_to_send_buffer(buf, &msg);
    baioWriteToBuffer(bb, buf, len);
    
    timeLineInsertEvent(UTIME_AFTER_MSEC(200), mavlinkSendBatteryStatusRegular, d);
}

void mavlinkSendGlobalPositionInt(void *d) {
    int 		baioMagic, dindex, len;
    struct baio		*bb;
    struct deviceData 	*dd;
    mavlink_message_t 	msg;
    uint8_t 		buf[MAVLINK_MAX_PACKET_LEN];

    baioMagic = (intptr_t) d;
    bb = baioFromMagic(baioMagic);
    if (bb == NULL) return;

    dindex = bb->userParam[0].i;
    dd = uu->device[dindex];

    // TODO: Figure out something accurate
    double latitude = uu->droneLastPosition[0] / 111000.0 * 180 / M_PI;
    double longitude =  uu->droneLastPosition[1] / 111321.0 * cos(latitude * M_PI/180);

    mavlink_msg_global_position_int_pack(dd->connection.u.mavlink.system_id,  dd->connection.u.mavlink.component_id, &msg,
					 currentTime.msec - uu->pilotStartingTime*1000,
					 /* actual values
					 latitude*10000000.0, longitude*10000000.0, uu->droneLastPosition[2]*1000,
					 uu->droneLastPosition[2]*1000, 
					 uu->droneLastVelocity[0]*100, uu->droneLastVelocity[1]*100, uu->droneLastVelocity[2]*100,
					 normalizeAngle(uu->droneLastRpy[2], 0, 2*M_PI)*18000/M_PI
					 */
					 // values used for debugging
					 latitude*10000000.0*100, longitude*10000000.0*100, uu->droneLastPosition[2]*10000,
					 uu->droneLastPosition[2]*10000, 
					 uu->droneLastVelocity[0]*10000, uu->droneLastVelocity[1]*10000, uu->droneLastVelocity[2]*10000,
					 normalizeAngle(uu->droneLastRpy[2], 0, 2*M_PI)*18000/M_PI
	);
    
    len = mavlink_msg_to_send_buffer(buf, &msg);
    baioWriteToBuffer(bb, buf, len);
    
    timeLineInsertEvent(UTIME_AFTER_MSEC(200), mavlinkSendGlobalPositionInt, d);
}


void mavlinkSendHomePosition(void *d) {
    int 		baioMagic, dindex, len;
    struct baio		*bb;
    struct deviceData 	*dd;
    mavlink_message_t 	msg;
    uint8_t 		buf[MAVLINK_MAX_PACKET_LEN];

    baioMagic = (intptr_t) d;
    bb = baioFromMagic(baioMagic);
    if (bb == NULL) return;

    dindex = bb->userParam[0].i;
    dd = uu->device[dindex];
#if 0
    // To be implemented
    mavlink_msg_home_position_pack(dd->connection.u.mavlink.system_id,  dd->connection.u.mavlink.component_id, &msg,

	);
    
    len = mavlink_msg_to_send_buffer(buf, &msg);
    baioWriteToBuffer(bb, buf, len);
    
    timeLineInsertEvent(UTIME_AFTER_MSEC(200), mavlinkSendHomePositionInt, d);
#endif    
}

void mavlinkSendStatusText(int baioMagic, char *text) {
    int 		dindex, len;
    struct baio		*bb;
    struct deviceData 	*dd;
    mavlink_message_t 	msg;
    uint8_t 		buf[MAVLINK_MAX_PACKET_LEN];

    bb = baioFromMagic(baioMagic);
    if (bb == NULL) return;

    dindex = bb->userParam[0].i;
    dd = uu->device[dindex];

    mavlink_msg_statustext_pack(dd->connection.u.mavlink.system_id,  dd->connection.u.mavlink.component_id, &msg,
				0, text, 0, 0
	);
    
    len = mavlink_msg_to_send_buffer(buf, &msg);
    baioWriteToBuffer(bb, buf, len);
    
}

void mavlinkSendStatusTextToListeners(char *text) {
    struct deviceStreamData 	*ddl;
    struct baio 		*bb;
    
    for(ddl=uu->deviceStreamDataByType[DT_MAVLINK_STATUSTEXT]; ddl!=NULL; ddl=ddl->nextWithSameType) {
	mavlinkSendStatusText(ddl->dd->baioMagic, text);
    }
    
}

void mavlinkVPrintfStatusTextToListeners(char *fmt, va_list arg_ptr) {
    int	 r;
    char text[TMP_STRING_SIZE];
    
    r = vsnprintf(text, sizeof(text), fmt, arg_ptr);
    if (r >= 50) {
	lprintf(22, "%s: Warning: mavlink status text truncated. Full text: %s\n", PPREFIX(), text);
    }
    mavlinkSendStatusTextToListeners(text);
}

void mavlinkPrintfStatusTextToListeners(char *fmt, ...) {
    va_list     ap;

    va_start(ap, fmt);
    mavlinkVPrintfStatusTextToListeners(fmt, ap);
    // There is some bug in openhd, so that sometimes the last message is not displayed, so always sens an empty message after
    mavlinkSendStatusTextToListeners("");
    va_end(ap);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////
// receiving

void mavlinkOnHeartbeat(struct deviceData *dd, struct baio *bb, mavlink_message_t *msg) {
    // lprintf(0, "%s: Warning: Got mavlink heartbeat\n", PPREFIX());
    lastReceivedMavlinkHeartbeatTime = currentTime.dtime;
}

void mavlinkOnPing(struct deviceData *dd, struct baio *bb, mavlink_message_t *msg) {
    mavlink_ping_t 	mm;
    mavlink_message_t 	oo;
    int			len;
    uint8_t 		buf[MAVLINK_MAX_PACKET_LEN];

    mavlink_msg_ping_decode(msg, &mm);
    if (mm.target_system == 0 && mm.target_component == 0) {
	mavlink_msg_ping_pack(dd->connection.u.mavlink.system_id,  dd->connection.u.mavlink.component_id, &oo,
			      currentTime.usec, mm.seq,
			      msg->sysid, msg->compid
	    );
	len = mavlink_msg_to_send_buffer(buf, &oo);
	baioWriteToBuffer(bb, buf, len);
    }
}

void mavlinkOnTimesync(struct deviceData *dd, struct baio *bb, mavlink_message_t *msg) {
    mavlink_timesync_t 	mm;
    mavlink_message_t 	oo;
    int			len;
    uint8_t 		buf[MAVLINK_MAX_PACKET_LEN];

    mavlink_msg_timesync_decode(msg, &mm);
    if (mm.tc1 == 0) {
	mavlink_msg_timesync_pack(dd->connection.u.mavlink.system_id,  dd->connection.u.mavlink.component_id, &oo,
				  currentTime.usec*1000, mm.ts1,
				  msg->sysid, msg->compid
	    );
	len = mavlink_msg_to_send_buffer(buf, &oo);
	baioWriteToBuffer(bb, buf, len);
    }
}

double mavlinkRcChannelsGetChannelRawValue(struct deviceStreamData *ss, mavlink_rc_channels_override_t *mm, int radioControlIndex) {
    int channel;

    assert(radioControlIndex >= RC_NONE && radioControlIndex <= RC_MAX);
    channel = ss->inverse_channel_map[radioControlIndex];
    // lprintf(0, "Index %d --> channel %d\n", radioControlIndex, channel);
    switch (channel) {
    case 0: return(mm->chan1_raw);
    case 1: return(mm->chan2_raw);
    case 2: return(mm->chan3_raw);
    case 3: return(mm->chan4_raw);
    case 4: return(mm->chan5_raw);
    case 5: return(mm->chan6_raw);
    case 6: return(mm->chan7_raw);
    case 7: return(mm->chan8_raw);
    case 8: return(mm->chan9_raw);
    case 9: return(mm->chan10_raw);
    case 10: return(mm->chan11_raw);
    case 11: return(mm->chan12_raw);
    case 12: return(mm->chan13_raw);
    case 13: return(mm->chan14_raw);
    case 14: return(mm->chan15_raw);
    case 15: return(mm->chan16_raw);
    case 16: return(mm->chan17_raw);
    case 17: return(mm->chan18_raw);
    default:
	lprintf(0, "%s: Internal Error: channel out of range 0-18 in %s\n", PPREFIX(), ss->dd->name);
    }
    return(0);
}

double mavlinkRcChannelsGetChannelNormalizedValue(struct deviceStreamData *ss, mavlink_rc_channels_override_t *mm, int radioControlIndex) {
    double vv;
    vv = mavlinkRcChannelsGetChannelRawValue(ss, mm, radioControlIndex);
    // lprintf(0, "Warning: Value of index %d --> %f\n", radioControlIndex, vv);
    if (vv < 1000) return(0.0);
    if (vv > 2000) return(1.0);
    return((vv-1000.0) / 1000.0);
}

void mavlinkOnRcChannelsOverride(struct deviceData *dd, struct baio *bb, mavlink_message_t *msg) {
    struct deviceStreamData 		*ss;
    mavlink_rc_channels_override_t 	mm;
    mavlink_message_t 			oo;
    int					i, len;
    uint8_t 				buf[MAVLINK_MAX_PACKET_LEN];
    static time_t			panicButtonPressedTime;
    
    mavlink_msg_rc_channels_override_decode(msg, &mm);

#if 0
    lprintf(0, "Got joystick: %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d \n",
	    mm.chan1_raw, mm.chan2_raw,  mm.chan3_raw,  mm.chan4_raw,  mm.chan5_raw,  mm.chan6_raw,  mm.chan7_raw,  mm.chan8_raw,  mm.chan9_raw,
	    mm.chan10_raw, mm.chan11_raw, mm.chan12_raw, mm.chan13_raw, mm.chan14_raw, mm.chan15_raw, mm.chan16_raw, mm.chan17_raw, mm.chan18_raw
	);
#endif
    
    ss = deviceFindStreamByType(dd, DT_MAVLINK_RC_CHANNELS_OVERRIDE);
    // ignore if not configured
    if (ss == NULL) return;

    // OK. we are basically getting only a few values from there
    // TODO: Adjust those value to reasonable ranges !!!!
    manualPilotSetRoll(mavlinkRcChannelsGetChannelNormalizedValue(ss, &mm, RC_ROLL));
    manualPilotSetPitch(mavlinkRcChannelsGetChannelNormalizedValue(ss, &mm, RC_PITCH));
    manualPilotSetYaw(1.0 - mavlinkRcChannelsGetChannelNormalizedValue(ss, &mm, RC_YAW));
    manualPilotSetAltitude(1.0 - mavlinkRcChannelsGetChannelNormalizedValue(ss, &mm, RC_ALTITUDE));

    if (mavlinkRcChannelsGetChannelNormalizedValue(ss, &mm, RC_BUTTON_LAUNCH_COUNTDOWN) < 0.5) {
	if (uu->flyStage == FS_STANDBY) {
	    uu->flyStage = FS_COUNTDOWN;
	}
    }
    if (mavlinkRcChannelsGetChannelNormalizedValue(ss, &mm, RC_BUTTON_STANDBY) < 0.5) {
	if (uu->flyStage > FS_STANDBY) {
	    lprintf(0, "%s: Joystick standby button pressed.\n", PPREFIX());
	    raspilotGotoStandby();
	}
    }
    if (mavlinkRcChannelsGetChannelNormalizedValue(ss, &mm, RC_BUTTON_PANIC_SHUTDOWN) < 0.5) {
	if (uu->flyStage > FS_STANDBY) {
	    lprintf(0, "%s: Joystick panic button pressed. Going to standby mode.\n", PPREFIX());
	    raspilotGotoStandby();
	}
	if (panicButtonPressedTime == 0) {
	    panicButtonPressedTime = currentTime.sec;
	} else if (currentTime.sec - panicButtonPressedTime > 5) {
	    lprintf(0, "%s: Joystick panic button hold for 5 seconds. Raspilot is exiting!\n", PPREFIX());
	    raspilotShutDownAndExit();
	}
    } else {
	panicButtonPressedTime = 0;
    }
		
    
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////


int mavlinkParseInput(struct deviceData *dd, struct baio *bb, int fromj, int num) {
    const mavlink_message_info_t 	*msginfo;
    int					i, ii;
    struct baioBuffer			*b;
    mavlink_message_t 			msg;

    b = &bb->readBuffer;
	
    // we have n available chars starting at s[i]
    assert(b->i >= 0 && b->j >= 0 && b->i < b->size && b->j < b->size && b->i < b->j && b->size > 0);

    for(i=b->i; i<b->j; i++) {
	if (mavlink_parse_char(chan, b->b[i], &msg, &status)) {
	    switch (msg.msgid) {
	    case MAVLINK_MSG_ID_HEARTBEAT:
		mavlinkOnHeartbeat(dd, bb, &msg);
		break;
	    case MAVLINK_MSG_ID_PING:
		mavlinkOnPing(dd, bb, &msg);
		break;
	    case MAVLINK_MSG_ID_TIMESYNC:
		mavlinkOnTimesync(dd, bb, &msg);
		break;
	    case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE:
		mavlinkOnRcChannelsOverride(dd, bb, &msg);
		break;
		//case MAVLINK_MSG_ID_COMMAND_LONG:
		//break;
	    default:
		msginfo = mavlink_get_message_info(&msg);
		lprintf(100 - dd->connection.u.mavlink.debug_level, "Received unknown message with ID %d (%s), sequence: %d from component %d of system %d\n", msg.msgid, msginfo->name, msg.seq, msg.compid, msg.sysid);
		break;
	    }
	}
    }

    b->i = i;
    return(0);
}

void mavlinkRegularCheckHeartbeat(void *d) {
    static int	recheckAfterMs = 200;
    static int	repeatBeepEveryNSeconds = 10;
    static int 	beepCounter = 0;
    // They do not seem to be particularly interested in this they send heartbeat once 10-20 seconds, sometimes more.
    // I remove this check for sacurity reasons.
    // TODO: rather check for last received mavlink message at all, not only heartbeat
    if (currentTime.dtime - lastReceivedMavlinkHeartbeatTime < 0.0) {
	if (uu->flyStage >= FS_FLY && uu->flyStage < FS_EMERGENCY_LANDING) {
	    lprintf(0, "%s: Error: mavlink connection timeout. Immediate landing !!!\n", PPREFIX());
	    uu->flyStage = FS_EMERGENCY_LANDING;
	}
	// repeat 
	beepCounter = (beepCounter + 1) % (repeatBeepEveryNSeconds * 1000 / recheckAfterMs);
	// of course do not 'beep' motor while in air.
	if (beepCounter == 0 && uu->flyStage < FS_FLY) {
	    mavlinkPrintfStatusTextToListeners("Mavlink lost: BEEP");
	    motorsBeep(NULL);
	}
    }
    timeLineInsertEvent(UTIME_AFTER_MSEC(recheckAfterMs), mavlinkRegularCheckHeartbeat, NULL);
}

void mavlinkInitiate(struct deviceData *dd, struct baio *bb) {
    int i;
    // Ensure that integer type is smaller than pointer, otherwise baioMagic can be too big to fit into void*
    assert(sizeof(int) <= sizeof(void*));
    
    // Note that nor dd and bb is completely setup at this time !!!!
    timeLineInsertEvent(UTIME_AFTER_SECONDS(1), mavlinkSendHeartbeatRegular, (void*)(intptr_t)bb->baioMagic);
    // first heartbeat check will be in 3 seconds
    lastReceivedMavlinkHeartbeatTime = currentTime.dtime + 3;
    timeLineInsertEvent(UTIME_AFTER_SECONDS(3), mavlinkRegularCheckHeartbeat, NULL);
    
    // find out debug level
    for(i=0; i<dd->ddtMax; i++) {
	if (dd->ddt[i] != NULL) {
	    if (dd->ddt[i]->type == DT_MAVLINK_ATTITUDE) {
		timeLineInsertEvent(UTIME_AFTER_SECONDS(5), mavlinkSendAttitudeRegular, (void*)(intptr_t)bb->baioMagic);
	    } else if (dd->ddt[i]->type == DT_MAVLINK_BATTERY_STATUS) {
		timeLineInsertEvent(UTIME_AFTER_SECONDS(5), mavlinkSendBatteryStatusRegular, (void*)(intptr_t)bb->baioMagic);
	    } else if (dd->ddt[i]->type == DT_MAVLINK_GLOBAL_POSITION) {
		timeLineInsertEvent(UTIME_AFTER_SECONDS(5), mavlinkSendGlobalPositionInt, (void*)(intptr_t)bb->baioMagic);
	    } else if (dd->ddt[i]->type == DT_MAVLINK_HOME_POSITION) {
		timeLineInsertEvent(UTIME_AFTER_SECONDS(5), mavlinkSendHomePosition, (void*)(intptr_t)bb->baioMagic);
	    } else if (dd->ddt[i]->type == DT_DEBUG) {
		dd->connection.u.mavlink.debug_level = dd->ddt[i]->debug_level;
	    }
	}
    }
}

