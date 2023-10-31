// load configuration and mission (and maybe other) files

#include "common.h"

#define CANT_LOAD_CONFIGURATION_FILE_FATAL(path) {			\
	lprintf(0, "%s: Error: can't load file %s!\n", PPREFIX(), path); \
	raspilotShutDownAndExit();			\
    }

#define LOAD_CONFIG_CONTEXT_PUSH(context, ...) {			\
	snprintf(context+strlen(context), TMP_STRING_SIZE-strlen(context), "." __VA_ARGS__); \
    }

#define LOAD_CONFIG_CONTEXT_POP(context) {	\
	char *_s_;				\
	_s_ = strrchr(context, '.');		\
	if (_s_ != NULL) *_s_ = 0;		\
    }

#define LOAD_CONFIG_ASSERT(s) {						\
	if (!(s)) {							\
	    lprintf(0, "%s: Error: %s is not valid for configuration file %s.\n", PPREFIX(), #s, path); \
	    CANT_LOAD_CONFIGURATION_FILE_FATAL(path);			\
	}								\
    }

#define LOAD_CONFIG_TYPE_OK(cc, jsonType) (cc != NULL && cc->type == jsonType)

#define LOAD_CONFIG_ERROR_ON_WRONG_TYPE(cc, path, context, id, jsonType) {	\
	if (cc == NULL) {						\
	    lprintf(0, "%s: Error: %s.%s is NULL in %s.\n", PPREFIX(), context, id, path); \
	    CANT_LOAD_CONFIGURATION_FILE_FATAL(path);			\
	    cc = &dummyJsonNode;					\
	}								\
	if (cc->type != jsonType) {					\
	    lprintf(0, "%s: Error: %s.%s is not of type %s in %s.\n", PPREFIX(), context, id, jsonNodeTypeEnumNames[jsonType], path); \
	    CANT_LOAD_CONFIGURATION_FILE_FATAL(path);			\
	}								\
    }

#define LOAD_CONFIG_STRING_OPTION(cc, context, d, name) {		\
	struct jsonnode *node;						\
	node = jsonFindObjectField(cc, #name);				\
	if (LOAD_CONFIG_TYPE_OK(node, JSON_NODE_TYPE_STRING)) {		\
	    (d)->name = strSafeDuplicate(node->u.s);			\
	    node->used = 1;						\
	    lprintf(10,"%s: Info: parameter %s.%-40s: %s\n", PPREFIX(), context, #name, (d)->name); \
	} else {							\
	    LOAD_CONFIG_ERROR_ON_WRONG_TYPE(node, path, context, #name, JSON_NODE_TYPE_STRING); \
	}								\
    }

#define LOAD_CONFIG_STRING_OPTION_WITH_DEFAULT_VALUE(cc, context, d, name, defaultValue) { \
	struct jsonnode *node;						\
	node = jsonFindObjectField(cc, #name);				\
	if (node == NULL) {	\
	    (d)->name = strDuplicate(defaultValue);			\
	    lprintf(10,"%s: Info: parameter %s.%-40s: %s\n", PPREFIX(), context, #name, (d)->name); \
	} else if (LOAD_CONFIG_TYPE_OK(node, JSON_NODE_TYPE_STRING)) {	\
	    (d)->name = strSafeDuplicate(node->u.s);			\
	    node->used = 1;						\
	    lprintf(10,"%s: Info: parameter %s.%-40s: %s\n", PPREFIX(), context, #name, (d)->name); \
	} else {			\
	    LOAD_CONFIG_ERROR_ON_WRONG_TYPE(node, path, context, #name, JSON_NODE_TYPE_STRING); \
	}								\
    }

#define LOAD_CONFIG_ENUM_STRING_OPTION(cc, context, d, enumnames, name) { \
	struct jsonnode *node;						\
	node = jsonFindObjectField(cc, #name);				\
	if (LOAD_CONFIG_TYPE_OK(node, JSON_NODE_TYPE_STRING)) {		\
	    (d)->name = enumNamesStringToInt(node->u.s, enumnames);	\
	    if ((d)->name < 0) {					\
		lprintf(0,"%s: Error wrong value %s in %s. One of ", PPREFIX(), node->u.s, #name); \
		enumNamesPrint(stdout, enumnames);			\
		lprintf(0," expected.\n");					\
		(d)->name = 0;						\
		CANT_LOAD_CONFIGURATION_FILE_FATAL(path);		\
	    }								\
	    node->used = 1;						\
	    lprintf(10,"%s: Info: parameter %s.%-40s: %s\n", PPREFIX(), context, #name, enumnames[(d)->name]); \
	} else {			\
	    LOAD_CONFIG_ERROR_ON_WRONG_TYPE(node, path, context, #name, JSON_NODE_TYPE_STRING); \
	}								\
    }

#define LOAD_CONFIG_ENUM_STRING_OPTION_WITH_DEFAULT_VALUE(cc, context, d, enumnames, name, defaultValue) { \
        struct jsonnode *node;						\
        node = jsonFindObjectField(cc, #name);				\
	if (node == NULL) {	\
	    (d)->name = (defaultValue);					\
	    lprintf(10,"%s: Info: parameter %s.%-40s: %s\n", PPREFIX(), context, #name, enumnames[(d)->name]); \
	} else {							\
	    LOAD_CONFIG_ENUM_STRING_OPTION(cc, context, d, enumnames, name); \
	}								\
    }
#define LOAD_CONFIG_DOUBLE_OPTION(cc, context, d, name) {		\
        struct jsonnode *node;                                          \
        node = jsonFindObjectField(cc, #name);                                \
        if (LOAD_CONFIG_TYPE_OK(node, JSON_NODE_TYPE_NUMBER)) {		\
	    (d)->name = node->u.n;					\
	    node->used = 1;						\
	    lprintf(10,"%s: Info: parameter %s.%-40s: %g\n", PPREFIX(), context, #name, (double)(d)->name); \
	} else {			\
	    LOAD_CONFIG_ERROR_ON_WRONG_TYPE(node, path, context, #name, JSON_NODE_TYPE_NUMBER); \
	}								\
    }
#define LOAD_CONFIG_DOUBLE_OPTION_WITH_DEFAULT_VALUE(cc, context, d, name, defaultValue) { \
        struct jsonnode *node;                                          \
        node = jsonFindObjectField(cc, #name);                                \
	if (node == NULL) {	\
	    (d)->name = (defaultValue);					\
	    lprintf(10,"%s: Info: parameter %s.%-40s: %g\n", PPREFIX(), context, #name, (double)(d)->name); \
	} else if (LOAD_CONFIG_TYPE_OK(node, JSON_NODE_TYPE_NUMBER)) {	\
	    (d)->name = node->u.n;					\
	    node->used = 1;						\
	    lprintf(10,"%s: Info: parameter %s.%-40s: %g\n", PPREFIX(), context, #name, (double)(d)->name); \
	} else {			\
	    LOAD_CONFIG_ERROR_ON_WRONG_TYPE(node, path, context, #name, JSON_NODE_TYPE_NUMBER); \
	}								\
    }

#define LOAD_CONFIG_BOOL_OPTION(cc, context, d, name) {			\
	struct jsonnode *node;						\
	node = jsonFindObjectField(cc, #name);				\
	if (LOAD_CONFIG_TYPE_OK(node, JSON_NODE_TYPE_BOOL)) {		\
	    (d)->name = node->u.b;					\
	    node->used = 1;						\
	    lprintf(10,"%s: Info: parameter %s.%-40s: %d\n", PPREFIX(), context, #name, (int)(d)->name); \
	} else {			\
	    LOAD_CONFIG_CHECK_TYPE(node, context, #name,  JSON_NODE_TYPE_BOOL);	\
	}								\
    }

#define LOAD_CONFIG_BOOL_OPTION_WITH_DEFAULT_VALUE(cc, context, d, name, defaultValue) { \
	struct jsonnode *node;						\
	node = jsonFindObjectField(cc, #name);				\
	if (node == NULL) {	\
	    (d)->name = (defaultValue);					\
	    lprintf(10,"%s: Info: parameter %s.%-40s: %d\n", PPREFIX(), context, #name, (int)(d)->name); \
	} else if (LOAD_CONFIG_TYPE_OK(node, JSON_NODE_TYPE_BOOL)) {	\
	    (d)->name = node->u.b;					\
	    node->used = 1;						\
	    lprintf(10,"%s: Info: parameter %s.%-40s: %d\n", PPREFIX(), context, #name, (int)(d)->name); \
	} else {			\
	    LOAD_CONFIG_ERROR_ON_WRONG_TYPE(node, path, context, #name, JSON_NODE_TYPE_BOOL); \
	}								\
    }


static int configGetDeviceDataIndex(struct deviceData *dl, char *name) {
    int i;
    
    // first look for the name in yet configured devices
    for(i = 0; i < dl->ddtMax; i++) {
	if (strSafeCmp(dl->ddt[i]->name, name) == 0) return(i);
    }
    // not found, create a new device
    if (i >= DEVICE_DATA_MAX) return(-1);
    dl->ddtMax ++;
    CALLOC(dl->ddt[i], struct deviceStreamData);
    dl->ddt[i]->name = name;
    return(i);
}

struct jsonFieldList *configFindFieldList(struct jsonnode *nn, char *name, int jsonType, char *context) {
    struct jsonnode *tt;
    
    tt = jsonFind(nn, name);
    if (tt == NULL) return(NULL);
    if (tt->type != jsonType) {
	// lprintf(0,"%s: Error: parameter %s.%s shall be of type %s!\n", PPREFIX(), context, name, jsonNodeTypeEnumNames[jsonType]);
	return(NULL);
    }
    tt->used = 1;
    return(tt->u.fields);
}

static int configMaybeLoadVector(struct jsonnode *cc, double *vv, int length, char *fieldname, char *path, char *context) {
    int 			i;
    struct jsonnode 		*c;
    struct jsonFieldList 	*ll;
    i = 0;
    for(ll=configFindFieldList(cc, fieldname, JSON_NODE_TYPE_ARRAY, context); ll!=NULL; ll=ll->next) {
	if (i < length) {
	    LOAD_CONFIG_CONTEXT_PUSH(context, "%s[%d]", fieldname, i);
	    c = ll->val;
	    LOAD_CONFIG_ERROR_ON_WRONG_TYPE(c, path, context, "", JSON_NODE_TYPE_NUMBER);
	    vv[i] = c->u.n;
	    lprintf(10,"%s: Info: parameter %.*s: %f\n", PPREFIX(), 40, context, vv[i]);
	    LOAD_CONFIG_CONTEXT_POP(context);
	}
	i ++;
    }
    return(i);
}

void configLoadVectorWithDefaultValue(struct jsonnode *cc, double *vv, int length, char *fieldname, double defaultFill, char *path, char *context, int warnOnMiss) {
    struct jsonFieldList 	*ll;
    struct jsonnode 		*c;
    int 			i;

    i = configMaybeLoadVector(cc, vv, length, fieldname, path, context);
    if (i != length) {
	if (i != 0 && warnOnMiss) {
	    lprintf(0,"%s: Warning: %s.%s shall be an array of %d elements!\n", PPREFIX(), context, fieldname, length);
	}
	for(; i<length; i++) {
	    LOAD_CONFIG_CONTEXT_PUSH(context, "%s[%d]", fieldname, i);
	    vv[i] = defaultFill;
	    lprintf(10,"%s: Info: parameter %.*s: %f\n", PPREFIX(), 40, context, vv[i]);
	    LOAD_CONFIG_CONTEXT_POP(context);
	}
    }
}


void configLoadVectorNoDefaultValue(struct jsonnode *cc, double *vv, int length, char *fieldname, char *path, char *context) {
    struct jsonFieldList 	*ll;
    struct jsonnode 		*c;
    int 			i;

    i = configMaybeLoadVector(cc, vv, length, fieldname, path, context);
    if (i != length) {
	if (i == 0) {
	    lprintf(0,"%s: Error: %s: missing configuration for %s !\n", PPREFIX(), context, fieldname);
	    CANT_LOAD_CONFIGURATION_FILE_FATAL(path);
	} else {
	    lprintf(0,"%s: Error: %s: %s has wrong length!\n", PPREFIX(), context, fieldname);
	    CANT_LOAD_CONFIGURATION_FILE_FATAL(path);
	}
    }
}

static void configInputBufferInit(struct deviceData *dl, struct deviceStreamData *ddl) {
    int 	shmFlag, len;
    int		bufferSize, vectorSize;
    double	*memoryPool;
    char	*mem;
    
    vectorSize = deviceDataStreamVectorLength[ddl->type];
    bufferSize = ddl->history_size;
    len = sizeof(struct raspilotInputBuffer) + bufferSize * (vectorSize + 1) * sizeof(double);
	
    
    if (deviceIsSharedMemoryDataStream(ddl)) {
	// TODO: to be implemented !!!!
	// Allocate/map shared memory struct raspilotInputBuffer
	ddl->input = raspilotCreateSharedMemory(ddl);
    } else {
	CALLOCC(mem, RASPILOT_INPUT_BUFFER_SIZE(bufferSize, vectorSize), char);
	ddl->input = (struct raspilotInputBuffer *) mem;
	ddl->input->status = RIBS_NOT_SHARED;
	raspilotRingBufferInit(&ddl->input->buffer, vectorSize, bufferSize, "%s.%s stream", dl->name, ddl->name);
    }
}

static void configVectorMaybeSpecifiedByUniqueNumber(struct jsonnode *d, double *vv, int length, char *field, double defaultValue, char *path, char *context) {
    double 		dval;
    struct jsonnode 	*ww;
    
    dval = defaultValue;
    ww = jsonFindObjectField(d, field);
    if (ww != NULL && ww->type == JSON_NODE_TYPE_NUMBER) {
	dval = ww->u.n;
	ww->used = 1;
    }
    configLoadVectorWithDefaultValue(d, vv, length, field, dval, path, context, 0);
}

void configLoadDeviceStreams(struct jsonnode *c, struct deviceData	*dl, char *path, char *context) {
    int					oldtype, i;
    struct jsonnode			*d, *ww;
    struct jsonFieldList		*ll;
    struct deviceStreamData		*ddl;
    char				*name;
    double				dweight;

    for(ll=configFindFieldList(c, "stream", JSON_NODE_TYPE_ARRAY, context); ll!=NULL; ll=ll->next) {
	d = ll->val;
	LOAD_CONFIG_ERROR_ON_WRONG_TYPE(d, path, context, "", JSON_NODE_TYPE_OBJECT);

	name = jsonFindString(d, "name", NULL);
	i = configGetDeviceDataIndex(dl, name);
	if (i >= 0 && i < DEVICE_DATA_MAX) {
	    ddl = dl->ddt[i];
	    oldtype = ddl->type;
	    ddl->dd = dl;
	    LOAD_CONFIG_CONTEXT_PUSH(context, "stream[%d]", i);
	    LOAD_CONFIG_ENUM_STRING_OPTION(d, context, ddl, deviceDataTypeNames, type);
	    if (ddl->type < DT_NONE || ddl->type >= DT_MAX) {
		lprintf(0,"%s: Error: stream type in %s. Exiting.\n", PPREFIX(), context);
		// Actuall this probably means serious configuration problem. Prefer not to continue
		CANT_LOAD_CONFIGURATION_FILE_FATAL(path);
		ddl->type = DT_NONE;
	    }
	    LOAD_CONFIG_STRING_OPTION_WITH_DEFAULT_VALUE(d, context, ddl, tag, NULL);
	    if (strSafeLen(ddl->tag) < 2) {
		// TODO: remove this
		lprintf(0,"%s: Error: %s.tag is too short. Tags shall be at least two letters long!\n", PPREFIX(), context);
		CANT_LOAD_CONFIGURATION_FILE_FATAL(path);
	    }
	    if (strSafeNCmp(ddl->tag, "BN", 2) == 0) {
		// TODO: remove this
		lprintf(0,"%s: Error: %s.tag starts with prefix BN. This prefix is reserved for binary messages!\n", PPREFIX(), context);
		CANT_LOAD_CONFIGURATION_FILE_FATAL(path);
	    }
	    // Hmm. default name is derived from type. Maybe would be better to use tag
	    // LOAD_CONFIG_STRING_OPTION_WITH_DEFAULT_VALUE(d, context, ddl, name, deviceDataTypeNames[ddl->type]);
	    LOAD_CONFIG_STRING_OPTION_WITH_DEFAULT_VALUE(d, context, ddl, name, ddl->tag);
	    LOAD_CONFIG_DOUBLE_OPTION_WITH_DEFAULT_VALUE(d, context, ddl, latency, 0);
	    if (fabs(ddl->latency) >= 1.0) {
		lprintf(0,"%s: Warning: %s: latency above 1 second? Are you sure?.\n", PPREFIX(), context);
	    }
	    LOAD_CONFIG_DOUBLE_OPTION_WITH_DEFAULT_VALUE(d, context, ddl, timeout, 1.0);
	    LOAD_CONFIG_DOUBLE_OPTION_WITH_DEFAULT_VALUE(d, context, ddl, min_range, 0.001);
	    LOAD_CONFIG_DOUBLE_OPTION_WITH_DEFAULT_VALUE(d, context, ddl, max_range, 1e33);
	    LOAD_CONFIG_DOUBLE_OPTION_WITH_DEFAULT_VALUE(d, context, ddl, min_altitude, 0.001);
	    LOAD_CONFIG_DOUBLE_OPTION_WITH_DEFAULT_VALUE(d, context, ddl, max_altitude, 1e33);
	    LOAD_CONFIG_DOUBLE_OPTION_WITH_DEFAULT_VALUE(d, context, ddl, mandatory, 0);
	    LOAD_CONFIG_DOUBLE_OPTION_WITH_DEFAULT_VALUE(d, context, ddl, history_size, 2);
	    LOAD_CONFIG_DOUBLE_OPTION_WITH_DEFAULT_VALUE(d, context, ddl, debug_level, 30);
	    configVectorMaybeSpecifiedByUniqueNumber(d, ddl->weight, 4, "weight", 1.0, path, context);
	    //if (ddl->weight[0] != ddl->weight[1]) {
	    //lprintf(0,"%s: Warning: different weight for X and Y axis! Not yet implemented!\n", PPREFIX());
	    //}
	     LOAD_CONFIG_DOUBLE_OPTION_WITH_DEFAULT_VALUE(d, context, ddl, drift_fix_period, 0);
	    // we usually process all pending input from ParsedVector to RegressionBuffer at each tick
	    configInputBufferInit(dl, ddl);
	    regressionBufferInit(&ddl->outputBuffer, deviceDataStreamVectorLength[ddl->type], ddl->history_size, "%s.%s stream out buffer", dl->name, ddl->name);
	    memset(&ddl->launchData, 0, sizeof(ddl->launchData));
	    ddl->nextWithSameType = uu->deviceStreamDataByType[ddl->type];
	    uu->deviceStreamDataByType[ddl->type] = ddl;
	    LOAD_CONFIG_CONTEXT_POP(context);
	} else {
	    lprintf(0,"%s: Warning: maximum number of device data %d exceeded in configuration file %s. Ignoring.\n", PPREFIX(), DEVICE_DATA_MAX, path);
	}
    }
}


static int configGetDeviceIndex(char *name) {
    int i;

    // first look for the name in yet configured devices
    for(i = 0; i < uu->deviceMax; i++) {
	if (strSafeCmp(uu->device[i]->name, name) == 0) return(i);
    }
    // not found, create a new device
    if (i >= DEVICE_MAX) return(-1);
    uu->deviceMax++;
    CALLOC(uu->device[i], struct deviceData);
    uu->device[i]->name = name;
    return(i);
}

void configLoadDeviceConnection(struct jsonnode *cc, struct deviceData *dl, char *path, char *context) {
    struct jsonnode			*c;

    c = jsonFindObjectField(cc, "connection");
    if (c == NULL) {
	printf("%s: Error: Missing \"connection\" part for device %s in %s.\n", PPREFIX(), dl->name, path);
	CANT_LOAD_CONFIGURATION_FILE_FATAL(path);
	raspilotShutDownAndExit();
    }
    c->used = 1;
    LOAD_CONFIG_CONTEXT_PUSH(context, "connection");
    LOAD_CONFIG_ERROR_ON_WRONG_TYPE(c, path, context, "", JSON_NODE_TYPE_OBJECT);
    LOAD_CONFIG_ENUM_STRING_OPTION(c, context, &dl->connection, deviceConnectionTypeNames, type);
    switch (dl->connection.type) {
    case DCT_INTERNAL_ZEROPOSE:
	break;
    case DCT_COMMAND_BASH:
    case DCT_COMMAND_EXEC:
	LOAD_CONFIG_STRING_OPTION_WITH_DEFAULT_VALUE(c, context, &dl->connection.u, command, NULL);
	break;
    case DCT_NAMED_PIPES:
	LOAD_CONFIG_STRING_OPTION_WITH_DEFAULT_VALUE(c, context, &dl->connection.u.pp, read_pipe, NULL);
	LOAD_CONFIG_STRING_OPTION_WITH_DEFAULT_VALUE(c, context, &dl->connection.u.pp, write_pipe, NULL);
	break;
    default:
	lprintf(0,"%s: Error: None or unknown connection type for device %s in %s.\n", PPREFIX(), dl->name, path);
	CANT_LOAD_CONFIGURATION_FILE_FATAL(path);
	raspilotShutDownAndExit();
    }
    LOAD_CONFIG_CONTEXT_POP(context);
}

static void configLoadPidController(struct jsonnode *c, struct pidController *pp, char *name, double integralMax, double derivativeMax, char *path, char *context) {
    struct jsonnode *cc;

    memset(pp, 0, sizeof(*pp));
    pp->name = name;
    pp->constant.integralMax = integralMax;
    pp->constant.derivativeMax = derivativeMax;
    cc = jsonFindObjectField(c, name);
    if (cc != NULL) cc->used = 1;
    LOAD_CONFIG_CONTEXT_PUSH(context, "%s", name);
    LOAD_CONFIG_ERROR_ON_WRONG_TYPE(cc, path, context, "", JSON_NODE_TYPE_OBJECT);
    LOAD_CONFIG_DOUBLE_OPTION_WITH_DEFAULT_VALUE(cc, context, &pp->constant, p, 0.0);
    LOAD_CONFIG_DOUBLE_OPTION_WITH_DEFAULT_VALUE(cc, context, &pp->constant, i, 0.0);
    LOAD_CONFIG_DOUBLE_OPTION_WITH_DEFAULT_VALUE(cc, context, &pp->constant, d, 0.0);
    LOAD_CONFIG_DOUBLE_OPTION_WITH_DEFAULT_VALUE(cc, context, &pp->constant, ci, 0.0);
    LOAD_CONFIG_CONTEXT_POP(context);
}

void configLoadFromJsonNode(struct jsonnode *cc, char *path, char *context) {
    int					i;
    struct jsonnode			*c;
    struct jsonFieldList		*ll;
    struct deviceData			*dl;
    char				*name;
    struct pidController		pp;
    double				vv[MOTOR_MAX];
    struct config			*cfg;
    double				is1, is2;
    vec3				vvv;
	
    cfg = &uu->config;
    LOAD_CONFIG_DOUBLE_OPTION_WITH_DEFAULT_VALUE(cc, context, uu, motor_number, 4);

    // vector lengths may depend on some configuration, so I can not initialize before that point
    mainInitDeviceDataStreamVectorLengths(uu->motor_number);

    LOAD_CONFIG_DOUBLE_OPTION_WITH_DEFAULT_VALUE(cc, context, uu, autopilot_loop_Hz, 100);
    LOAD_CONFIG_DOUBLE_OPTION_WITH_DEFAULT_VALUE(cc, context, uu, stabilization_loop_Hz, 100);
    LOAD_CONFIG_DOUBLE_OPTION_WITH_DEFAULT_VALUE(cc, context, cfg, short_buffer_seconds, 0.05);
    if (uu->config.short_buffer_seconds < 1.0/uu->autopilot_loop_Hz) {
	lprintf(0, "%s: Error: short_buffer_seconds must be larger than 1/stabilization_loop_Hz\n", PPREFIX());
	uu->config.short_buffer_seconds = 2.0/uu->autopilot_loop_Hz;
    }
    LOAD_CONFIG_DOUBLE_OPTION_WITH_DEFAULT_VALUE(cc, context, cfg, long_buffer_seconds, 0.2);
    if (uu->config.long_buffer_seconds < 1.0/uu->autopilot_loop_Hz) {
	lprintf(0, "%s: Error: long_buffer_seconds must be larger than 1/stabilization_loop_Hz\n", PPREFIX());
	uu->config.long_buffer_seconds = 20.0/uu->autopilot_loop_Hz;
    }
    
    LOAD_CONFIG_DOUBLE_OPTION_WITH_DEFAULT_VALUE(cc, context, cfg, motor_thrust_min_spin, 0.01);
    LOAD_CONFIG_DOUBLE_OPTION_WITH_DEFAULT_VALUE(cc, context, cfg, pilot_reach_goal_orientation_time, 0.2);
    LOAD_CONFIG_DOUBLE_OPTION_WITH_DEFAULT_VALUE(cc, context, cfg, pilot_reach_goal_position_time, cfg->pilot_reach_goal_orientation_time*2+0.5);
    if (cfg->pilot_reach_goal_position_time < 2*cfg->pilot_reach_goal_orientation_time) {
	lprintf(0, "%s: Error: pilot_reach_goal_position_time has to be 2*pilot_reach_goal_orientation_time plus the time of fly\n", PPREFIX());
	cfg->pilot_reach_goal_position_time = 2 * cfg->pilot_reach_goal_orientation_time;
    }
    LOAD_CONFIG_DOUBLE_OPTION_WITH_DEFAULT_VALUE(cc, context, cfg, drone_max_inclination, M_PI/8);
    LOAD_CONFIG_DOUBLE_OPTION_WITH_DEFAULT_VALUE(cc, context, cfg, drone_panic_inclination, M_PI/2);
    LOAD_CONFIG_DOUBLE_OPTION_WITH_DEFAULT_VALUE(cc, context, cfg, drone_max_speed, 1.0);
    LOAD_CONFIG_DOUBLE_OPTION_WITH_DEFAULT_VALUE(cc, context, cfg, drone_max_rotation_speed, 0.5);
    LOAD_CONFIG_DOUBLE_OPTION_WITH_DEFAULT_VALUE(cc, context, cfg, drone_min_altitude, 0.05);
    LOAD_CONFIG_DOUBLE_OPTION_WITH_DEFAULT_VALUE(cc, context, cfg, drone_max_altitude, 20000.0);
    LOAD_CONFIG_DOUBLE_OPTION_WITH_DEFAULT_VALUE(cc, context, cfg, drone_waypoint_reached_range, 0.10);
    LOAD_CONFIG_DOUBLE_OPTION_WITH_DEFAULT_VALUE(cc, context, cfg, drone_waypoint_reached_angle, 0.20);
    
    configLoadVectorNoDefaultValue(cc, uu->motor_roll_forces, uu->motor_number, "motor_roll_forces", path, context);
    configLoadVectorNoDefaultValue(cc, uu->motor_pitch_forces, uu->motor_number, "motor_pitch_forces", path, context);
    configLoadVectorNoDefaultValue(cc, uu->motor_yaw_forces, uu->motor_number, "motor_yaw_forces", path, context);

    // Load PID values. Do this at the end because some default values may depend on other configuration values.
    configLoadPidController(cc, &pp, "pidXY", 50.0*cfg->drone_max_speed, 1.0*cfg->drone_max_speed, path, context);
    pp.name = "pidX";
    uu->pidX = pp;
    pp.name = "pidY";
    uu->pidY = pp;
    configLoadPidController(cc, &uu->pidRoll, "pidRoll", 1.0, 0.05, path, context);
    configLoadPidController(cc, &uu->pidPitch, "pidPitch", 1.0, 0.05, path, context);
    configLoadPidController(cc, &uu->pidYaw, "pidYaw", 1.0, 0.05, path, context);
    configLoadPidController(cc, &uu->pidAltitude, "pidAltitude", 1.0, 0.1, path, context);

    
    is1 = fabs(uu->pidX.constant.i) + fabs(uu->pidY.constant.i);
    is2 = fabs(uu->pidRoll.constant.i) + fabs(uu->pidPitch.constant.i);
    if (is1 != 0 && is2 != 0) {
	lprintf(0, "%s: Warning: Both pid{X,Y} and pid{Roll,Pitch} have I factor non-zero!\n", PPREFIX());
	lprintf(0, "%s: Warning: It is higly recommended to set I to zero for either pidX and pidY or pidRoll and pidPitch!\n", PPREFIX());
    }
    
    for(ll=configFindFieldList(cc, "device", JSON_NODE_TYPE_ARRAY, context); ll!=NULL; ll=ll->next) {
	c = ll->val;
	LOAD_CONFIG_ERROR_ON_WRONG_TYPE(c, path, context, "", JSON_NODE_TYPE_OBJECT);

	name = jsonFindString(c, "name", NULL);
	i = configGetDeviceIndex(name);
	if (i >= 0 && i < DEVICE_MAX) {
	    dl = uu->device[i];
	    LOAD_CONFIG_CONTEXT_PUSH(context, "device[%d]", i);
	    // reassign name to have it debug output
	    LOAD_CONFIG_STRING_OPTION_WITH_DEFAULT_VALUE(c, context, dl, name, NULL);
	    configLoadDeviceConnection(c, dl, path, context);
	    configLoadVectorWithDefaultValue(c, dl->mount_position, 3, "mount_position", 0.0, path, context, 0);
	    if (jsonFind(c, "mount_pry") != NULL) {
		configLoadVectorWithDefaultValue(c, vvv, 3, "mount_pry", 0.0, path, context, 0);
		dl->mount_rpy[0] = vvv[1]; dl->mount_rpy[1] = vvv[0]; dl->mount_rpy[2] = vvv[2]; 
	    } else {
		configLoadVectorWithDefaultValue(c, dl->mount_rpy, 3, "mount_rpy", 0.0, path, context, 0);
	    }
	    LOAD_CONFIG_BOOL_OPTION_WITH_DEFAULT_VALUE(c, context, dl, shutdownExit, 0);
	    LOAD_CONFIG_BOOL_OPTION_WITH_DEFAULT_VALUE(c, context, dl, data_ignore_unknown_tags, 0);
	    LOAD_CONFIG_DOUBLE_OPTION_WITH_DEFAULT_VALUE(c, context, dl, warming_time, 1.0);
	    configLoadDeviceStreams(c, dl, path, context);
	    LOAD_CONFIG_CONTEXT_POP(context);
	} else {
	    lprintf(0,"%s: Warning: maximum number of devices %d exceeded in configuration file %s. Ignoring.\n", PPREFIX(), DEVICE_MAX, path);
	}
    }
}

void configCheckForUnusedObjects(struct jsonnode *nn, char *context) {
    struct jsonFieldList	*ll;
    int 			contextlen;
    
    if (nn == NULL) return;

    contextlen = strlen(context);
    switch (nn->type) {
    case JSON_NODE_TYPE_ARRAY:
	for(ll=nn->u.fields; ll!=NULL; ll=ll->next) {
	    snprintf(context+contextlen, TMP_STRING_SIZE-contextlen, "[%d]", ll->u.index);
	    configCheckForUnusedObjects(ll->val, context);
	}
	break;
    case JSON_NODE_TYPE_OBJECT:
	for(ll=nn->u.fields; ll!=NULL; ll=ll->next) {
	    snprintf(context+contextlen, TMP_STRING_SIZE-contextlen, ".%s", ll->u.name);
	    if (ll->val != NULL && ll->val->used == 0) {
		lprintf(0,"%s: %s:%d: Warning: unknown option '%s' !\n", PPREFIX(), ll->val->pos.file, ll->val->pos.line, context);
	    }
	    configCheckForUnusedObjects(ll->val, context);
	}
	break;
    }
}

void configloadFile() {
    char			*path;
    char			hname[TMP_STRING_SIZE];
    char			ttt[TMP_STRING_SIZE];
    char			context[TMP_STRING_SIZE];
    char			*s, *ss;
    struct jsonnode		*cc, *c;
    struct jsonFieldList	*ll;

    path = uu->cfgFileName;
    if (path == NULL) path = "config.json";
    
#if 0
    // Old versions loaded configuration file based on hostname
    gethostname(hname, sizeof(hname));
    hname[sizeof(hname)-1] = 0;
    snprintf(ttt, sizeof(ttt)-1, "../cfg/%s.json", hname);
    ttt[sizeof(ttt)-1] = 0;
    path = ttt;
#endif
	
    ss = fileLoadToNewlyAllocatedString(path, 1);
    if (ss == NULL) {CANT_LOAD_CONFIGURATION_FILE_FATAL(path); raspilotShutDownAndExit();};

    s = jsonParseString(ss, path, &cc);
    if (s == NULL) {CANT_LOAD_CONFIGURATION_FILE_FATAL(path); raspilotShutDownAndExit();};
    FREE(ss);

    // go through parsed JSON
    memset(context, 0, sizeof(context));
    configLoadFromJsonNode(cc, path, context);
    
    memset(context, 0, sizeof(context));
    configCheckForUnusedObjects(cc, context);

    jsonFree(cc);
}

