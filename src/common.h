#ifndef _COMMON__H_
#define _COMMON__H_ 1


#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdarg.h>
#include <float.h>
#include <math.h>
#include <inttypes.h>
#include <fcntl.h>
#include <string.h>
#include <limits.h>
#include <time.h>
#include <errno.h>
#include <termios.h>
#include <signal.h>
#include <ctype.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/wait.h>

#include <pthread.h>

#include "expmem.h"
#include "sglib.h"

// we are using linmath library for quaternions.
// where q[0] == x; q[1] == y; q[2] == z; q[3] == w;

#include "linmath.h"
#include "linmath2.h"

//////////////////////////////////////////////////////////////

#ifndef EDEBUG
#define EDEBUG		0
#endif

//////////////////////////////////////////////////////////////
// Some constants controling pre flight behaviour

#define PILOT_SENSOR_MERGE_DEBUG_LEVEL 		20
#define PILOT_THRUST_NORMALIZATION_DEBUG_LEVEL 	22
#define PILOT_LAND_ALTITUDE			1.0

// The default configuration is overwritten at launch and land time
#define PILOT_LAUNCH_SPEED			1.0
#define PILOT_LAUNCH_GOAL_ORIENTATION_TIME	0.2
#define PILOT_LAUNCH_GOAL_POSITION_TIME		0.6
#define PILOT_LAUNCH_MAX_TIME			30.0
#define PILOT_LAND_SPEED			0.2

#define PILOT_PRELAUNCH_FREQUENCY_HZ		20

// put there 30 sec if using MPU-6050 which calibrates itself during that time
#define PILOT_WARMING_WARNING_ROTATION_TIME		0.5
#define PILOT_WARMING_WARNING_ROTATIONS_DELAY		3.0
#define PILOT_WARMING_WARNING_ROTATIONS_TO_LAUNCH	2.0

// Whether D action in PID is based on error or PV
#define PID_USES_ERROR_BASED_DERIVATIVE 	0
#define PID_PREVENT_INTEGRAL_WINDUP		1

//////////////////////////////////////////////////////////////

#if 0
#define select(...) (checkTimeLimit("", 0, 0), checkTimeLimit("select", 0.001, select(__VA_ARGS__)))
#define read(...)   (checkTimeLimit("", 0, 0), checkTimeLimit("read",   0.001, read(__VA_ARGS__)))
#define write(...)  (checkTimeLimit("", 0, 0), checkTimeLimit("write",  0.001, write(__VA_ARGS__)))
#define printf(...) (checkTimeLimit("", 0, 0), checkTimeLimit("printf", 0.001, printf(__VA_ARGS__)))
#define fflush(...) (checkTimeLimit("", 0, 0), checkTimeLimit("fflush", 0.001, fflush(__VA_ARGS__)))
#endif

#define PILOT_TICK_HZ			10

#define DEFAULT_DEBUG_LEVEL		10
#define MOTOR_MAX			16
#define DEVICE_MAX			64
#define DEVICE_DATA_MAX			32
#define DEVICE_DATA_VECTOR_MAX 		16

/////////////////////////////////////////////////////////////

#define TMP_STRING_SIZE                 255
#define STATIC_STRINGS_RING_SIZE        64
#define ZERO_SIZED_ARRAY_SIZE           0
#define MAGIC_NUMBER			0xcafe
#define ANGLE_NAN			0.12345e-67
// do not send thrust changes less than that because PWM has smal resolution anyway
#define MOTOR_THRUST_EPSILON		0.001

#define GRAVITY_ACCELERATION		9.8

/////////////////////////////////////////////////////////////

// spaecial valu fro motor setting function
#define MOTORS_ALL			-1

#define POSE_HISTORY_LAST(hh) (&(hh)->a[(hh)->ailast])
#define POSE_HISTORY_PREVIOUS(hh) (&(hh)->a[(hh)->aiprev])

/////////////////////////////////////////////////////////////////////////////////////////////

#define MAGIC_CHECK(uu)				(assert((uu)->magicNumber == MAGIC_NUMBER))
#define MIN(x,y)                        	((x)<(y)?(x):(y))
#define MAX(x,y)                        	((x)>(y)?(x):(y))
#define SIGN(x)					((x)>0?1:(x)<0?-1:0)
#define DIM(x)                          	(sizeof(x) / sizeof(x[0]))
#define va_copy_end(x)                  	{}
#define CORE_DUMP()                     	{if (fork()==0) {signal(SIGABRT, SIG_DFL); abort();}}
#define PRINTF(msg, ...)                	{???printf("%s:%d: " msg, __FILE__, __LINE__ __VA_OPT__(,) __VA_ARGS__);}
#define PRINTF_AND_RETURN(val, msg, ...)    	{PRINTF(msg, __VA_ARGS__); return val;}
#define PRINTF_AND_EXIT(val, msg, ...)    	{PRINTF(msg, __VA_ARGS__); exit(val) ;}
#define PRINTF_AND_ABORT(msg, ...)    		{PRINTF(msg, __VA_ARGS__); abort() ;}

#define ALLOC(p,t)          {(p) = (t*) expmemMalloc(sizeof(t)); if((p)==NULL) {printf("apsarapilot: Out of memory\n"); assert(0);shutdown();}}
#define CALLOC(p,t)         {ALLOC(p,t); memset((p), 0, sizeof(t));}
#define REALLOC(p,t)        {(p) = (t*) expmemRealloc((p), sizeof(t)); if((p)==NULL && (n)!=0) {printf("apsarapilot: Out of memory\n"); assert(0);shutdown();}}
#define ALLOCC(p,n,t)       {(p) = (t*) expmemMalloc((n)*sizeof(t)); if((p)==NULL && (n)!=0) {printf("apsarapilot: Out of memory\n"); assert(0);shutdown();}}
#define CALLOCC(p,n,t)      {ALLOCC(p,n,t); memset((p), 0, (n)*sizeof(t));}
#define REALLOCC(p,n,t)     {(p) = (t*) expmemRealloc((p), (n)*sizeof(t)); if((p)==NULL && (n)!=0) {printf("apsarapilot Out of memory\n"); assert(0);shutdown();}}
#define ALLOC_SIZE(p,t,n)   {(p) = (t*) expmemMalloc(n); if((p)==NULL && (n)!=0) {printf("apsarapilot: Out of memory\n"); assert(0); exit(1);shutdown();}}
#define FREE(p)             {expmemFree(p); }

#define UTIME_AFTER_MINUTES(n)    (currentTime.usec + 1000000LL*60*(n))
#define UTIME_AFTER_SECONDS(n)    (currentTime.usec + 1000000LL*(n))
#define UTIME_AFTER_MSEC(n)       (currentTime.usec + 1000LL*(n))
#define UTIME_AFTER_USEC(n)       (currentTime.usec + (n))

#define TLINE_UTIME_AFTER_MINUTES(n)    (currentTimeLineTimeUsec + 1000000LL*60*(n))
#define TLINE_UTIME_AFTER_SECONDS(n)    (currentTimeLineTimeUsec + 1000000LL*(n))
#define TLINE_UTIME_AFTER_MSEC(n)       (currentTimeLineTimeUsec + 1000LL*(n))
#define TLINE_UTIME_AFTER_USEC(n)       (currentTimeLineTimeUsec + (n))

#define PPREFIX()            (printPrefix_st(uu, __FILE__, __LINE__))
#define STR_ERRNO()               (strerror(errno))
#define STRINGIFY(x)		  #x
#define FILE_LINE_ID_STR()	  __FILE__ ":" STRINGIFY(__LINE__) 

#define DEBUG_HERE_I_AM()         {printf("%s: H.I.AM: %s:%d\n", PRINT_PREFIX(), __FILE__, __LINE__); fflush(stdout);}

#define ENUM_NAME_SET(e, x) {assert(x<DIM(e)); e[x] = #x;}
#define ENUM_NAME_NO_NULL_CHECK(names, i) {if (names[i] == NULL) {printf("%s:%s:%d: Internal error: Enumeration \"%s\" name item %d is not filled! Did you add an item and not updated names? Fatal, exiting!\n", PPREFIX(), __FILE__, __LINE__, #names, i); exit(-1);}}
#define ENUM_NAME_CHECK(names, prefix) {				\
	int i;								\
	if (names[prefix##MAX] == NULL || strcmp(names[prefix##MAX], #prefix "MAX") != 0) { \
	    printf("%s: Internal error: prefix \"%s\" does not match enumeration \"%s\". Fatal, exiting!\n", PPREFIX(), #prefix, #names); \
	    exit(-1);							\
	};								\
	for(i=0;i<=prefix##MAX;i++) { ENUM_NAME_NO_NULL_CHECK(names, i); } ; \
    }
#define ENUM_NAME_AND_DESC_SET(e, x, d, y) {ENUM_NAME_SET(e, x); d[x] = y;}


#define SKIP_SPACE(p) {while (*p==' ' || *p=='\t' || *p=='\f' || *p=='\v' || *p == '\r') p ++;}

#define NMEA_NEXT_FIELD(p) {						\
	while (*p != 0 && *p != ',') p++;				\
	if (*p == 0) {							\
	    lprintf(0, "%s: Error: Unexpected end of msg reached in %s\n", PPREFIX(), s); \
	    return(-1);							\
	}								\
	p ++;								\
    }


#define CALLBACK_CALL(hook, command) {				\
        int _i_;						\
        for(_i_=(hook).i-1; _i_ >= 0; _i_--) {			\
            callBackHookFunType callBack = (hook).a[_i_];	\
            if (command) break;					\
        }							\
    }


#undef assert
#define assert(x)             {                                         \
        if (!(x)) {                                                     \
            printf("%s:%d: assertion %s failed  !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n", __FILE__, __LINE__, #x); \
            fflush(stdout);                                             \
            if (EDEBUG) CORE_DUMP();                                     \
            shutdown();					\
        }                                                               \
    }

#define InternalCheck(x) {						\
        if (! (x)) {                                                    \
            printf("%s: Error: Internal check %s failed at %s:%d\n", PPREFIX(), #x, __FILE__, __LINE__); \
            fflush(stdout);                                             \
        }                                                               \
    }


////////////////////////////////////////////////////////////////////////////////////////////////////////
// forward decls

struct universe;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////

// TODO: Remove  all this coordinate system stuff. It is overcomplicated. Instead implement
// gotoWaypointToGps(), gotoWaypointRelative(), etc. ...
enum coordinateSystemEnum {
    
    // CS stands for Coordinate System
    CS_NONE,
    
    // GBASE stands for "Gravity Base", it is a static CS in meters. Point [0,0,0] is at the point where the drone starts and its
    // Z-axis points up from earth center (that's why CS_GBASE and not CS_BASE),
    // X-axis points forward from the drone perspective at the moment of start,
    // Y-axis points left from the drone perspective at the moment of start,
    // GBASE is 'created' at the moment of the start, and stay unchanged no matter how drone moves.
    // Coordinates of anything in the worlds do not change in GBASE since the drone initalization.
    CS_GBASE,
    
    // Drone is a CS relative to the drone center. Point [0,0,0] refers to the center of gravity of the drone.
    // Coordinate system moves with drone movement so that
    // Z-axis points up from the drone perspective
    // X-axis points forward from the drone perspective
    // Y-axix points left from the drone perspective
    // CS_DRONE is not the same as CS_GBASE even at the moment of the start, because its Z-axis does not point
    // from the earth center, but is relative to the drone orientation. X,Y axis of CS_DRONE are the same as CS_GBASE
    // at the moment of start. Then CS_DRONE moves as the drone moves while CS_GBASE remains stable.
    CS_DRONE,

    // GPS (not yet implemented) is a static earth fixed CS:
    // Z-axis corresponds to the altitude in meters from sea level?
    // X-axis corresponds to the longitude (East positive, West negative) in degrees from Greenwich 
    // Y-axis corresponds to the latitude (North positive, South negative) in degrees from Equator
    CS_GPS,

    // Earth (not yet implemented) is an earth fixed CS expressed in meters (not degrees) starting from an absolute "Zero Point" where
    // Z-axis corresponds to the altitude in meters from the level of "Zero Point" 
    // X-axis corresponds to the longitude (East positive, West negative) in meters from "Zero Point"
    // Y-axis corresponds to the latitude (North positive, South negative) in meters from "Zero Point"
    // It is a short range "local version" of GPS. GPS is to be transformed to CS_EARTH before beeing used.
    CS_EARTH,

    // Global is a deep space fixed CS (envisaged for our Mars mission :))
    CS_GLOBAL,



    ///////////////////////////////////
    
    CS_APRIL,	// temporary for milestone 1
    
    CS_MAX,
};

enum deviceDataTypes {
    DT_NONE,
    
    DT_DEBUG,
    DT_PONG,
    DT_POSITION_VECTOR,
    DT_GROUND_DISTANCE,
    DT_ORIENTATION_RPY_COMPASS,
    DT_ORIENTATION_QUATERNION,
    
    DT_POSITION_NMEA,
    DT_MAGNETIC_HEADING_NMEA,
    
    DT_MAX,
};

enum deviceConnectionTypeEnum {
    DCT_NONE,
    DCT_INTERNAL_ZEROPOSE,
    DCT_INTERNAL_GYROPOSE,
    DCT_COMMAND_BASH,
    DCT_COMMAND_EXEC,
    DCT_NAMED_PIPES,
    /* in the future we can do something like:
    DCT_TCPIP_CLIENT_SOCKET,
    DCT_TCPIP_SERVER_SOCKET,
    DCT_UDP_SOCKET,
    */
    DCT_MAX,
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
// json stuff
    
#define JSON_ACCEPT_CPP_LINE_DIRECTIVES 	1

// TODO: This is just for testing, set following two to 0 for production
#define JSON_OBJECT_TREE 			1
#define JSON_ARRAY_TABLE 			1
#define JSON_ARRAY_TABLE_ONLY 			0
#define JSON_SOURCE_POSITIONS 			1    

// If you change enum, do not forget to chage corresponing ENUM_NAMES in the #define
enum jsonNodeTypeEnum {
    JSON_NODE_TYPE_BOOL,
    JSON_NODE_TYPE_NUMBER,
    JSON_NODE_TYPE_STRING,
    JSON_NODE_TYPE_OBJECT,
    JSON_NODE_TYPE_ARRAY,
};
#define JSON_NODE_TYPE_ENUM_NAMES {		\
	"JSON_NODE_TYPE_BOOL",			\
	    "JSON_NODE_TYPE_NUMBER",		\
	    "JSON_NODE_TYPE_STRING",		\
	    "JSON_NODE_TYPE_OBJECT",		\
	    "JSON_NODE_TYPE_ARRAY",		\
	    }

// structure storing where in the source file the node occurs
struct jsonPosition {
    char *file;
    int line;
};

struct jsonnode {
    int type;
    int	used;			// raspilot flag whether the configuration was read by pilot
    union {
        int                     b;
        double                  n;
        char                    *s;
        struct jsonFieldList    *fields;
#if JSON_OBJECT_TREE
	struct {
	    struct jsonFieldList *fields;
	    struct jsonFieldList *tree;
	} t;
#endif
#if JSON_ARRAY_TABLE || JSON_ARRAY_TABLE_ONLY
	struct {
#if ! JSON_ARRAY_TABLE_ONLY	    
	    struct jsonFieldList *fields;
#endif	    
	    int			 tableSize;
	    struct jsonnode 	**table;
	} a;
#endif
    } u;
#if JSON_SOURCE_POSITIONS
    struct jsonPosition		pos;
#endif
};

struct jsonFieldList {
    union {
        int                     index;
        char                    *name;
    } u;
    struct jsonnode         	*val;
    struct jsonFieldList    	*next;
#if JSON_OBJECT_TREE
    char 			nodeColor;
    struct jsonFieldList    	*left, *right;
#endif
};

/////////////////////////////////////////////////////////////
// time / timeline 
struct globalTimeInfo {
    long long int   usec;       // micro seconds since epoch
    long long int   msec;       // milli seconds since epoch
    time_t          sec;        // seconds since epoch
    double          dtime;      // time since "zero moment" in seconds with microsecond precission
    int             hour;       // hours since epoch
    int             msecPart;   // milli seconds since last second (0 .. 999) 
    int             usecPart;   // micro seconds since last second (0 .. 999999)

    struct tm       gmttm;      // GMT time in form year, month, day, etc.
    struct tm       lcltm;      // Local time in form year, month, day, etc.
};


// timeline is a sorted list of timeLineEvents which shall be executed at given time
struct timeLineEvent {
    long long int           usecond;                // event time: micro seconds since epoch
    void                    (*event)(void *arg);    // function to be called at given usecond
    void                    *arg;                   // argument to be used when calling event()
    struct timeLineEvent    *next;                  // next event in the timeline
};

typedef void (timelineEventFunType)(void *arg);

//////////////////////////////////////////////////////////////////////////////////////////////
// baio == basic/buffered assynchronous input  output

#define BAIO_MAX_CONNECTIONS                1024
#define SIZEOF_FD_SETSIZE                   (((BAIO_MAX_CONNECTIONS - 1) / 1024 + 1) * sizeof(fd_set))
#define EXTENDED_FD_ZERO(rset)              memset(rset, 0, SIZEOF_FD_SETSIZE)

#if INT_MAX <= 32767
#error INT size is too small to encode baio magic numbers
#elif INT_MAX <= 2147483647
#define BAIO_MAGIC_MASK			    0x01ffe
#else
#define BAIO_MAGIC_MASK			    0x01fffffe
#endif


enum baioDirection {
    BAIO_IO_DIRECTION_NONE,
    BAIO_IO_DIRECTION_READ,
    BAIO_IO_DIRECTION_WRITE,
    BAIO_IO_DIRECTION_RW,
    BAIO_IO_DIRECTION_MAX,
};

enum baioTypes {
    BAIO_TYPE_NONE,
    BAIO_TYPE_FD,                   // basic baio, fd is supplied from outside
    BAIO_TYPE_FILE,                 // file i/o
    BAIO_TYPE_SERIAL,               // file representing serial port (needs init at open)
    BAIO_TYPE_PIPED_COMMAND,        // bash command connected via a single pipe
    BAIO_TYPE_NAMED_PIPES,          // a pair of named Unix pipes
    BAIO_TYPE_UDP,
    BAIO_TYPE_MAX,
};

/////////////////////////////////////////////////////////////////////////////////////
// max user defined parameters inside assynchronous I/O structures
#define BAIO_MAX_USER_PARAMS                        8
#define BAIO_MAX_INACTIVITY_TIME_INFINITY           (-1)

// Not all combinations of following masks are possible
#define BAIO_STATUS_ZOMBIE                          0x000000
#define BAIO_STATUS_ACTIVE                          0x000001
#define BAIO_STATUS_EOF_READ                        0x000002
#define BAIO_STATUS_PENDING_CLOSE                   0x000004

// event statuses, listen is not clearable status
#define BAIO_BLOCKED_FOR_READ_IN_READ               0x001000
#define BAIO_BLOCKED_FOR_WRITE_IN_WRITE             0x002000
#define BAIO_READ_BUFFER_HAS_SPACE_FOR_READ(bb)     (bb->initialReadBufferSize - (bb->readBuffer.j - bb->readBuffer.i) > bb->minFreeSizeBeforeRead)

// user's can have values stored in structure baio. Those params are stored in an array
// of the following UNION type
union baioUserParam {
    void                *p;         // a pointer parameter
    int                 i;          // an integer parameter
    double              d;          // a double parameter
};

struct baioBuffer {
    // buffered characters
    char    *b;
    // buffer is simply a linear array. Values between b[i] and b[j]
    // contains valid chars
    // Baio can move whole buffer (i.e. move b[i .. j] -> b[0 .. j-i) and consequently i,j -> 0,j-i; at any time.
    int     i,j;
    // allocated size
    int     size;       
};

typedef int (*callBackHookFunType)(void *x, ...);
typedef callBackHookFunType callBackHookFunArgType;

struct callBackHook {
    callBackHookFunType    *a;
    int                         i, dim;
};

struct baio {
    int                     baioType;           // type of baio BAIO_TYPE_...
    int                     ioDirections;
    int                     rfd;		// reading fd
    int                     wfd;		// writting fd
    unsigned                status;
    int                     index;
    int                     baioMagic;
    struct baioBuffer       readBuffer;
    struct baioBuffer       writeBuffer;

    // when the connection is inactive for a time longer then "maxInactivityTime", it is closed
    time_t                  lastActivityTime;

    // If you add a callback hook, do not forget to FREE it in baioFreeZombie and CLONE it in cloneCallBackHooks
    // callbackhooks
    struct callBackHook        callBackOnRead;
    struct callBackHook        callBackOnWrite;
    struct callBackHook        callBackOnError;
    struct callBackHook        callBackOnEof;
    struct callBackHook        callBackOnDelete;
    struct callBackHook        callBackOnBufferShift;

    // config values
    int                     initialReadBufferSize;
    int                     initialWriteBufferSize;
    int                     minFreeSizeBeforeRead;
    int                     minFreeSizeAtEndOfReadBuffer;   // if you want to put zero for example
    time_t                  maxInactivityTime;

    // when we allocate this structure we allocate some additional space
    // for other extensions and we keep the size of additional space
    int                     additionalSpaceAllocated;
    // user defined values, they must be at the end of the structure!
    union baioUserParam     userParam[BAIO_MAX_USER_PARAMS];

    // here is the end of the structure
    void                    *endOfBaio[ZERO_SIZED_ARRAY_SIZE];
};

//////////////////////////////
// misc

struct terminalSettingStr {
    int			fd;
    int			ioSavedFlag;
    struct termios 	io;
};

struct pidControllerConstants {
    double 	p,i,d;
    
    // We are using a simple extension of PID controllers because
    // we want to set 'i' to zero in some PIDs accumulating always the same value in their integral.
    // To achieve this we have added 'ci' value which is a constant
    // added to the output.
    // If you have a PID controller accumulating always the same
    // value X in its 'integral' variable then you can replace that PID with
    // the one having 'ci' set to 'i*X' where 'i' is the 'i' from the original PID.
    // Then you can set 'i' in the new controller to zero.
    // Such a controller can not 'diverge' as it has no internal state.
    double	ci;

    // some safety value to be checked to prevent PID's windup (overflow of integral sum)
    double	integralMax;

};

struct pidControllerData {
    double 	integral;

    double 	previous_error;
    double 	previous_output;
    double 	previous_setpoint;
    double 	previous_measured_value;

    // some values for statistics
    int		statNumberOfSamples;
    double	statIntegralSum;
    double	statErrorSum;
};

struct pidController {
    char				*name;
    struct pidControllerConstants 	constant;
    struct pidControllerData		d;
};

//////////////////////////////////////////////////////////////////////////////////////////////
// configuration

// first 3 values are position then 4 values are orientation quaternion, then 3 are eulers
#define POSE_VECTOR_LENGTH 10
#define GET_ROLL_PITCH_YAW_FROM_POSE_VECTOR(pr, roll, pitch, yaw) {(roll)=(pr)[7]; (pitch)=(pr)[8]; (yaw)=(pr)[9];}

struct pose {
    double			time;
    // position vector: first 3 values are position then 4 values are orientation quaternion, then 3 are orientation in RPY
    // i.e. pr[7] - roll; pr[8] - pitch; pr[9] - yaw
    // roll  - negative == left wing down; positive == left wing up
    // pitch - negative == nose down;      positive == nose up
    // yaw   - positive == rotated counterclockwise (view from up down) (

    double			pr[POSE_VECTOR_LENGTH];	
};

// TODO: rename to poseBuffer or something liket that
struct poseHistory {
    char		name[256];	// for debug output only, to be removed or replaced by char *name;
    struct pose		*a;		// a[size]
    int			ai;		// index where next elem will be stored
    int			ailast;		// index where last elem was added, i.e.  (ai-1) % size
    int			aiprev;		// index where previous last elem was added, i.e. (ai-2)%size
    int			n;		// number of total inserted elements (not only the stored elems)
    int			size;

    // sums used for least square method
    double		sumsTimeOffset;
    double		sumTime;
    double		sumTimeSquare;
    double		sumPr[POSE_VECTOR_LENGTH];
    double		sumPrMulTime[POSE_VECTOR_LENGTH];

    // sums used for total statistics
    double		totalSumForStatistics[POSE_VECTOR_LENGTH];    
};

union deviceDataDataUnion {
    // basically all devices provide some vector of doubles,
    // however, keep this union for case you will need to implement
    // some special devices which do not fit
    double			v[DEVICE_DATA_VECTOR_MAX];
};

struct deviceDataData {
    // values from config
    char			*name;
    char			*tag;
    enum coordinateSystemEnum	cs;
    enum deviceDataTypes	type;
    int				length;		// length of the vector if type == DDT_VECTOR
    double			latency;	// data received refers to the time lastDataTime - latency
    double			timeout;	// if last value is more then timeout old, sensor is ignored
    double			min_range;	// minimal valid range for radar
    double			max_range;	// minimal valid range for radar
    vec3			weight;		// weight is per axis in GBASE
    uint8_t			mandatory;	// whether device has to be active before launch
    int				history_size;   // the size of the history saved for linear interpolation, rename to regression_points or similar
    int				debug_level;

    // run time values
    struct deviceData		*dd;		// "back" point to device data where I belong

    // history of last received data
    struct poseHistory   	history;
    
    // maybe confidence shall be per sample and stored in history? Then the average confidence will be used.
    double			confidence;	

    // for statistics
    double			pongTotalTimeForStatistics;
    int				totalNumberOfRecordsReceivedForStatistics;
    
    // This is the pose the device reported before drone launch
    // It is used to translate device poses to actual poses of GBASE coordinate system during the flight
    struct pose   		launchPose;
    uint8_t			launchPoseSetFlag;	// whether we have yet stored the launch pose

    struct deviceDataData 	*nextWithSameType;
};

struct connection {
    enum deviceConnectionTypeEnum type;
    union {
	char		*command;
	struct {
	    char	*read_pipe;
	    char	*write_pipe;
	} pp;
    } u;
};

struct deviceData {
    char			*name;

    // for now, consider all devices are sending line after line
    // with the relevant parts split by ddt names
    char			*command;
    struct connection		connection;
    struct deviceDataData	*ddt[DEVICE_DATA_MAX];
    int				ddtMax;
    vec3			mount_position;
    vec3			mount_rpy;		// roll pitch yaw
    double			warming_time;
    uint8_t			data_ignore_unknown_tags;
    
    uint8_t			enabled;
    int 			baioMagic;
    double 			lastActivityTime;
};

/////////////////////////////////////////

struct motorStr {
    double			thrust;
    int				rotationSpeed;
    int				lastSentRotationSpeed;

    double			lastSentThrust;
    double			totalWork;
};

struct waypoint {
    vec3			position;
    double			yaw;
};

struct config {
    double			motor_thrust_min_spin;
    double			pilot_reach_goal_orientation_time;
    double			pilot_reach_goal_position_time;
    double			drone_max_inclination;
    double			drone_max_speed;
    double			drone_max_rotation_speed;
    double			drone_min_altitude;
    double			drone_max_altitude;
    double			drone_waypoint_reached_range;
    double			drone_waypoint_reached_angle;
    double			short_history_seconds;
};

struct universe {
    // configuration
    char			*cfgFileName;
    
    // if non NULL, ping to the controllingHostIp to check if we are connected to it
    // if not, initiate landing or return to home
    char			*pingToHost;
    
    double			stabilization_loop_Hz;
    int 			motor_number;
    double			motor_roll_forces[MOTOR_MAX];
    double			motor_pitch_forces[MOTOR_MAX];
    double			motor_yaw_forces[MOTOR_MAX];

    struct pidController	pidRoll;
    struct pidController	pidPitch;
    struct pidController	pidYaw;
    struct pidController	pidX;
    struct pidController	pidY;
    struct pidController	pidAltitude;

    // TODO: put all this to some "struct config" which i can save and overwrite during start
    struct config		config;
    
    struct waypoint 		currentWaypoint;
    
    
    struct deviceData		*device[DEVICE_MAX];
    int				deviceMax;

    struct deviceDataData	*deviceDataDataByType[DT_MAX];	// lists by data types
    
    // run time values
    double			pilotStartingTime;		// the time when pilot was launched
    
    struct motorStr		motor[MOTOR_MAX];

    int				motorBaioMagic;
    double			pilotLaunchTime;		// time when pilot was started
    double			flyStartTime;			// start flying time
    double			motorFirstSendTime;		// when we lastly send a command to motors
    double			motorLastSendTime;		// when we lastly send a command to motors

    struct poseHistory		shortHistoryPose;
    struct poseHistory		shortHistoryVelo;

    // end of universe
    unsigned			magicNumber;
};

typedef int deviceDataParseFunction(char *tag, char *p, struct deviceData *dd, struct deviceDataData *ddd);

//////////////////////////////////////////////////////////////////////////////////////////////
//
// common.c
extern int 			debugLevel;
extern struct universe		*uu;
extern struct globalTimeInfo	currentTime;
extern struct timeLineEvent     *timeLine;
extern uint64_t			currentTimeLineTimeUsec;
extern int64_t 			nextStabilizationTickUsec;
extern int			shutDownInProgress;
extern struct jsonnode 		dummyJsonNode;
extern char 			*signalInterruptNames[258];
extern char 			*deviceDataTypeNames[DT_MAX+2];
extern char 			*coordinateSystemNames[CS_MAX+2];
extern char			*deviceConnectionTypeNames[DCT_MAX+2];

char *getTemporaryStringPtrFromStaticStringRing();
int hexDigitCharToInt(int hx) ;
int intDigitToHexChar(int x) ;
char *strDuplicate(char *s) ;
char *strnDuplicate(char *s, int max) ;
char *strSafeDuplicate(char *s) ;
int strSafeCmp(char *s1, char *s2) ;
int isspaceString(char *s) ;
void writeToFd(int fd, char *buf, int bufsize) ;
char *printPrefix_st(struct universe *uu, char *file, int line) ;
void dumpHex(char *msg, char *d, int len) ;
double normalizeToRange(double value, double min, double max) ;
double angleSubstract(double a1, double a2) ;
char *fileLoadToNewlyAllocatedString(char *path, int useCppFlag) ;
double normalizeAngle(double omega, double min, double max) ;
double normalizeToRange(double value, double min, double max) ;
double truncateToRange(double x, double min, double max) ;
void vec2Rotate(double *res, double *v, double theta) ;

char *currentLocalTime_st() ;
char *sprintSecTime_st(long long int utime) ;
char *sprintUsecTime_st(long long int utime) ;
void setCurrentTimeToTimeVal(struct timeval *tv) ;
void setCurrentTime() ;
void incrementCurrentTime() ;
int checkTimeLimit(char *op, double maxTime, int res) ;
void timeLineInsertEvent(long long int usec, void (*event)(void *arg), void *arg) ;
struct timeLineEvent *timeLineFindEventAtUnknownTime(void (*event)(void *arg), void *arg) ;
void timeLineRemoveEvent(long long int usec, void (*event)(void *arg), void *arg) ;
int timeLineRemoveEventAtUnknownTime(void (*event)(void *arg), void *arg) ;
int timeLineRemoveEventAtUnknownTimeAndArg(void (*event)(void *arg)) ;
int timeLineRemoveAllEvents() ;
int timeLineRescheduleUniqueEvent(long long int usec, void (*event)(void *arg), void *arg) ;
int timeLineRescheduleUniqueEventIfExisted(long long int usec, void (*event)(void *arg), void *arg) ;
int timeLineInsertUniqEventIfNotYetInserted(long long int usec, void (*event)(void *arg), void *arg) ;
void timeLineTimeToNextEvent(struct timeval *tv, int maxseconds) ;
int timeLineExecuteScheduledEvents(int updateCurrenttimeFlag) ;
void timeLineDump() ;
char *arrayWithDimToStr_st(double *a, int dim) ;
void deviceInitiate(int i) ;
void sendToAllDevices(char *fmt, ...) ;
void enumNamesInit() ;
void enumNamesPrint(FILE *ff, char **names) ;
int enumNamesStringToInt(char *s, char **names) ;
void terminalResume() ;
int stdbaioStdinMaybeGetPendingChar() ;
void stdbaioStdinClearBuffer();
void poseHistoryAddElem(struct poseHistory *hh, struct pose *pp) ;
void poseHistoryInit(struct poseHistory *hh, int size) ;
int poseHistoryGetRegressionCoefficients(struct poseHistory *hh, int i, double *k0, double *k1) ;
void poseVectorAssign(struct pose *res, struct pose *a) ;
void poseVectorScale(struct pose *res, struct pose *a, double factor) ;
void poseVectorAdd(struct pose *res, struct pose *a, struct pose *b) ;
void poseVectorSubstract(struct pose *res, struct pose *a, struct pose *b) ;
void poseHistoryGetMean(struct poseHistory *hh, struct pose *res) ;
int poseHistoryEstimatePoseForTimeByLinearRegression(struct poseHistory *hh, double time, struct pose *res) ;
void vec3TruncateToSize(vec3 r, double size, char *warningId) ;
void pidControllerReset(struct pidController *pp) ;
char *pidControllerStatistics(struct pidController *pp, int showProposedCiFlag) ;
double pidControllerStep(struct pidController *pp, double setpoint, double measured_value, double dt) ;
void quatToYpr(quat qq, double *yaw, double *pitch, double *roll);
void yprToQuat(double yaw, double pitch, double roll, quat q) ;
void stdbaioInit() ;
void stdbaioClose() ;
int lprintf(int level, char *fmt, ...);
void trajectoryLogInit() ;
int trajectoryLogPrintf(char *fmt, ...) ;
void trajectoryLogClose() ;
void pingToHostInit() ;
void pingToHostRegularCheck(void *d) ;
void pingToHostClose() ;

// json.c
extern char *jsonNodeTypeEnumNames[];

char *jsonParseString(char *jsonString, char *inputName, struct jsonnode **res) ;
void jsonFree(struct jsonnode *nn) ;
void jsonPrintString(char *ss, FILE *ff) ;
void jsonPrint(struct jsonnode *nn, FILE *ff) ;
struct jsonnode *jsonFindArrayIndex(struct jsonnode *nn, int index) ;
struct jsonnode *jsonFindObjectField(struct jsonnode  *nn, char *name) ;
struct jsonnode *jsonFind(struct jsonnode *nn, char *composedField) ;
double jsonFindDouble(struct jsonnode  *nn, char *name, double defaultValue) ;
char *jsonFindString(struct jsonnode   *nn, char *name, char *defaultValue) ;
struct jsonFieldList *jsonFindFieldList(struct jsonnode	*nn, char *name, int jsonType) ;
char *jsonParseStringLiteral(char *ss, char **res) ;

// config.c
void configloadFile() ;
void mainLoadMissionFile() ;

// pilot.c
int raspilotPoll() ;
void raspilotBusyWait(double sleeptime) ;
int raspilotInit(int argc, char **argv) ;
int raspilotShutDownAndExit() ;
void raspilotLand(double x, double y) ;
void raspilotPreLaunchSequence() ;
void raspilotLaunch(double altitude) ;
void raspilotWaypointSet(double x, double y, double z, double yaw) ;
void raspilotGotoWaypoint(double x, double y, double z, double yaw) ;
double raspilotCurrentAltitude() ;
int raspilotWaypointReached() ;
void motorsThrustSend(void *d) ;
void motorsStandby(void *d) ;
void motorsThrustSet(double thrust) ;
void motorThrustSetAndSend(int i, double thrust) ;
void motorsThrustSetAndSend(double thrust) ;
void motorsStop(void *d) ;
void motorsEmmergencyLand() ;
void pilotImmediateLanding() ;
void pilotInteractiveInputRegularCheck(void *d) ;
void pilotRegularSpecialTick(void *d) ;
void pilotRegularPreLaunchTick(void *d) ;
void pilotRegularStabilizationLoopTick(void *d) ;
int pilotAreAllDevicesReady() ;
void pilotStoreLaunchPose(void *d) ;
void pilotRegularStabilizationLoopRescheduleToSoon() ;
void pilotRegularMotorPing(void *d) ;
void pilotRegularSaveTrajectory(void *d) ;
void pilotRegularWaitForDevicesAndStartPilot(void *d) ;
void pilotUpdatePositionHistoryAndRecomputeMotorThrust() ;

// baio.c
extern int baioDebugLevel;
void callBackClearHook(struct callBackHook *h) ;
int callBackAddToHook(struct callBackHook *h, callBackHookFunArgType ptr) ;
void callBackRemoveFromHook(struct callBackHook *h, callBackHookFunArgType ptr) ;
int baioLibraryInit(int deInitializationFlag) ;
int baioClose(struct baio *bb) ;
int baioCloseMagic(int baioMagic) ;
int baioCloseMagicOnError(int baioMagic) ;
void baioCloseFd(struct baio *bb) ;
struct baio *baioFromMagic(int baioMagic) ;
void baioPurgeInactiveConnections(int maxCheckedItems) ;
int baioWriteToBuffer(struct baio *bb, char *s, int len) ;
int baioVprintfToBuffer(struct baio *bb, char *fmt, va_list arg_ptr) ;
int baioPrintfToBuffer(struct baio *bb, char *fmt, ...) ;
int baioPoll(int timeOutUsec);
pid_t popen2(char *command, int *in_fd, int *out_fd, int useBashFlag) ;
struct baio *baioNewBasic(int baioType, int ioDirections, int additionalSpaceToAllocate) ;
struct baio *baioNewFile(char *path, int ioDirection, int additionalSpaceToAllocate) ;
struct baio *baioNewSerial(char *path, int baudrate, int ioDirection, int additionalSpaceToAllocate) ;
struct baio *baioNewPipedCommand(char *command, int ioDirection, int useBashFlag, int additionalSpaceToAllocate) ;
struct baio *baioNewNamedPipes(char *readPipePath, char *writePipePath, int additionalSpaceToAllocate) ;
struct baio *baioNewUDP(char *ip, int port, int ioDirection, int additionalSpaceToAllocate);

// mission.c
void missionProcessInteractiveInput(int c) ;
void mission();

// main.c
void mainStatistics(int action) ;
void shutdown();
void mainStandardShutdown(void *d);







#endif
