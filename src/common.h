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
#include <sys/mman.h>

#include <pthread.h>

#include "raspilotshm.h"
#include "expmem.h"
#include "sglib.h"
#include "pi2c.h"

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

// The default configuration is overwritten during launch and land time
#define PILOT_LAUNCH_SPEED			1.0
#define PILOT_LAUNCH_GOAL_ORIENTATION_TIME	0.2
#define PILOT_LAUNCH_GOAL_POSITION_TIME		0.6
#define PILOT_LAUNCH_MAX_TIME			30.0
#define PILOT_LAND_SPEED			0.2

#define PILOT_PRELAUNCH_FREQUENCY_HZ		50

#define PILOT_WARMING_WARNING_ROTATION_TIME		0.5
#define PILOT_WARMING_WARNING_ROTATIONS_DELAY		3.0
#define PILOT_WARMING_WARNING_ROTATIONS_TO_LAUNCH	5.0

// Whether D action in PID is based on error or PV
#define PID_USES_ERROR_BASED_DERIVATIVE 	0
#define PID_PREVENT_INTEGRAL_WINDUP		1

//////////////////////////////////////////////////////////////

#if 0
// some stuff used for latency profiling/checking
#define select(...) (checkTimeLimit("", 0, 0), checkTimeLimit("select", 0.001, select(__VA_ARGS__)))
#define read(...)   (checkTimeLimit("", 0, 0), checkTimeLimit("read",   0.001, read(__VA_ARGS__)))
#define write(...)  (checkTimeLimit("", 0, 0), checkTimeLimit("write",  0.001, write(__VA_ARGS__)))
#define printf(...) (checkTimeLimit("", 0, 0), checkTimeLimit("printf", 0.001, printf(__VA_ARGS__)))
#define fflush(...) (checkTimeLimit("", 0, 0), checkTimeLimit("fflush", 0.001, fflush(__VA_ARGS__)))
#endif

#define DEFAULT_DEBUG_LEVEL		5
#define DEFAULT_LOG_LEVEL		10
#define MOTOR_MAX			16
#define DEVICE_MAX			64
#define DEVICE_DATA_MAX			32
#define DEVICE_DATA_VECTOR_MAX 		16
// In order to speed up parsing of thrust sent to motorsd, we are sending integers instead of double.
// The actual thrust <0..1> will be multiplied by this factor before being sent to motors.
#define MOTOR_STREAM_THRUST_FACTOR			10000

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

// special value for motor setting/testing function
#define MOTORS_ALL			-1

#define REGRESSION_BUFFER_LAST(hh) (&(hh)->a[(hh)->ailast * (hh)->vectorsize])

/////////////////////////////////////////////////////////////////////////////////////////////

#define lprintf(level, ...) {					\
	if ((level) < debugLevel) stdbaioPrintf(level, __VA_ARGS__);	\
	if ((level) < logLevel) logbaioPrintf(level, __VA_ARGS__);	\
    }

#define STRINGIFY(x)		  		#x
#define FILE_LINE_ID_STR()	  		__FILE__ ":" STRINGIFY(__LINE__) 
#define MAGIC_CHECK(uu)				(assert((uu)->magicNumber == MAGIC_NUMBER))
#define MIN(x,y)                        	((x)<(y)?(x):(y))
#define MAX(x,y)                        	((x)>(y)?(x):(y))
#define SIGN(x)					((x)>0?1:(x)<0?-1:0)
#define DIM(x)                          	(sizeof(x) / sizeof(x[0]))
#define va_copy_end(x)                  	{}
#define DO_SIGSEGV()				{*((int*)(NULL)) = 0;}
// Hmm. it seems that abort corrupts stack on my RPi
#define ABORT()					{signal(SIGSEGV, SIG_DFL); DO_SIGSEGV();}
#define CORE_DUMP()                     	{if (fork()==0) {ABORT();}}
// #define CORE_DUMP()                     	{if (fork()==0) {signal(SIGABRT, SIG_DFL); abort();}}
#define PRINTF(msg, ...)                	{???printf("%s:%d: " msg, __FILE__, __LINE__ __VA_OPT__(,) __VA_ARGS__);}
#define PRINTF_AND_RETURN(val, msg, ...)    	{PRINTF(msg, __VA_ARGS__); return val;}
#define PRINTF_AND_EXIT(val, msg, ...)    	{PRINTF(msg, __VA_ARGS__); exit(val) ;}
#define PRINTF_AND_ABORT(msg, ...)    		{PRINTF(msg, __VA_ARGS__); abort() ;}

#if 1

#define ALLOC(p,t)          {(p) = (t*) expmemMalloc(sizeof(t)); if((p)==NULL) {printf("apsarapilot: Out of memory\n"); assert(0);shutdown();}}
#define CALLOC(p,t)         {ALLOC(p,t); memset((p), 0, sizeof(t));}
#define REALLOC(p,t)        {(p) = (t*) expmemRealloc((p), sizeof(t)); if((p)==NULL && (n)!=0) {printf("apsarapilot: Out of memory\n"); assert(0);shutdown();}}
#define ALLOCC(p,n,t)       {(p) = (t*) expmemMalloc((n)*sizeof(t)); if((p)==NULL && (n)!=0) {printf("apsarapilot: Out of memory\n"); assert(0);shutdown();}}
#define CALLOCC(p,n,t)      {ALLOCC(p,n,t); memset((p), 0, (n)*sizeof(t));}
#define REALLOCC(p,n,t)     {(p) = (t*) expmemRealloc((p), (n)*sizeof(t)); if((p)==NULL && (n)!=0) {printf("apsarapilot Out of memory\n"); assert(0);shutdown();}}
#define ALLOC_SIZE(p,t,n)   {(p) = (t*) expmemMalloc(n); if((p)==NULL && (n)!=0) {printf("apsarapilot: Out of memory\n"); assert(0); exit(1);shutdown();}}
#define FREE(p)             {expmemFree(p); }

#else

#define ALLOC(p,t)          {(p) = (t*) malloc(sizeof(t)); if((p)==NULL) {printf("apsarapilot: Out of memory\n"); assert(0);shutdown();}}
#define CALLOC(p,t)         {ALLOC(p,t); memset((p), 0, sizeof(t));}
#define REALLOC(p,t)        {(p) = (t*) realloc((p), sizeof(t)); if((p)==NULL && (n)!=0) {printf("apsarapilot: Out of memory\n"); assert(0);shutdown();}}
#define ALLOCC(p,n,t)       {(p) = (t*) malloc((n)*sizeof(t)); if((p)==NULL && (n)!=0) {printf("apsarapilot: Out of memory\n"); assert(0);shutdown();}}
#define CALLOCC(p,n,t)      {ALLOCC(p,n,t); memset((p), 0, (n)*sizeof(t));}
#define REALLOCC(p,n,t)     {(p) = (t*) realloc((p), (n)*sizeof(t)); if((p)==NULL && (n)!=0) {printf("apsarapilot Out of memory\n"); assert(0);shutdown();}}
#define ALLOC_SIZE(p,t,n)   {(p) = (t*) malloc(n); if((p)==NULL && (n)!=0) {printf("apsarapilot: Out of memory\n"); assert(0); exit(1);shutdown();}}
#define FREE(p)             {free(p); }

#endif

#define UTIME_AFTER_MINUTES(n)    (currentTime.usec + 1000000LL*60*(n))
#define UTIME_AFTER_SECONDS(n)    (currentTime.usec + 1000000LL*(n))
#define UTIME_AFTER_MSEC(n)       (currentTime.usec + 1000LL*(n))
#define UTIME_AFTER_USEC(n)       (currentTime.usec + (n))

#define TLINE_UTIME_AFTER_MINUTES(n)    (currentTimeLineTimeUsec + 1000000LL*60*(n))
#define TLINE_UTIME_AFTER_SECONDS(n)    (currentTimeLineTimeUsec + 1000000LL*(n))
#define TLINE_UTIME_AFTER_MSEC(n)       (currentTimeLineTimeUsec + 1000LL*(n))
#define TLINE_UTIME_AFTER_USEC(n)       (currentTimeLineTimeUsec + (n))

#define PPREFIX()            		(printPrefix_st(uu, __FILE__, __LINE__))
#define STR_ERRNO()               	(strerror(errno))

#define DEBUG_HERE_I_AM()         	{printf("%s: H.I.AM: %s:%d\n", PPREFIX(), __FILE__, __LINE__); fflush(stdout);}

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

enum flyStageEnum {
    FS_NONE,
    FS_START,
    FS_WAITING_FOR_SENSORS,
    FS_PRE_FLY,
    FS_FLY,
    // Following stages are not yet implemented, TODO: instead shutdown in progress test uu->stageFly >= FS_SHUTDOWN
    FS_LAND,
    FS_SHUTDOWN,
    FS_MAX,
};


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
    
    DT_VOID,

    // Sending streams (to motors)
    // Sending to motors
    DT_PING,
    DT_THRUST,
    DT_THRUST_SHM,

    // Reading streams from sensors
    // Text based streams through pipes/sockets
    DT_DEBUG,
    DT_PONG,
    DT_POSITION_VECTOR,
    DT_BOTTOM_RANGE,
    DT_FLOW_XY,
    DT_ALTITUDE,
    DT_ORIENTATION_RPY,
    // DT_ORIENTATION_QUATERNION,    
    DT_POSITION_NMEA,
    DT_MAGNETIC_HEADING_NMEA,
    // Exotic stuff
    DT_JSTEST,		// joystick


    // Shared memory streams
    DT_POSITION_SHM,
    DT_ORIENTATION_RPY_SHM,
    
    DT_MAX,
};

// Maybe this is useless, you can implement all of them as DCT_COMMAND_BASH
enum deviceConnectionTypeEnum {
    DCT_NONE,
    DCT_INTERNAL_ZEROPOSE,
    DCT_COMMAND_BASH,
    DCT_COMMAND_EXEC,
    // Named pipes looked nice. Unfortunately, on raspberry pi they are unstable. They have jitters making them unusable for low latency devices.
    DCT_NAMED_PIPES,     
    /* in the future we can do something like:
    DCT_TCPIP_CLIENT_SOCKET,
    DCT_TCPIP_SERVER_SOCKET,
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
    double          dtime;      // time since epoch in seconds with microsecond precission
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
    double	derivativeMax;

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


// Regression buffer is a ring buffer storing vectors of size vectorsize in each cell.
// Usualy, it also maintains sums and sums of squares of stored elements. Those sums are used
// for fast extrapolation (for given time) of values by linear regression.
struct regressionBuffer {
    char		name[256];	// for debug output only, to be removed or replaced by char *name;
    int			size;
    int			vectorsize;
    int			ai;		// index where next elem will be stored
    int			ailast;		// index where the last elem was added, i.e.  (ai-1) % size
    int			aiprev;		// index where the previous elem to the last was added, i.e. (ai-2)%size
    int			n;		// number of total inserted elements, not only currently stored (ai == n%size)

    // the actual data stored in the buffer
    double		*time;		// time[size]          // time when the vector was added
    double		*a;		// a[size][vectorsize] // the actual numbers stored in the buffer

    int			keepSumsFlag;	// if zero, sums are not maintained (used for buffers not using regression too much)
    double		timeOffsetForSums; // time is decresed by this offset in sum and sumSquare to avoid overflow
    // sums used for least square method (from times and elements currently hold in the 'a' array)
    double		sumTime;
    double		sumTimeSquare;
    double		*suma;		// suma[vectorsize];
    double		*sumaMulTime;	// sumaMulTime[vectorsize];

    // sums used for flight statistics
    // sums from all elements inserted since the launch time, not only currently stored
    double		*totalSumForStatistics;		// totalSumForStatistics[vectorsize];
    int			totalElemsForStatistics;    
};

//////////////////////////////////////////////////////////////////////////////////////////////
// configuration

struct deviceStreamData {
    // values from config
    char			*name;
    char			*tag;
    int				type;			// enum deviceDataTypes
    double			latency;		// data received refers to the time lastDataTime - latency
    double			timeout;		// if last value is more then timeout old, sensor is ignored
    double			min_range;		// minimal valid range for rangefinders (radar)
    double			max_range;		// minimal valid range for rangefinders (radar)
    double			min_altitude;		// min valid altitude (for flow detectors)
    double			max_altitude;		// max valid altitude (for flow detectors)
    vec4			weight;			// weight is per axis in GBASE, or rpy per quat elem for quat
    double			drift_fix_period;	// Not yet implemented, period to recalibrate drift of drifting devices
    uint8_t			mandatory;		// whether device has to be active before launch
    int				history_size;   	// the size of the history saved for linear interpolation, rename to regression_points or similar
    int				debug_level;
    struct deviceData		*dd;			// "back" pointer to device data where I belong

    // This is the place where the 'raw' values read from the device are stored.
    // Old way.
    // raw data coming from the sensor are parsed, "timestamped" and put into this buffer
    //struct raspilotRingBuffer		inputBuffer;
    //  New way allowing shared memory access.    
    // Data coming from the sensor through a text pipe are parsed, "timestamped" and put into this buffer.
    // Shared memory devices send us a pointer to this structure on init and write directly into this shared.
    struct raspilotInputBuffer	*input;
    
    // data from inputBuffer are transformed to what pilot needs (usualy orientation/position)
    // when pilot needs
    struct regressionBuffer	outputBuffer;
    int				inputToOutputN;	// the indice of the first item in inputBuffer not moved to outputBuffer yet

    
    // TODO move from scalar confidence to vector, even better per record!
    double			confidence;	

    // something for final statistics
    double			pongTotalTimeForStatistics;
    int				totalNumberOfRecordsReceivedForStatistics;
    
    // This is the value the device reported before drone launch
    // It may or may not (depending on device type) be used to correct device data
    double   			launchData[DEVICE_DATA_VECTOR_MAX];
    uint8_t			launchPoseSetFlag;	// whether we have yet stored the launch pose

    // devicedata are linked also by the type of data for faster fusion of sensors
    struct deviceStreamData 	*nextWithSameType;

};

struct connection {
    int type; // enum deviceConnectionTypeEnum
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

    // for now, we consider that all devices are providing data in form of a text streams,
    // line after line, each line starting by the tag determining which data are there.
    struct connection		connection;
    struct deviceStreamData	*ddt[DEVICE_DATA_MAX];
    int				ddtMax;
    vec3			mount_position;		// w.r.t. the center of gravity
    vec3			mount_rpy;		// roll pitch yaw
    double			warming_time;
    
    // Flag whether to send 'exit' command to motors at raspilot shutdown.
    // the ide was that PWM process will stay running sending idle PWM
    // while dshot motor process can exit.
    uint8_t			shutdownExit;
    // do not print warning if unknow data line read from data stream
    uint8_t			data_ignore_unknown_tags;
    
    uint8_t			enabled;
    int 			baioMagic;
    double 			lastActivityTime;
};

/////////////////////////////////////////

struct motorStr {
    double			thrust;

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
    double			drone_panic_inclination;
    double			drone_max_speed;
    double			drone_max_rotation_speed;
    double			drone_min_altitude;
    double			drone_max_altitude;
    double			drone_waypoint_reached_range;
    double			drone_waypoint_reached_angle;
    double			short_buffer_seconds;
    double			long_buffer_seconds;
};

struct universe {
    // configuration
    char			*cfgFileName;
    
    // TODO: put all this to "struct config"
    // if non NULL, ping to the controllingHostIp to check if we are connected to it
    // if not, initiate landing or return to home
    char			*pingToHost;
    char			*logFileName;
    
    double			autopilot_loop_Hz;
    double			stabilization_loop_Hz;
    int 			motor_number;
    double			motor_roll_forces[MOTOR_MAX];
    double			motor_pitch_forces[MOTOR_MAX];
    double			motor_yaw_forces[MOTOR_MAX];

    struct config		config;
    
    struct pidController	pidRoll;
    struct pidController	pidPitch;
    struct pidController	pidYaw;
    struct pidController	pidX;
    struct pidController	pidY;
    struct pidController	pidAltitude;

    struct waypoint 		currentWaypoint;
    
    
    struct deviceData		*device[DEVICE_MAX];
    int				deviceMax;
    int				deviceMotors;

    // Following optimizes sensor fusion
    struct deviceStreamData	*deviceStreamDataByType[DT_MAX];	// lists by data types
    
    // run time values
    int				flyStage; 		// enum flyStageEnum
    double			pilotStartingTime;	// the time when pilot was launched
    
    struct motorStr		motor[MOTOR_MAX];

    int				motorBaioMagic;
    double			pilotLaunchTime;		// time when pilot was started
    double			previousTotalFlyTime;		// total fly time since last reset (battery change)
    double			flyStartTime;			// start flying time
    double			motorFirstSendTime;		// when we lastly send a command to motors
    double			motorLastSendTime;		// when we lastly send a command to motors

    // Long buffer is used to smooth position and velo from sensor fusion
    struct regressionBuffer	longBufferPosition;
    struct regressionBuffer	longBufferRpy;

    // Short buffer is used to smooth roll and pitch
    struct regressionBuffer	shortBufferPosition;
    struct regressionBuffer	shortBufferRpy;

    // Those values may be used as the current position, orientation and velocity of the drone
    vec3			droneLastPosition;
    vec3			droneLastRpy;
    vec3			droneLastVelocity;
    vec3			droneLastRpyRotationSpeed;
    // The time when previous values were updated
    double			droneLastTickTime;

    // Target roll, pitch yaw [speed] for PID controller
    double 			targetRoll, targetPitch, targetYaw;
    double			targetYawRotationSpeed, targetRollRotationSpeed, targetPitchRotationSpeed;

    // Thrust to be sent to motors for altitude goal.
    double			altitudeThrust;
    
    // hold a few seconds of historical poses for case somebody needs it
    // TODO: split into historyPosition and historyRpy, so that position sensors
    // can find the new orientation there
    struct raspilotRingBuffer	*historyPose;
    
    // end of universe
    unsigned			magicNumber;
};

typedef int deviceDataParseFunction(char *tag, char *p, struct deviceData *dd, struct deviceStreamData *ddd);

//////////////////////////////////////////////////////////////////////////////////////////////
//
// common.c
extern int 			debugLevel;
extern int 			logLevel;
extern int 			baseLogLevel;
extern int			log10TicksPerSecond;
extern struct universe		*uu;
extern struct globalTimeInfo	currentTime;
extern struct timeLineEvent     *timeLine;
extern uint64_t			tickCounter;
extern uint64_t			currentTimeLineTimeUsec;
extern int64_t 			nextStabilizationTickUsec;
extern int64_t 			nextPidTickUsec;
extern int			shutDownInProgress;
extern struct jsonnode 		dummyJsonNode;
extern char 			*signalInterruptNames[258];
extern int 			deviceDataStreamVectorLength[DT_MAX];
extern char 			*deviceDataTypeNames[DT_MAX+2];
extern char 			*coordinateSystemNames[CS_MAX+2];
extern char			*deviceConnectionTypeNames[DCT_MAX+2];

char *getTemporaryStringPtrFromStaticStringRing();
int strtoint(char *s, char **ee) ;
void strtodninit() ;
double strtodn(char *p, char **ee) ;
int hexDigitCharToInt(int hx) ;
int intDigitToHexChar(int x) ;
char *strDuplicate(char *s) ;
char *strnDuplicate(char *s, int max) ;
char *strSafeDuplicate(char *s) ;
int strSafeLen(char *s) ;
int strSafeNCmp(char *s1, char *s2, int n) ;
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
double truncateToRange(double x, double min, double max, char *warningTag) ;
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
void enumNamesInit() ;
void enumNamesPrint(FILE *ff, char **names) ;
int enumNamesStringToInt(char *s, char **names) ;
void terminalResume() ;
int stdbaioStdinMaybeGetPendingChar() ;
void stdbaioStdinClearBuffer();

double *raspilotRingBufferGetFirstFreeVector(struct raspilotRingBuffer *hh) ;
void raspilotRingBufferFindRecordForTime(struct raspilotRingBuffer *hh, double time, double *restime, double **res) ;

void regressionBufferPrintSums(struct regressionBuffer *hh) ;
void regressionBufferAddToSums(struct regressionBuffer *hh, double time, double *vec) ;
void regressionBufferSubstractFromSums(struct regressionBuffer *hh, double time, double *vec) ;
void regressionBufferRecalculateSums(struct regressionBuffer *hh) ;
void regressionBufferAddElem(struct regressionBuffer *hh, double time, double *vec) ;
void regressionBufferInit(struct regressionBuffer *hh, int vectorSize, int bufferSize, char *namefmt, ...) ;
void regressionBufferFindRecordForTime(struct regressionBuffer *hh, double time, double *restime, double *res) ;
int regressionBufferGetRegressionCoefficients(struct regressionBuffer *hh, int i, double *k0, double *k1) ;
void regressionBufferGetMean(struct regressionBuffer *hh, double *time, double *res) ;
int regressionBufferEstimateForTime(struct regressionBuffer *hh, double time, double *res) ;

int vec1TruncateToSize(double *r, double size, int warningFlag, char *warningId) ;
int vec2TruncateToSize(vec2 r, double size, int warningFlag, char *warningId) ;
int vec3TruncateToSize(vec3 r, double size, int warningFlag, char *warningId) ;
void pidControllerReset(struct pidController *pp) ;
char *pidControllerStatistics(struct pidController *pp, int showProposedCiFlag) ;
double pidControllerStep(struct pidController *pp, double setpoint, double measured_value, double dt) ;
void quatToRpy(quat qq, double *roll, double *pitch, double *yaw);
void rpyToQuat(double roll, double pitch, double yaw, quat q) ;
void stdbaioInit() ;
void stdbaioClose() ;
void logbaioInit() ;
void logbaioClose() ;
int stdbaioPrintf(int level, char *fmt, ...) ;
int logbaioPrintf(int level, char *fmt, ...) ;
void trajectoryLogInit() ;
int trajectoryLogPrintf(char *fmt, ...) ;
void trajectoryLogClose() ;
void pingToHostInit() ;
void pingToHostRegularCheck(void *d) ;
void pingToHostClose() ;

struct raspilotInputBuffer *raspilotCreateSharedMemory(struct deviceStreamData *ddd) ;

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

// device.c
int deviceIsSharedMemoryDataStream(struct deviceStreamData *ddl) ;
void deviceParseInputStreamLineToInputBuffer(struct deviceData *dd, char *s, int n) ;
void deviceTranslateInputToOutput(struct deviceStreamData *ddd) ;
void deviceInitiate(int i) ;
void deviceFinalize(int i) ;
void deviceSendToAllDevices(char *fmt, ...) ;

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
void pilotSendThrusts(void *d) ;
void motorsExit(void *d) ;
void motorsStandby(void *d) ;
void motorsSendStreamThrustFactor(void *d) ;
void motorsThrustSet(double thrust) ;
void motorThrustSetAndSend(int i, double thrust) ;
void motorsThrustSetAndSend(double thrust) ;
void motorsStop(void *d) ;
void motorsEmmergencyLand() ;
void motorsEmmergencyShutdown() ;
void pilotImmediateLanding() ;
void pilotInteractiveInputRegularCheck(void *d) ;
void pilotRegularSpecialModeTick(void *d) ;
void pilotRegularStabilizationTick(void *d) ;
void pilotAutopilotLoopTick(void *d) ;
int pilotAreAllDevicesReady() ;
void pilotStoreLaunchPose(void *d) ;
void pilotRegularStabilizationLoopRescheduleToSoon() ;
void pilotRegularSendPings(void *d) ;
void pilotRegularSaveTrajectory(void *d) ;
void pilotRegularWaitForDevicesAndStartPilot(void *d) ;
void pilotUpdateBufferAndRecomputeMotorThrust() ;

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
void missionLandImmediately() ;
void mission();

// main.c
void mainInitDeviceDataStreamVectorLengths(int motor_number) ;
void mainStatistics(int action) ;
void shutdown();
void mainStandardShutdown(void *d);







#endif
