#include "common.h"

int 			debugLevel;
int 			logLevel;
int 			baseLogLevel;
struct universe		uuu;
struct universe		*uu = &uuu;
struct timeLineEvent    *timeLine = NULL;
uint64_t		currentTimeLineTimeUsec;
struct globalTimeInfo   currentTime;
int			shutDownInProgress = 0;
struct jsonnode 	dummyJsonNode;
int64_t 		nextStabilizationTickUsec;
int64_t 		nextPidTickUsec;
int			stdbaioBaioMagic = 0;
int			logbaioBaioMagic = 0;
int			trajectoryLogBaioMagic = 0;
int			pingToHostBaioMagic = 0;
double			pingToHostLastAnswerTime = 0;

// enumeration names
char 			*signalInterruptNames[258];
int 			deviceDataStreamVectorLength[DT_MAX];
char 			*deviceDataTypeNames[DT_MAX+2];
char			*deviceConnectionTypeNames[DCT_MAX+2];
char			*radioControlNames[RC_MAX+2];
char			*pilotMainModeNames[MODE_MAX+2];
char			*remoteControlModeNames[RCM_MAX+2];


////////////////////////////////////////////////////////////////////////////////////////////////

static char baioStaticStringsRing[STATIC_STRINGS_RING_SIZE][TMP_STRING_SIZE];
static int  baioStaticStringsRingIndex = 0;

char *getTemporaryStringPtrFromStaticStringRing() {
    char *res;
    
    res = baioStaticStringsRing[baioStaticStringsRingIndex];
    baioStaticStringsRingIndex  = (baioStaticStringsRingIndex+1) % STATIC_STRINGS_RING_SIZE;
    // make sure that snprint-ed string will be zero terminating
    res[TMP_STRING_SIZE-1] = 0;
    return(res);
}

/////////////////////////////////////////////////////////////////////////////////////////////////
// "fast" parsing

#define STRTODN_TABULATED_POWERS 32

static double strtodnExpPositive[STRTODN_TABULATED_POWERS];
static double strtodnExpNegative[STRTODN_TABULATED_POWERS];

int strtoint(char *s, char **ee) {
    int sign, r, res;
    
    while(isspace(*s)) s++;
    
    sign = 1;
    if (*s == '-') {
        sign = -1; s++;
    } else if (*s == '+') {
        sign = 1; s++;
    }

    res = 0;
    while (*s >= '0' && *s <= '9') {
	res = res * 10 + *s - '0';
	s++;
    }
    *ee = s;
    if (sign < 0) res = - res;
    return(res);
}

void strtodninit() {
    int i;
    for(i=0; i<STRTODN_TABULATED_POWERS; i++) {
	strtodnExpPositive[i] = pow(10, i);
	strtodnExpNegative[i] = pow(10, -i);
    }
}

// This is a naive parsing of double numbers. It is faster than strtod for a limited range of exponents and does not conform to any standard.
double strtodn(char *p, char **ee) {
    double     	res;
    int        	sign, e, i;
    char	*pp, *pd;
    
    // check if exponent table was initialized
    assert(strtodnExpPositive[0] != 0);
    
    while (isspace(*p)) p++;
    
    sign = 1;
    if (*p == '-') {
        sign = -1; p++;
    } else if (*p == '+') {
        sign = 1; p++;
    }

    res = 0;
    e = 0;

    // Double has 15 valid digits plus dot
    pp = p + 16;
    
    while (*p >= '0' && *p <= '9') {
        res = res * 10 + (*p - '0');
	p++;
    }

    if (*p == '.') {
	p++;
	pd = p;
	while (*p >= '0' && *p <= '9') {
	    if (p < pp) {
		res = res * 10 + (*p - '0');
	    }
	    p++;
	}
	if (p <= pp) e = pd - p;
	else if (pd <= pp) e = pd - pp;
    }

    if (sign < 0) res = - res;

    *ee = p;
    if (*p == 'e' || *p == 'E') {
	e += strtoint(p+1, ee);
    }
    
    if (e >= 0 && e < STRTODN_TABULATED_POWERS) {
	res = strtodnExpPositive[e] * res;
    } else if (-e >= 0 && -e < STRTODN_TABULATED_POWERS) {
	res = strtodnExpNegative[-e] * res;
    } else {		
	res = pow(10, e) * res;
    }

    return(res);
}

/////////////////////////////////////////////////////////////////////////////////////

int hexDigitCharToInt(int hx) {
    int res;
    if (hx >= '0' && hx <= '9') return(hx - '0');
    if (hx >= 'A' && hx <= 'F') return(hx - 'A' + 10);
    if (hx >= 'a' && hx <= 'f') return(hx - 'a' + 10);
    return(-1);
}

int intDigitToHexChar(int x) {
    if (x >= 0 && x <= 9) return(x + '0');
    if (x >= 10 && x <= 15) return(x + 'A' - 10);
    return(-1);
}

char *strDuplicate(char *s) {
    int     n;
    char    *ss, *res;
    if (s == NULL) return(NULL);
    n = strlen(s);
    ALLOCC(res, n+1, char);
    strcpy(res, s);
    return(res);
}

char *strnDuplicate(char *s, int max) {
    int     n;
    char    *ss, *res;
    
    if (s == NULL) return(NULL);
    n = strlen(s);
    if (n > max) n = max;
    ALLOCC(res, n+1, char);
    strncpy(res, s, n);
    res[n] = 0;
    return(res);
}

char *strSafeDuplicate(char *s) {
    if (s == NULL) return(strDuplicate(""));
    return(strDuplicate(s));
}

int strSafeLen(char *s) {
    if (s == NULL) return(0);
    return(strlen(s));
}

int strSafeNCmp(char *s1, char *s2, int n) {
    if (s1 == NULL && s2 == NULL) return(0);
    if (s1 == NULL) return(-1);
    if (s2 == NULL) return(1);
    return(strncmp(s1, s2, n));
}

int strSafeCmp(char *s1, char *s2) {
    if (s1 == NULL || s2 == NULL) return(-1);
    return(strcmp(s1, s2));
}

int isspaceString(char *s) {
    char	*p;
    if (s == NULL) return(1);
    for(p=s; *p; p++) {
	if (! isspace(*p)) return(0);
    }
    return(1);
}

void writeToFd(int fd, char *buf, int bufsize) {
    int 	r, n;
	
    for(n=0; n<bufsize; ) {
	errno = 0;
	r = write(fd, buf+n, bufsize-n);
	// printf("Written %d bytes to %d\n", r, fd);
	if (r < 0 || (r == 0 && errno != 0)) break;
	n += r;
    }
}

char *printPrefix_st(struct universe *uu, char *file, int line) {
    int		i, r;
    char	*res;

    // Hmm. BTW this is a costly function if there is a lot of debug output. Maybe optimized a bit.
    res = getTemporaryStringPtrFromStaticStringRing();
    r = snprintf(res, TMP_STRING_SIZE-1, "%s: %s:%d", currentLocalTime_st(), file, line);
    for(i=r; i>=0 && i<40; i++) res[i] = ' ';
    res[i] = 0;
    return(res);
}

void dumpHex(char *msg, char *d, int len) {
    int i;
    printf("%s: ", msg);
    for(i=0; i<len; i++) printf("%02x ", d[i]);
    printf("\n");
}    

char *fileLoadToNewlyAllocatedString(char *path, int useCppFlag) {
    FILE 	*ff;
    char	*b;
    int		blen;
    int		i, n;
    char	*ccc;
	
    ccc = getTemporaryStringPtrFromStaticStringRing();
    if (useCppFlag) {
	snprintf(ccc, TMP_STRING_SIZE-1, "cpp %s", path);	
	ff = popen(ccc, "r");
    } else {
	ff = fopen(path, "r");
    }
    if (ff == NULL) {
	printf("%s:%s:%d: Can't open %s\n", PPREFIX(), __FILE__, __LINE__, path);
	return(NULL);
    }
    i = 0;
    blen = 1024;
    ALLOCC(b, blen, char);
    n = 1;
    while (n > 0) {
	n = fread(b+i, 1, blen-i, ff);
	// lprintf(0, "read %d bytes: %*.*s\n", n, n, n, b+i);fflush(stdout);
	i += n;
	if (i >= blen-1) {
	    blen *= 2;
	    REALLOCC(b, blen, char);
	}
    }
    b[i] = 0;
    REALLOCC(b, i+1, char);	
    fclose(ff);
    return(b);
}

double signd(double x) {
    if (x > 0) return(1.0);
    if (x < 0) return(-1.0);
    return(0);
}

double normalizeToRangeOld(double value, double min, double max) {
    double res;
    double r;
    assert(min < max);
    r = max - min;

    res = value;
    while (res < min) res += r;
    while (res >= max) res -= r;
    return(res);
}

double normalizeToRange(double value, double min, double max) {
    double 	res;
    double 	r;
    
    r = max - min;

    res = fmod(value, r);
    res += floor(min / r) * r;
    // loop should not be executed more then once
    while (res < min) res += r;
    while (res >= max) res -= r;

#if 0 && DEBUG
    if (fabs(res - normalizeToRangeOld(value, min, max)) > 0.001) {
	lprintf(0, "%s: Internal warning: something wrong in the new implementation of normalizeToRange %g <-> %g\n", PPREFIX(), res, normalizeToRangeOld(value, min, max));
    }
#endif
    return(res);
}

double normalizeAngle(double omega, double min, double max) {
    double res;
    if (omega == ANGLE_NAN) {
	lprintf(0, "%s: Internal error: Something is wrong, trying to normalize ANGLE_NAN. \n", PPREFIX());
    }
    res = normalizeToRange(omega, min, max);
    return(res);
}

double truncateToRange(double x, double min, double max, char *warningTag, int warningIndex) {
    if (x < min) {
	if (warningTag != NULL) {
	    lprintf(10, "%s: Warning: %s has been truncated from %f to %f.", PPREFIX(), warningTag, x, min);
	    if (warningIndex != INDEX_NAN) lprintf(10, " Index: %d", warningIndex);
	    lprintf(10, "\n");
	}
	return(min);
    }
    if (x > max) {
	if (warningTag != NULL) {
	    lprintf(10, "%s: Warning: %s has been truncated from %f to %f.", PPREFIX(), warningTag, x, max);
	    if (warningIndex != INDEX_NAN) lprintf(10, "Index: %d", warningIndex);
	    lprintf(10, "\n");
	}
	return(max);
    }
    return(x);
}

void vecTruncateInnerToRange(vec3 r, int dim, double min, double max, char *warningId) {
    int 	i;

    for(i=0; i<dim; i++) r[i] = truncateToRange(r[i], min, max, warningId, i);
}

double angleSubstract(double a1, double a2) {
    return(normalizeAngle(a1 - a2, -M_PI, M_PI));
}

void vec2Rotate(double *res, double *v, double theta) {
    double sint, cost, xx, yy;

    // printf("rotating %f %f by %f degree counter clockwise \n", v[0], v[1], theta*180/M_PI);
    sint = sin(theta);
    cost = cos(theta);
    // get values to local variables for case res == v
    xx = v[0];
    yy = v[1];

    if (0) {
	// clockwise rotation
	res[0] = xx * cost + yy * sint;
	res[1] = yy * cost - xx * sint;
    } else {
	// counter clockwise rotation
	res[0] = xx * cost - yy * sint;
	res[1] = yy * cost + xx * sint;
    }	
    // printf("rotated to %f %f\n", v[0], v[1]);
}


//////////////////////////////////////////////////////////////////////////////////

char *currentLocalTime_st() {
    char            *res;
    time_t          t;
    int             u;
    struct tm       *tm, ttm;

    res = getTemporaryStringPtrFromStaticStringRing();

    snprintf(res, TMP_STRING_SIZE-1, "%4d-%02d-%02d %02d:%02d:%02d.%03d", 
             1900+currentTime.lcltm.tm_year, currentTime.lcltm.tm_mon+1, currentTime.lcltm.tm_mday, 
             currentTime.lcltm.tm_hour, currentTime.lcltm.tm_min, currentTime.lcltm.tm_sec,
             currentTime.msecPart);
    return(res);
}

char *sprintSecTime_st(long long int utime) {
    static char     *res;
    time_t          t;
    int             u;
    struct tm       *tm, ttm;

    res = getTemporaryStringPtrFromStaticStringRing();
    t = utime / 1000000;
#if _WIN32
    ttm = *localtime(&t);
    tm = &ttm;
#else
    tm = localtime_r(&t, &ttm);
#endif
    snprintf(res, TMP_STRING_SIZE-1, "%4d-%02d-%02d %02d:%02d:%02d", 
	     1900+tm->tm_year, tm->tm_mon+1, tm->tm_mday, tm->tm_hour, tm->tm_min, tm->tm_sec
	);
    return(res);
}

char *sprintUsecTime_st(long long int utime) {
    char     		*res;
    time_t          	t;
    int             	u;
    struct tm       	*tm, ttm;

    res = getTemporaryStringPtrFromStaticStringRing();
    t = utime / 1000000;
    u = utime % 1000000;
#if _WIN32
    ttm = *localtime(&t);
    tm = &ttm;
#else
    tm =  localtime_r(&t, &ttm);
#endif
    snprintf(res, TMP_STRING_SIZE-1, "%4d-%02d-%02d %02d:%02d:%02d.%03d %03d", 
	     1900+tm->tm_year, tm->tm_mon+1, tm->tm_mday, tm->tm_hour, tm->tm_min, tm->tm_sec,
	     u/1000, u%1000);
    return(res);
}

void setCurrentTimeToTimeVal(struct timeval *tv) {
    int             previousTimeHour, m, s;

    if (currentTime.sec < tv->tv_sec || (currentTime.sec == tv->tv_sec && currentTime.usecPart < tv->tv_usec)) {

        previousTimeHour = currentTime.hour;

        // update current time
        currentTime.sec = tv->tv_sec;
        currentTime.hour = tv->tv_sec / (60*60);
        currentTime.usecPart = tv->tv_usec;
        currentTime.msecPart = tv->tv_usec / 1000;
        currentTime.usec = ((long long int)tv->tv_sec) * 1000000LL + tv->tv_usec;
        currentTime.dtime = tv->tv_sec + tv->tv_usec / 1000000.0;
        // currentTime.dtime = (tv->tv_sec - 1640995200) + tv->tv_usec / 1000000.0;	// like this since 1.1.2022
	currentTime.msec = currentTime.usec / 1000;
	//currentTime.msec = currentTime.dtime * 1000.0;
	
	// update tm structures
	// TODO: check if we are the same halfhour, some timezones are half an hour shifted
        if (currentTime.hour == previousTimeHour) {
            // we are the same hour as previously, no need to call localtime, update only minutes and seconds in tm structures
            s = currentTime.sec % 60;
            m = (currentTime.sec / 60) % 60;
            currentTime.gmttm.tm_sec = currentTime.lcltm.tm_sec = s;
            currentTime.gmttm.tm_min = currentTime.lcltm.tm_min = m;
        } else {
#if _WIN32
	    currentTime.gmttm = *gmtime(&currentTime.sec);
            currentTime.lcltm = *localtime(&currentTime.sec);
#else
            gmtime_r(&currentTime.sec, &currentTime.gmttm);
            localtime_r(&currentTime.sec, &currentTime.lcltm);
#endif
        }
    }
}

void setCurrentTime() {
    struct timeval  tv;

    gettimeofday(&tv, NULL);
    setCurrentTimeToTimeVal(&tv);
}

void incrementCurrentTime() {
    struct timeval  tv;	

    tv.tv_sec = currentTime.sec;
    tv.tv_usec = currentTime.usecPart;
    tv.tv_usec ++;
    if (tv.tv_usec >= 1000000) {
	tv.tv_sec += tv.tv_usec / 1000000;
	tv.tv_usec = tv.tv_usec % 1000000;		
    }
    setCurrentTimeToTimeVal(&tv);
}

int checkTimeLimit(char *op, double maxTime, int res) {
    static char	 	b[TMP_STRING_SIZE];
    static double 	lastTime;
    double		diff;
    
    setCurrentTime();
    if (maxTime == 0) {
	lastTime = currentTime.dtime;
    } else {
	diff = currentTime.dtime - lastTime;
	if (diff > maxTime) {
	    snprintf(b, TMP_STRING_SIZE-1, "%s: Warning: Operation %s took more than limit %g ms. Time %g ms.\n", PPREFIX(), op, maxTime*1000, diff*1000);
	    fwrite(b, strlen(b), 1, stdout);
	}
    }
    return(res);
}

//////////////////////////////////////////////////////////////////////////////////////////////////
// timeline functions
// timeline is a list of functions to be executed in the future at given time.
// The list is ordered by execution time

static int timeLineEventComparator(struct timeLineEvent *e1, struct timeLineEvent *e2) {
    if (e1->usecond < e2->usecond) return(-1);
    if (e1->usecond > e2->usecond) return(1);
    if (((char *)e1->event) < ((char *)e2->event)) return(-1);
    if (((char *)e1->event) > ((char *)e2->event)) return(1);
    if (((char *)e1->arg) < ((char *)e2->arg)) return(-1);
    if (((char *)e1->arg) > ((char *)e2->arg)) return(1);
    return(0);
}

static int timeFindEventNoMatterTimeComparator(struct timeLineEvent *e1, struct timeLineEvent *e2) {
    if (((char *)e1->event) != ((char *)e2->event)) return(-1);
    if (((char *)e1->arg) != ((char *)e2->arg)) return(-1);
    return(0);
}

static int timeFindEventNoMatterTimeAndArgComparator(struct timeLineEvent *e1, struct timeLineEvent *e2) {
    if (((char *)e1->event) != ((char *)e2->event)) return(-1);
    return(0);
}

// TODO: Make timeline a double linked list, insert will return a pointer and delete will delete in constant time
void timeLineInsertEvent(long long int usec, void (*event)(void *arg), void *arg) {
    struct timeLineEvent *tt;

    // You shall never insert an event at current time (with UTIME_AFTER_SECONDS(0) or UTIME_AFTER_MILLIS(0)) !!!
    // Because if this is entered from a timeline event, it is executed immediately which is probably not what you
    // want, if it is what you want you can call it directly.
    if (usec <= currentTime.usec) usec = currentTime.usec + 1;

    ALLOC(tt, struct timeLineEvent);
    tt->usecond = usec;
    tt->event = event;
    tt->arg = arg;
    tt->next = NULL;
    SGLIB_SORTED_LIST_ADD(struct timeLineEvent, timeLine, tt, timeLineEventComparator, next);
    // lprintf(0, "timeLineInsertEvent for %s\n", sprintUsecTime_st(usec));
}

struct timeLineEvent *timeLineFindEventAtUnknownTime(void (*event)(void *arg), void *arg) {
    struct timeLineEvent ttt, *memb;
    ttt.event = event;
    ttt.arg = arg;
    SGLIB_LIST_FIND_MEMBER(struct timeLineEvent, timeLine, &ttt, timeFindEventNoMatterTimeComparator, next, memb);
    return(memb);
}

void timeLineRemoveEvent(long long int usec, void (*event)(void *arg), void *arg) {
    struct timeLineEvent ttt, *memb;
    ttt.usecond = usec;
    ttt.event = event;
    ttt.arg = arg;
    SGLIB_SORTED_LIST_DELETE_IF_MEMBER(struct timeLineEvent, timeLine, &ttt, timeLineEventComparator, next, memb);
    if (memb != NULL) {
        FREE(memb);
    }
}

int timeLineRemoveEventAtUnknownTime(void (*event)(void *arg), void *arg) {
    struct timeLineEvent ttt, *memb;
    ttt.event = event;
    ttt.arg = arg;
    SGLIB_LIST_DELETE_IF_MEMBER(struct timeLineEvent, timeLine, &ttt, timeFindEventNoMatterTimeComparator, next, memb);
    if (memb != NULL) {
        FREE(memb);
	return(0);
    }
    return(-1);
}

int timeLineRemoveEventAtUnknownTimeAndArg(void (*event)(void *arg)) {
    struct timeLineEvent ttt, *memb;
    ttt.event = event;
    SGLIB_LIST_DELETE_IF_MEMBER(struct timeLineEvent, timeLine, &ttt, timeFindEventNoMatterTimeAndArgComparator, next, memb);
    if (memb != NULL) {
        FREE(memb);
	return(0);
    }
    return(-1);
}

int timeLineRemoveAllEvents() {
    struct timeLineEvent *tt;
	
    while (timeLine != NULL) {
	tt = timeLine->next;
	FREE(timeLine);
	timeLine = tt;
    }
    return(0);
}


int timeLineRescheduleUniqueEvent(long long int usec, void (*event)(void *arg), void *arg) {
    int res;
    res = timeLineRemoveEventAtUnknownTime(event, arg);
    timeLineInsertEvent(usec, event, arg);
    return(res);
}

int timeLineRescheduleUniqueEventIfExisted(long long int usec, void (*event)(void *arg), void *arg) {
    int res;
    res = timeLineRemoveEventAtUnknownTime(event, arg);
    if (res == 0) timeLineInsertEvent(usec, event, arg);
    return(res);
}

int timeLineInsertUniqEventIfNotYetInserted(long long int usec, void (*event)(void *arg), void *arg) {
    struct timeLineEvent *memb;
    memb = timeLineFindEventAtUnknownTime(event, arg);
    if (memb == NULL) {
	timeLineInsertEvent(usec, event, arg);
	return(0);
    } else {
	return(1);
    }
}

void timeLineTimeToNextEvent(struct timeval *tv, int maxseconds) {
    long long int diff;

    if (timeLine == NULL) goto timeLineReturnMax;

    diff = timeLine->usecond - currentTime.usec;
    if (diff <= 0) goto timeLineReturnZero;
    tv->tv_sec =  diff / 1000000LL;
    tv->tv_usec = diff % 1000000LL;
    if (tv->tv_sec >= maxseconds) goto timeLineReturnMax;
    goto timeLineExitPoint;

timeLineReturnMax:
    tv->tv_sec = maxseconds;
    tv->tv_usec = 0;
    goto timeLineExitPoint;

timeLineReturnZero:
    tv->tv_sec = 0;
    tv->tv_usec = 0;
    goto timeLineExitPoint;

timeLineExitPoint:;
    // lprintf(0, "time to next event %lld - %lld == %d %d\n", (long long)currentTime.usec, (long long)timeLine->usecond, tv->tv_sec, tv->tv_usec);
    return;
}

int timeLineExecuteScheduledEvents(int updateCurrenttimeFlag) {
    int 			res;
    struct timeLineEvent 	*tt;
    long long int		untilUsec;

    res = 0;
    // memorize stop time, because otherwise if updateCurrenttimeFlag is true,
    // then actions can add immediate actions which are executed in this loop
    // and can potentially loop forever
    untilUsec = currentTime.usec;
    while (timeLine != NULL && timeLine->usecond <= untilUsec) {
	currentTimeLineTimeUsec = timeLine->usecond;
	if (updateCurrenttimeFlag) setCurrentTime();
	tt = timeLine;
	timeLine = tt->next;
        if (tt->event != NULL) {
            tt->event(tt->arg);
	    res ++;
        }		
        FREE(tt);
    }
    return(res);
}

void timeLineDump() {
    struct timeLineEvent *ll;
    lprintf(0, "TIMELINE DUMP: ");
    lprintf(0, "Currenttime %s, ", sprintUsecTime_st(currentTime.usec));
    lprintf(0, "currentTimelineTime %s \n", sprintUsecTime_st(currentTimeLineTimeUsec)); 
    for(ll=timeLine; ll!=NULL; ll=ll->next) {
        lprintf(0, "%p:%s:%p(%p) ", ll, sprintUsecTime_st(ll->usecond), ll->event, ll->arg);fflush(stdout);
    }
    lprintf(0, "\nEND.\n"); fflush(stdout);
}

//////////////////////////////////////////////////////////////////////////////////////////////////

char *arrayWithDimToStr_st(double *a, int dim) {
    char	*res, *separator;
    int		i, j;
    
    res = getTemporaryStringPtrFromStaticStringRing();
    i = 0;
    if (i>=TMP_STRING_SIZE-1) return(FILE_LINE_ID_STR() ": Error");
    i += snprintf(res+i, TMP_STRING_SIZE-i-1, "[");
    separator = "";
    for(j=0; j<dim; j++) {
	if (i>=TMP_STRING_SIZE-1) return("Error: vector too large to print");
	i += snprintf(res+i, TMP_STRING_SIZE-i-1, "%s%7.3f", separator, a[j]);
	// i += snprintf(res+i, TMP_STRING_SIZE-i-1, "%s%9.5f", separator, a[j]);
	separator = " ";
    }
    if (i>=TMP_STRING_SIZE-1) return("Error: vector too large to print");
    i += snprintf(res+i, TMP_STRING_SIZE-i-1, "]");
    return(res);
}

//////////////////////////////////////////////////////////////////////////////////////////////////

void enumNamesInit() {
    // signal interrupt names
    ENUM_NAME_SET(signalInterruptNames, SIGINT);
    ENUM_NAME_SET(signalInterruptNames, SIGILL);
    ENUM_NAME_SET(signalInterruptNames, SIGABRT);
    ENUM_NAME_SET(signalInterruptNames, SIGFPE);
    ENUM_NAME_SET(signalInterruptNames, SIGSEGV);
    ENUM_NAME_SET(signalInterruptNames, SIGTERM);
    ENUM_NAME_SET(signalInterruptNames, SIGHUP);
    ENUM_NAME_SET(signalInterruptNames, SIGQUIT);
    ENUM_NAME_SET(signalInterruptNames, SIGTRAP);
    ENUM_NAME_SET(signalInterruptNames, SIGKILL);
    ENUM_NAME_SET(signalInterruptNames, SIGBUS);
    ENUM_NAME_SET(signalInterruptNames, SIGSYS);
    ENUM_NAME_SET(signalInterruptNames, SIGPIPE);
    ENUM_NAME_SET(signalInterruptNames, SIGALRM);
    ENUM_NAME_SET(signalInterruptNames, SIGURG);
    ENUM_NAME_SET(signalInterruptNames, SIGSTOP);
    ENUM_NAME_SET(signalInterruptNames, SIGTSTP);
    ENUM_NAME_SET(signalInterruptNames, SIGCONT);
    ENUM_NAME_SET(signalInterruptNames, SIGCHLD);
    ENUM_NAME_SET(signalInterruptNames, SIGTTIN);
    ENUM_NAME_SET(signalInterruptNames, SIGTTOU);
    ENUM_NAME_SET(signalInterruptNames, SIGPOLL);
    ENUM_NAME_SET(signalInterruptNames, SIGXCPU);
    ENUM_NAME_SET(signalInterruptNames, SIGXFSZ);
    ENUM_NAME_SET(signalInterruptNames, SIGVTALRM);
    ENUM_NAME_SET(signalInterruptNames, SIGPROF);
    ENUM_NAME_SET(signalInterruptNames, SIGUSR1);
    ENUM_NAME_SET(signalInterruptNames, SIGUSR2);

    ENUM_NAME_SET(deviceDataTypeNames, DT_NONE);
    ENUM_NAME_SET(deviceDataTypeNames, DT_VOID);
    ENUM_NAME_SET(deviceDataTypeNames, DT_PING);
    ENUM_NAME_SET(deviceDataTypeNames, DT_THRUST);
    ENUM_NAME_SET(deviceDataTypeNames, DT_THRUST_SHM);
    ENUM_NAME_SET(deviceDataTypeNames, DT_GIMBAL_X);
    ENUM_NAME_SET(deviceDataTypeNames, DT_GIMBAL_Y);
    ENUM_NAME_SET(deviceDataTypeNames, DT_DEBUG);
    ENUM_NAME_SET(deviceDataTypeNames, DT_PONG);
    ENUM_NAME_SET(deviceDataTypeNames, DT_POSITION_VECTOR);
    ENUM_NAME_SET(deviceDataTypeNames, DT_BOTTOM_RANGE);
    ENUM_NAME_SET(deviceDataTypeNames, DT_FLOW_XY);
    ENUM_NAME_SET(deviceDataTypeNames, DT_ALTITUDE);
    ENUM_NAME_SET(deviceDataTypeNames, DT_TEMPERATURE);
    ENUM_NAME_SET(deviceDataTypeNames, DT_MAGNETIC_HEADING);
    ENUM_NAME_SET(deviceDataTypeNames, DT_EARTH_ACCELERATION);
    ENUM_NAME_SET(deviceDataTypeNames, DT_ORIENTATION_RPY);
    // ENUM_NAME_SET(deviceDataTypeNames, DT_ORIENTATION_QUATERNION);
    ENUM_NAME_SET(deviceDataTypeNames, DT_POSITION_NMEA);
    ENUM_NAME_SET(deviceDataTypeNames, DT_MAGNETIC_HEADING_NMEA);
    ENUM_NAME_SET(deviceDataTypeNames, DT_JSTEST);
    ENUM_NAME_SET(deviceDataTypeNames, DT_POSITION_SHM);
    ENUM_NAME_SET(deviceDataTypeNames, DT_ORIENTATION_RPY_SHM);
    ENUM_NAME_SET(deviceDataTypeNames, DT_EARTH_ACCELERATION_SHM);
    ENUM_NAME_SET(deviceDataTypeNames, DT_MAVLINK_RC_CHANNELS_OVERRIDE);
    ENUM_NAME_SET(deviceDataTypeNames, DT_MAVLINK_ATTITUDE);
    ENUM_NAME_SET(deviceDataTypeNames, DT_MAVLINK_BATTERY_STATUS);
    ENUM_NAME_SET(deviceDataTypeNames, DT_MAVLINK_GLOBAL_POSITION);
    ENUM_NAME_SET(deviceDataTypeNames, DT_MAVLINK_HOME_POSITION);
    ENUM_NAME_SET(deviceDataTypeNames, DT_MAVLINK_STATUSTEXT);
    ENUM_NAME_SET(deviceDataTypeNames, DT_MAX);
    ENUM_NAME_CHECK(deviceDataTypeNames, DT_);
    
    ENUM_NAME_SET(deviceConnectionTypeNames, DCT_NONE);
    ENUM_NAME_SET(deviceConnectionTypeNames, DCT_INTERNAL_ZEROPOSE);
    ENUM_NAME_SET(deviceConnectionTypeNames, DCT_COMMAND_BASH);
    ENUM_NAME_SET(deviceConnectionTypeNames, DCT_COMMAND_EXEC);
    ENUM_NAME_SET(deviceConnectionTypeNames, DCT_NAMED_PIPES);
    ENUM_NAME_SET(deviceConnectionTypeNames, DCT_MAVLINK_PTTY);
    ENUM_NAME_SET(deviceConnectionTypeNames, DCT_MAX);
    ENUM_NAME_CHECK(deviceConnectionTypeNames, DCT_);

    ENUM_NAME_SET(radioControlNames, RC_NONE);
    ENUM_NAME_SET(radioControlNames, RC_ROLL);
    ENUM_NAME_SET(radioControlNames, RC_PITCH);
    ENUM_NAME_SET(radioControlNames, RC_YAW);
    ENUM_NAME_SET(radioControlNames, RC_ALTITUDE);
    ENUM_NAME_SET(radioControlNames, RC_BUTTON_LAUNCH_COUNTDOWN);
    ENUM_NAME_SET(radioControlNames, RC_BUTTON_STANDBY);
    ENUM_NAME_SET(radioControlNames, RC_BUTTON_PANIC_SHUTDOWN);
    ENUM_NAME_SET(radioControlNames, RC_MAX);
    ENUM_NAME_CHECK(radioControlNames, RC_);
    
    ENUM_NAME_SET(pilotMainModeNames, MODE_NONE);
    ENUM_NAME_SET(pilotMainModeNames, MODE_MOTOR_PWM_CALIBRATION);
    ENUM_NAME_SET(pilotMainModeNames, MODE_MOTOR_TEST);
    ENUM_NAME_SET(pilotMainModeNames, MODE_SINGLE_MISSION);
    ENUM_NAME_SET(pilotMainModeNames, MODE_MANUAL_RC);
    ENUM_NAME_SET(pilotMainModeNames, MODE_MAX);
    ENUM_NAME_CHECK(pilotMainModeNames, MODE_);
    
    ENUM_NAME_SET(remoteControlModeNames, RCM_NONE);
    ENUM_NAME_SET(remoteControlModeNames, RCM_PASSTHROUGH);
    ENUM_NAME_SET(remoteControlModeNames, RCM_ACRO);
    ENUM_NAME_SET(remoteControlModeNames, RCM_TARGET);
    ENUM_NAME_SET(remoteControlModeNames, RCM_AUTO);
    ENUM_NAME_SET(remoteControlModeNames, RCM_MAX);
    ENUM_NAME_CHECK(remoteControlModeNames, RCM_);

}

void logEnumNames(int loglevel, char **names) {
    int i;
    for(i=0; names[i] != NULL; i++) lprintf(loglevel, "%s\"%s\"", (i==0?"":", "), names[i]);
}

int enumNamesStringToInt(char *s, char **names) {
    int i;
    for(i=0; names[i] != NULL; i++) {
	if (strcmp(s, names[i]) == 0) return(i);
    }
    return(-1);
}

//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////
// regressionBuffer

void regressionBufferPrintSums(struct regressionBuffer *hh) {
    int i;
    lprintf(0, "regressionBufferSums: %f %f: ", hh->sumTime, hh->sumTimeSquare);
    for(i=0; i<hh->vectorsize; i++) {
	lprintf(0, "%f %f, ",  hh->suma[i],hh->sumaMulTime[i]); 
    }	
    lprintf(0, "\n");
}
void regressionBufferAddToSums(struct regressionBuffer *hh, double time, double *vec) {
    int 	i;
    double	tt;
    // In sums we store and count (time - hh->timeOffsetForSums). This avoids rounding errors when
    // square of sums overflows over double precission range.
    tt = time - hh->timeOffsetForSums;
    hh->sumTime += tt;
    hh->sumTimeSquare += tt * tt;
    for(i=0; i<hh->vectorsize; i++) {
	hh->suma[i] += vec[i];
	hh->sumaMulTime[i] += vec[i] * tt;
    }
}
void regressionBufferSubstractFromSums(struct regressionBuffer *hh, double time, double *vec) {
    int 	i;
    double	tt;
    tt = time - hh->timeOffsetForSums;
    hh->sumTime -= tt;
    hh->sumTimeSquare -= tt * tt;
    for(i=0; i<hh->vectorsize; i++) {
	hh->suma[i] -= vec[i];
	hh->sumaMulTime[i] -= vec[i] * tt;
    }
}

void regressionBufferRecalculateSums(struct regressionBuffer *hh) {
    int 	i,ii,j,n;

    lprintf(90, "%s: Info: %s: recomputing sums.\n", PPREFIX(), hh->name);
    hh->sumTime = 0;
    hh->sumTimeSquare = 0;
    for(i=0; i<hh->vectorsize; i++) {
	hh->suma[i] = 0;
	hh->sumaMulTime[i] = 0;
    }
    n = hh->n;
    if (n > hh->size) n = hh->size;
    //lprintf(0, "Starting: sums: "); regressionBufferPrintSums(hh);
    for(i=0; i<n; i++) {
	ii = (hh->ai + hh->size - i - 1) % hh->size;
	assert(ii >= 0 && ii < hh->size);
	regressionBufferAddToSums(hh, hh->time[ii], &hh->a[ii*hh->vectorsize]);
	//lprintf(0, "Adding line %d: sums: ", ii); regressionBufferPrintSums(hh);
    }
}

void regressionBufferAddElem(struct regressionBuffer *hh, double time, double *vec) {
    int 	i;
    double	t;
    
    lprintf(40, "%s: regressionBufferAddElem: %s: %f:  %s\n", PPREFIX(), hh->name, time, arrayWithDimToStr_st(vec, hh->vectorsize));

    if (hh->size == 0 || hh->vectorsize == 0) return;
    if (hh->keepSumsFlag && hh->n >= hh->size) regressionBufferSubstractFromSums(hh, hh->time[hh->ai], &hh->a[hh->ai*hh->vectorsize]);
    hh->time[hh->ai] = time;
    memmove(&hh->a[hh->ai*hh->vectorsize], vec, hh->vectorsize * sizeof(double));
    if (hh->keepSumsFlag) regressionBufferAddToSums(hh, time, &hh->a[hh->ai*hh->vectorsize]);
    hh->n ++;
    hh->aiprev = hh->ailast;
    hh->ailast = hh->ai;
    hh->ai = (hh->ai + 1) % hh->size;
#if SMOOTH_REGRESSION
    if (hh->keepSumsFlag && hh->n >= hh->size) {
	// The idea of smoothing is that a wrong value is deviating regression line
	// not only at the beginning of buffer but also later when that value is 
	// shifted at the end of the buffer. We can fix that by removing dirty values
	// somewhere inside the buffer. (2/3 or maybe even 3/4 of buffer).
	i = (hh->ailast + hh->size*3/4) % hh->size;
	if (i != hh->ailast) {
	    t = hh->time[i];
	    regressionBufferEstimateForTime(hh, t, hh->tmp);
	    regressionBufferSubstractFromSums(hh, t, &hh->a[i*hh->vectorsize]);
	    memmove(&hh->a[i*hh->vectorsize],  hh->tmp, hh->vectorsize * sizeof(double));
	    regressionBufferAddToSums(hh, t, &hh->a[i*hh->vectorsize]);
	}
    }
#endif    
    // from time to time, recalculate sums to avoid cumulative errors due to floating point arithmetics
    if (hh->keepSumsFlag && hh->n % 10000 == 0) regressionBufferRecalculateSums(hh);

    // for statistics
    for(i=0; i<hh->vectorsize; i++) {
    	hh->totalSumForStatistics[i] += vec[i];
	hh->totalElemsForStatistics ++;
    }
}

void regressionBufferReset(struct regressionBuffer *hh) {
    int i;
    
    hh->ai = hh->ailast = hh->aiprev = 0;
    hh->n = 0;
    hh->sumTime = hh->sumTimeSquare = 0;
    
    if (hh->vectorsize > 0) {
	for(i=0; i<hh->vectorsize; i++) {
	    hh->suma[i] = hh->sumaMulTime[i] = 0;
	}
    }

}

void regressionBufferInit(struct regressionBuffer *hh, int vectorSize, int bufferSize, char *namefmt, ...) {
    int		i;
    va_list     ap;

    va_start(ap, namefmt);
    memset(hh, 0, sizeof(*hh));
    vsnprintf(hh->name, sizeof(hh->name)-1, namefmt, ap);
    assert(bufferSize > 0);
    if (bufferSize == 1) {
	lprintf(0, "%s: Warning: regression buffer %s has size %d!\n", PPREFIX(), hh->name, (bufferSize));
    }
    // In sums we store and count (time - hh->timeOffsetForSums). This avoids rounding errors when
    // square of sums overflows over double precission range.
    hh->keepSumsFlag = 1;
    hh->timeOffsetForSums = uu->pilotStartingTime;
    hh->ai = hh->ailast = hh->aiprev = 0;
    hh->n = 0;
    hh->size = bufferSize;
    hh->vectorsize = vectorSize;
    CALLOCC(hh->time,  bufferSize, double);
    if (vectorSize > 0) {
	CALLOCC(hh->a, bufferSize*vectorSize, double);
#if SMOOTH_REGRESSION
	CALLOCC(hh->tmp, vectorSize, double);
#endif	
	CALLOCC(hh->suma, vectorSize, double);
	CALLOCC(hh->sumaMulTime, vectorSize, double);
	CALLOCC(hh->totalSumForStatistics, vectorSize, double);
    }
    va_end(ap);
}

void regressionBufferFindRecordForTime(struct regressionBuffer *hh, double time, double *restime, double *res) {
    int 	i, mini, maxi, ci, ri;
    double	tt;

    // find the closes record for the time, it supposes that records are ordered by time
    if (hh->n < 1) return;

    if (hh->n >= hh->size) {
	mini = hh->ai;
	maxi = hh->ai+hh->size-1;
    } else {
	maxi = hh->ai-1;
	mini = hh->ai-hh->n;
    }
    i = 0;
    while (maxi - mini > 1) {
	// Binary search.
	// ci = (maxi + mini) / 2;
	// Approximative search
	ci = mini + (time - hh->time[mini%hh->size]) * (maxi-mini)/(hh->time[maxi%hh->size] - hh->time[mini%hh->size]);
	// printf("<%f, %f> : %f :: <%d, %d> --> %d\n", hh->time[mini%hh->size], hh->time[maxi%hh->size], time, mini, maxi, ci);
	if (ci <= mini) ci = mini+1;
	if (ci >= maxi) ci = maxi-1;
	tt = hh->time[ci%hh->size];
	if (tt > time) {
	    maxi = ci;
	} else if (tt < time) {
	    mini = ci;
	} else {
	    ri = ci;
	    goto finito;
	}
	i++;
    }
    if (fabs(time - hh->time[mini%hh->size]) < fabs(time - hh->time[maxi%hh->size])) {
	ri = mini;
    } else {
	ri = maxi;
    }
finito:
    // printf("found index after %d loops\n", i);
    *restime = hh->time[ri%hh->size];
    memcpy(res, &hh->a[(ri%hh->size)*hh->vectorsize], hh->vectorsize*sizeof(double));
}

// res = k1 * time + k0
int regressionBufferGetRegressionCoefficients(struct regressionBuffer *hh, int i, double *k0, double *k1) {
    int 	n;
    double	d, deviation;

    n = hh->n;
    if (n > hh->size) n = hh->size;

    // It is useless to recalculate d for each i, TODO: make it better, maybe move it to addtosums
    d = n * hh->sumTimeSquare - hh->sumTime * hh->sumTime ;
    // I know that d can not be negative, but to be sure, use fabs
    deviation = fabs(d/n);
    // Hmm. what is the necessary deviation to proceed the regression?
    // lprintf(0, "dev: %f\n", deviation);
    if (deviation <= 1e-6) {
	// lprintf(0, "%s: Warning: no or wrong value for linear regression\n", PPREFIX());
	*k0 = *k1 = 0;
	return(-1);
    }
    *k1 = ( n * hh->sumaMulTime[i] - hh->sumTime * hh->suma[i] ) / d ;
    *k0 = ( hh->sumTimeSquare * hh->suma[i] - hh->sumTime * hh->sumaMulTime[i] ) / d ;
    return(0);
}

void regressionBufferGetMean(struct regressionBuffer *hh, double *time, double *res) {
    int 	i, n;

    if (hh->keepSumsFlag == 0) regressionBufferRecalculateSums(hh);

    n = hh->n;
    if (n > hh->size) n = hh->size;
    if (n == 0) n = 1;	// avoid division by zero
    
    if (time != NULL) *time = hh->sumTime / n + hh->timeOffsetForSums;
    for(i=0; i<hh->vectorsize; i++) {
	res[i] = hh->suma[i] / n;
    }
}

// TODO: rename to regressionBufferEstimateVectorForTime
int regressionBufferEstimateForTime(struct regressionBuffer *hh, double time, double *res) {
    int 		i, j, n, r, rr;
    double 		k0, k1;
    double		mtime;

    if (hh->keepSumsFlag == 0) regressionBufferRecalculateSums(hh);

#if 0
    //lprintf(0, "Starting regression %s:\n", hh->name);
    regressionBufferPrintSums(hh);
    //regressionBufferRecalculateSums(hh);
    //regressionBufferPrintSums(hh);
#endif
    
    rr = 0;
    n = hh->vectorsize;
    for(i=0; i<n; i++) {
	r = regressionBufferGetRegressionCoefficients(hh, i, &k0, &k1);
	if (r != 0) break;
	res[i] = (time - hh->timeOffsetForSums) * k1 + k0;
    }

    if (i<n) {
	// if something went wrong during computation of regression coefs return mean
	regressionBufferGetMean(hh, &mtime, res);
	rr = -1;
    }

    
#if 0
    lprintf(0, "Doing regression, total added values %d:\n", hh->n);
    for(j=0; j<hh->size; j++) {
	lprintf(0, "%18.5f: ", hh->time[j]);
        for(i=0; i<n; i++) {
	    lprintf(0, "%7.3f ",  hh->a[j*hh->vectorsize+i]);
	}
	lprintf(0, "\n");
    }
    lprintf(0, "----------------------------------------------\n");
    lprintf(0, "%18.5f: ", time);
    for(i=0; i<n; i++) {
	lprintf(0, "%7.3f ",  res[i]);
    }
    lprintf(0, "\n");
#endif

    return(rr);
}

//////////////////////////////////////////////////////////////////////////////////////////////
// raspilotRingBuffer

double *raspilotRingBufferGetFirstFreeVector(struct raspilotRingBuffer *hh) {
    if (hh->size == 0 || hh->vectorsize == 0) return(NULL);
    return(&hh->a[hh->ai*(hh->vectorsize+1)+1]);
}

void raspilotRingBufferFindRecordForTime(struct raspilotRingBuffer *hh, double time, double *restime, double **res) {
    int 	i, mini, maxi, ci, ri;
    double	tt;

    // find the closes record for the time, it supposes that records are ordered by time
    if (hh == NULL || hh->n < 1) {
	*res = NULL;
	return;
    }

    if (hh->n >= hh->size) {
	mini = hh->ai;
	maxi = hh->ai+hh->size-1;
    } else {
	maxi = hh->ai-1;
	mini = hh->ai-hh->n;
    }
    i = 0;
    while (maxi - mini > 1) {
	// Binary search.
	// ci = (maxi + mini) / 2;
	// Approximative search
	ci = mini + (time - hh->a[(mini%hh->size)  * (hh->vectorsize+1)]) * (maxi-mini)/(hh->a[(maxi%hh->size)  * (hh->vectorsize+1)] - hh->a[(mini%hh->size)  * (hh->vectorsize+1)]);
	// printf("<%f, %f> : %f :: <%d, %d> --> %d\n", hh->time[mini%hh->size], hh->time[maxi%hh->size], time, mini, maxi, ci);
	if (ci <= mini) ci = mini+1;
	if (ci >= maxi) ci = maxi-1;
	tt = hh->a[(ci%hh->size)  * (hh->vectorsize+1)];
	if (tt > time) {
	    maxi = ci;
	} else if (tt < time) {
	    mini = ci;
	} else {
	    ri = ci;
	    goto finito;
	}
	i++;
    }
    if (fabs(time - hh->a[(mini%hh->size)  * (hh->vectorsize+1)]) < fabs(time - hh->a[(maxi%hh->size) * (hh->vectorsize+1)])) {
	ri = mini;
    } else {
	ri = maxi;
    }
finito:
    // printf("found index after %d loops\n", i);
    if (restime != NULL) *restime = hh->a[(ri%hh->size) * (hh->vectorsize+1)];
    *res = &hh->a[(ri%hh->size)*(hh->vectorsize+1)+1];
}

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

int vec1TruncateToSize(double *r, double size, int warningFlag, char *warningId) {
    double 	s;
    int 	res;

    res = 0;
    s = fabs(*r);
    if (s > size) {
	*r = *r * size / s;
	res = 1;
	if (warningFlag) lprintf(0, "%s: Warning: vector %s had to be truncated from size %g to %g --> %g\n", PPREFIX(), warningId, s, size, *r);
    }
    return(res);
}

int vec2TruncateToSize(vec2 r, double size, int warningFlag, char *warningId) {
    double 	s;
    int 	res;
    
    res = 0;
    s = vec2_len(r);
    if (s > size) {
	vec2_scale(r, r, size/s);
	res = 1;
	if (warningFlag) lprintf(0, "%s: Warning: vector %s had to be truncated from size %g to %g --> %s\n", PPREFIX(), warningId, s, size, vec2ToString_st(r));
    }
    return(res);
}

int vec3TruncateToSize(vec3 r, double size, int warningFlag, char *warningId) {
    double 	s;
    int 	res;
    
    res = 0;
    s = vec3_len(r);
    if (s > size) {
	vec3_scale(r, r, size/s);
	res = 1;
	if (warningFlag) lprintf(0, "%s: Warning: vector %s had to be truncated from size %g to %g --> %s\n", PPREFIX(), warningId, s, size, vec3ToString_st(r));
    }
    return(res);
}

double vectorLength(double *a, int dim) {
    int 	i;
    double 	s;

    s = 0;
    for(i=0; i<dim; i++) s += a[i] * a[i];
    return(sqrt(s));
}

//////////////////////////////////////////////////////////////////////////////////////////////

void pidControllerReset(struct pidController *pp, double dt) {
    struct pidControllerData	*d;
    double 			Kp, Ki, Kd;
    int				N;
    double			tau;
    
    // "naive"
    memset(&pp->d, 0, sizeof(pp->d));
    // IIR
    d = &pp->d;
    Kp = pp->constant.p;
    Ki = pp->constant.i;
    Kd = pp->constant.d;
    d->A0 = Kp + Ki*dt;
    d->A1 = -Kp;
    d->error[2] = 0; // e(t-2)
    d->error[1] = 0; // e(t-1)
    d->error[0] = 0; // e(t)
    d->output = 0; // u0;  // Usually the current value of the actuator
    d->A0d = Kd/dt;
    d->A1d = - 2.0*Kd/dt;
    d->A2d = Kd/dt;
    N = 5;
    tau = Kd / (Kp*N); // IIR filter time constant
    d->alpha = dt / (2*tau);
    d->d0 = 0;
    d->d1 = 0;
    d->fd0 = 0;
    d->fd1 = 0;    
}

char *pidControllerStatistics(struct pidController *pp, int showProposedCiFlag) {
    char 	*res;
    int		r;
    
    res = getTemporaryStringPtrFromStaticStringRing();
    if (pp->d.statNumberOfSamples == 0) {
	res[0] = 0;
    } else {
	r = snprintf(res, TMP_STRING_SIZE-1,
		     "Integral: Last: %7.4f, Average: %7.4f;  ErrorAverage: %7.4f.",
		     pp->d.integral,
		     pp->d.statIntegralSum/pp->d.statNumberOfSamples,
		     pp->d.statErrorSum/pp->d.statNumberOfSamples
	    );
	if (showProposedCiFlag) {
	    r += snprintf(res+r, TMP_STRING_SIZE-1-r,
			  "  PropCI: %10.8f",
			  pp->constant.ci + pp->constant.p / 10.0 * pp->d.statIntegralSum / pp->d.statNumberOfSamples
		);
	}
    }
    return(res);
}

double pidControllerStep(struct pidController *pp, double setpoint, double measured_value, double dt) {
    double 				error;
    double 				derivative, newIntegral;
    double				output;
    struct pidControllerData 		*dd;
    struct pidControllerConstants 	*cc;

    dd = &pp->d;
    cc = &pp->constant;
    
    error = setpoint - measured_value;

    newIntegral = dd->integral + error * dt;
    // lprintf(0, "PID: %s: integral %f === %f + %f * %f\n", pp->name, newIntegral, dd->integral, error, dt);

    if (PID_PREVENT_INTEGRAL_WINDUP) {
	// If error is too big it will blow up integral and make the drone crash. This probably means a problem in a sensor or software bug.
	// Prefer to ignore that value completely.
	if (fabs(cc->p * error) > 2.0 * cc->integralMax) {
	    lprintf(0, "%s: Error: PID: %s: value of error (%g) is out of range, ignoring it.\n", PPREFIX(), pp->name, error);
	    error = 0; // dd->previous_error / 2;
	    newIntegral = dd->integral;
	}
	// Also restrict continuous growing of integral output over predefined max value.
	if (fabs(cc->i * newIntegral) >= cc->integralMax) {
	    lprintf(10, "%s: Error: PID: %s: Integral value %g grows out of range, restricting it.\n", PPREFIX(), pp->name, newIntegral);
	    newIntegral = dd->integral;
	}
    }
    
    dd->integral = newIntegral;

    if (PID_USES_ERROR_BASED_DERIVATIVE) {
	// The "standard" way of computing derivative.
	derivative = (error - dd->previous_error) / dt;
    } else {
	// Standard derivative is based on error - previous_error == setpoint - measured_value - previous_setpoint + previous_measured_value.
	// If setpoint does not change it gives previous_measured_value - measured_value
	// This should be a "better" alternative as it does not make peaks on changes in setpoint.
	derivative = (dd->previous_measured_value - measured_value) / dt;
    }

    // Also restrict derivative value in some way. It happens that it is completely determining for the value of the whole PID due
    // to very small value of dt. Let's try it like this for the moment:
    if (fabs(cc->d * derivative) >= cc->derivativeMax) {
	lprintf(10, "%s: Error: PID: %s: Derivative value %g is out of range, restricting it.\n", PPREFIX(), pp->name, derivative);
	derivative = SIGN(derivative) * cc->derivativeMax;
    }
    
    output = cc->p * error + cc->i * dd->integral + cc->d * derivative + cc->ci;
    lprintf(22, "%s: PID %10s: %g == P:%f (%g * %g)   +   I:%f (%g * %g)   +  D:%f (%g * %g) + CI:%g.", PPREFIX(), pp->name, output, cc->p * error, cc->p, error, cc->i * dd->integral, cc->i, dd->integral, cc->d * derivative, cc->d, derivative, cc->ci);
    lprintf(9999, " (setpoint:%f, measured_value: %f)\n", setpoint, measured_value);
    lprintf(22, "\n");
    dd->previous_error = error;
    dd->previous_output = output;
    dd->previous_setpoint = setpoint;
    dd->previous_measured_value = measured_value;

    // for statistics
    dd->statNumberOfSamples ++;
    dd->statErrorSum += error;
    dd->statIntegralSum += dd->integral;
    return(output);
}

double pidIirControllerStep(struct pidController *pp, double setpoint, double measured_value, double dt) {
    struct pidControllerData	*d;
    double 			Kp, Ki, Kd;
    int				N;
    
    d = &pp->d;
    Kp = pp->constant.p;
    Ki = pp->constant.i;
    Kd = pp->constant.d;

    d->error[2] = d->error[1];
    d->error[1] = d->error[0];
    d->error[0] = setpoint - measured_value;
    
    // PI
    d->output = d->output + d->A0 * d->error[0] + d->A1 * d->error[1];
    // Filtered D
    d->d1 = d->d0;
    d->d0 = d->A0d * d->error[0] + d->A1d * d->error[1] + d->A2d * d->error[2];
    d->fd1 = d->fd0;
    d->fd0 = ((d->alpha) / (d->alpha + 1)) * (d->d0 + d->d1) - ((d->alpha - 1) / (d->alpha + 1)) * d->fd1;
    d->output = d->output + d->fd0;
    return(d->output);
}
    
/////////////////////////////////////////////////////////////////////////////////////////////
// those two functions were deliberately copied from Jeff Rowberg's demo_dmp.cpp of MPU6050

static void mpuGetGravity(vec3 v, quat q) {
    v[0] = 2 * (q[0]*q[2] - q[3]*q[1]);
    v[1] = 2 * (q[3]*q[0] + q[1]*q[2]);
    v[2] = q[3]*q[3] - q[0]*q[0] - q[1]*q[1] + q[2]*q[2];
}
static void mpuGetYawPitchRoll(vec3 data, quat q, vec3 gravity) {
    // yaw: (about Z axis)
    data[0] = atan2(2*q[0]*q[1] - 2*q[3]*q[2], 2*q[3]*q[3] + 2*q[0]*q[0] - 1);
    // pitch: (nose up/down, about Y axis)
    data[1] = atan(gravity[0] / sqrt(gravity[1]*gravity[1] + gravity[2]*gravity[2]));
    // roll: (tilt left/right, about X axis)
    data[2] = atan(gravity[1] / sqrt(gravity[0]*gravity[0] + gravity[2]*gravity[2]));
}
static void mpuQuatToYpr(quat qq, double *yaw, double *pitch, double *roll) {
    vec3	gr, vv;
    
    mpuGetGravity(gr, qq);
    mpuGetYawPitchRoll(vv, qq, gr);
    // [MV] I changed sign for yaw to get counterclockwise yaw
#if 0
    *yaw = - normalizeToRange(vv[0], -M_PI, M_PI);
    *pitch = normalizeToRange(vv[1], -M_PI, M_PI);
    *roll = normalizeToRange(vv[2], -M_PI, M_PI);
#else    
    *yaw = - vv[0];
    *pitch = vv[1];
    *roll = vv[2];
#endif
}

// quat to rpi and back by wiki
// not sure what is the correspondance between mpu and this
static void wikiQuaternionToEulerAngles(quat q, double *yaw, double *pitch, double *roll) {
    double x, y, z, w;
    double sinr_cosp, cosr_cosp, sinp, siny_cosp, cosy_cosp;
    
    x = q[0];
    y = q[1];
    z = q[2];
    w = q[3];
    
    // roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z);
    cosr_cosp = 1 - 2 * (x * x + y * y);
    *roll = atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    // [MV] I had to change the sign here to get my pitch
    sinp = 2 * (w * y - z * x);
    if (fabs(sinp) >= 1) {
        *pitch = - copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    } else {
        *pitch = - asin(sinp);
    }
	
    // yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y);
    cosy_cosp = 1 - 2 * (y * y + z * z);
    *yaw = atan2(siny_cosp, cosy_cosp);
	
}

static void wikiEulerAnglesToQuaternion(double yaw, double pitch, double roll, quat q) {
    double cy, sy, cp, sp, cr, sr;

    cy = cos(yaw * 0.5);
    sy = sin(yaw * 0.5);
    // [MV] Use pitch with inversed sign to get back to original quaternion
    cp = cos(-pitch * 0.5);
    sp = sin(-pitch * 0.5);
    cr = cos(roll * 0.5);
    sr = sin(roll * 0.5);

    q[3] = cr * cp * cy + sr * sp * sy;
    q[0] = sr * cp * cy - cr * sp * sy;
    q[1] = cr * sp * cy + sr * cp * sy;
    q[2] = cr * cp * sy - sr * sp * cy;
}

//////////////////////////////////////////////////////////////////////////////////
// main functions using either mpu conversion or wiki, choose one

void quatToRpy(quat qq, double *roll, double *pitch, double *yaw) {
    // Experiment with those two
    if (0) {
	mpuQuatToYpr(qq, yaw, pitch, roll);
    } else {
	wikiQuaternionToEulerAngles(qq, yaw, pitch, roll);
    }
}

void rpyToQuat(double roll, double pitch, double yaw, quat q) {
    wikiEulerAnglesToQuaternion(yaw, pitch, roll, q);
}

//////////////////////////////////////////////////////////////////////////////////
int baudrateToSpeed_t(int baudrate) {
    switch (baudrate) {
    case 9600:
        return B9600;
    case 19200:
        return B19200;
    case 38400:
        return B38400;
    case 57600:
        return B57600;
    case 115200:
        return B115200;
    case 230400:
        return B230400;
    case 460800:
        return B460800;
    case 500000:
        return B500000;
    case 576000:
        return B576000;
    case 921600:
        return B921600;
    case 1000000:
        return B1000000;
    case 1152000:
        return B1152000;
    case 1500000:
        return B1500000;
    case 2000000:
        return B2000000;
    case 2500000:
        return B2500000;
    case 3000000:
        return B3000000;
    case 3500000:
        return B3500000;
    case 4000000:
        return B4000000;
    default: 
        return 0;
    }
}

void initSerialPort(int fd, int baudrate) {
    struct termios 	tty;
    speed_t			speed;
	
    memset (&tty, 0, sizeof tty);

    if ( tcgetattr ( fd, &tty ) != 0 ) {
	lprintf(0, "%s: Error: tcgetattr: %s\n", PPREFIX(), STR_ERRNO());
    }

    speed = baudrateToSpeed_t(baudrate);
    if (speed <= 0) {
	lprintf(0, "%s: Error: Invalid serial port baud rate %d. Using 9600 instead!\n", PPREFIX(), baudrate);
	speed = B9600;
    }

#if 0
    tty.c_cflag     &=  ~PARENB;            // Make 8n1
    tty.c_cflag     &=  ~CSTOPB;
    tty.c_cflag     &=  ~CSIZE;
    tty.c_cflag     |=  CS8;

    tty.c_cflag     &=  ~CRTSCTS;           // no flow control
    tty.c_cflag     &=  ~ECHO;           	// no echo
    tty.c_cflag     &=  ~ICRNL;
    tty.c_cflag     &=  ~IXON;
    tty.c_cflag     &=  ~IXOFF;
    tty.c_cflag     &=  ~OPOST;
    tty.c_cflag     &=  ~ISIG;
    tty.c_cflag     &=  ~ICANON;

    tty.c_cc[VMIN]   =  1;
    tty.c_cc[VTIME]  =  5;
    tty.c_cflag     |=  CREAD | CLOCAL;     // turn on READ & ignore ctrl lines
#endif
	
    cfmakeraw(&tty);

    cfsetospeed (&tty, speed);
    cfsetispeed (&tty, speed);

    // tcflush( fd, TCIFLUSH );

    if ( tcsetattr ( fd, TCSANOW, &tty ) != 0) {
	lprintf(0, "%s: Error: tcsetattr: %s\n", PPREFIX(), STR_ERRNO());
    }
}

//////////////////////////////////////////////////////////////////////////////////
// interactive terminal

struct terminalSettingStr stdinTerminal = {STDIN_FILENO, 0, };
	
int terminalSetInteractive(struct terminalSettingStr *tt) {
    struct termios ttt;

    if (tt->ioSavedFlag) return(1);
    tcgetattr(tt->fd, &tt->io);
    tt->ioSavedFlag = 1;
    ttt = tt->io;
    ttt.c_lflag &=(~ICANON & ~ECHO);
    tcsetattr(tt->fd, TCSANOW, &ttt);
    return(0);
}

int terminalRestore(struct terminalSettingStr *tt) {
    if (tt->ioSavedFlag == 0) return(1);
    tcsetattr(tt->fd, TCSANOW, &tt->io);
    tt->ioSavedFlag = 0;
    return(0);
}

void terminalResume() {
    terminalRestore(&stdinTerminal);
}

/////////////////////////////////////////////////////////////////////////////////////
// stdio through baio

static int stdbaioStdinOnRead(struct baio *bb) {
    // we fetch interactive commands regularly from timeline
    // here just watch that readbuffer does not overflow
    // discard old input
    if (bb->readBuffer.j > bb->readBuffer.size/2) {
	bb->readBuffer.i = bb->readBuffer.j;
    }
    return(0);
}

void stdbaioInit() {
    struct baio	*bb;

    if (stdbaioBaioMagic == 0) {
	bb = baioNewBasic(BAIO_TYPE_FD, BAIO_IO_DIRECTION_RW, 0);
	if (bb == NULL) {
	    printf("%s: Error: can't open terminal i/o! Fatal!\n", PPREFIX());
	    exit(-1);
	}
	bb->rfd = 0;
	bb->wfd = 1;
	bb->initialWriteBufferSize = (1<<18);
	terminalSetInteractive(&stdinTerminal);
	callBackAddToHook(&bb->callBackOnRead, (callBackHookFunArgType) stdbaioStdinOnRead);
	stdbaioBaioMagic = bb->baioMagic;
    }
}

void stdbaioClose() {
    baioCloseMagic(stdbaioBaioMagic);
    stdbaioBaioMagic = 0;
    terminalResume();
}

void stdbaioStdinClearBuffer() {
    struct baio		*bb;
    struct baioBuffer	*b;

    bb = baioFromMagic(stdbaioBaioMagic);
    if (bb == NULL) return;
    b = &bb->readBuffer;
    b->i = b->j = 0;
}

int stdbaioStdinMaybeGetPendingChar() {
    int		res;
    struct baio	*bb;

    bb = baioFromMagic(stdbaioBaioMagic);
    if (bb == NULL) return(-1);
    if (bb->readBuffer.i < bb->readBuffer.j) {
	res = bb->readBuffer.b[bb->readBuffer.i];
	bb->readBuffer.i ++;
	return(res);
    }
    return(-1);
}

// log

void logbaioInit() {
    char	cmd[TMP_STRING_SIZE];
    struct baio	*bb;
    int		r;

    if (uu->logFileName == NULL) {
	snprintf(cmd, TMP_STRING_SIZE,
		 "../log/log-%04d-%02d-%02d---%02d:%02d:%02d.txt",
		 1900+currentTime.lcltm.tm_year, currentTime.lcltm.tm_mon+1, currentTime.lcltm.tm_mday, 
		 currentTime.lcltm.tm_hour, currentTime.lcltm.tm_min, currentTime.lcltm.tm_sec
	    );	
	cmd[TMP_STRING_SIZE-1] = 0;
	uu->logFileName = strDuplicate(cmd);
	// printf("logfilename == %s\n", uu->logFileName);
    }

    if (logbaioBaioMagic == 0) {
	// create directory of does not exists
	snprintf(cmd, TMP_STRING_SIZE, "mkdir -p `dirname %s`\n", uu->logFileName);
	cmd[TMP_STRING_SIZE-1] = 0;
	r = system(cmd);
	
	// start writing to log file. Buffer it through cat,
	// otherwise it might block raspilot (select on file write signals non-block always)
	snprintf(cmd, TMP_STRING_SIZE, "cat > %s\n", uu->logFileName);
	cmd[TMP_STRING_SIZE-1] = 0;
	bb = baioNewPipedCommand(cmd, BAIO_IO_DIRECTION_WRITE, 1, 0);
	if (bb == NULL) {
	    printf("%s: Error: can't open log file! Fatal!\n", PPREFIX());
	    exit(-1);
	}
	bb->initialWriteBufferSize = (1<<20);
	logbaioBaioMagic = bb->baioMagic;
	
	// Create symbolic link
	usleep(100000);
	snprintf(cmd, TMP_STRING_SIZE, "ln -f -s %s currentlog.txt\n", uu->logFileName);
	cmd[TMP_STRING_SIZE-1] = 0;
	// printf("cmd == %s\n", cmd);
	r = system(cmd);
	if (r < 0) {
	    printf("%s: Error: can't create symbolic link for currentlog.txt!\n", PPREFIX());
	}
    }
}

void logbaioClose() {
    baioCloseMagic(logbaioBaioMagic);
    logbaioBaioMagic = 0;
}

int stdbaioPrintf(int level, char *fmt, ...) {
    int		r;
    va_list     ap;
    struct baio	*bb;

    r = 0;
    if (level < debugLevel) {
	va_start(ap, fmt);
	bb = baioFromMagic(stdbaioBaioMagic);
	if (bb == NULL) {
	    printf("!");
	    r = vprintf(fmt, ap);
	} else {
	    baioPrintfToBuffer(bb, " ");
	    r = baioVprintfToBuffer(bb, fmt, ap);
	}
	va_end(ap);
    }
    return(r);
}

int logbaioPrintf(int level, char *fmt, ...) {
    int		r;
    va_list     ap;
    struct baio	*bb;

#if 0
    if (1 || level == 0) {
	va_start(ap, fmt);
	vprintf(fmt, ap);
	fflush(stdout);
	va_end(ap);
	return(0);
    }
#endif
    
    r = 0;
    if (level < logLevel) {
	va_start(ap, fmt);
	bb = baioFromMagic(logbaioBaioMagic);
	if (bb != NULL) {
	    r = baioVprintfToBuffer(bb, fmt, ap);
	}
	va_end(ap);
    }
    return(r);
}

/////////////////////////////////////////////////////////////////////////////////////////////

void trajectoryLogInit() {
    struct baio	*bb;

    if (trajectoryLogBaioMagic == 0) {
	// maybe try also "exec dd of=trajectory.dat"
	bb = baioNewPipedCommand("exec cat > trajectory.dat", BAIO_IO_DIRECTION_WRITE, 1, 0);
	if (bb == NULL) {
	    printf("%s: Error: can't open trajectory log file!\n", PPREFIX());
	    return;
	}
	bb->initialWriteBufferSize = (1<<16);
	trajectoryLogBaioMagic = bb->baioMagic;
    }
}

int trajectoryLogPrintf(char *fmt, ...) {
    int		r;
    va_list     ap;
    struct baio	*bb;

    bb = baioFromMagic(trajectoryLogBaioMagic);
    if (bb == NULL) return(0);
    va_start(ap, fmt);
    r = baioVprintfToBuffer(bb, fmt, ap);
    va_end(ap);
    return(r);
}

void trajectoryLogClose() {
    trajectoryLogPrintf("# end\n");
    baioCloseMagic(trajectoryLogBaioMagic);
    trajectoryLogBaioMagic = 0;
}

/////////////////////////////////////////////////////////////////////////////////////////////

static int pingToHostOnRead(struct baio *bb) {
    struct baioBuffer 	*b;
    char		*p;
    
    b = &bb->readBuffer;
    // I think I can do this
    assert(b->j >= b->i && b->j >= 0 && b->j < b->size);
    b->b[b->j] = 0;
    // printf("Warning: Got Pong %s\n", b->b+b->i);fflush(stdout);
    // succesfull ping reports time, search for it
    p = strstr(b->b+b->i, "time=");
    if (p != NULL) pingToHostLastAnswerTime = currentTime.dtime;
    // empty read buffer
    b->i = b->j;
    return(0);
}

void pingToHostInit() {
    struct baio		*bb;
    char		ttt[TMP_STRING_SIZE];

    if (pingToHostBaioMagic == 0) {
	snprintf(ttt, TMP_STRING_SIZE-1, "exec ping %s", uu->pingToHost);
	bb = baioNewPipedCommand(ttt, BAIO_IO_DIRECTION_READ, 1, 0);
	if (bb == NULL) {
	    printf("%s: Error: can't execute pingToHost command!\n", PPREFIX());
	    return;
	}
	bb->initialReadBufferSize = (1<<10);
	bb->minFreeSizeAtEndOfReadBuffer = 2;
	// printf("%s: Executing %s\n", PPREFIX(), ttt);fflush(stdout);
	callBackAddToHook(&bb->callBackOnRead, (callBackHookFunArgType) pingToHostOnRead);
	pingToHostBaioMagic = bb->baioMagic;
    }
}

void pingToHostRegularCheck(void *d) {
    // we have to receive ping answer within 2 seconds
    if (pingToHostLastAnswerTime < currentTime.dtime - 3.0) {
	// problem. We have lost the connection.
	lprintf(0, "%s: Error: connection to host %s lost. Immediate landing !!!\n", PPREFIX(), uu->pingToHost);
	if (uu->flyStage >= FS_FLY && uu->flyStage < FS_EMERGENCY_LANDING) uu->flyStage = FS_EMERGENCY_LANDING;
    }
    timeLineInsertEvent(UTIME_AFTER_MSEC(500), pingToHostRegularCheck, d);
}

void pingToHostClose() {
    baioCloseMagic(pingToHostBaioMagic);
    pingToHostBaioMagic = 0;
}

/////////////////////////////////////////////////////////////////////////////////////////

void *createSharedMemory(int size, char *namefmt, ...) {
    int 	fd, r;
    char	name[TMP_STRING_SIZE];
    va_list	ap;
    void	*res;

    // DEBUG_HERE_I_AM();
    
    va_start(ap, namefmt);
    vsnprintf(name, TMP_STRING_SIZE-1, namefmt, ap);
    //printf("%s: Info: Creating shared memory %s\n", PPREFIX(), name); fflush(stdout);
    lprintf(1, "%s: Info: Creating shared memory %s\n", PPREFIX(), name);
    va_end(ap);

    fd = shm_open(name, O_CREAT | O_RDWR, S_IRWXU);
    if (fd == -1) {
	lprintf(0, "%s: Error: Can't open shared memory %s: %s\n", PPREFIX(), name, strerror(errno));
	return(NULL);
    }
    if (size > 0) {
	r = ftruncate(fd, size);
	if (r == -1) {
	    lprintf(0, "%s: Error: Can't truncate shared memory %s: %s\n", PPREFIX(), name, strerror(errno));
	    close(fd);
	    return(NULL);
	}
    }
    res = mmap(NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    close(fd);
    
    if (res == NULL) {
	lprintf(0, "%s: Error: Can't mmap shared memory %s: %s\n", PPREFIX(), name, strerror(errno));
	return(NULL);
    }
    return(res);
}

struct raspilotInputBuffer *raspilotCreateSharedMemory(struct deviceStreamData *ddd) {
    struct raspilotInputBuffer 	*res;
    int				len;

    len = RASPILOT_INPUT_BUFFER_SIZE(ddd->regression_size, deviceDataStreamVectorLength[ddd->type]);
    res = createSharedMemory(len, "raspilot.%s.%s", ddd->dd->name, ddd->name);
    if (res == NULL) return(NULL);
    res->buffer.vectorsize = deviceDataStreamVectorLength[ddd->type];
    res->buffer.size = ddd->regression_size;
    res->status = RIBS_SHARED_INITIALIZE;
    res->magicVersion = RASPILOT_SHM_MAGIC_VERSION;
    res->confidence = 0;
    return(res);
}

