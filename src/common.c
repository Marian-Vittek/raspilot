#include "common.h"

int 			debugLevel;
struct universe		uuu;
struct universe		*uu = &uuu;
struct timeLineEvent    *timeLine = NULL;
uint64_t		currentTimeLineTimeUsec;
struct globalTimeInfo   currentTime;
int			shutDownInProgress = 0;
struct jsonnode 	dummyJsonNode;
int64_t 		nextStabilizationTickUsec;
int			stdbaioBaioMagic = 0;
int			trajectoryLogBaioMagic = 0;
int			pingToHostBaioMagic = 0;
double			pingToHostLastAnswerTime = 0;

// enumeration names
char 			*signalInterruptNames[258];
char 			*deviceDataTypeNames[DT_MAX+2];
char 			*coordinateSystemNames[CS_MAX+2];
char			*deviceConnectionTypeNames[DCT_MAX+2];

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
	
    res = getTemporaryStringPtrFromStaticStringRing();
    r = snprintf(res, TMP_STRING_SIZE-1, "%s: %s:%d", currentLocalTime_st(), file, line);
    for(i=r; i<40; i++) res[i] = ' ';
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
	res = 0;
    } else {
	res = normalizeToRange(omega, min, max);
    }
    return(res);
}

double truncateToRange(double x, double min, double max) {
    if (x < min) return(min);
    if (x > max) return(max);
    return(x);
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
// parsing output from devices

static int parsePong(char *tag, char *p, struct deviceData *dd, struct deviceDataData *ddd) {
    double tstamp, lat;
    
    tstamp = atof(p) / 1000000.0;
    lat = currentTime.dtime-tstamp;
    lprintf(100 - ddd->debug_level, "%s: Info: %s: latency: %g ms.\n", PPREFIX(), dd->name, lat*1000);
    // for statistics
    ddd->pongTotalTimeForStatistics += lat;
    return(0);
}

static int parseDeviceDebugPrint(char *tag, char *p, struct deviceData *dd, struct deviceDataData *ddd) {
    lprintf(100 - ddd->debug_level, "%s: Info: %s: Debug: %s\n", PPREFIX(), dd->name, p);
    return(0);
}

static int parseVector(double *rr, int length, char *tag, char *p, struct deviceData *dd, struct deviceDataData *ddd) {
    int		i;
    char 	*pe;
    double 	*v;

    lprintf( 50, "%s: parsing vector%d %s\n", PPREFIX(), length, p);

    for(i=0; i<length; i++) {
	rr[i] = strtod(p, &pe);
	if (pe == p) return(-1);
	p = pe;    
    }
    ddd->confidence = 1.0;
    lprintf(100 - ddd->debug_level, "%s: %s: %s: got %s\n", PPREFIX(), dd->name, tag, arrayWithDimToStr_st(rr, length));
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

int parseNmeaPosition(double *rr, char *tag, char *s, struct deviceData *dd, struct deviceDataData *ddd) {
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
	ddd->confidence = 0.0;
	break;
    case '1':
	// Autonomous GPS fix, no correction data used.
	ddd->confidence = 0.7;
	break;
    case '2':
	// DGPS fix, using a local DGPS base station or correction service such as WAAS or EGNOS.
	ddd->confidence = 0.8;
	break;
    case '3':
	// PPS fix ???
	ddd->confidence = 0.6;
	break;
    case '4':
	// RTK fix, high accuracy Real Time Kinematic.
	ddd->confidence = 1.0;
	break;
    case '5':
	// RTK Float, better than DGPS, but not quite RTK.
	ddd->confidence = 0.9;
	break;
    case '6':
	// Estimated fix (dead reckoning).
	ddd->confidence = 0.5;
	break;
    default:
	// We do not know what
	ddd->confidence = 0.3;
	break;
    }

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
    lprintf(100 - ddd->debug_level, "%s: %s: %s: got %s\n", PPREFIX(), dd->name, tag, arrayWithDimToStr_st(rr, 3));
    return(0);
}

// Joystick lines look like: Event: type 2, time 8695440, number 0, value 6808
int parseJstestJoystickSetWaypoint(double *rr, char *tag, char *s, struct deviceData *dd, struct deviceDataData *ddd) {
    char 		*stype;
    char 		*snumber;
    char 		*svalue;
    int 		type, number, value;
    vec3		offset;
    
    stype = strstr(s, "type ");
    if (stype == NULL) return(-1);
    snumber = strstr(s, "number ");
    if (snumber == NULL) return(-1);
    svalue = strstr(s, "value ");
    if (svalue == NULL) return(-1);
    
    type = atoi(stype + strlen("type "));
    number = atoi(snumber + strlen("number "));
    value = atoi(svalue + strlen("value "));

    if (type == 1) {
	// button click
	// For the moment evry button is emergency land
	missionLandImmediately();
    }
    
    if (type != 2) return(0);
    
    switch (number) {
    case 0:
	// In my joystick, axes 0 controls X
	offset[0] = value / 32767.0 / 10.0;
	break;
    case 1:
	// In my joystick, axes 1 controls Y
	offset[1] = - value / 32767.0 / 10.0;
	break;
    case 3:
	// In my joystick, axes 3 controls Z
	offset[2] = (32767.0 - value) / 32767.0 / 10.0;
	break;
    default:
	break;
    }

    lprintf(10, "%s: Joystick setting waypoint to: %s\n", PPREFIX(), vec3ToString_st(offset));
    // actually you can not add to the waypoint all the time, instead set it hard
    // TODO: figure this out.
    vec3_assign(uu->currentWaypoint.position, offset);
    
    return(0);
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
	if (i>=TMP_STRING_SIZE-1) return(FILE_LINE_ID_STR() ": Error");
	i += snprintf(res+i, TMP_STRING_SIZE-i-1, "%s%7.3f", separator, a[j]);
	// i += snprintf(res+i, TMP_STRING_SIZE-i-1, "%s%9.5f", separator, a[j]);
	separator = " ";
    }
    if (i>=TMP_STRING_SIZE-1) return(FILE_LINE_ID_STR() ": Error");
    i += snprintf(res+i, TMP_STRING_SIZE-i-1, "]");
    return(res);
}

//////////////////////////////////////////////////////////////////////////////////////////////////
// devices communication

static int baioLineInputDevCallBackOnDelete(struct baio *b) {
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
    /*
    if (dd->autoReconnectFlag) {
	timeLineInsertUniqEventIfNotYetInserted(UTIME_AFTER_MSEC(100), (timelineEventFunType*)baioLineIOConnectInternal, (void*)dd);
    }
    */
    return(0);
}

static int baioLineInputDevCallBackOnError(struct baio *b) {
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

int baioLineDispatchInputLine(struct deviceData *dd, char *s, int n) {	
    struct deviceDataData 		*ddd;
    struct receivedDataHistory		*hh;
    struct pose			pp;			
    struct deviceDataDataHistoryElem	*ee;
    char				*p, *tag;
    int					i, taglen, r;

    lprintf( 66, "%s: %s: Parsing line: %*.*s\n", PPREFIX(), dd->name, n, n, s);

    p = s;
    SKIP_SPACE(p);

    // TODO: maybe do it better, do some hash or something from the first identifier of the line
    // for now, there are 3,4 tags per device, so it's ok.
    for(i=0; i<dd->ddtMax; i++) {
	ddd = dd->ddt[i];
	assert(ddd != NULL);
	tag = ddd->tag;
	taglen = strlen(tag);
	if (strncmp(p, tag, taglen) == 0) {	
	    p += taglen;
	    memset(&pp, 0, sizeof(pp));
	    pp.time = currentTime.dtime - ddd->latency;
	    switch(ddd->type) {
	    case DT_VOID:
		r = 0;
		break;
	    case DT_DEBUG:
		r = parseDeviceDebugPrint(tag, p, dd, ddd);
		break;
	    case DT_PONG:
		r = parsePong(tag, p, dd, ddd);
		break;
	    case DT_POSITION_VECTOR:
		r = parseVector(&pp.pr[0], 3, tag, p, dd, ddd);
		break;
	    case DT_ORIENTATION_RPY_COMPASS:
		r = parseVector(&pp.pr[7], 3, tag, p, dd, ddd);
		// apply mount correction
		vec3_sub(&pp.pr[7], &pp.pr[7], dd->mount_rpy);
		if (r == 0) yprToQuat(pp.pr[9], pp.pr[8], pp.pr[7], &pp.pr[3]);
		break;		    
	    case DT_ORIENTATION_QUATERNION:
		// read as quaternion, store both as quaternion and roll-pitch-yaw
		r = parseVector(&pp.pr[3], 4, tag, p, dd, ddd);
		if (r == 0) {
		    // Add mount RPY.
		    // TODO: maybe I shall convert mount rpy to quaternion and multipy quats here!
		    quatToYpr(&pp.pr[3], &pp.pr[9], &pp.pr[8], &pp.pr[7]);
		    // lprintf(1, "%s: debug got orientation rpy %s\n", PPREFIX(), vec3ToString_st(&pp.pr[7]));
		    // apply mount correction
		    vec3_sub(&pp.pr[7], &pp.pr[7], dd->mount_rpy);
		    // lprintf(1, "%s: debug after sub mount %s\n", PPREFIX(), vec3ToString_st(&pp.pr[7]));
		    yprToQuat(pp.pr[9], pp.pr[8], pp.pr[7], &pp.pr[3]);
		}
		break;
	    case DT_POSITION_NMEA:
		r = parseNmeaPosition(&pp.pr[0], tag, p, dd, ddd);
		break;
	    case DT_MAGNETIC_HEADING_NMEA:
		lprintf(0, "%s: Not yet implemented\n", PPREFIX());
		break;
	    case DT_GROUND_DISTANCE:
		r = parseVector(&pp.pr[2], 1, tag, p, dd, ddd);
		break;
	    case DT_ALTITUDE:
		r = parseVector(&pp.pr[2], 1, tag, p, dd, ddd);
		break;
	    case DT_JSTEST:
		r = parseJstestJoystickSetWaypoint(&pp.pr[0], tag, p, dd, ddd);
		break;
	    default:
		if (! dd->data_ignore_unknown_tags) printf("%s: %s: Error: Tag %s not implemented!\n", PPREFIX(), dd->name, tag);
		r = -1;
	    }
	    if (r == 0) {
		ddd->totalNumberOfRecordsReceivedForStatistics ++;
		poseHistoryAddElem(&ddd->history, &pp);
	    } else {
		if (! dd->data_ignore_unknown_tags) printf("%s:  %s: Error %d when parsing line: %*.*s\n", PPREFIX(), dd->name, r, n, n, s);
	    }
	    return(r);
	}
    }
    lprintf( 22, "%s:  %s: Warning: No data tag found in line: %*.*s\n", PPREFIX(), dd->name, n, n, s);
    return(0);
}

static int baioLineInputDevCallBackOnRead(struct baio *bb, int fromj, int num) {
    int				i, ii;
    struct deviceData 		*dd;
    struct baioBuffer		*b;

    lprintf( 99, "%s: READ %d bytes (fromj==%d): %.*s", PPREFIX(), num, fromj, num, bb->readBuffer.b+bb->readBuffer.j-num);
    lprintf( 35, "%s: READ %d bytes\n", PPREFIX(), num);
	
    ii = bb->userParam[0].i;
    assert(ii >= 0 && ii < uu->deviceMax);
    dd = uu->device[ii];
    assert(dd != NULL);

    dd->lastActivityTime = currentTime.dtime;
    b = &bb->readBuffer;
	
    // we have n available chars starting at s[i]
    assert(b->i >= 0 && b->j >= 0 && b->i < b->size && b->j < b->size && b->i < b->j && b->size > 0);

    i = fromj;
    for(;;) {
		
	// did we read newline?
	while (i < b->j && b->b[i] != '\n') i++;
	
	// the newline was not read
	if (i >= b->j) {
	    // if the buffer is full, flush it to be able to read next possible lines
	    if (b->i == 0 && i >= b->size - bb->minFreeSizeAtEndOfReadBuffer - 1) {
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
		if (! shutDownInProgress) baioLineDispatchInputLine(dd, &b->b[b->i], i - b->i);
	    }
	    b->b[i] = '\n';
	}
	i = b->i = i+1;
    }
    
    //lprintf(0, "%s:%d\n", __FILE__, __LINE__);
    return(0);
}

void deviceInitiate(int i) {
    struct deviceData 	*dd;
    struct baio 	*bb;
    
    dd = uu->device[i];
    dd->enabled = 0;
    dd->baioMagic = 0;

    switch (dd->connection.type) {
    case DCT_INTERNAL_ZEROPOSE:
    case DCT_INTERNAL_GYROPOSE:
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
	bb = baioNewNamedPipes(dd->connection.u.pp.read_pipe, dd->connection.u.pp.write_pipe, 0);
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
    callBackAddToHook(&bb->callBackOnRead, (callBackHookFunArgType) baioLineInputDevCallBackOnRead);
    callBackAddToHook(&bb->callBackOnError, (callBackHookFunArgType) baioLineInputDevCallBackOnError);
    callBackAddToHook(&bb->callBackOnDelete, (callBackHookFunArgType) baioLineInputDevCallBackOnDelete);
    bb->userParam[0].i = i;
}

void sendToAllDevices(char *fmt, ...) {
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
    ENUM_NAME_SET(deviceDataTypeNames, DT_DEBUG);
    ENUM_NAME_SET(deviceDataTypeNames, DT_PONG);
    ENUM_NAME_SET(deviceDataTypeNames, DT_POSITION_VECTOR);
    ENUM_NAME_SET(deviceDataTypeNames, DT_GROUND_DISTANCE);
    ENUM_NAME_SET(deviceDataTypeNames, DT_ALTITUDE);
    ENUM_NAME_SET(deviceDataTypeNames, DT_ORIENTATION_RPY_COMPASS);
    ENUM_NAME_SET(deviceDataTypeNames, DT_ORIENTATION_QUATERNION);
    ENUM_NAME_SET(deviceDataTypeNames, DT_POSITION_NMEA);
    ENUM_NAME_SET(deviceDataTypeNames, DT_MAGNETIC_HEADING_NMEA);
    ENUM_NAME_SET(deviceDataTypeNames, DT_JSTEST);
    ENUM_NAME_SET(deviceDataTypeNames, DT_MAX);
    ENUM_NAME_CHECK(deviceDataTypeNames, DT_);
    
    ENUM_NAME_SET(coordinateSystemNames, CS_NONE);
    ENUM_NAME_SET(coordinateSystemNames, CS_GBASE);
    ENUM_NAME_SET(coordinateSystemNames, CS_DRONE);
    ENUM_NAME_SET(coordinateSystemNames, CS_GPS);
    ENUM_NAME_SET(coordinateSystemNames, CS_EARTH);
    ENUM_NAME_SET(coordinateSystemNames, CS_GLOBAL);
    ENUM_NAME_SET(coordinateSystemNames, CS_APRIL);
    ENUM_NAME_SET(coordinateSystemNames, CS_MAX);
    ENUM_NAME_CHECK(coordinateSystemNames, CS_);

    ENUM_NAME_SET(deviceConnectionTypeNames, DCT_NONE);
    ENUM_NAME_SET(deviceConnectionTypeNames, DCT_INTERNAL_ZEROPOSE);
    ENUM_NAME_SET(deviceConnectionTypeNames, DCT_INTERNAL_GYROPOSE);
    ENUM_NAME_SET(deviceConnectionTypeNames, DCT_COMMAND_BASH);
    ENUM_NAME_SET(deviceConnectionTypeNames, DCT_COMMAND_EXEC);
    ENUM_NAME_SET(deviceConnectionTypeNames, DCT_NAMED_PIPES);
    ENUM_NAME_SET(deviceConnectionTypeNames, DCT_MAX);
    ENUM_NAME_CHECK(deviceConnectionTypeNames, DCT_);

}

void enumNamesPrint(FILE *ff, char **names) {
    int i;
    for(i=0; names[i] != NULL; i++) fprintf(ff, "%s\"%s\"", (i==0?"":", "), names[i]);
}

int enumNamesStringToInt(char *s, char **names) {
    int i;
    for(i=0; names[i] != NULL; i++) {
	if (strcmp(s, names[i]) == 0) return(i);
    }
    return(-1);
}

//////////////////////////////////////////////////////////////////////////////////////////////

void poseHistoryPrintSums(struct poseHistory *hh) {
    int i;
    lprintf(0, "poseHistorySums: %f %f: ", hh->sumTime, hh->sumTimeSquare);
    for(i=0; i<DIM(hh->a[0].pr); i++) {
	lprintf(0, "%f %f, ",  hh->sumPr[i],hh->sumPrMulTime[i]); 
    }	
    lprintf(0, "\n");
}
void poseHistoryAddToSums(struct poseHistory *hh, struct pose *pp) {
    int 	i;
    double	tt;
    // in sums we count time - hh->sumsTimeOffset in order to avoid too much different values and
    // floating point rounding errors.
    tt = pp->time - hh->sumsTimeOffset;
    hh->sumTime += tt;
    hh->sumTimeSquare += tt * tt;
    for(i=0; i<DIM(pp->pr); i++) {
	hh->sumPr[i] += pp->pr[i];
	hh->totalSumForStatistics[i] += pp->pr[i];
	hh->sumPrMulTime[i] += pp->pr[i] * tt;
    }

#if 0
    if (fabs(pp->pr[7]) > 1.0 || fabs(pp->pr[8]) > 1.0 || fabs(pp->pr[9]) > 1.0) {
	// Hmm. why is this there?
	lprintf(0, "%s: Warning: angles in %s too large, regression for such angles not yet implemented!\n", PPREFIX(), hh->name);
    }
#endif    
}
void poseHistorySubstractFromSums(struct poseHistory *hh, struct pose *pp) {
    int 	i;
    double	tt;
    tt = pp->time - hh->sumsTimeOffset;
    hh->sumTime -= tt;
    hh->sumTimeSquare -= tt * tt;
    for(i=0; i<DIM(pp->pr); i++) {
	hh->sumPr[i] -= pp->pr[i];
	hh->sumPrMulTime[i] -= pp->pr[i] * tt;
    }
}

void poseHistoryRecalculateSums(struct poseHistory *hh) {
    int 	i,ii,j,n;

    lprintf(10, "%s: Info: %s: recomputing sums.\n", PPREFIX(), hh->name);
    hh->sumTime = 0;
    hh->sumTimeSquare = 0;
    for(i=0; i<DIM(hh->a[0].pr); i++) {
	hh->sumPr[i] = 0;
	hh->sumPrMulTime[i] = 0;
    }
    n = hh->n;
    if (n > hh->size) n = hh->size;
    //lprintf(0, "Starting: sums: "); poseHistoryPrintSums(hh);
    for(i=0; i<n; i++) {
	ii = (hh->ai + hh->size - i - 1) % hh->size;
	assert(ii >= 0 && ii < hh->size);
	poseHistoryAddToSums(hh, &hh->a[ii]);
	//lprintf(0, "Adding line %d: sums: ", ii); poseHistoryPrintSums(hh);
    }
}

void poseHistoryAddElem(struct poseHistory *hh, struct pose *pp) {
    lprintf(40, "%s: poseHistoryAddElem: %s: %f:  %s\n", PPREFIX(), hh->name, pp->time, vecToString_st(pp->pr));
    
    if (hh->n >= hh->size) poseHistorySubstractFromSums(hh, &hh->a[hh->ai]);
    memmove(&hh->a[hh->ai], pp, sizeof(struct pose));
    poseHistoryAddToSums(hh, &hh->a[hh->ai]);
    hh->n ++;
    hh->aiprev = hh->ailast;
    hh->ailast = hh->ai;
    hh->ai = (hh->ai + 1) % hh->size;
    // from time to time, recalculate sums to avoid cumulative errors due to floating point arithmetics
    if (hh->n % 10000 == 0) poseHistoryRecalculateSums(hh);
}

void poseHistoryInit(struct poseHistory *hh, int size) {
    assert(size > 0);
    memset(hh, 0, sizeof(*hh));
    hh->sumsTimeOffset = uu->pilotStartingTime;
    hh->ai = hh->ailast = hh->aiprev = 0;
    hh->n = 0;
    hh->size = size;
    ALLOCC(hh->a, size, struct pose);
    memset(hh->a, 0, size * sizeof(struct pose));
}

static void poseVectorResetQuatFromRpy(struct pose *res) {
    yprToQuat(res->pr[9], res->pr[8], res->pr[7], &res->pr[3]);
}

// res = k1 * time + k0
int poseHistoryGetRegressionCoefficients(struct poseHistory *hh, int i, double *k0, double *k1) {
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
    *k1 = ( n * hh->sumPrMulTime[i] - hh->sumTime * hh->sumPr[i] ) / d ;
    *k0 = ( hh->sumTimeSquare * hh->sumPr[i] - hh->sumTime * hh->sumPrMulTime[i] ) / d ;
    return(0);
}

void poseHistoryGetMean(struct poseHistory *hh, struct pose *res) {
    int 	i, n;

    n = hh->n;
    if (n > hh->size) n = hh->size;

    res->time = hh->sumTime / n + hh->sumsTimeOffset;
    for(i=0; i<DIM(hh->a[0].pr); i++) {
	res->pr[i] = hh->sumPr[i] / n;
    }
    quat_safe_norm(&res->pr[3], &res->pr[3]);
    // For mean, it is probably better to use quaternion and recalculate mean RPY from it
    quatToYpr(&res->pr[3], &res->pr[9], &res->pr[8], &res->pr[7]);
}

int poseHistoryEstimatePoseForTimeByLinearRegression(struct poseHistory *hh, double time, struct pose *res) {
    int 		i, j, n, r, rr;
    double 		k0, k1;
    struct pose 	mean;

#if 0
    //lprintf(0, "Starting regression %s:\n", hh->name);
    poseHistoryPrintSums(hh);
    //poseHistoryRecalculateSums(hh);
    //poseHistoryPrintSums(hh);
#endif
    
    rr = 0;
    n = DIM(hh->a[0].pr);
    res->time = time;
    for(i=0; i<n; i++) {
	r = poseHistoryGetRegressionCoefficients(hh, i, &k0, &k1);
	if (r != 0) break;
	res->pr[i] = (time - hh->sumsTimeOffset) * k1 + k0;
    }

    if (i<n) {
	// if something went wrong during computation of regression coefs return mean
	poseHistoryGetMean(hh, &mean);
	memcpy(res->pr, mean.pr, sizeof(res->pr));
	rr = -1;
    }

    
#if 0
    lprintf(0, "Doing regression, total added values %d:\n", hh->n);
    for(j=0; j<hh->size; j++) {
	lprintf(0, "%18.5f: ", hh->a[j].time);
        for(i=0; i<n; i++) {
	    lprintf(0, "%7.3f ",  hh->a[j].pr[i]);
	}
	lprintf(0, "\n");
    }
    lprintf(0, "----------------------------------------------\n");
    lprintf(0, "%18.5f: ", res->time);
    for(i=0; i<n; i++) {
	lprintf(0, "%7.3f ",  res->pr[i]);
    }
    lprintf(0, "\n");
#endif

    if (0) {
	// quaternion here: mpuYprToQuat(&res->pr[3], &res->pr[9], &res->pr[8], &res->pr[7]);
	// ! always do the translation in order to hold valid quaternion
	quat_safe_norm(&res->pr[3], &res->pr[3]);
    } else {
	// Take regression made on RPY, normalize to range <-Pi, Pi> then
	// translate RPY to quanternion
	for(i=7; i<=9; i++) res->pr[i] = normalizeToRange(res->pr[i], -M_PI, M_PI);
	poseVectorResetQuatFromRpy(res);
    }

    
#if 0
    lprintf(0, "%18.5f: ", res->time);
    for(i=0; i<n; i++) {
	lprintf(0, "%7.3f ",  res->pr[i]);
    }
    lprintf(0, "\n\n");
#endif
    
    return(rr);
}

void poseVectorAssign(struct pose *res, struct pose *a) {
    int 	i;

    res->time = a->time;
    for(i=0; i<DIM(a->pr); i++) res->pr[i] = a->pr[i];
}

void poseVectorScale(struct pose *res, struct pose *a, double factor) {
    int 	i;
    
    // simple substraction for position and angles
    res->time = a->time;
    for(i=0; i<DIM(a->pr); i++) res->pr[i] = a->pr[i] * factor;
    poseVectorResetQuatFromRpy(res);
}

void poseVectorAdd(struct pose *res, struct pose *a, struct pose *b) {
    int 	i;
    
    res->time = a->time + b->time;
    // simple substraction for position and angles
    for(i=0; i<DIM(a->pr); i++) res->pr[i] = a->pr[i] + b->pr[i];
    poseVectorResetQuatFromRpy(res);
}

void poseVectorSubstract(struct pose *res, struct pose *a, struct pose *b) {
    int 	i;
    
    res->time = a->time - b->time;
    // simple substraction for position and angles
    for(i=0; i<DIM(a->pr); i++) res->pr[i] = a->pr[i] - b->pr[i];
    poseVectorResetQuatFromRpy(res);
}

void vec3TruncateToSize(vec3 r, double size, char *warningId) {
    double s;
    
    s = vec3_len(r);
    if (s > size) {
	vec3_scale(r, r, size/s);
	if (warningId != NULL) lprintf(0, "%s: Warning: vector %s had to be truncated to size %g.\n", PPREFIX(), warningId, size);
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////

void pidControllerReset(struct pidController *pp) {
    memset(&pp->d, 0, sizeof(pp->d));
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
	}
	// Also restrict continuous growing of integral output over predefined max value.
	if (fabs(cc->i * newIntegral) >= cc->integralMax) {
	    lprintf(10, "%s: Error: PID: %s: Integral value %g grows out of range, restricting it.\n", PPREFIX(), pp->name, newIntegral);
	    newIntegral = dd->integral;
	}
	dd->integral = newIntegral;
    }

    if (PID_USES_ERROR_BASED_DERIVATIVE) {
	// The "standard" way of computing derivative.
	derivative = (error - dd->previous_error) / dt;
    } else {
	// Standard derivative is based on error - previous_error == setpoint - measured_value - previous_setpoint + previous_measured_value.
	// If setpoint does not change it gives previous_measured_value - measured_value
	// This should be a "better" alternative as it does not make peaks on changes in setpoint.
	derivative = (dd->previous_measured_value - measured_value) / dt;
    }
    output = cc->p * error + cc->i * dd->integral + cc->d * derivative + cc->ci;
    lprintf(99, "%s:PID %s: %g == %g * %g   +   %g * %g   +  %g * %g (setpoint:%f, measured_value: %f)\n", PPREFIX(), pp->name, output, cc->p, error, cc->i, dd->integral, cc->d, derivative, setpoint, measured_value);
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

void quatToYpr(quat qq, double *yaw, double *pitch, double *roll) {
    // Experiment with those two
    if (0) {
	mpuQuatToYpr(qq, yaw, pitch, roll);
    } else {
	wikiQuaternionToEulerAngles(qq, yaw, pitch, roll);
    }
}

void yprToQuat(double yaw, double pitch, double roll, quat q) {
    wikiEulerAnglesToQuaternion(yaw, pitch, roll, q);
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
	bb->initialWriteBufferSize = (1<<22);
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

int lprintf(int level, char *fmt, ...) {
    int		r;
    va_list     ap;
    struct baio	*bb;

    if (debugLevel < level) return(0);
    
    va_start(ap, fmt);
    bb = baioFromMagic(stdbaioBaioMagic);
    if (bb == NULL) {
	printf("!");
	if (strncmp(fmt, "2023-", 4) == 0) printf("%02d: ", debugLevel);
	r = vprintf(fmt, ap);
    } else {
	baioPrintfToBuffer(bb, " ");
	if (strncmp(fmt, "2023-", 4) == 0) baioPrintfToBuffer(bb, "%02d: ", debugLevel);
	r = baioVprintfToBuffer(bb, fmt, ap);
    }
    va_end(ap);
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
    // succesfull ping reports time, search for it
    p = strstr(b->b+b->i, "time=");
    if (p != NULL) pingToHostLastAnswerTime = currentTime.dtime;
    // empty read buffer
    bb->readBuffer.i = bb->readBuffer.j;
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
	callBackAddToHook(&bb->callBackOnRead, (callBackHookFunArgType) pingToHostOnRead);
	pingToHostBaioMagic = bb->baioMagic;
    }
}

void pingToHostRegularCheck(void *d) {
    // we have to receive ping answer within 2 seconds
    if (pingToHostLastAnswerTime < currentTime.dtime - 2.0) {
	// problem. We have lost the connection.
	lprintf(0, "%s: Error: connection to host %s lost. Immediate landing !!!\n", PPREFIX(), uu->pingToHost);
	raspilotShutDownAndExit();
    }
    timeLineInsertEvent(UTIME_AFTER_MSEC(500), pingToHostRegularCheck, d);
}

void pingToHostClose() {
    baioCloseMagic(pingToHostBaioMagic);
    pingToHostBaioMagic = 0;
}
