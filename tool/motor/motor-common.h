
/////////////////////////////////////////////////////////////

#define MOTOR_MAX		64

// motor-common.c exports a variable indicating debug level
extern int debug;

// each motor implementation have to implement following three
// functions

void motorImplementationInitialize(int motorPins[], int motorMax);
void motorImplementationFinalize(int motorPins[], int motorMax);
void motorImplementationSet3dModeAndSpinDirection(int motorPins[], int motorMax, int mode3dFlag, int reverseDirectionFlag) ;
void motorImplementationSendThrottles(int motorPins[], int motorMax, double motorThrottle[]);




