
/////////////////////////////////////////////////////////////

#define MOTOR_MAX		64

// each motor implementation have to implement following three
// functions

void motorImplementationInitialize(int motorPins[], int motorMax);
void motorImplementationFinalize(int motorPins[], int motorMax);
void motorImplementationSendThrottles(int motorPins[], int motorMax, double motorThrottle[]);




