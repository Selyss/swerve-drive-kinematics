#ifndef PID_H
#define PID_H

#include <algorithm>

class PID
{
private:
    float kP, kI, kD;
    float integral;
    float previousError;
    float integralMax;

public:
    PID(float p, float i, float d, float integralLimit = 1.0f);
    float calculate(float setpoint, float measurement, float dt);
    void reset();
};

#endif // PID_H