#include "pid.h"

#include "pid.h"
#include <algorithm>

PID::PID(float p, float i, float d, float integralLimit)
    : kP(p), kI(i), kD(d), integral(0.0f), previousError(0.0f), integralMax(integralLimit) {}

float PID::calculate(float setpoint, float measurement, float dt)
{
    float error = setpoint - measurement;

    // proportional term
    float proportional = kP * error;

    // integral term (accumulated error over time)
    integral += error * dt;
    integral = std::clamp(integral, -integralMax, integralMax);
    float integralTerm = kI * integral;

    // derivative term (rate of change of error)
    float derivative = (error - previousError) / dt;
    float derivativeTerm = kD * derivative;

    previousError = error;

    return proportional + integralTerm + derivativeTerm;
}

void PID::reset()
{
    integral = 0.0f;
    previousError = 0.0f;
}