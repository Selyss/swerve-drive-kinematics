#ifndef SWERVE_MODULE_H
#define SWERVE_MODULE_H

#include <cmath>
#include <algorithm>
#include "pid.h"

class SwerveModule
{
private:
    float cTheta, cVelocity;
    float tTheta, tSpeed;

    // PID
    PID steerPID;
    PID drivePID;

    float steerMotorCommand;
    float driveMotorCommand;

    void normalizeAngle();
    void optimizeTarget();
    float shortestAngleDiff(float a, float b);

public:
    SwerveModule();
    SwerveModule(float steerP, float steerI, float steerD, float driveP, float driveI, float driveD);

    void setTarget(float angle, float speed);
    void update(float dt);

    void setCurrentAngle(float angle);
    void setCurrentVelocity(float velocity);

    float getSteerMotorCommand() const;
    float getDriveMotorCommand() const;

    float getDriveOutput() const;
    float getSteerOutput() const;
};

#endif // SWERVE_MODULE_H