#include "swerveModule.h"

SwerveModule::SwerveModule() : cTheta(0.0), cVelocity(0.0), tTheta(0.0), tSpeed(0.0) {}

void SwerveModule::setTarget(float angle, float speed)
{
    // validate later lol
    tTheta = angle; // radians?
    tSpeed = speed;
}

float SwerveModule::normalizeAngle(float angle)
{
    float normalizedAngle = fmod(angle + 1, 2);
    if (normalizedAngle <= 0)
    {
        normalizedAngle += 2;
    }
    normalizedAngle -= 1;
    return normalizedAngle;
}

float SwerveModule::shortestAngleDiff(float a, float b)
{
    return std::abs(a - b) < std::abs(b - a) ? a - b : b - a;
}

void SwerveModule::optimizeTarget()
{
    if (std::abs(tTheta - cTheta) > 0.5)
    {
        tSpeed = -tSpeed;
        tTheta = -tTheta;
    }
}