#include "swerveModule.h"

SwerveModule::SwerveModule() : cTheta(0.0), cVelocity(0.0), tTheta(0.0), tSpeed(0.0) {}

float SwerveModule::getDriveOutput() const
{
    return tSpeed;
}

float SwerveModule::getSteerOutput() const
{
    return tTheta;
}

void SwerveModule::setTarget(float angle, float speed)
{
    // validate later lol
    tTheta = angle; // radians?
    tSpeed = speed;
}

void SwerveModule::normalizeAngle()
{
    float normalizedAngle = fmod(tTheta + 1, 2);
    if (normalizedAngle <= 0)
    {
        normalizedAngle += 2;
    }
    normalizedAngle -= 1;
    tTheta = normalizedAngle;

    float normalizedAngle = fmod(cTheta + 1, 2);
    if (normalizedAngle <= 0)
    {
        normalizedAngle += 2;
    }
    normalizedAngle -= 1;
    tTheta = normalizedAngle;
}

// Calculates the shortest signed angle to the target
float SwerveModule::shortestAngleDiff(float a, float b)
{
    return std::abs(a - b) < std::abs(b - a) ? a - b : b - a;
}

void SwerveModule::optimizeTarget()
{
    float diff = shortestAngleDiff(cTheta, tTheta);

    if (std::abs(diff) > 0.5)
    {
        tSpeed = -tSpeed;
        tTheta = 1 - tTheta;
    }
}

void SwerveModule::update()
{
    normalizeAngle();
    optimizeTarget();
}