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

void SwerveModule::setCurrentAngle(float angle) // FIXME: inconsistent with setTarget
{
    cTheta = angle;
}

/**
 * @brief Normalize the target angle and current angle to the range [-1, 1].
 */
void SwerveModule::normalizeAngle()
{
    // Normalize target angle
    float tNormalizedAngle = fmod(tTheta + 1, 2);
    if (tNormalizedAngle <= 0)
    {
        tNormalizedAngle += 2;
    }
    tNormalizedAngle -= 1;
    tTheta = tNormalizedAngle;

    // Normalize current angle
    float cNormalizedAngle = fmod(cTheta + 1, 2);
    if (cNormalizedAngle <= 0)
    {
        cNormalizedAngle += 2;
    }
    cNormalizedAngle -= 1;
    cTheta = cNormalizedAngle;
}

/**
 * @brief Calculate the shortest angle difference between two angles.
 *
 * @param a First angle in radians.
 * @param b Second angle in radians.
 *
 * @return The shortest angle difference in radians.
 */
float SwerveModule::shortestAngleDiff(float a, float b)
{
    float diff = a - b;
    while (diff > 1.0f)
        diff -= 2.0f;
    while (diff < -1.0f)
        diff += 2.0f;
    return diff;
}

/**
 * @brief Optimize the target angle and speed based on the current angle.
 */
void SwerveModule::optimizeTarget()
{
    float diff = shortestAngleDiff(tTheta, cTheta);

    // if the difference is greater than 0.5 (half a rotation), reverse the speed and flip the angle
    if (std::abs(diff) > 0.5f)
    {
        tSpeed = -tSpeed;
        tTheta = tTheta + 1.0f;
        // normalize again after flipping
        float tNormalizedAngle = fmod(tTheta + 1, 2);
        if (tNormalizedAngle <= 0)
        {
            tNormalizedAngle += 2;
        }
        tNormalizedAngle -= 1;
        tTheta = tNormalizedAngle;
    }
}

/**
 * @brief Update the SwerveModule state by normalizing angles and optimizing the target.
 */
void SwerveModule::update()
{
    normalizeAngle();
    optimizeTarget();
}