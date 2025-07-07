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

/**
 * @brief Normalize the target angle and current angle to the range [-1, 1].
 */
void SwerveModule::normalizeAngle()
{
    // Normalize target angle
    float normalizedAngle = fmod(tTheta + 1, 2);
    if (normalizedAngle <= 0)
    {
        normalizedAngle += 2;
    }
    normalizedAngle -= 1;
    tTheta = normalizedAngle;

    // Normalize current angle
    float normalizedAngle = fmod(cTheta + 1, 2);
    if (normalizedAngle <= 0)
    {
        normalizedAngle += 2;
    }
    normalizedAngle -= 1;
    tTheta = normalizedAngle;
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
    return std::abs(a - b) < std::abs(b - a) ? a - b : b - a;
}

/**
 * @brief Optimize the target angle and speed based on the current angle.
 */
void SwerveModule::optimizeTarget()
{
    float diff = shortestAngleDiff(cTheta, tTheta);

    if (std::abs(diff) > 0.5)
    {
        tSpeed = -tSpeed;
        tTheta = 1 - tTheta;
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