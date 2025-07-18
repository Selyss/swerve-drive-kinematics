#include "swerveModule.h"

SwerveModule::SwerveModule() : cTheta(0.0), cVelocity(0.0), tTheta(0.0), tSpeed(0.0) {}

/**
 * @brief Get the drive output for the swerve module.
 *
 * @return The target speed (normalized).
 */
float SwerveModule::getDriveOutput() const
{
    return tSpeed;
}

/**
 * @brief Get the steering output for the swerve module.
 *
 * @return The target angle in radians, normalized to the range [-1, 1].
 */
float SwerveModule::getSteerOutput() const
{
    return tTheta;
}

/**
 * @brief Set the target angle and speed for the swerve module.
 *
 * @param angle The target angle in radians, normalized to the range [-1, 1].
 * @param speed The target speed (normalized).
 */
void SwerveModule::setTarget(float angle, float speed)
{
    // validate later lol
    tTheta = angle; // radians?
    tSpeed = speed;
}

/**
 * @brief Set the current angle of the swerve module.
 *
 * @param angle The current angle in radians, normalized to the range [-1, 1].
 */
void SwerveModule::setCurrentAngle(float angle) // FIXME: inconsistent with setTarget
{
    cTheta = angle;
}

/**
 * @brief Normalize the target angle and current angle to the range [-1, 1].
 */
void SwerveModule::normalizeAngle()
{
    // Branchless normalization to [-1, 1)
    tTheta += 1.0f;
    tTheta -= 2.0f * std::floor(tTheta * 0.5f);
    tTheta -= 1.0f;

    cTheta += 1.0f;
    cTheta -= 2.0f * std::floor(cTheta * 0.5f);
    cTheta -= 1.0f;
}

/**
 * @brief Calculate the shortest angle difference between two angles.
 *
 * @param a First angle in radians.
 * @param b Second angle in radians.
 *
 * @return The shortest angle difference in radians.
 */
inline float SwerveModule::shortestAngleDiff(float a, float b)
{
    float diff = a - b;

    diff -= 2.0f * std::round(diff * 0.5f);
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