#include "robot.h"

Robot::Robot(float dim) : dim(dim), x(0), y(0), theta(0), vx(0), vy(0), omega(0)
{
    topLeftPosition = std::make_pair(dim / 2, dim / 2);
    topRightPosition = std::make_pair(-dim / 2, dim / 2);
    bottomLeftPosition = std::make_pair(dim / 2, -dim / 2);
    bottomRightPosition = std::make_pair(-dim / 2, -dim / 2);
}

float Robot::getX() const { return x; }
float Robot::getY() const { return y; }
float Robot::getTheta() const { return theta; }

/**
 * @brief Compute the speed and angle for a swerve module based on translational and rotational velocities.
 *
 * This function calculates the speed and angle for a swerve module given the
 * translational velocities (tvx, tvy) and the rotational velocity (tOmega).
 * It uses the position of the module to compute the effective velocities.
 *
 * @param tvx The translational velocity in the x direction.
 * @param tvy The translational velocity in the y direction.
 * @param tOmega The rotational velocity (angular velocity).
 * @param pos The position of the swerve module.
 * @return A pair containing the speed and angle of the swerve module.
 */
static std::pair<float, float> computeModule(float tvx, float tvy, float tOmega, const std::pair<float, float> &pos)
{
    float speed = std::sqrt(
        std::pow(tvx - tOmega * pos.second, 2) +
        std::pow(tvy + tOmega * pos.first, 2));
    float theta = atan2f(tvy + tOmega * pos.first, tvx - tOmega * pos.second) / M_PI;
    return {speed, theta};
}

/**
 * @brief Set the target velocities for the robot's swerve modules.
 *
 * This function calculates the target speeds and angles for each swerve module
 * based on the desired translational and rotational velocities. It ensures that
 * the speeds are normalized if they exceed a maximum speed of 1.0.
 *
 * @param tvx The translational velocity in the x direction.
 * @param tvy The translational velocity in the y direction.
 * @param tOmega The rotational velocity (angular velocity).
 */
void Robot::drive(float tvx, float tvy, float tOmega)
{
    float maxSpeed;

    vx = tvx;
    vy = tvy;
    omega = tOmega;

    auto [topLeftSpeed, topLeftTheta] = computeModule(tvx, tvy, tOmega, topLeftPosition);
    auto [topRightSpeed, topRightTheta] = computeModule(tvx, tvy, tOmega, topRightPosition);
    auto [bottomLeftSpeed, bottomLeftTheta] = computeModule(tvx, tvy, tOmega, bottomLeftPosition);
    auto [bottomRightSpeed, bottomRightTheta] = computeModule(tvx, tvy, tOmega, bottomRightPosition);

    maxSpeed = std::max({topLeftSpeed, topRightSpeed, bottomLeftSpeed, bottomRightSpeed});

    if (maxSpeed > 1.0f)
    {
        topLeftSpeed = topLeftSpeed / maxSpeed;
        topRightSpeed = topRightSpeed / maxSpeed;
        bottomLeftSpeed = bottomLeftSpeed / maxSpeed;
        bottomRightSpeed = bottomRightSpeed / maxSpeed;
    }

    topLeftModule.setTarget(topLeftTheta, topLeftSpeed);
    topRightModule.setTarget(topRightTheta, topRightSpeed);
    bottomLeftModule.setTarget(bottomLeftTheta, bottomLeftSpeed);
    bottomRightModule.setTarget(bottomRightTheta, bottomRightSpeed);
}

/**
 * @brief Update the robot's state by updating each swerve module.
 *
 * This function updates the state of each swerve module, which in turn
 * updates the robot's position and orientation based on the current
 * velocities and angles of the modules.
 *
 * @param dt The time delta for the update (default is 1.0f).
 */
void Robot::update(float dt)
{
    // avoid recompute
    float cosTheta = cos(theta * M_PI);
    float sinTheta = sin(theta * M_PI);

    // convert body-frame velocities to world-frame
    float dx = vx * cosTheta - vy * sinTheta;
    float dy = vx * sinTheta + vy * cosTheta;

    x += dx * dt;
    y += dy * dt;
    // Debug: print integration step
    // std::cout << "theta: " << theta << ", omega: " << omega << ", dx: " << dx << ", x: " << x << std::endl;
    theta += omega * dt / M_PI;

    // keep theta within [-pi, pi]. the pi is not included in the range, we add it at the end.
    if (theta > 1.0f)
        theta -= 2.0f;
    if (theta < -1.0f)
        theta += 2.0f;

    topLeftModule.update();
    topRightModule.update();
    bottomLeftModule.update();
    bottomRightModule.update();
}