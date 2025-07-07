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

void Robot::drive(float tvx, float tvy, float tOmega)
{
    float topLeftSpeed, topRightSpeed, bottomLeftSpeed, bottomRightSpeed;
    float topLeftTheta, topRightTheta, bottomLeftTheta, bottomRightTheta;
    float maxSpeed;

    vx = tvx;
    vy = tvy;
    omega = tOmega;

    topLeftSpeed = std::sqrt((tvx - (tOmega * topLeftPosition.second)) * (tvx - (tOmega * topLeftPosition.second)) + (tvy + (tOmega * topLeftPosition.first)) * (tvy + (tOmega * topLeftPosition.first)));
    topLeftTheta = atan2f((tvy + (tOmega * topLeftPosition.first)), (tvx - (tOmega * topLeftPosition.second)));

    topRightSpeed = std::sqrt((tvx - (tOmega * topRightPosition.second)) * (tvx - (tOmega * topRightPosition.second)) + (tvy + (tOmega * topRightPosition.first)) * (tvy + (tOmega * topRightPosition.first)));
    topRightTheta = atan2f((tvy + (tOmega * topRightPosition.first)), (tvx - (tOmega * topRightPosition.second)));

    bottomLeftSpeed = std::sqrt((tvx - (tOmega * bottomLeftPosition.second)) * (tvx - (tOmega * bottomLeftPosition.second)) + (tvy + (tOmega * bottomLeftPosition.first)) * (tvy + (tOmega * bottomLeftPosition.first)));
    bottomLeftTheta = atan2f((tvy + (tOmega * bottomLeftPosition.first)), (tvx - (tOmega * bottomLeftPosition.second)));

    bottomRightSpeed = std::sqrt((tvx - (tOmega * bottomRightPosition.second)) * (tvx - (tOmega * bottomRightPosition.second)) + (tvy + (tOmega * bottomRightPosition.first)) * (tvy + (tOmega * bottomRightPosition.first)));
    bottomRightTheta = atan2f((tvy + (tOmega * bottomRightPosition.first)), (tvx - (tOmega * bottomRightPosition.second)));

    maxSpeed = std::max(std::max(topLeftSpeed, topRightSpeed), std::max(bottomLeftSpeed, bottomRightSpeed));

    if (maxSpeed > 1.0)
    {
        topLeftSpeed = topLeftSpeed / maxSpeed;
        topRightSpeed = topRightSpeed / maxSpeed;
        bottomLeftSpeed = bottomLeftSpeed / maxSpeed;
        bottomRightSpeed = bottomRightSpeed / maxSpeed;
    }

    topLeftModule.setTarget(topLeftTheta, topLeftSpeed);
    topRightModule.setTarget(topRightTheta, topLeftSpeed);
    bottomLeftModule.setTarget(bottomLeftTheta, topLeftSpeed);
    bottomRightModule.setTarget(bottomRightTheta, topLeftSpeed);
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
    float cosTheta = cos(theta);
    float sinTheta = sin(theta);

    // convert body-frame velocities to world-frame
    float dx = vx * cosTheta - vy * sinTheta;
    float dy = vx * sinTheta + vy * cosTheta;

    x += dx * dt;
    y += dy * dt;
    theta += omega * dt;

    // keep theta within [-pi, pi]. the pi is not included in the range, we add it at the end.
    // if something doesnt work its probably because of this section...
    if (theta > 1.0f)
        theta -= 2.0f;
    if (theta < -1.0f)
        theta += 2.0f;

    topLeftModule.update();
    topRightModule.update();
    bottomLeftModule.update();
    bottomRightModule.update();
}