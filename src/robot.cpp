#include "robot.h"

Robot::Robot(float dim) : dim(dim)
{
    topLeftPosition = std::make_pair(dim / 2, dim / 2);
    topRightPosition = std::make_pair(-dim / 2, dim / 2);
    bottomLeftPosition = std::make_pair(dim / 2, -dim / 2);
    bottomRightPosition = std::make_pair(-dim / 2, -dim / 2);
}

void Robot::drive(float tvx, float tvy, float tOmega)
{
    float topLeftSpeed, topRightSpeed, bottomLeftSpeed, bottomRightSpeed;
    float topLeftTheta, topRightTheta, bottomLeftTheta, bottomRightTheta;
    float maxSpeed;

    topLeftSpeed = std::sqrt((vx - (omega * topLeftPosition.second)) * (vx - (omega * topLeftPosition.second)) + (vy + (omega * topLeftPosition.first)) * (vy + (omega * topLeftPosition.first)));
    topLeftTheta = atan2f((vy + (omega * topLeftPosition.first)), (vx - (omega * topLeftPosition.second)));

    topRightSpeed = std::sqrt((vx - (omega * topRightPosition.second)) * (vx - (omega * topRightPosition.second)) + (vy + (omega * topRightPosition.first)) * (vy + (omega * topRightPosition.first)));
    topRightTheta = atan2f((vy + (omega * topRightPosition.first)), (vx - (omega * topRightPosition.second)));

    bottomLeftSpeed = std::sqrt((vx - (omega * bottomLeftPosition.second)) * (vx - (omega * bottomLeftPosition.second)) + (vy + (omega * bottomLeftPosition.first)) * (vy + (omega * bottomLeftPosition.first)));
    bottomLeftTheta = atan2f((vy + (omega * bottomLeftPosition.first)), (vx - (omega * bottomLeftPosition.second)));

    bottomRightSpeed = std::sqrt((vx - (omega * bottomRightPosition.second)) * (vx - (omega * bottomRightPosition.second)) + (vy + (omega * bottomRightPosition.first)) * (vy + (omega * bottomRightPosition.first)));
    bottomRightTheta = atan2f((vy + (omega * bottomRightPosition.first)), (vx - (omega * bottomRightPosition.second)));

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

void Robot::update()
{
    topLeftModule.update();
    topRightModule.update();
    bottomLeftModule.update();
    bottomRightModule.update();
}