#include "robot.h"

Robot::Robot(float dim) : dim(dim)
{
    topLeftPosition = std::make_pair(dim / 2, dim / 2);
    topRightPosition = std::make_pair(-dim / 2, dim / 2);
    bottomLeftPosition = std::make_pair(dim / 2, -dim / 2);
    bottomRightPosition = std::make_pair(-dim / 2, -dim / 2);
}

float Robot::getX() const
{
    return (topLeftPosition.first + topRightPosition.first + bottomLeftPosition.first + bottomRightPosition.first) / 4.0f;
}

float Robot::getY() const
{
    return (topLeftPosition.second + topRightPosition.second + bottomLeftPosition.second + bottomRightPosition.second) / 4.0f;
}

float Robot::getTheta() const
{
    return atan2f((topLeftPosition.second - bottomLeftPosition.second), (topLeftPosition.first - bottomLeftPosition.first));
}

void Robot::drive(float tvx, float tvy, float tOmega)
{
    float topLeftSpeed, topRightSpeed, bottomLeftSpeed, bottomRightSpeed;
    float topLeftTheta, topRightTheta, bottomLeftTheta, bottomRightTheta;
    float maxSpeed;

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

void Robot::update()
{
    topLeftModule.update();
    topRightModule.update();
    bottomLeftModule.update();
    bottomRightModule.update();
}