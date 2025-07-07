#include "robot.h"

Robot::Robot(float dim) : dim(dim)
{
    tlPos = std::make_pair(dim / 2, dim / 2);
    trPos = std::make_pair(-dim / 2, dim / 2);
    blPos = std::make_pair(dim / 2, -dim / 2);
    brPos = std::make_pair(-dim / 2, -dim / 2);
}

void Robot::drive(float tvx, float tvy, float tOmega) {
    float tlSpeed, trSpeed, blSpeed, brSpeed;
    float tlTheta, trTheta, blTheta, brTheta;
    float maxSpeed;

    tlSpeed = std::sqrt((vx - (omega * tlPos.second))*(vx - (omega * tlPos.second)) + (vy + (omega * tlPos.first))*(vy + (omega * tlPos.first)));
    tlTheta = std::atan2f((vy + (omega * tlPos.first)), (vx - (omega * tlPos.second)));

    trSpeed = std::sqrt((vx - (omega * trPos.second))*(vx - (omega * trPos.second)) + (vy + (omega * trPos.first))*(vy + (omega * trPos.first)));
    trTheta = std::atan2f((vy + (omega * trPos.first)), (vx - (omega * trPos.second)));

    blSpeed = std::sqrt((vx - (omega * blPos.second))*(vx - (omega * blPos.second)) + (vy + (omega * blPos.first))*(vy + (omega * blPos.first)));
    blTheta = std::atan2f((vy + (omega * blPos.first)), (vx - (omega * blPos.second)));

    brSpeed = std::sqrt((vx - (omega * brPos.second))*(vx - (omega * brPos.second)) + (vy + (omega * brPos.first))*(vy + (omega * brPos.first)));
    brTheta = std::atan2f((vy + (omega * brPos.first)), (vx - (omega * brPos.second)));

    maxSpeed = std::max(std::max(tlSpeed, trSpeed), std::max(blSpeed, brSpeed));

    if (maxSpeed > 1.0) {
        tlSpeed = tlSpeed / maxSpeed;
        trSpeed = trSpeed / maxSpeed;
        blSpeed = blSpeed / maxSpeed;
        brSpeed = brSpeed / maxSpeed;
    }

    tlModule.setTarget(tlTheta, tlSpeed);
    trModule.setTarget(trTheta, tlSpeed);
    blModule.setTarget(blTheta, tlSpeed);
    brModule.setTarget(brTheta, tlSpeed);
}