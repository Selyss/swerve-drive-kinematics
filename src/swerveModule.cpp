#include "swerveModule.h"

SwerveModule::SwerveModule() : cTheta(0.0), cVelocity(0.0), tTheta(0.0), tVelocity(0.0) {}

void SwerveModule::setTarget(float angle, float speed) {
    // validate later lol
    tTheta = angle; // radians?
    tVelocity = speed;
}

float SwerveModule::normalizeAngle(float angle) {
    float normalizedAngle = fmod(angle + 1, 2);
    if (normalizedAngle <= 0) { 
        normalizedAngle += 2;
    }
    normalizedAngle -= 1;
    return normalizedAngle;
}