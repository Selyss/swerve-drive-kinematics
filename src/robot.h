#include <cmath>
#include <vector>
#include "swerveModule.h"
// #include <iostream>

class Robot
{
private:
    float dim;
    float vx;
    float vy;
    float omega;

    // position and orientations
    float x;
    float y;
    float theta;

    SwerveModule topRightModule, topLeftModule, bottomRightModule, bottomLeftModule;
    std::pair<float, float> topRightPosition, topLeftPosition, bottomRightPosition, bottomLeftPosition;

public:
    Robot(float dim);

    void update(float dt = 1.0f);
    void drive(float tvx, float tvy, float tOmega);

    float getX() const;
    float getY() const;
    float getTheta() const;

    const SwerveModule &getTopLeftModule() const { return topLeftModule; }
    const SwerveModule &getTopRightModule() const { return topRightModule; }
    const SwerveModule &getBottomLeftModule() const { return bottomLeftModule; }
    const SwerveModule &getBottomRightModule() const { return bottomRightModule; }
};