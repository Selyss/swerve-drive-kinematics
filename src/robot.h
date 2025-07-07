#include <cmath>
#include <vector>
#include "swerveModule.h"

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

    void update();
    void drive(float tvx, float tvy, float tOmega);

    float getX() const;
    float getY() const;
    float getTheta() const;
};