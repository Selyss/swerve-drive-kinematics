#include <cmath>
#include <vector>
#include "swerveModule.h"

class Robot
{
private:
    float dim;
    float vx = 0.0f;
    float vy = 0.0f;
    float omega = 0.0f;

    // position and orientations
    float x = 0.0f;
    float y = 0.0f;
    float theta = 0.0f;

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