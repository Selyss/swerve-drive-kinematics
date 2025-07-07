#include <cmath>
#include <vector>
#include "swerveModule.h"

class Robot
{
private:
    float dim;
    int vx, vy, omega;
    SwerveModule topRightModule, topLeftModule, bottomRightModule, bottomLeftModule;
    std::pair<float, float> topRightPosition, topLeftPosition, bottomRightPosition, bottomLeftPosition;

public:
    Robot(float dim);

    void update();
    void drive(float tvx, float tvy, float tOmega);
};