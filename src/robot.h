#include <vector>
#include "swerveModule.h"

class Robot
{
private:
    float dim;
    int vx, vy, omega;
    SwerveModule trModule, tlModule, brModule, blModule;
    std::pair<float, float> trPos, tlPos, brPos, blPos;

public:
    Robot(float dim);

    void update();
    void drive(float tvx, float tvy, float tOmega);
};