#include <cmath>
#include <algorithm>

class SwerveModule
{
private:
    float cTheta, cVelocity;
    float tTheta, tSpeed;

    static float normalizeAngle(float angle);
    void optimizeTarget();
    float shortestAngleDiff(float a, float b);

public:
    SwerveModule();

    void setTarget(float angle, float speed);
    void update();
    float getDriveOutput() const;
    float getSteerOutput() const;
};