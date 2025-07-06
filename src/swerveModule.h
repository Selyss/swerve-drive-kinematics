#include <cmath>
#include <algorithm>

class SwerveModule
{
private:
    float cTheta, cVelocity;
    float tTheta, tSpeed;

    void normalizeAngle();
    void optimizeTarget();
    float shortestAngleDiff(float a, float b);

public:
    SwerveModule();

    void setTarget(float angle, float speed);
    void update();
    float getDriveOutput() const;
    float getSteerOutput() const;
};