#include <swerveModule.h>
#include <robot.h>
#include <unity.h>

// windows does not like M_PI
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

Robot robot(1.0f);

void setUp(void) {
    // runs before each test
    robot = Robot(1.0f);
}

void tearDown(void) {
    // runs after each test
}


void test_robot_initial_state(void) {
    TEST_ASSERT_EQUAL_FLOAT(0.0f, robot.getX());
    TEST_ASSERT_EQUAL_FLOAT(0.0f, robot.getY());
    TEST_ASSERT_EQUAL_FLOAT(0.0f, robot.getTheta());
}


int main(int argc, char **argv)
{
    UNITY_BEGIN();
    RUN_TEST(test_robot_initial_state);
    UNITY_END();
    


    return 0;
}