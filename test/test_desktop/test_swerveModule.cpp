#include <robot.h>
#include <unity.h>
#include <swerveModule.h>

SwerveModule module;
Robot robot;

void setUp(void)
{
    // runs before each test
    module = SwerveModule();
    robot = Robot(1.0f);
}

void tearDown(void)
{
    // runs after each test
}

void test_swerveModule_initialization(void)
{
    TEST_ASSERT_EQUAL_FLOAT(0.0f, module.getDriveOutput());
    TEST_ASSERT_EQUAL_FLOAT(0.0f, module.getSteerOutput());
}

void test_setTarget_sets_output(void)
{
    module.setTarget(0.5f, 0.8f);
    TEST_ASSERT_EQUAL_FLOAT(0.5f, module.getSteerOutput());
    TEST_ASSERT_EQUAL_FLOAT(0.8f, module.getDriveOutput());
}

void test_normalize_angle(void)
{
    module.setTarget(2.5f, 1.0f); // outside [-1, 1]
    module.update();
    TEST_ASSERT_LESS_OR_EQUAL_FLOAT(1.0f, module.getSteerOutput());
    TEST_ASSERT_GREATER_OR_EQUAL_FLOAT(-1.0f, module.getSteerOutput());
}

void test_optimize_target(void)
{
    module.setCurrentAngle(0.0f);
    module.setTarget(1.0f, 1.0f); // 180 degrees away in normalized [-1, 1] space
    module.update();
    // if optimized, drive should be reversed
    TEST_ASSERT_EQUAL_FLOAT(-1.0f, module.getDriveOutput());
}

void test_setCurrentAngle_affects_steer(void)
{
    module.setCurrentAngle(0.5f);
    module.setTarget(0.8f, 1.0f);
    module.update();
    TEST_ASSERT_EQUAL_FLOAT(0.8f, module.getSteerOutput());
}

void test_handles_negative_speed(void)
{
    module.setTarget(0.5f, -1.0f);
    module.update();
    TEST_ASSERT_EQUAL_FLOAT(-1.0f, module.getDriveOutput());
}

void test_normalizes_angle_at_boundary(void)
{
    module.setTarget(-1.1f, 1.0f);
    module.update();
    TEST_ASSERT_LESS_OR_EQUAL_FLOAT(module.getSteerOutput(), -1.0f);
    TEST_ASSERT_LESS_OR_EQUAL_FLOAT(1.0f, module.getSteerOutput());
}

void test_repeated_updates(void)
{
    module.setTarget(0.3f, 0.7f);
    for (int i = 0; i < 10; ++i)
    {
        module.update();
    }
    TEST_ASSERT_EQUAL_FLOAT(0.3f, module.getSteerOutput());
    TEST_ASSERT_EQUAL_FLOAT(0.7f, module.getDriveOutput());
}

void test_modules_are_ninety_deg_apart_when_rotating(void)
{
    // rotate in place, rotational velocity of 1
    robot.drive(0.0f, 0.0f, 1.0f);

    // update over dt = 1s
    robot.update();

    float tl = robot.getTopLeftModule().getSteerOutput();
    float tr = robot.getTopRightModule().getSteerOutput();
    float bl = robot.getBottomLeftModule().getSteerOutput();
    float br = robot.getBottomRightModule().getSteerOutput();

    // Check that each module is 90 degrees (0.5 in normalized [-1,1] range), allowing for wrapping
    auto angle_diff = [](float a, float b)
    {
        float diff = a - b;
        while (diff > 1.0f)
            diff -= 2.0f;
        while (diff < -1.0f)
            diff += 2.0f;
        return std::fabs(std::fabs(diff) - 0.5f) < 0.01f;
    };

    // FIXME: ? does this work
    TEST_ASSERT_TRUE(angle_diff(tl, tr));
    TEST_ASSERT_TRUE(angle_diff(tr, br));
    TEST_ASSERT_TRUE(angle_diff(br, bl));
    TEST_ASSERT_TRUE(angle_diff(bl, tl));
}

int main(int argc, char **argv)
{
    UNITY_BEGIN();
    RUN_TEST(test_swerveModule_initialization);
    RUN_TEST(test_setTarget_sets_output);
    RUN_TEST(test_normalize_angle);
    RUN_TEST(test_optimize_target);
    RUN_TEST(test_setCurrentAngle_affects_steer);
    RUN_TEST(test_handles_negative_speed);
    RUN_TEST(test_normalizes_angle_at_boundary);
    RUN_TEST(test_repeated_updates);
    RUN_TEST(test_modules_are_ninety_deg_apart_when_rotating);
    UNITY_END();

    return 0;
}