#include <swerveModule.h>
#include <robot.h>
#include <unity.h>

// windows does not like M_PI
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

SwerveModule module;
Robot robot(1.0f);

void setUp(void)
{
    // runs before each test
    robot = Robot(1.0f);
    module = SwerveModule();
}

void tearDown(void)
{
    // runs after each test
}

void test_robot_initial_state(void)
{
    TEST_ASSERT_EQUAL_FLOAT(0.0f, robot.getX());
    TEST_ASSERT_EQUAL_FLOAT(0.0f, robot.getY());
    TEST_ASSERT_EQUAL_FLOAT(0.0f, robot.getTheta());
}

void test_robot_moves_forward(void)
{
    robot.drive(1.0f, 0.0f, 0.0f);
    robot.update();

    TEST_ASSERT_EQUAL_FLOAT(1.0f, robot.getX());
    TEST_ASSERT_EQUAL_FLOAT(0.0f, robot.getY());
    TEST_ASSERT_EQUAL_FLOAT(0.0f, robot.getTheta());
}

void test_robot_rotates_in_place(void)
{
    robot.drive(0.0f, 0.0f, 1.0f);
    robot.update();

    TEST_ASSERT_EQUAL_FLOAT(0.0f, robot.getX());
    TEST_ASSERT_EQUAL_FLOAT(0.0f, robot.getY());
    TEST_ASSERT_EQUAL_FLOAT(1.0f / M_PI, robot.getTheta());
}

void test_robot_moves_diagonally(void)
{
    robot.drive(1.0f, 1.0f, 0.0f);
    robot.update();

    TEST_ASSERT_EQUAL_FLOAT(1.0f, robot.getX());
    TEST_ASSERT_EQUAL_FLOAT(1.0f, robot.getY());
    TEST_ASSERT_EQUAL_FLOAT(0.0f, robot.getTheta());
}

void test_robot_moves_and_rotates(void)
{
    robot.drive(1.0f, 1.0f, 1.0f);
    robot.update();

    TEST_ASSERT_EQUAL_FLOAT(1.0f, robot.getX());
    TEST_ASSERT_EQUAL_FLOAT(1.0f, robot.getY());
    TEST_ASSERT_EQUAL_FLOAT(1.0f / M_PI, robot.getTheta());
}

void test_robot_accumulates_movement_over_updates(void)
{
    robot.drive(1.0f, 0.0f, 0.5f);
    robot.update(1.0f);
    robot.update(1.0f);

    // The robot integrates body-frame velocity into world-frame position.
    // After first update: theta = 0.5, x = cos(0.0) = 1.0, y = sin(0.0) = 0.0
    // After second update: theta = 1.0, x += cos(0.5), y += sin(0.5)
    float x_expected = std::cos(0.0f) + std::cos(0.5f);
    float y_expected = std::sin(0.0f) + std::sin(0.5f);
    TEST_ASSERT_EQUAL_FLOAT(x_expected, robot.getX());
    TEST_ASSERT_EQUAL_FLOAT(y_expected, robot.getY());
    TEST_ASSERT_EQUAL_FLOAT(1.0f / M_PI, robot.getTheta());
}

void test_robot_moves_backwards_and_rotates_negatively(void)
{
    robot.drive(-1.0f, 0.0f, -1.0f);
    robot.update();

    TEST_ASSERT_EQUAL_FLOAT(-1.0f, robot.getX());
    TEST_ASSERT_EQUAL_FLOAT(-1.0f / M_PI, robot.getTheta());
}

void test_theta_wraps_correctly(void)
{
    robot.drive(0.0f, 0.0f, 2.0f * M_PI);
    robot.update(1.0f); // theta should be 2.0, but should wrap

    TEST_ASSERT_EQUAL_FLOAT(0.0f, robot.getTheta());
}

void test_robot_sequential_drive_commands(void)
{
    robot.drive(1.0f, 0.0f, 0.0f);
    robot.update();
    robot.drive(0.0f, 1.0f, 0.0f);
    robot.update();

    TEST_ASSERT_EQUAL_FLOAT(1.0f, robot.getX());
    TEST_ASSERT_EQUAL_FLOAT(1.0f, robot.getY());
}

void test_robot_stops(void)
{
    robot.drive(1.0f, 0.0f, 0.0f);
    robot.update();
    robot.drive(0.0f, 0.0f, 0.0f);
    robot.update();

    TEST_ASSERT_EQUAL_FLOAT(1.0f, robot.getX());
    TEST_ASSERT_EQUAL_FLOAT(0.0f, robot.getY());
}

// SWERVE MODULE TESTS

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
    RUN_TEST(test_robot_initial_state);
    RUN_TEST(test_robot_moves_forward);
    RUN_TEST(test_robot_rotates_in_place);
    RUN_TEST(test_robot_moves_diagonally);
    RUN_TEST(test_robot_moves_and_rotates);
    RUN_TEST(test_robot_accumulates_movement_over_updates);
    RUN_TEST(test_robot_moves_backwards_and_rotates_negatively);
    RUN_TEST(test_theta_wraps_correctly);
    RUN_TEST(test_robot_sequential_drive_commands);
    RUN_TEST(test_robot_stops);

    // Swerve Module Tests

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