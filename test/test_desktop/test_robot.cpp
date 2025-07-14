#include <swerveModule.h>
#include <robot.h>
#include <unity.h>

// windows does not like M_PI
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

Robot robot(1.0f);

void setUp(void)
{
    // runs before each test
    robot = Robot(1.0f);
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
    UNITY_END();

    return 0;
}