#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include "../src/robot.h"

TEST_CASE("Robot initial state", "[robot]")
{
    Robot robot(1.0f);
    REQUIRE(robot.getX() == Catch::Approx(0.0f));
    REQUIRE(robot.getY() == Catch::Approx(0.0f));
    REQUIRE(robot.getTheta() == Catch::Approx(0.0f));
}

TEST_CASE("Robot moves forward", "[robot]")
{
    Robot robot(1.0f);
    robot.drive(1.0f, 0.0f, 0.0f);
    robot.update();

    REQUIRE(robot.getX() == Catch::Approx(1.0f));
    REQUIRE(robot.getY() == Catch::Approx(0.0f));
    REQUIRE(robot.getTheta() == Catch::Approx(0.0f));
}

TEST_CASE("Robot rotates in place", "[robot]")
{
    Robot robot(1.0f);
    robot.drive(0.0f, 0.0f, 1.0f);
    robot.update();

    REQUIRE(robot.getX() == Catch::Approx(0.0f));
    REQUIRE(robot.getY() == Catch::Approx(0.0f));
    REQUIRE(robot.getTheta() == Catch::Approx(1.0f / M_PI));
}

TEST_CASE("Robot moves diagonally", "[robot]")
{
    Robot robot(1.0f);
    robot.drive(1.0f, 1.0f, 0.0f);
    robot.update();

    REQUIRE(robot.getX() == Catch::Approx(1.0f));
    REQUIRE(robot.getY() == Catch::Approx(1.0f));
    REQUIRE(robot.getTheta() == Catch::Approx(0.0f));
}

TEST_CASE("Robot moves diagonally while rotating", "[robot]")
{
    Robot robot(1.0f);
    robot.drive(1.0f, 1.0f, 1.0f);
    robot.update();

    REQUIRE(robot.getX() == Catch::Approx(1.0f));
    REQUIRE(robot.getY() == Catch::Approx(1.0f));
    REQUIRE(robot.getTheta() == Catch::Approx(1.0f / M_PI));
}

TEST_CASE("Robot accumulates movement over multiple updates", "[robot]")
{
    Robot robot(1.0f);
    robot.drive(1.0f, 0.0f, 0.5f);
    robot.update(1.0f);
    robot.update(1.0f);

    // The robot integrates body-frame velocity into world-frame position.
    // After first update: theta = 0.5, x = cos(0.0) = 1.0, y = sin(0.0) = 0.0
    // After second update: theta = 1.0, x += cos(0.5), y += sin(0.5)
    float x_expected = std::cos(0.0f) + std::cos(0.5f); // 1.0 + cos(0.5)
    float y_expected = std::sin(0.0f) + std::sin(0.5f); // 0.0 + sin(0.5)
    REQUIRE(robot.getX() == Catch::Approx(x_expected));
    REQUIRE(robot.getY() == Catch::Approx(y_expected));
    REQUIRE(robot.getTheta() == Catch::Approx(1.0f / M_PI));
}

TEST_CASE("Robot moves backward and rotates negatively", "[robot]")
{
    Robot robot(1.0f);
    robot.drive(-1.0f, 0.0f, -1.0f);
    robot.update();

    REQUIRE(robot.getX() == Catch::Approx(-1.0f));
    REQUIRE(robot.getTheta() == Catch::Approx(-1.0f / M_PI));
}

TEST_CASE("Robot theta wraps correctly", "[robot]")
{
    Robot robot(1.0f);
    robot.drive(0.0f, 0.0f, 2.0f * M_PI);
    robot.update(1.0f); // theta should be 2.0, but should wrap

    REQUIRE(robot.getTheta() == Catch::Approx(0.0f));
}

TEST_CASE("Robot does not move with zero input", "[robot]")
{
    Robot robot(1.0f);
    robot.drive(0.0f, 0.0f, 0.0f);
    robot.update();

    REQUIRE(robot.getX() == Catch::Approx(0.0f));
    REQUIRE(robot.getY() == Catch::Approx(0.0f));
    REQUIRE(robot.getTheta() == Catch::Approx(0.0f));
}

TEST_CASE("Robot moves in both X and Y without rotation", "[robot]")
{
    Robot robot(1.0f);
    robot.drive(1.0f, 2.0f, 0.0f);
    robot.update();
    REQUIRE(robot.getX() == Catch::Approx(1.0f));
    REQUIRE(robot.getY() == Catch::Approx(2.0f));
}

TEST_CASE("Robot responds to sequential drive commands", "[robot]")
{
    Robot robot(1.0f);
    robot.drive(1.0f, 0.0f, 0.0f);
    robot.update();
    robot.drive(0.0f, 1.0f, 0.0f);
    robot.update();
    REQUIRE(robot.getX() == Catch::Approx(1.0f));
    REQUIRE(robot.getY() == Catch::Approx(1.0f));
}

TEST_CASE("Robot stops after drive set to zero", "[robot]")
{
    Robot robot(1.0f);
    robot.drive(1.0f, 0.0f, 0.0f);
    robot.update();
    robot.drive(0.0f, 0.0f, 0.0f);
    robot.update();
    REQUIRE(robot.getX() == Catch::Approx(1.0f));
    REQUIRE(robot.getY() == Catch::Approx(0.0f));
}