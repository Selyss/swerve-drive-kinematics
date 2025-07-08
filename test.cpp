#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include "src/robot.h"

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
    REQUIRE(robot.getTheta() == Catch::Approx(1.0f));
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
    REQUIRE(robot.getTheta() == Catch::Approx(1.0f));
}

TEST_CASE("Robot accumulates movement over multiple updates", "[robot]")
{
    Robot robot(1.0f);
    robot.drive(1.0f, 0.0f, 0.5f);
    robot.update(1.0f);
    robot.update(1.0f);

    REQUIRE(robot.getX() == Catch::Approx(std::cos(0.0f) + std::cos(0.5f)));
    REQUIRE(robot.getY() == Catch::Approx(std::sin(0.0f) + std::sin(0.5f)));
    REQUIRE(robot.getTheta() == Catch::Approx(1.0f));
}

TEST_CASE("Robot moves backward and rotates negatively", "[robot]")
{
    Robot robot(1.0f);
    robot.drive(-1.0f, 0.0f, -1.0f);
    robot.update();

    REQUIRE(robot.getX() == Catch::Approx(-1.0f));
    REQUIRE(robot.getTheta() == Catch::Approx(-1.0f));
}

TEST_CASE("Robot theta wraps correctly", "[robot]")
{
    Robot robot(1.0f);
    robot.drive(0.0f, 0.0f, 2.0f);
    robot.update(1.0f); // theta should be 2.0, but should wrap

    REQUIRE(robot.getTheta() == Catch::Approx(0.0f));
}
