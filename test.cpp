#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include "src/robot.h"

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
