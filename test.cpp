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

/*
SWERVE MODULE TESTS
*/

TEST_CASE("SwerveModule initializes to zero", "[swerve]")
{
    SwerveModule module;
    REQUIRE(module.getDriveOutput() == Catch::Approx(0.0f));
    REQUIRE(module.getSteerOutput() == Catch::Approx(0.0f));
}

TEST_CASE("SwerveModule setTarget sets outputs", "[swerve]")
{
    SwerveModule module;
    module.setTarget(0.5f, 0.8f);
    REQUIRE(module.getSteerOutput() == Catch::Approx(0.5f));
    REQUIRE(module.getDriveOutput() == Catch::Approx(0.8f));
}

TEST_CASE("SwerveModule normalizes angles", "[swerve]")
{
    SwerveModule module;
    module.setTarget(2.5f, 1.0f); // outside [-1, 1]
    module.update();
    REQUIRE(module.getSteerOutput() <= 1.0f);
    REQUIRE(module.getSteerOutput() >= -1.0f);
}

TEST_CASE("SwerveModule optimizes target for large angle diff", "[swerve]")
{
    SwerveModule module;
    module.setCurrentAngle(0.0f);
    module.setTarget(1.0f, 1.0f); // 180 degrees away in normalized [-1,1] space
    module.update();
    // If optimized, drive should be reversed
    REQUIRE(module.getDriveOutput() == Catch::Approx(-1.0f));
}

TEST_CASE("SwerveModule setCurrentAngle affects steer output", "[swerve]")
{
    SwerveModule module;
    module.setCurrentAngle(0.5f);
    module.setTarget(0.8f, 1.0f);
    module.update();
    REQUIRE(module.getSteerOutput() == Catch::Approx(0.8f));
}

TEST_CASE("SwerveModule handles negative speed", "[swerve]")
{
    SwerveModule module;
    module.setTarget(0.5f, -1.0f);
    module.update();
    REQUIRE(module.getDriveOutput() == Catch::Approx(-1.0f));
}

TEST_CASE("SwerveModule normalizes angle at boundary", "[swerve]")
{
    SwerveModule module;
    module.setTarget(-1.1f, 1.0f);
    module.update();
    REQUIRE(module.getSteerOutput() >= -1.0f);
    REQUIRE(module.getSteerOutput() <= 1.0f);
}

TEST_CASE("SwerveModule outputs are stable over repeated updates", "[swerve]")
{
    SwerveModule module;
    module.setTarget(0.3f, 0.7f);
    for (int i = 0; i < 10; ++i)
    {
        module.update();
    }
    REQUIRE(module.getSteerOutput() == Catch::Approx(0.3f));
    REQUIRE(module.getDriveOutput() == Catch::Approx(0.7f));
}
