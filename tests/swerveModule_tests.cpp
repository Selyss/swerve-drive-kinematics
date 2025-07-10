#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <iostream>
#include "../src/robot.h"

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

TEST_CASE("Swerve modules are 90 degrees apart when rotating in place", "[robot][swerve]")
{
    // 1m square
    Robot robot(1.0f);

    // rotate in place, rotational velocity of 1
    robot.drive(0.0f, 0.0f, 1.0f);

    // update over dt = 1s
    robot.update();

    float tl = robot.getTopLeftModule().getSteerOutput();
    float tr = robot.getTopRightModule().getSteerOutput();
    float bl = robot.getBottomLeftModule().getSteerOutput();
    float br = robot.getBottomRightModule().getSteerOutput();

    std::cout << "TL: " << tl << " TR: " << tr << " BL: " << bl << " BR: " << br << std::endl;

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
    REQUIRE(angle_diff(tl, tr));
    REQUIRE(angle_diff(tr, br));
    REQUIRE(angle_diff(br, bl));
    REQUIRE(angle_diff(bl, tl));
}