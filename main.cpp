#include <iostream>
#include "robot.h"

int main(int argc, char *argv[])
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
}