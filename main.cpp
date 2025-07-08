#include <iostream>
#include "robot.h"

int main(int argc, char *argv[])
{
    if (argc != 4)
    {
        std::cerr << "Usage: " << argv[0] << " <tx> <ty> <tOmega>" << std::endl;
        return 1;
    }

    float tx = std::stof(argv[1]);
    float ty = std::stof(argv[2]);
    float tOmega = std::stof(argv[3]);

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

    return 0;
}