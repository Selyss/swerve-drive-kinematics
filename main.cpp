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

    Robot robot(1.0f); // 1m square for testing

    robot.drive(tx, ty, tOmega);
    robot.update();

    std::cout << "Robot position after applying input: "
              << "X: " << robot.getX() << ", "
              << "Y: " << robot.getY() << ", "
              << "Theta: " << robot.getTheta() << std::endl;

    return 0;
}