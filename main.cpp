#include <iostream>
#include "robot.h"

int main()
{
    Robot robot(1.0f); // 1m square for testing

    // test driving forward with no rotation
    robot.drive(1.0f, 0.0f, 0.0f);
    robot.update();

    std::cout << "Robot position after driving forward 1m: "
              << "X: " << robot.getX() << ", "
              << "Y: " << robot.getY() << ", "
              << "Theta: " << robot.getTheta() << std::endl;

    return 0;
}