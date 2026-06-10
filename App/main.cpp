#include "Robot_control.hpp"

RobotSystem robot;

int main()
{
    robot.init();
    robot.spin();
    return 0;
}