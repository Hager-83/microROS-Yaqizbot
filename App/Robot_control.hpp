#ifndef ROBOT_CONTROL_HPP
#define ROBOT_CONTROL_HPP

class RobotSystem
{
public:
    void init();
    void spin();

private:
    void init_transport();
    void init_hardware();
    void init_ros();
};

#endif // ROBOT_CONTROL_HPP