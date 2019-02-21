#include <iostream>

#include <ros/ros.h>

int main(int argc, char *argv[])
{
    // std::cout << "Hello world" << std::endl;
    ros::init(argc, argv, "player_drato_node");

    ros::NodeHandle n;

    for (int i = 0; i < 10; i++)
    {
        std::cout << i << std::endl;
    }

    return 0;
}
