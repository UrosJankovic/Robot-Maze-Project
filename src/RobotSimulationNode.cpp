//
// Created by janw on 04.06.2020.
//

#include "ros/ros.h"

#include "robot_simulation/RobotSimulation.hpp"

using namespace std;

int main(int argc, char** argv){
    ros::init( argc, argv, "robot_simulation" );

    RobotSimulation robotSimulation;
    robotSimulation.run();

    return EXIT_SUCCESS;
}
