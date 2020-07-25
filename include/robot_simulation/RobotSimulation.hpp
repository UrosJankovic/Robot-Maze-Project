//
// Created by janw on 04.06.2020.
//

#ifndef ROBOT_SIMULATION_ROBOTSIMULATION_HPP
#define ROBOT_SIMULATION_ROBOTSIMULATION_HPP

// ROS
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

class RobotSimulation {
public:
    RobotSimulation();

    /**
     * This function is called whenever a new message with laser scan is received
     * @param laserScanMsg message with a laser scan
     */
    void laserScanCB(const sensor_msgs::LaserScan::ConstPtr &laserScanMsg);

    /**
     * Main control loop. Runs until ROS is shut down.
     */
    void run();


private:
    /**
     * Converts wheel linear velocities to robot linear and angular velocities and publishes the message.
     * @param leftVel left wheel linear velocity.
     * @param rightVel right wheel linear velocity.
     */
    void publishVel(const double &leftVel, const double &rightVel);

    ros::NodeHandle nh;

    ros::Subscriber subLaserScan;
    ros::Publisher pubVel;

    sensor_msgs::LaserScan::ConstPtr lastLaserScanMsg;
};


#endif //ROBOT_SIMULATION_ROBOTSIMULATION_HPP
