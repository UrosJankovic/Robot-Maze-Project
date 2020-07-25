//
// Created by janw on 04.06.2020.
//

// STL
#include <random>
#include <chrono>

// ROS
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/PointCloud.h>

#include "robot_simulation/RobotSimulation.hpp"

RobotSimulation::RobotSimulation() :
    nh("~")
{
    // listening to the topic with laser scans
    subLaserScan = nh.subscribe<sensor_msgs::LaserScan>("/scan", 10, &RobotSimulation::laserScanCB, this);

    // publishing topic with velocity commands
    pubVel = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
}

void RobotSimulation::laserScanCB(const sensor_msgs::LaserScan_<std::allocator<void>>::ConstPtr &laserScanMsg) {
    lastLaserScanMsg = laserScanMsg;
}

void RobotSimulation::run() {
    // frequency of the control loop
    static constexpr int loopFreq = 10;
    ros::Rate loopRate(loopFreq);

    // additional variables
    // PUT YOUR CODE HERE



    // ------------------

    while (ros::ok()) {
        ros::Time curTimestamp = ros::Time::now();

        // simulating 3 range sensors from laser scanner
        // angles around which mean range will be calculated
        // angles start from 0 in x direction (front of the robot) and continue counter-clockwise
        static constexpr double leftAngle = 30.0 * M_PI / 180.0;
        static constexpr double rightAngle = 330.0 * M_PI / 180.0;
        static constexpr double frontAngle = 0.0 * M_PI / 180.0;
        // angular range used for computing mean range
        static constexpr double angleRange = 5 * M_PI / 180.0;

        double leftRange = 0.0;
        int leftCnt = 0;
        double rightRange = 0.0;
        int rightCnt = 0;
        double frontRange = 0.0;
        int frontCnt = 0;
        if(lastLaserScanMsg) {
            for (int i = 0; i < lastLaserScanMsg->ranges.size(); ++i) {
                double curAngle = lastLaserScanMsg->angle_min + i * lastLaserScanMsg->angle_increment;

                // if current angle is in range for left range computation
                if (leftAngle - angleRange / 2.0 < curAngle && curAngle < leftAngle + angleRange / 2.0) {
                    leftRange += lastLaserScanMsg->ranges[i];
                    ++leftCnt;
                }
                if (rightAngle - angleRange / 2.0 < curAngle && curAngle < rightAngle + angleRange / 2.0) {
                    rightRange += lastLaserScanMsg->ranges[i];
                    ++rightCnt;
                }
                if (frontAngle - angleRange / 2.0 < curAngle && curAngle < frontAngle + angleRange / 2.0) {
                    frontRange += lastLaserScanMsg->ranges[i];
                    ++frontCnt;
                }
            }
            if (leftCnt > 0) {
                leftRange /= leftCnt;
            }
            if (rightCnt > 0) {
                rightRange /= rightCnt;
            }
            if (frontCnt > 0) {
                frontRange /= frontCnt;
            }
        }

        // uncomment to print computed ranges
        ROS_INFO_STREAM("Left range: " << leftRange);
        ROS_INFO_STREAM("Right range: " << rightRange);
        ROS_INFO_STREAM("Front range: " << frontRange);

        // computing setpoint wheel velocities
        double leftVel = 0.02;
        double rightVel = 0.02;

        // PUT YOUR CODE HERE
            leftVel = rightRange / 5.0;
            rightVel = leftRange / 5.0;

        // ------------------

        publishVel(leftVel, rightVel);

        ros::spinOnce();
        loopRate.sleep();
    }
}

void RobotSimulation::publishVel(const double &leftVel, const double &rightVel) {
    // distance between wheels
    static constexpr double l = 0.154;
    // maximum linear velocity of wheels
    static constexpr double maxVel = 0.22;

    static std::default_random_engine gen(std::chrono::system_clock::now().time_since_epoch().count());
    std::normal_distribution<double> dist(0.0, 0.03);

    // adding noise and truncating to maximum values
    double leftVelTrunc = std::max(std::min(leftVel + dist(gen), maxVel), -maxVel);
    double rightVelTrunc = std::max(std::min(rightVel + dist(gen), maxVel), -maxVel);

    // kinematics model for differential drive
    double linVel = (leftVelTrunc + rightVelTrunc) / 2.0;
    double angVel = (rightVelTrunc - leftVelTrunc) / l;

    geometry_msgs::Twist velMsg;
    velMsg.linear.x = linVel;
    velMsg.linear.y = 0.0;
    velMsg.linear.z = 0.0;
    velMsg.angular.x = 0.0;
    velMsg.angular.y = 0.0;
    velMsg.angular.z = angVel;

    pubVel.publish(velMsg);
}
