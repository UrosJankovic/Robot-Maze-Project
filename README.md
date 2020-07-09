# robot_simulation

This is a package for simulating Turtlebot3 in a simple maze using ROS and Gazebo. Your task is to write an algorithm that will control robot's wheel velocites in order to traverse the maze as quickly as possible. You should place your code in `RobotSimulation.cpp` file in marked lines. You can use 3 simulated range sensors or the whole laser scan.

Please mirror this repository as a private repository using [this instruction](https://gist.github.com/0xjac/85097472043b697ab57ba1b1c7530274), add me to your mirror and keep commiting your changes on a regular basis. You can write a report in this README at the bottom or in a separate file. Include description of your algorithm and add a link to a video of the robot traversing the maze.

## prerequisites

The package was tested on Ubuntu 18.04 and ROS Melodic. You can install Ubuntu on a free space on your hard drive or use virtual machine (eq. VirtualBox). Having freshly installed Ubuntu you can install and configure necessary software:

1. Install ROS using [this instruction](http://wiki.ros.org/melodic/Installation/Ubuntu) up to the point 1.5 (included).

2. Install necessary software:

    ```bash
   sudo apt install git python-catkin-tools ros-melodic-turtlebot3 ros-melodic-turtlebot3-gazebo ros-melodic-gazebo-plugins
   ```
   
3. If you're not familiar with ROS complete [this tutorials 1-6](http://wiki.ros.org/ROS/Tutorials). Instead of `catkin_make` we recommend using newer `catkin build` command.

## building

1. Clone this repository into your workspace:

    ```bash
   cd ~/catkin_ws/src
   git clone 
   ```
   
2. Copy maze model, so it can be used by Gazebo:

    ```bash
    cp ~/catkin_ws/src/robot_simulation/models ~/.gazebo/
    ```

3. Build the package:

    ```bash
   cd ..
   catkin build -DCMAKE_BUILD_TYPE=Release
   ```
   
4. Run the simulation:

    ```bash
    roslaunch robot_simulation turtlebot3_world.launch
   ```