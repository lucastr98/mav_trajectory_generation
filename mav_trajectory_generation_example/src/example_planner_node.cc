/*
 * Simple example that shows a trajectory planner using
 *  mav_trajectory_generation.
 *
 *
 * Launch via
 *   roslaunch mav_trajectory_generation_example example.launch
 *
 * Wait for console to run through all gazebo/rviz messages and then
 * you should see the example below
 *  - After Enter, it receives the current uav position
 *  - After second enter, publishes trajectory information
 *  - After third enter, executes trajectory (sends it to the sampler)
 */

#include  "ros/ros.h"
#include <mav_trajectory_generation_example/example_planner.h>

#include <math.h>
#define PI 3.14159265

#include <iostream>

int main(int argc, char** argv) {

  ros::init(argc, argv, "simple_planner");

  ros::NodeHandle n;
  ExamplePlanner planner(n);
  ROS_WARN_STREAM("SLEEPING FOR 5s TO WAIT FOR CLEAR CONSOLE");
  ros::Duration(5.0).sleep();
  ROS_WARN_STREAM("WARNING: CONSOLE INPUT/OUTPUT ONLY FOR DEMONSTRATION!");

  // THIS SHOULD NORMALLY RUN INSIDE ROS::SPIN!!! JUST FOR DEMO PURPOSES LIKE THIS.
  ROS_WARN_STREAM("PRESS ENTER TO UPDATE CURRENT POSITION AND SEND TRAJECTORY");
  std::cin.get();
  for (int i = 0; i < 10; i++) {
    ros::spinOnce();  // process a few messages in the background - causes the uavPoseCallback to happen
  }

  mav_trajectory_generation::Trajectory trajectory;
  Eigen::Vector3d position, middle_pos, velocity;

  position << 0.0, 0.0, 2.0;
  velocity << 0.0, 0.0, 0.0;

  planner.planTrajectory(position, velocity, &trajectory);
  planner.publishTrajectory(trajectory);
  while(std::sqrt(std::pow(position[0] - planner.current_pose_.translation()[0], 2) + std::pow(position[1] - planner.current_pose_.translation()[1], 2) + std::pow(position[2] - planner.current_pose_.translation()[2], 2)) > 0.3){
    ros::spinOnce();
  }
  ros::Duration(1.0).sleep();

  while(ros::ok()){
    position << 10.0, 0.0, 2.0;
    velocity << 0.0, 0.0, 0.0;

    for (int i = 0; i < 10; i++) {
      ros::spinOnce();  // process a few messages in the background - causes the uavPoseCallback to happen
    }

    planner.planTrajectory(position, velocity, &trajectory);
    planner.publishTrajectory(trajectory);
    while(std::sqrt(std::pow(position[0] - planner.current_pose_.translation()[0], 2) + std::pow(position[1] - planner.current_pose_.translation()[1], 2) + std::pow(position[2] - planner.current_pose_.translation()[2], 2)) > 0.3){
      ros::spinOnce();
    }
    ros::Duration(0.5).sleep();


    position[0] = 5;
    position[1] = sqrt(25/2);
    position[2] = sqrt(25/2) + 2;
    middle_pos[0] = sqrt(25/3) + 5;
    middle_pos[1] = sqrt(25/3);
    middle_pos[2] = sqrt(25/3) + 2;
    velocity << 0.0, 0.0, 0.0;

    for (int i = 0; i < 10; i++) {
      ros::spinOnce();  // process a few messages in the background - causes the uavPoseCallback to happen
    }

    planner.planTrajectory2(position, middle_pos, velocity, &trajectory);
    planner.publishTrajectory(trajectory);
    while(std::sqrt(std::pow(position[0] - planner.current_pose_.translation()[0], 2) + std::pow(position[1] - planner.current_pose_.translation()[1], 2) + std::pow(position[2] - planner.current_pose_.translation()[2], 2)) > 0.3){
      ros::spinOnce();
    }
    ros::Duration(0.5).sleep();


    position[0] = 0;
    position[1] = 0;
    position[2] = 2;
    middle_pos[0] = -sqrt(25/3) + 5;
    middle_pos[1] = sqrt(25/3);
    middle_pos[2] = sqrt(25/3) + 2;
    velocity << 0.0, 0.0, 0.0;

    for (int i = 0; i < 10; i++) {
      ros::spinOnce();  // process a few messages in the background - causes the uavPoseCallback to happen
    }

    planner.planTrajectory2(position, middle_pos, velocity, &trajectory);
    planner.publishTrajectory(trajectory);
    while(std::sqrt(std::pow(position[0] - planner.current_pose_.translation()[0], 2) + std::pow(position[1] - planner.current_pose_.translation()[1], 2) + std::pow(position[2] - planner.current_pose_.translation()[2], 2)) > 0.3){
      ros::spinOnce();
    }
    ros::Duration(0.5).sleep();
  }

  return 0;
}
