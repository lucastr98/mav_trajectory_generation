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
  Eigen::Vector3d position1, position2, position3, position4, position5, velocity;
  std::vector<double> position_param1, position_param2, position_param3, position_param4, position_param5;
  double accuracy;
  int num_pos_to_use;

  if (!n.getParam(ros::this_node::getName() + "/position1", position_param1)){
    ROS_WARN("[example_planner] param position1 not found");
  }
  if (!n.getParam(ros::this_node::getName() + "/position2", position_param2)){
    ROS_WARN("[example_planner] param position2 not found");
  }
  if (!n.getParam(ros::this_node::getName() + "/position3", position_param3)){
    ROS_WARN("[example_planner] param position3 not found");
  }
  if (!n.getParam(ros::this_node::getName() + "/position4", position_param4)){
    ROS_WARN("[example_planner] param position4 not found");
  }
  if (!n.getParam(ros::this_node::getName() + "/position5", position_param5)){
    ROS_WARN("[example_planner] param position5 not found");
  }
  if (!n.getParam(ros::this_node::getName() + "/accuracy", accuracy)){
    ROS_WARN("[example_planner] param accuracy not found");
  }
  if (!n.getParam(ros::this_node::getName() + "/num_pos_to_use", num_pos_to_use)){
    ROS_WARN("[example_planner] num_pos_to_use accuracy not found");
  }

  if(num_pos_to_use > 5){
    ROS_WARN_STREAM("YOU CANNOT HAVE MORE THAN 5 POSITIONS!");
  }

  position1[0] = position_param1[0];
  position1[1] = position_param1[1];
  position1[2] = position_param1[2];
  position2[0] = position_param2[0];
  position2[1] = position_param2[1];
  position2[2] = position_param2[2];
  position3[0] = position_param3[0];
  position3[1] = position_param3[1];
  position3[2] = position_param3[2];
  position4[0] = position_param4[0];
  position4[1] = position_param4[1];
  position4[2] = position_param4[2];
  position5[0] = position_param5[0];
  position5[1] = position_param5[1];
  position5[2] = position_param5[2];

  std::vector<Eigen::Vector3d> position_vec;
  position_vec.push_back(position1);
  position_vec.push_back(position2);
  position_vec.push_back(position3);
  position_vec.push_back(position4);
  position_vec.push_back(position5);

  velocity << 0.0, 0.0, 0.0;

  for(int i = 0; i < num_pos_to_use; ++i){
    planner.planTrajectory(position_vec[i], velocity, &trajectory);
    planner.publishTrajectory(trajectory);
    while(std::sqrt(std::pow(position_vec[i][0] - planner.current_pose_.translation()[0], 2) +
                    std::pow(position_vec[i][1] - planner.current_pose_.translation()[1], 2) +
                    std::pow(position_vec[i][2] - planner.current_pose_.translation()[2], 2)) > accuracy){
      ros::spinOnce();
    }
    ros::Duration(1.0).sleep();
    ros::spinOnce();
  }

  ROS_WARN_STREAM("PROGRAM ENDED");

  return 0;
}
