/*
 * Copyright (c) 2016, Markus Achtelik, ASL, ETH Zurich, Switzerland
 * Copyright (c) 2016, Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright (c) 2016, Helen Oleynikova, ASL, ETH Zurich, Switzerland
 * Copyright (c) 2016, Rik Bähnemann, ASL, ETH Zurich, Switzerland
 * Copyright (c) 2016, Marija Popovic, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "mav_trajectory_generation/trajectory_sampling.h"

namespace mav_trajectory_generation {

bool check_for_outlier(Eigen::Vector3d final_position){
  // global variables, limits of the arnea. 1 stands for minimum value, the highest number for max value
  double outlier_x_1 = -48.6;
  double outlier_x_2 = -45;
  double outlier_x_3 = -7.2;
  double outlier_x_4 = 6;
  double outlier_x_5 = 44;
  double outlier_x_6 = 49;
  double outlier_x_7 = 41.5;
  double outlier_y_1 = -17.5;
  double outlier_y_2 = -12;
  double outlier_y_3 = 14;
  double outlier_y_4 = 20;
  double outlier_y_5 = -14;
  double outlier_z_1 = 0;
  double outlier_z_2 = 20;

  if  (final_position[0] <  outlier_x_1 || final_position[0] >  outlier_x_6 ||
    final_position[1] <  outlier_y_2 || final_position[1] >  outlier_y_3 ||
    final_position[2] <  outlier_z_1 || final_position[2] >  outlier_z_2  ){
      // ROS_WARN_STREAM("MP ----- OUTLIER first");

     if (((final_position[1] >  outlier_y_3 && final_position[1] < outlier_y_4) ||
         (final_position[1] > outlier_y_1 && final_position[1] < outlier_y_2)) &&
         (((final_position[0] > outlier_x_2 && final_position[0] < outlier_x_5) ||
         (final_position[0] > outlier_x_3 && final_position[0] < outlier_x_7 )))){
      // ROS_WARN_STREAM("MP ----- OUTLIER 1");
      return false;
    }
    else if(((final_position[1] >  outlier_y_3 && final_position[1] <  outlier_y_4) ||
            (final_position[1] >  outlier_y_5 && final_position[1] <  outlier_y_2)) &&
            ( final_position[0] >  outlier_x_7 && final_position[0] <  outlier_x_5)){
      // ROS_WARN_STREAM("MP ----- OUTLIER 2");
      return false;
    }
    else if(((final_position[1] >  outlier_y_5 && final_position[1] <  outlier_y_2)) &&
          ( final_position[0] >  outlier_x_3 && final_position[0] <  outlier_x_4)){
      // ROS_WARN_STREAM("MP ----- OUTLIER 3");
      return false;
    }

    else{
      return true;
    }
  }
  else{
    return false;
  }
}

const double kNumNanosecondsPerSecond = 1.e9;

bool sampleTrajectoryAtTime(const Trajectory& trajectory, double sample_time,
                            mav_msgs::EigenTrajectoryPoint* state) {
  CHECK_NOTNULL(state);
  if (sample_time < trajectory.getMinTime() ||
      sample_time > trajectory.getMaxTime()) {
    LOG(ERROR) << "Sample time should be within [" << trajectory.getMinTime()
               << " " << trajectory.getMaxTime() << "] but is " << sample_time;
    return false;
  }

  if (trajectory.D() < 3) {
    LOG(ERROR) << "Dimension has to be at least 3, but is " << trajectory.D();
    return false;
  }

  return sampleFlatStateAtTime<Trajectory>(trajectory, sample_time, state);
}

bool sampleTrajectoryInRange(const Trajectory& trajectory, double min_time,
                             double max_time, double sampling_interval,
                             mav_msgs::EigenTrajectoryPointVector* states) {
  CHECK_NOTNULL(states);
  if (min_time < trajectory.getMinTime() ||
      max_time > trajectory.getMaxTime()) {
    LOG(ERROR) << "Sample time should be within [" << trajectory.getMinTime()
               << " " << trajectory.getMaxTime() << "] but is [" << min_time
               << " " << max_time << "]";
    return false;
  }

  if (trajectory.D() < 3) {
    LOG(ERROR) << "Dimension has to be at least 3, but is " << trajectory.D();
    return false;
  }

  std::vector<Eigen::VectorXd> position, velocity, acceleration, jerk, snap,
      yaw, yaw_rate;

  trajectory.evaluateRange(min_time, max_time, sampling_interval,
                           derivative_order::POSITION, &position);
  trajectory.evaluateRange(min_time, max_time, sampling_interval,
                           derivative_order::VELOCITY, &velocity);
  trajectory.evaluateRange(min_time, max_time, sampling_interval,
                           derivative_order::ACCELERATION, &acceleration);
  trajectory.evaluateRange(min_time, max_time, sampling_interval,
                           derivative_order::JERK, &jerk);
  trajectory.evaluateRange(min_time, max_time, sampling_interval,
                           derivative_order::SNAP, &snap);

  size_t n_samples = position.size();

  states->resize(n_samples);
  for (size_t i = 0; i < n_samples; ++i) {
    mav_msgs::EigenTrajectoryPoint& state = (*states)[i];

    state.degrees_of_freedom = mav_msgs::MavActuation::DOF4;
    state.position_W = position[i].head<3>();                   // Eigen::Vector3d
    state.velocity_W = velocity[i].head<3>();
    state.acceleration_W = acceleration[i].head<3>();
    state.jerk_W = jerk[i].head<3>();
    state.snap_W = snap[i].head<3>();
    state.time_from_start_ns = static_cast<int64_t>(
        (min_time + sampling_interval * i) * kNumNanosecondsPerSecond);
    if (trajectory.D() == 4) {
      state.setFromYaw(position[i](3));
      state.setFromYawRate(velocity[i](3));
      state.setFromYawAcc(acceleration[i](3));
    }
    else if (trajectory.D() == 6) {
      // overactuated, write quaternion from interpolated rotation vector
      Eigen::Vector3d rot_vec, rot_vec_vel, rot_vec_acc;
      rot_vec = position[i].tail<3>();
      rot_vec_vel  = velocity[i].tail<3>();
      rot_vec_acc  = acceleration[i].tail<3>();
      Eigen::Matrix3d rot_matrix;
      mav_msgs::matrixFromRotationVector(rot_vec, &rot_matrix);
      state.orientation_W_B = Eigen::Quaterniond(rot_matrix);
      state.angular_velocity_W = mav_msgs::omegaFromRotationVector(rot_vec, rot_vec_vel);
      state.angular_acceleration_W = mav_msgs::omegaDotFromRotationVector(rot_vec, rot_vec_vel, rot_vec_acc);
      state.degrees_of_freedom = mav_msgs::MavActuation::DOF6;
    }
  }
  return true;
}
bool sampleTrajectoryInRange_new(const Trajectory& trajectory, double min_time,
                             double max_time, double sampling_interval,
                             mav_msgs::EigenTrajectoryPointVector* states,
                             Eigen::Matrix<double, 3, 2> outlier_rej) {
  CHECK_NOTNULL(states);
  if (min_time < trajectory.getMinTime() ||
      max_time > trajectory.getMaxTime()) {
    LOG(ERROR) << "Sample time should be within [" << trajectory.getMinTime()
               << " " << trajectory.getMaxTime() << "] but is [" << min_time
               << " " << max_time << "]";
    return false;
  }

  if (trajectory.D() < 3) {
    LOG(ERROR) << "Dimension has to be at least 3, but is " << trajectory.D();
    return false;
  }

  std::vector<Eigen::VectorXd> position, velocity, acceleration, jerk, snap,
      yaw, yaw_rate;

  trajectory.evaluateRange(min_time, max_time, sampling_interval,
                           derivative_order::POSITION, &position);
  trajectory.evaluateRange(min_time, max_time, sampling_interval,
                           derivative_order::VELOCITY, &velocity);
  trajectory.evaluateRange(min_time, max_time, sampling_interval,
                           derivative_order::ACCELERATION, &acceleration);
  trajectory.evaluateRange(min_time, max_time, sampling_interval,
                           derivative_order::JERK, &jerk);
  trajectory.evaluateRange(min_time, max_time, sampling_interval,
                           derivative_order::SNAP, &snap);

  size_t n_samples = position.size();

  states->resize(n_samples);
  for (size_t i = 0; i < n_samples; ++i) {
    mav_msgs::EigenTrajectoryPoint& state = (*states)[i];

    state.degrees_of_freedom = mav_msgs::MavActuation::DOF4;
    state.position_W = position[i].head<3>();                                   // Eigen::Vector3d
    state.velocity_W = velocity[i].head<3>();
    state.acceleration_W = acceleration[i].head<3>();
    state.jerk_W = jerk[i].head<3>();
    state.snap_W = snap[i].head<3>();
    state.time_from_start_ns = static_cast<int64_t>(
        (min_time + sampling_interval * i) * kNumNanosecondsPerSecond);


    Eigen::Vector3d position_arena;
    Eigen::Affine3d pose;
    pose.translation() = state.position_W;                                      // pose in world

    Eigen::Affine3d T_world_arena;
    Eigen::Quaterniond quat(0, 0, -0.148350, 0.988935);
    Eigen::Vector3d vec;
    vec << -1.8661, 22.8, -1.6867;
    T_world_arena.translation() = vec;
    T_world_arena.linear() = quat.toRotationMatrix();
    T_world_arena = T_world_arena.inverse();
    // T_world_arena.linear() = quat.toRotationMatrix();
    // T_world_arena.translation() = vec;

    // T_world_arena.linear() = T_world_arena.linear().inverse();

    pose = T_world_arena * pose;                                                // transform pose to arena

    position_arena = pose.translation();                                        // position in arena

    if(check_for_outlier(position_arena)){
      std::cout << "outlier at: " <<  std::endl << "x = " << position_arena[0] << std::endl << "y = " << position_arena[1] << std::endl << "z = " << position_arena[2]<< std::endl;
      return false;
    }
    else{
      continue;
    }



    // if(state.position_W[0] < outlier_rej(0, 0) || state.position_W[0] > outlier_rej(0, 1) ||
    //    state.position_W[1] < outlier_rej(1, 0) || state.position_W[1] > outlier_rej(1, 1) ||
    //    state.position_W[2] < outlier_rej(2, 0) || state.position_W[2] > outlier_rej(2, 1)){
    //   std::cout << outlier_rej(0, 0) << " < " << state.position_W[0] << " < " << outlier_rej(0, 1) << std::endl;
    //   std::cout << outlier_rej(1, 0) << " < " << state.position_W[1] << " < " << outlier_rej(1, 1) << std::endl;
    //   std::cout << outlier_rej(2, 0) << " < " << state.position_W[2] << " < " << outlier_rej(2, 1) << std::endl;
    //   return false;
    // }
    // else{
    //   continue;
    // }

    if (trajectory.D() == 4) {
      state.setFromYaw(position[i](3));
      state.setFromYawRate(velocity[i](3));
      state.setFromYawAcc(acceleration[i](3));
    }
    else if (trajectory.D() == 6) {
      // overactuated, write quaternion from interpolated rotation vector
      Eigen::Vector3d rot_vec, rot_vec_vel, rot_vec_acc;
      rot_vec = position[i].tail<3>();
      rot_vec_vel  = velocity[i].tail<3>();
      rot_vec_acc  = acceleration[i].tail<3>();
      Eigen::Matrix3d rot_matrix;
      mav_msgs::matrixFromRotationVector(rot_vec, &rot_matrix);
      state.orientation_W_B = Eigen::Quaterniond(rot_matrix);
      state.angular_velocity_W = mav_msgs::omegaFromRotationVector(rot_vec, rot_vec_vel);
      state.angular_acceleration_W = mav_msgs::omegaDotFromRotationVector(rot_vec, rot_vec_vel, rot_vec_acc);
      state.degrees_of_freedom = mav_msgs::MavActuation::DOF6;
    }
  }
  return true;
}

bool sampleTrajectoryStartDuration(
    const Trajectory& trajectory, double start_time, double duration,
    double sampling_interval, mav_msgs::EigenTrajectoryPointVector* states) {
  return sampleTrajectoryInRange(trajectory, start_time, start_time + duration,
                                 sampling_interval, states);
}

bool sampleWholeTrajectory(const Trajectory& trajectory,
                           double sampling_interval,
                           mav_msgs::EigenTrajectoryPoint::Vector* states) {
  const double min_time = trajectory.getMinTime();
  const double max_time = trajectory.getMaxTime();

  return sampleTrajectoryInRange(trajectory, min_time, max_time,
                                 sampling_interval, states);
}
bool sampleWholeTrajectory_new(const Trajectory& trajectory,
                           double sampling_interval,
                           mav_msgs::EigenTrajectoryPoint::Vector* states,
                           Eigen::Matrix<double, 3, 2> outlier_rej,
                           bool cut_traj) {
  const double min_time = trajectory.getMinTime();
  double max_time = trajectory.getMaxTime();

  if(cut_traj){
    if(max_time <= (min_time + 0.21)){}
    else{
      max_time = min_time + 0.21;
    }
  }

  return sampleTrajectoryInRange_new(trajectory, min_time, max_time,
                                 sampling_interval, states, outlier_rej);
}

bool sampleSegmentAtTime(const Segment& segment, double sample_time,
                         mav_msgs::EigenTrajectoryPoint* state) {
  CHECK_NOTNULL(state);
  if (sample_time < 0.0 || sample_time > segment.getTime()) {
    LOG(ERROR) << "Sample time should be within [" << 0.0 << " "
               << segment.getTime() << "] but is " << sample_time;
    return false;
  }

  return sampleFlatStateAtTime<Segment>(segment, sample_time, state);
}

template <class T>
bool sampleFlatStateAtTime(const T& type, double sample_time,
                           mav_msgs::EigenTrajectoryPoint* state) {
  if (type.D() < 3) {
    LOG(ERROR) << "Dimension has to be 3, 4, or 6 but is " << type.D();
    return false;
  }

  Eigen::VectorXd position = type.evaluate(sample_time, derivative_order::POSITION);
  Eigen::VectorXd velocity = type.evaluate(sample_time, derivative_order::VELOCITY);
  Eigen::VectorXd acceleration = type.evaluate(sample_time, derivative_order::ACCELERATION);

  state->degrees_of_freedom = mav_msgs::MavActuation::DOF4;
  state->position_W = position.head(3);
  state->velocity_W = velocity.head(3);
  state->acceleration_W = acceleration.head(3);
  state->jerk_W = type.evaluate(sample_time, derivative_order::JERK).head(3);
  state->snap_W = type.evaluate(sample_time, derivative_order::SNAP).head(3);

  if (type.D() == 4) {
    state->setFromYaw(position(3));
    state->setFromYawRate(velocity(3));
    state->setFromYawAcc(acceleration(3));
  }
  else if (type.D() == 6) {
    // overactuated, write quaternion from interpolated rotation vector
    Eigen::Vector3d rot_vec, rot_vec_vel, rot_vec_acc;
    rot_vec  = position.tail(3);
    rot_vec_vel = velocity.tail(3);
    rot_vec_acc = acceleration.tail(3);
    Eigen::Matrix3d rot_matrix;
    mav_msgs::matrixFromRotationVector(rot_vec, &rot_matrix);
    state->orientation_W_B = Eigen::Quaterniond(rot_matrix);
    state->angular_velocity_W = mav_msgs::omegaFromRotationVector(rot_vec, rot_vec_vel);
    state->angular_acceleration_W = mav_msgs::omegaDotFromRotationVector(rot_vec, rot_vec_vel, rot_vec_acc);
    state->degrees_of_freedom = mav_msgs::MavActuation::DOF6;
  }

  state->time_from_start_ns =
      static_cast<int64_t>(sample_time * kNumNanosecondsPerSecond);
  return true;
}

}  // namespace mav_trajectory_generation
