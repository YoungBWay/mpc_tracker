/*    rpg_quadrotor_mpc
 *    A model predictive control implementation for quadrotors.
 *    Copyright (C) 2017-2018 Philipp Foehn, 
 *    Robotics and Perception Group, University of Zurich
 * 
 *    Intended to be used with rpg_quadrotor_control and rpg_quadrotor_common.
 *    https://github.com/uzh-rpg/rpg_quadrotor_control
 *
 *    This program is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */


#include "rpg_mpc/mpc_controller.h"

#include <ctime>

namespace rpg_mpc {

template<typename T>
MpcController<T>::MpcController(
    const ros::NodeHandle& nh, const ros::NodeHandle& pnh, const std::string& topic) :
    nh_(nh),
    pnh_(pnh),
    mpc_wrapper_(MpcWrapper<T>()),
    timing_feedback_(T(1e-3)),
    timing_preparation_(T(1e-3)),
    est_state_((Eigen::Matrix<T, kStateSize, 1>() <<
                                                  0, 0, 0, 1, 0, 0, 0, 0, 0, 0).finished()),
    reference_states_(Eigen::Matrix<T, kStateSize, kSamples + 1>::Zero()),
    reference_inputs_(Eigen::Matrix<T, kInputSize, kSamples + 1>::Zero()),
    predicted_states_(Eigen::Matrix<T, kStateSize, kSamples + 1>::Zero()),
    predicted_inputs_(Eigen::Matrix<T, kInputSize, kSamples>::Zero()),
    point_of_interest_(Eigen::Matrix<T, 3, 1>::Zero()) {
  pub_predicted_trajectory_ =
      nh_.advertise<nav_msgs::Path>(topic, 1);

  sub_point_of_interest_ = nh_.subscribe("mpc/point_of_interest", 1,
                                         &MpcController<T>::pointOfInterestCallback, this);
  sub_autopilot_off_ = nh_.subscribe("autopilot/off", 1,
                                     &MpcController<T>::offCallback, this);
  
  // Sim to MPC communication
  sub_traj = nh_.subscribe("/quadrotor/trackers_manager/to_mpc",1,&MpcController<T>::trajCallback, this);
  // MPC result visualization
  marker_pub = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  // MPC to SO3 communication
  res_pub = nh_.advertise<rpg_mpc::ToSo3>("/quadrotor/position_cmd",1);

  Visualization_init(vis_points, line_strip, vis_points1, line_strip1);
 
  if (!params_.loadParameters(pnh_)) {
    ROS_ERROR("[%s] Could not load parameters.", pnh_.getNamespace().c_str());
    ros::shutdown();
    return;
  }
  setNewParams(params_);

  solve_from_scratch_ = true;
  preparation_thread_ = std::thread(&MpcWrapper<T>::prepare, mpc_wrapper_);
}

template<typename T>
void MpcController<T>::pointOfInterestCallback(
    const geometry_msgs::PointStamped::ConstPtr& msg) {
  point_of_interest_(0) = msg->point.x;
  point_of_interest_(1) = msg->point.y;
  point_of_interest_(2) = msg->point.z;
  mpc_wrapper_.setPointOfInterest(point_of_interest_);
}

template<typename T>
void MpcController<T>::offCallback(
    const std_msgs::Empty::ConstPtr& msg) {
  solve_from_scratch_ = true;
}

//template<typename T>
//quadrotor_common::ControlCommand MpcController<T>::off() {
//  quadrotor_common::ControlCommand command;
//
//  command.zero();
//
//  return command;
//}

//template<typename T>
//quadrotor_common::ControlCommand MpcController<T>::run(
//    const quadrotor_common::QuadStateEstimate& state_estimate,
//    const quadrotor_common::Trajectory& reference_trajectory,
//    const MpcParams<T>& params) {
//  ros::Time call_time = ros::Time::now();
//  const clock_t start = clock();
//  if (params.changed_) {
//    params_ = params;
//    setNewParams(params_);
//  }
//
//  preparation_thread_.join();
//
//  // Convert everything into Eigen format.
//  setStateEstimate(state_estimate);
//  setReference(reference_trajectory);
//
//  static const bool do_preparation_step(false);
//
//  // Get the feedback from MPC.
//  mpc_wrapper_.setTrajectory(reference_states_, reference_inputs_);
//  if (solve_from_scratch_) {
//    ROS_INFO("Solving MPC with hover as initial guess.");
//    mpc_wrapper_.solve(est_state_);
//    solve_from_scratch_ = false;
//  } else {
//    mpc_wrapper_.update(est_state_, do_preparation_step);
//  }
//  mpc_wrapper_.getStates(predicted_states_);
//  mpc_wrapper_.getInputs(predicted_inputs_);
//
//  // Publish the predicted trajectory.
//  publishPrediction(predicted_states_, predicted_inputs_, call_time);
//
//  // Start a thread to prepare for the next execution.
//  preparation_thread_ = std::thread(&MpcController<T>::preparationThread, this);
//
//  // Timing
//  const clock_t end = clock();
//  timing_feedback_ = 0.9 * timing_feedback_ +
//                     0.1 * double(end - start) / CLOCKS_PER_SEC;
//  if (params_.print_info_)
//    ROS_INFO_THROTTLE(1.0, "MPC Timing: Latency: %1.1f ms  |  Total: %1.1f ms",
//                      timing_feedback_ * 1000, (timing_feedback_ + timing_preparation_) * 1000);
//
//  // Return the input control command.
//  return updateControlCommand(predicted_states_.col(0),
//                              predicted_inputs_.col(0),
//                              call_time);
//}

//template<typename T>
//bool MpcController<T>::setStateEstimate(
//    const quadrotor_common::QuadStateEstimate& state_estimate) {
//  est_state_(kPosX) = state_estimate.position.x();
//  est_state_(kPosY) = state_estimate.position.y();
//  est_state_(kPosZ) = state_estimate.position.z();
//  est_state_(kOriW) = state_estimate.orientation.w();
//  est_state_(kOriX) = state_estimate.orientation.x();
//  est_state_(kOriY) = state_estimate.orientation.y();
//  est_state_(kOriZ) = state_estimate.orientation.z();
//  est_state_(kVelX) = state_estimate.velocity.x();
//  est_state_(kVelY) = state_estimate.velocity.y();
//  est_state_(kVelZ) = state_estimate.velocity.z();
//  const bool quaternion_norm_ok = abs(est_state_.segment(kOriW, 4).norm() - 1.0) < 0.1;
//  return quaternion_norm_ok;
//}

//template<typename T>
//bool MpcController<T>::setReference(
//    const quadrotor_common::Trajectory& reference_trajectory) {
//  ROS_INFO("%d",reference_trajectory.points.size());
//  reference_states_.setZero();
//  reference_inputs_.setZero();
//
//  const T dt = mpc_wrapper_.getTimestep();
//  Eigen::Matrix<T, 3, 1> acceleration;
//  const Eigen::Matrix<T, 3, 1> gravity(0.0, 0.0, -9.81);
//  Eigen::Quaternion<T> q_heading;
//  Eigen::Quaternion<T> q_orientation;
//  bool quaternion_norm_ok(true);
//  if (reference_trajectory.points.size() == 1) {
//    q_heading = Eigen::Quaternion<T>(Eigen::AngleAxis<T>(
//        reference_trajectory.points.front().heading,
//        Eigen::Matrix<T, 3, 1>::UnitZ()));
//    q_orientation = reference_trajectory.points.front().orientation.template cast<T>() * q_heading;
//    reference_states_ = (Eigen::Matrix<T, kStateSize, 1>()
//        << reference_trajectory.points.front().position.template cast<T>(),
//        q_orientation.w(),
//        q_orientation.x(),
//        q_orientation.y(),
//        q_orientation.z(),
//        reference_trajectory.points.front().velocity.template cast<T>()
//    ).finished().replicate(1, kSamples + 1);
//
//    acceleration << reference_trajectory.points.front().acceleration.template cast<T>() - gravity;
//    reference_inputs_ = (Eigen::Matrix<T, kInputSize, 1>() << acceleration.norm(),
//        reference_trajectory.points.front().bodyrates.template cast<T>()
//    ).finished().replicate(1, kSamples + 1);
//  } else {
//    auto iterator(reference_trajectory.points.begin());
//    ros::Duration t_start = reference_trajectory.points.begin()->time_from_start;
//    auto last_element = reference_trajectory.points.end();
//    last_element = std::prev(last_element);
//
//    for (int i = 0; i < kSamples + 1; i++) {
//      while ((iterator->time_from_start - t_start).toSec() <= i * dt &&
//             iterator != last_element) {
//        iterator++;
//      }
//
//      q_heading = Eigen::Quaternion<T>(Eigen::AngleAxis<T>(
//          iterator->heading, Eigen::Matrix<T, 3, 1>::UnitZ()));
//      q_orientation = q_heading * iterator->orientation.template cast<T>();
//      reference_states_.col(i) << iterator->position.template cast<T>(),
//          q_orientation.w(),
//          q_orientation.x(),
//          q_orientation.y(),
//          q_orientation.z(),
//          iterator->velocity.template cast<T>();
//      if (reference_states_.col(i).segment(kOriW, 4).dot(
//          est_state_.segment(kOriW, 4)) < 0.0)
//        reference_states_.block(kOriW, i, 4, 1) =
//            -reference_states_.block(kOriW, i, 4, 1);
//      acceleration << iterator->acceleration.template cast<T>() - gravity;
//      reference_inputs_.col(i) << acceleration.norm(),
//          iterator->bodyrates.template cast<T>();
//      quaternion_norm_ok &= abs(est_state_.segment(kOriW, 4).norm() - 1.0) < 0.1;
//    }
//  }
//  return quaternion_norm_ok;
//}

//template<typename T>
//quadrotor_common::ControlCommand MpcController<T>::updateControlCommand(
//    const Eigen::Ref<const Eigen::Matrix<T, kStateSize, 1>> state,
//    const Eigen::Ref<const Eigen::Matrix<T, kInputSize, 1>> input,
//    ros::Time& time) {
//  Eigen::Matrix<T, kInputSize, 1> input_bounded = input.template cast<T>();
//
//  // Bound inputs for sanity.
//  input_bounded(INPUT::kThrust) = std::max(params_.min_thrust_,
//                                           std::min(params_.max_thrust_, input_bounded(INPUT::kThrust)));
//  input_bounded(INPUT::kRateX) = std::max(-params_.max_bodyrate_xy_,
//                                          std::min(params_.max_bodyrate_xy_, input_bounded(INPUT::kRateX)));
//  input_bounded(INPUT::kRateY) = std::max(-params_.max_bodyrate_xy_,
//                                          std::min(params_.max_bodyrate_xy_, input_bounded(INPUT::kRateY)));
//  input_bounded(INPUT::kRateZ) = std::max(-params_.max_bodyrate_z_,
//                                          std::min(params_.max_bodyrate_z_, input_bounded(INPUT::kRateZ)));
//
//  quadrotor_common::ControlCommand command;
//
//  command.timestamp = time;
//  command.armed = true;
//  command.control_mode = quadrotor_common::ControlMode::BODY_RATES;
//  command.expected_execution_time = time;
//  command.collective_thrust = input_bounded(INPUT::kThrust);
//  command.bodyrates.x() = input_bounded(INPUT::kRateX);
//  command.bodyrates.y() = input_bounded(INPUT::kRateY);
//  command.bodyrates.z() = input_bounded(INPUT::kRateZ);
//  command.orientation.w() = state(STATE::kOriW);
//  command.orientation.x() = state(STATE::kOriX);
//  command.orientation.y() = state(STATE::kOriY);
//  command.orientation.z() = state(STATE::kOriZ);
//  return command;
//}

template<typename T>
bool MpcController<T>::publishPrediction(
    const Eigen::Ref<const Eigen::Matrix<T, kStateSize, kSamples + 1>> states,
    const Eigen::Ref<const Eigen::Matrix<T, kInputSize, kSamples>> inputs,
    ros::Time& time) {
  nav_msgs::Path path_msg;
  path_msg.header.stamp = time;
  path_msg.header.frame_id = "world";
  geometry_msgs::PoseStamped pose;
  T dt = mpc_wrapper_.getTimestep();

  for (int i = 0; i < kSamples; i++) {
    pose.header.stamp = time + ros::Duration(i * dt);
    pose.header.seq = i;
    pose.pose.position.x = states(kPosX, i);
    pose.pose.position.y = states(kPosY, i);
    pose.pose.position.z = states(kPosZ, i);
    pose.pose.orientation.w = states(kOriW, i);
    pose.pose.orientation.x = states(kOriX, i);
    pose.pose.orientation.y = states(kOriY, i);
    pose.pose.orientation.z = states(kOriZ, i);
    path_msg.poses.push_back(pose);
  }

  pub_predicted_trajectory_.publish(path_msg);
  return true;
}

template<typename T>
void MpcController<T>::preparationThread() {
  const clock_t start = clock();

  mpc_wrapper_.prepare();

  // Timing
  const clock_t end = clock();
  timing_preparation_ = 0.9 * timing_preparation_ +
                        0.1 * double(end - start) / CLOCKS_PER_SEC;
}

template<typename T>
bool MpcController<T>::setNewParams(MpcParams<T>& params) {
  mpc_wrapper_.setCosts(params.Q_, params.R_);
  mpc_wrapper_.setLimits(
      params.min_thrust_, params.max_thrust_,
      params.max_bodyrate_xy_, params.max_bodyrate_z_);
  mpc_wrapper_.setCameraParameters(params.p_B_C_, params.q_B_C_);
  params.changed_ = false;
  return true;
}


template<typename T>
void MpcController<T>::trajCallback(const rpg_mpc::MPCTrajState& msg) {

  ros::Time time_at_callback = msg.header.stamp;
  kr_setStateEstimate(msg);
  kr_setReference(msg);
  kr_run(msg, time_at_callback);
  
}
template<typename T>
int MpcController<T>::kr_run(const rpg_mpc::MPCTrajState& msg, ros::Time time_at_callback) {
  ros::Time call_time = ros::Time::now();
  const clock_t start = clock();

  // The following code may be useful for the end of trajectory MPC control(different parameters)

  // if (params.changed_) {
  //   params_ = params;
  //   setNewParams(params_);
  // }

  preparation_thread_.join();

  static const bool do_preparation_step(false);

  // Get the feedback from MPC.
  mpc_wrapper_.setTrajectory(reference_states_, reference_inputs_);
  if (solve_from_scratch_) {
    ROS_INFO("Solving MPC with hover as initial guess.");
    mpc_wrapper_.solve(est_state_);
    solve_from_scratch_ = false;
  } else {
    mpc_wrapper_.update(est_state_, do_preparation_step);
  }
  mpc_wrapper_.getStates(predicted_states_);
  mpc_wrapper_.getInputs(predicted_inputs_);

  // Publish the predicted trajectory.
  publishPrediction(predicted_states_, predicted_inputs_, call_time);

  // Start a thread to prepare for the next execution.
  preparation_thread_ = std::thread(&MpcController<T>::preparationThread, this);

  // Timing
  const clock_t end = clock();
  timing_feedback_ = 0.9 * timing_feedback_ +
                     0.1 * double(end - start) / CLOCKS_PER_SEC;
  if (params_.print_info_)
    ROS_INFO_THROTTLE(1.0, "MPC Timing: Latency: %1.1f ms  |  Total: %1.1f ms",
                      timing_feedback_ * 1000, (timing_feedback_ + timing_preparation_) * 1000);

  // send predicted point to SO3 control
  mpc_to_so3(predicted_states_, predicted_inputs_, msg, time_at_callback);

  return 1;
}

template<typename T>
bool MpcController<T>::kr_setStateEstimate(
    const rpg_mpc::MPCTrajState& msg) {
  est_state_(kPosX) = msg.state_posi.x;
  est_state_(kPosY) = msg.state_posi.y;
  est_state_(kPosZ) = msg.state_posi.z;
  est_state_(kOriW) = msg.state_q.w;
  est_state_(kOriX) = msg.state_q.x;
  est_state_(kOriY) = msg.state_q.y;
  est_state_(kOriZ) = msg.state_q.z;
  est_state_(kVelX) = msg.state_vel.x;
  est_state_(kVelY) = msg.state_vel.y;
  est_state_(kVelZ) = msg.state_vel.z;
  const bool quaternion_norm_ok = abs(est_state_.segment(kOriW, 4).norm() - 1.0) < 0.1;

  // ROS_INFO("%s",quaternion_norm_ok ? "true" : "false");
  return quaternion_norm_ok;
}

template<typename T>
bool MpcController<T>::kr_setReference(
    const rpg_mpc::MPCTrajState& msg) {
  reference_states_.setZero();
  reference_inputs_.setZero();

  const T dt = mpc_wrapper_.getTimestep();
  Eigen::Matrix<T, 3, 1> acceleration;
  const Eigen::Matrix<T, 3, 1> gravity(0.0, 0.0, -9.81);
  Eigen::Quaternion<T> q_heading;
  Eigen::Quaternion<T> q_orientation;
  bool quaternion_norm_ok(true);
  if (msg.points.size() == 1) {
    // ROS_INFO("reference trajectory points size is 1");
    q_heading = Eigen::Quaternion<T>(Eigen::AngleAxis<T>(
        msg.points.front().yaw,
        Eigen::Matrix<T, 3, 1>::UnitZ()));
    Eigen::Quaternion<T> q;
    q.w() = msg.points.front().orientation.w;
    q.x() = msg.points.front().orientation.x;
    q.y() = msg.points.front().orientation.y;
    q.z() = msg.points.front().orientation.z;
    // q_orientation = msg.points.front().orientation.template cast<T>() * q_heading;
    q_orientation = q.template cast<T>() * q_heading;

    Eigen::Vector3d p;
    p.x() = msg.points.front().position.x;
    p.y() = msg.points.front().position.y;
    p.z() = msg.points.front().position.z;
    Eigen::Vector3d v;
    v.x() = msg.points.front().velocity.x;
    v.y() = msg.points.front().velocity.y;
    v.z() = msg.points.front().velocity.z;
    // reference_states_ = (Eigen::Matrix<T, kStateSize, 1>()
    //     << msg.points.front().position.template cast<T>(),
    //     q_orientation.w(),
    //     q_orientation.x(),
    //     q_orientation.y(),
    //     q_orientation.z(),
    //     msg.points.front().velocity.template cast<T>()
    // ).finished().replicate(1, kSamples + 1);
    reference_states_ = (Eigen::Matrix<T, kStateSize, 1>()
        << p.template cast<T>(),
        q_orientation.w(),
        q_orientation.x(),
        q_orientation.y(),
        q_orientation.z(),
        v.template cast<T>()
    ).finished().replicate(1, kSamples + 1);
    Eigen::Vector3d a;
    a.x() = msg.points.front().acceleration.x;
    a.y() = msg.points.front().acceleration.y;
    a.z() = msg.points.front().acceleration.z;
    // acceleration << msg.points.front().acceleration.template cast<T>() - gravity;
    acceleration << a.template cast<T>() - gravity;

    Eigen::Vector3d angular_v;
    angular_v.x() = msg.points.front().bodyrates.x;
    angular_v.y() = msg.points.front().bodyrates.y;
    angular_v.z() = msg.points.front().bodyrates.z;

    // reference_inputs_ = (Eigen::Matrix<T, kInputSize, 1>() << acceleration.norm(),
    //     msg.points.front().bodyrates.template cast<T>()
    // ).finished().replicate(1, kSamples + 1);
    reference_inputs_ = (Eigen::Matrix<T, kInputSize, 1>() << acceleration.norm(),
        angular_v.template cast<T>()
    ).finished().replicate(1, kSamples + 1);
  } else {
    // ROS_INFO("reference trajectory points size is many");
    auto iterator(msg.points.begin());

    // Visualize the reference trajectory
    Eigen::Vector3d temp;
    temp.x() = msg.points.begin()->position.x;
    temp.y() = msg.points.begin()->position.y;
    temp.z() = msg.points.begin()->position.z;
    visualization(temp);

    // ros::Duration t_start = msg.points.begin()->time_from_start;
    ros::Time t_start = msg.points.begin()->header.stamp;

    auto last_element = msg.points.end();
    last_element = std::prev(last_element);

    for (int i = 0; i < kSamples + 1; i++) {
      while ((iterator->header.stamp - t_start).toSec() <= i * dt &&
             iterator != last_element) {
        iterator++;
      }

      q_heading = Eigen::Quaternion<T>(Eigen::AngleAxis<T>(
          iterator->yaw, Eigen::Matrix<T, 3, 1>::UnitZ()));
      Eigen::Quaterniond q;
      q.w() = iterator->orientation.w;
      q.x() = iterator->orientation.x;
      q.y() = iterator->orientation.y;
      q.z() = iterator->orientation.z;
      // q_orientation = q_heading * iterator->orientation.template cast<T>();
      q_orientation = q_heading * q.template cast<T>();
      Eigen::Vector3d p;
      Eigen::Vector3d v;
      p.x() = iterator->position.x;
      p.y() = iterator->position.y;
      p.z() = iterator->position.z;
      v.x() = iterator->velocity.x;
      v.y() = iterator->velocity.y;
      v.z() = iterator->velocity.z;

      // reference_states_.col(i) << iterator->position.template cast<T>(),
      //     q_orientation.w(),
      //     q_orientation.x(),
      //     q_orientation.y(),
      //     q_orientation.z(),
      //     iterator->velocity.template cast<T>();
      reference_states_.col(i) << p.template cast<T>(),
          q_orientation.w(),
          q_orientation.x(),
          q_orientation.y(),
          q_orientation.z(),
          v.template cast<T>();
      if (reference_states_.col(i).segment(kOriW, 4).dot(
          est_state_.segment(kOriW, 4)) < 0.0)
        reference_states_.block(kOriW, i, 4, 1) =
            -reference_states_.block(kOriW, i, 4, 1);
      Eigen::Vector3d a;
      a.x() = iterator->acceleration.x;
      a.y() = iterator->acceleration.y;
      a.z() = iterator->acceleration.z;
      // acceleration << iterator->acceleration.template cast<T>() - gravity;
      acceleration << a.template cast<T>() - gravity;
      Eigen::Vector3d angular_v;
      angular_v.x() = iterator->bodyrates.x;
      angular_v.y() = iterator->bodyrates.y;
      angular_v.z() = iterator->bodyrates.z;
      // reference_inputs_.col(i) << acceleration.norm(),
      //     iterator->bodyrates.template cast<T>();
      reference_inputs_.col(i) << acceleration.norm(),
          angular_v.template cast<T>();
      quaternion_norm_ok &= abs(est_state_.segment(kOriW, 4).norm() - 1.0) < 0.1;
    }
    // ROS_INFO("%s",quaternion_norm_ok ? "true" : "false");
  }
  return quaternion_norm_ok;
}

template<typename T>
void MpcController<T>::visualization(Eigen::Vector3d point){
  
  geometry_msgs::Point p;
  p.x = point.x();
  p.y = point.y();
  p.z = point.z();
  vis_points.points.push_back(p);
  line_strip.points.push_back(p);
  vis_points.header.stamp = ros::Time::now();
  line_strip.header.stamp = ros::Time::now();
  marker_pub.publish(vis_points);

  return ;
}

template<typename T>
void MpcController<T>::visualization1(Eigen::Vector3d point1){
  
  geometry_msgs::Point p1;
  p1.x = point1.x();
  p1.y = point1.y();
  p1.z = point1.z();
  vis_points1.points.push_back(p1);
  line_strip1.points.push_back(p1);
  vis_points1.header.stamp = ros::Time::now();
  line_strip1.header.stamp = ros::Time::now();
  marker_pub.publish(vis_points1);
  marker_pub.publish(line_strip1);

  return ;
}

template<typename T>
void MpcController<T>::mpc_to_so3(const Eigen::Ref<const Eigen::Matrix<T, kStateSize, kSamples + 1>> states,
                                  const Eigen::Ref<const Eigen::Matrix<T, kInputSize, kSamples>> inputs,
                                  const rpg_mpc::MPCTrajState& msg,
                                  ros::Time time_at_callback){
  // line interpolation for calculation time
  ros::Time time_at_pub = ros::Time::now();
  double time_diff = time_at_pub.toSec() - time_at_callback.toSec();
  double time_interpolation_percentage = time_diff/mpc_wrapper_.getTimestep();

  // construct Position Command message
  rpg_mpc::ToSo3 to_so3;
  to_so3.header.stamp = ros::Time::now();
  to_so3.header.frame_id = msg.points.front().header.frame_id;
  to_so3.position.x = states(kPosX,1)+(states(kPosX,2)-states(kPosX,1))*time_interpolation_percentage;
  to_so3.position.y = states(kPosY,1)+(states(kPosY,2)-states(kPosY,1))*time_interpolation_percentage;
  to_so3.position.z = states(kPosZ,1)+(states(kPosZ,2)-states(kPosZ,1))*time_interpolation_percentage;
  to_so3.velocity.x = states(kVelX,1)+(states(kVelX,2)-states(kVelX,1))*time_interpolation_percentage;
  to_so3.velocity.y = states(kVelY,1)+(states(kVelY,2)-states(kVelY,1))*time_interpolation_percentage;
  to_so3.velocity.z = states(kVelZ,1)+(states(kVelZ,2)-states(kVelZ,1))*time_interpolation_percentage;

  double thrust_a = inputs(kThrust,0);
  Eigen::Vector3d vector_z(0.0, 0.0, thrust_a);
  Eigen::Quaterniond orientation_a;
  orientation_a.w() = states(kOriW,1);
  orientation_a.x() = states(kOriX,1);
  orientation_a.y() = states(kOriY,1);
  orientation_a.z() = states(kOriZ,1);
  Eigen::Matrix3d rotation_matrix = orientation_a.toRotationMatrix();
  Eigen::Vector3d acc = rotation_matrix * vector_z;
  to_so3.acceleration.x = acc(0);
  to_so3.acceleration.y = acc(1);
  to_so3.acceleration.z = acc(2);

  // to_so3.acceleration.x = to_so3.acceleration.y = to_so3.acceleration.z = 0;
  to_so3.jerk.x = to_so3.jerk.y = to_so3.jerk.z = 0;
  to_so3.yaw = 0;
  to_so3.yaw_dot = 0;
  to_so3.kx[0] = to_so3.kx[1] = to_so3.kx[2] = 0;
  to_so3.kv[0] = to_so3.kv[1] = to_so3.kv[2] = 0;

  // publish Position Command to SO3 control
  res_pub.publish(to_so3);

  // Visualize the mpc predicted next point
  Eigen::Vector3d first_point;
  Eigen::Vector3d zero_point;

  zero_point.x() = states(kPosX, 0);
  zero_point.y() = states(kPosY, 0);
  zero_point.z() = states(kPosZ, 0);

  first_point.x() = states(kPosX, 1);
  first_point.y() = states(kPosY, 1);
  first_point.z() = states(kPosZ, 1);

  visualization1(first_point);
}



template
class MpcController<float>;

template
class MpcController<double>;

} // namespace rpg_mpc
