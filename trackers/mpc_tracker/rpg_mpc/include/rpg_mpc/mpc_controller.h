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


#pragma once

#include <thread>

#include <Eigen/Eigen>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Path.h>
//#include <quadrotor_common/control_command.h>
//#include <quadrotor_common/quad_state_estimate.h>
//#include <quadrotor_common/trajectory.h>
//#include <quadrotor_common/trajectory_point.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include <rpg_mpc/MPCTrajState.h>
#include <rpg_mpc/PositionCommand.h>
#include <Eigen/Dense>
#include <visualization_msgs/Marker.h>
#include <rpg_mpc/ToSo3.h>
#include <cmath>

#include "rpg_mpc/mpc_wrapper.h"
#include "rpg_mpc/mpc_params.h"

namespace rpg_mpc {

enum STATE {
  kPosX = 0,
  kPosY = 1,
  kPosZ = 2,
  kOriW = 3,
  kOriX = 4,
  kOriY = 5,
  kOriZ = 6,
  kVelX = 7,
  kVelY = 8,
  kVelZ = 9
};

enum INPUT {
  kThrust = 0,
  kRateX = 1,
  kRateY = 2,
  kRateZ = 3
};

template<typename T>
class MpcController {
public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  static_assert(kStateSize == 10,
                "MpcController: Wrong model size. Number of states does not match.");
  static_assert(kInputSize == 4,
                "MpcController: Wrong model size. Number of inputs does not match.");

  MpcController(const ros::NodeHandle& nh,
                const ros::NodeHandle& pnh,
                const std::string& topic = "mpc/trajectory_predicted");

  MpcController() : MpcController(ros::NodeHandle(), ros::NodeHandle("~")) {}

//  quadrotor_common::ControlCommand off();

//  quadrotor_common::ControlCommand run(
//      const quadrotor_common::QuadStateEstimate& state_estimate,
//      const quadrotor_common::Trajectory& reference_trajectory,
//      const MpcParams<T>& params);


  int kr_run(const rpg_mpc::MPCTrajState& msg, ros::Time time_at_callback);
  void visualization(Eigen::Vector3d point);
  void visualization1(Eigen::Vector3d point1);
  void mpc_to_so3(const Eigen::Ref<const Eigen::Matrix<T, kStateSize, kSamples + 1>> states,
                  const Eigen::Ref<const Eigen::Matrix<T, kInputSize, kSamples>> inputs,
                  const rpg_mpc::MPCTrajState& msg,
                  ros::Time time_at_callback);

private:
  // Internal helper functions.

  void pointOfInterestCallback(
      const geometry_msgs::PointStamped::ConstPtr& msg);

  void offCallback(const std_msgs::Empty::ConstPtr& msg);

  // sim to MPC communication test
  void trajCallback(const rpg_mpc::MPCTrajState& msg);

  bool kr_setStateEstimate(const rpg_mpc::MPCTrajState& msg);

  bool kr_setReference(const rpg_mpc::MPCTrajState& msg);

//  bool setStateEstimate(
//      const quadrotor_common::QuadStateEstimate& state_estimate);

//  bool setReference(const quadrotor_common::Trajectory& reference_trajectory);

//  quadrotor_common::ControlCommand updateControlCommand(
//      const Eigen::Ref<const Eigen::Matrix<T, kStateSize, 1>> state,
//      const Eigen::Ref<const Eigen::Matrix<T, kInputSize, 1>> input,
//      ros::Time& time);

  bool publishPrediction(
      const Eigen::Ref<const Eigen::Matrix<T, kStateSize, kSamples + 1>> states,
      const Eigen::Ref<const Eigen::Matrix<T, kInputSize, kSamples>> inputs,
      ros::Time& time);

  void preparationThread();

  bool setNewParams(MpcParams<T>& params);

  void Visualization_init(visualization_msgs::Marker &vis_points,
                          visualization_msgs::Marker &line_strip,
                          visualization_msgs::Marker &vis_points1,
                          visualization_msgs::Marker &line_strip1)
  {
    vis_points.header.frame_id = line_strip.header.frame_id = "simulator";
    vis_points.header.stamp = line_strip.header.stamp = ros::Time::now();
    vis_points.ns = line_strip.ns = "points_and_lines";
    vis_points.action = line_strip.action = visualization_msgs::Marker::ADD;
    vis_points.pose.orientation.w = line_strip.pose.orientation.w = 1.0;

    vis_points1.header.frame_id = line_strip1.header.frame_id = "simulator";
    vis_points1.header.stamp = line_strip1.header.stamp = ros::Time::now();
    vis_points1.ns = line_strip1.ns = "points_and_lines";
    vis_points1.action = line_strip1.action = visualization_msgs::Marker::ADD;
    vis_points1.pose.orientation.w = line_strip1.pose.orientation.w = 1.0;

    vis_points.id = 0;
    line_strip.id = 1;

    vis_points1.id = 2;
    line_strip1.id = 3;

    vis_points.type = visualization_msgs::Marker::POINTS;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;

    vis_points1.type = visualization_msgs::Marker::POINTS;
    line_strip1.type = visualization_msgs::Marker::LINE_STRIP;

    // POINTS marker uses scale.x and scale.y for width/height respectively
    vis_points.scale.x = 0.05;
    vis_points.scale.y = 0.05;

    vis_points1.scale.x = 0.05;
    vis_points1.scale.y = 0.05;

    // LINE_STRIP marker uses only the scale.x component for the line width
    line_strip.scale.x = 0.05;

    line_strip1.scale.x = 0.05;

    // 点和线条颜色
    vis_points.color.g = 1.0f;
    vis_points.color.a = 1.0;

    line_strip.color.b = 1.0;
    line_strip.color.a = 1.0;

    vis_points1.color.r = 1.0f;
    vis_points1.color.a = 1.0;

    line_strip1.color.b = 1.0;
    line_strip1.color.a = 1.0;
    return;
  }
  // Handles
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  // Subscribers and publisher.
  ros::Subscriber sub_point_of_interest_;
  ros::Subscriber sub_autopilot_off_;
  ros::Publisher pub_predicted_trajectory_;

  // Sim to MPC communication test
  ros::Subscriber sub_traj;
  ros::Publisher marker_pub;
  ros::Publisher res_pub;
  int setup_state;

  visualization_msgs::Marker vis_points, line_strip;
  visualization_msgs::Marker vis_points1, line_strip1;
  // Parameters
  MpcParams<T> params_;

  // MPC
  MpcWrapper<T> mpc_wrapper_;

  // Preparation Thread
  std::thread preparation_thread_;

  // Variables
  T timing_feedback_, timing_preparation_;
  bool solve_from_scratch_;
  Eigen::Matrix<T, kStateSize, 1> est_state_;
  Eigen::Matrix<T, kStateSize, kSamples + 1> reference_states_;
  Eigen::Matrix<T, kInputSize, kSamples + 1> reference_inputs_;
  Eigen::Matrix<T, kStateSize, kSamples + 1> predicted_states_;
  Eigen::Matrix<T, kInputSize, kSamples> predicted_inputs_;
  Eigen::Matrix<T, 3, 1> point_of_interest_;
};


} // namespace MPC
