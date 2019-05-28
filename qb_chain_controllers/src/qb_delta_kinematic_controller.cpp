/***
 *  Software License Agreement: BSD 3-Clause License
 *
 *  Copyright (c) 2016-2018, qbrobotics®
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
 *  following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this list of conditions and the
 *    following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
 *    following disclaimer in the documentation and/or other materials provided with the distribution.
 *
 *  * Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *  INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 *  WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 *  USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <qb_chain_controllers/qb_delta_kinematic_controller.h>

using namespace qb_chain_controllers;

DeltaKinematicController::DeltaKinematicController()
    : spinner_(1, callback_queue_.get()),  // the dedicated callback queue is needed to avoid deadlocks caused by action client calls (together with controller manager update loop)
      callback_queue_(boost::make_shared<ros::CallbackQueue>()),
      node_handle_control_(ros::NodeHandle(), "control") {
  node_handle_control_.setCallbackQueue(callback_queue_.get());
  spinner_.start();
}

DeltaKinematicController::~DeltaKinematicController() {
  spinner_.stop();
}

void DeltaKinematicController::actionActiveCallback(const std::string &controller) {
  ROS_INFO_STREAM_NAMED("delta_controller", "Controller [" << controller << "] action start.");
}

void DeltaKinematicController::actionDoneCallback(const actionlib::SimpleClientGoalState &state, const control_msgs::FollowJointTrajectoryResultConstPtr &result, const std::string &controller) {
  if (result->error_code != result->SUCCESSFUL) {
    ROS_WARN_STREAM_NAMED("delta_controller", "Controller [" << controller << "] action ended in state [" << state.toString() <<"] with error code [" << result->error_code << "]");
  }
  else {
    ROS_INFO_STREAM_NAMED("delta_controller", "Controller [" << controller << "] action ended in state [" << state.toString() <<"].");
  }

  if (!use_waypoints_) {
    // erase all but last point
    for (auto &trajectory : motor_joint_trajectories_) {
      trajectory.second.points.erase(trajectory.second.points.begin(), trajectory.second.points.end()-1);
      trajectory.second.points.front().time_from_start = ros::Duration(0.1);
    }
    return;
  }

  // if waypoints are active
  if (!motor_joint_trajectories_.empty()) {
    if (std::all_of(motor_names_.begin(), motor_names_.end(), [this](auto m){ return motor_action_clients_.at(m)->getState().isDone(); })) {
      startWaypointTrajectory();
    }
  }
}

void DeltaKinematicController::actionFeedbackCallback(const control_msgs::FollowJointTrajectoryFeedbackConstPtr &feedback, const std::string &controller) {
  for (int i=0; i<feedback->joint_names.size(); i++) {
    ROS_DEBUG_STREAM_NAMED("delta_controller", "Controller [" << controller << "] joint [" << feedback->joint_names.at(i) << "] state is [" << feedback->actual.positions.at(i) << "] (expecting [" << feedback->desired.positions.at(i) << "]).");
  }
}

void DeltaKinematicController::buildCube(visualization_msgs::InteractiveMarker &interactive_marker) {
  visualization_msgs::Marker marker;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.scale.x = interactive_marker.scale * 0.4;
  marker.scale.y = interactive_marker.scale * 0.4;
  marker.scale.z = interactive_marker.scale * 0.4;
  marker.color.r = 0.35;
  marker.color.g = 0.35;
  marker.color.b = 0.35;
  marker.color.a = 0.75;

  visualization_msgs::InteractiveMarkerControl control;
  control.always_visible = true;
  control.markers.push_back(marker);
  control.name = "move_xyz";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_3D;
  interactive_marker.controls.push_back(control);
}

void DeltaKinematicController::buildEndEffectorControl(visualization_msgs::InteractiveMarker &interactive_marker) {
  visualization_msgs::InteractiveMarkerControl control;
  control.orientation.w = 1;
  control.orientation.x = 1;
  control.orientation.y = 0;
  control.orientation.z = 0;
  control.name = "move_yz";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;
  interactive_marker.controls.push_back(control);
  control.name = "move_x";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  interactive_marker.controls.push_back(control);
  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.name = "move_xy";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;
  interactive_marker.controls.push_back(control);
  control.name = "move_z";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  interactive_marker.controls.push_back(control);
  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 1;
  control.name = "move_xz";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;
  interactive_marker.controls.push_back(control);
  control.name = "move_y";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  interactive_marker.controls.push_back(control);
}

bool DeltaKinematicController::cartesianLinearPlanner(const geometry_msgs::Point &target_pose, std::vector<std::vector<double>> &joint_positions) {
  geometry_msgs::Point target_pick_pose;
  target_pick_pose.x = target_pose.x - ee_offset_.x;
  target_pick_pose.y = target_pose.y - ee_offset_.y;
  target_pick_pose.z = target_pose.z - ee_offset_.z;

  std::vector<double> target_joint_positions;
  if (!inverseKinematics(target_pick_pose, target_joint_positions)) {
    return false;
  }

  std::vector<geometry_msgs::Point> intermediate_poses = computeIntermediatePosesTo(target_pick_pose);
  for (auto const &intermediate_pose : intermediate_poses) {
    std::vector<double> intermediate_joint_positions;
    if (!inverseKinematics(intermediate_pose, intermediate_joint_positions)) {
      return false;
    }
    joint_positions.push_back(intermediate_joint_positions);
  }
  return true;  // intermediate_poses can be empty if distance is 0, i.e. joint_positions is unchanged
}

double DeltaKinematicController::computeDistance(const geometry_msgs::Point &from_pose, const geometry_msgs::Point &to_pose) {
  return std::sqrt(std::pow(to_pose.x-from_pose.x,2) + std::pow(to_pose.y-from_pose.y,2) + std::pow(to_pose.z-from_pose.z,2));
}

std::vector<geometry_msgs::Point> DeltaKinematicController::computeIntermediatePosesTo(const geometry_msgs::Point &target_pose) {
  std::vector<geometry_msgs::Point> intermediate_poses;
  geometry_msgs::Point ee_pose;
  if (forwardKinematics(getTrajectoryLastPositions(), ee_pose)) {
    double distance = computeDistance(ee_pose, target_pose);
    if (distance > 0) {
      int size = std::floor(distance/max_displacement_)+1;
      double displacement_x = (target_pose.x - ee_pose.x)/size;
      double displacement_y = (target_pose.y - ee_pose.y)/size;
      double displacement_z = (target_pose.z - ee_pose.z)/size;
      for (int i=0; i<size+1; i++) {
        geometry_msgs::Point pose;
        pose.x = ee_pose.x + i*displacement_x;
        pose.y = ee_pose.y + i*displacement_y;
        pose.z = ee_pose.z + i*displacement_z;
        intermediate_poses.push_back(pose);
      }
    }
  }
  return intermediate_poses;
}

std::vector<std::vector<double>> DeltaKinematicController::computeIntermediateStiffnessesTo(const std::vector<double> &target_joint_stiffnesses, const int &size) {
  std::vector<std::vector<double>> joint_stiffnesses;
  std::vector<double> old_joint_stiffnesses = getTrajectoryLastStiffnesses();

  std::vector<double> displacements;
  for (int j=0; j<old_joint_stiffnesses.size(); j++) {
    displacements.push_back((target_joint_stiffnesses.at(j) - old_joint_stiffnesses.at(j))/size);
  }

  for (int i=0; i<size; i++) {
    std::vector<double> point;
    for (int j=0; j<displacements.size(); j++) {
      point.push_back(old_joint_stiffnesses.at(j) + (i+1)*displacements.at(j));
    }
    joint_stiffnesses.push_back(point);
  }

  return joint_stiffnesses;
}

std::map<std::string, trajectory_msgs::JointTrajectory> DeltaKinematicController::computeJointTrajectories(const std::vector<std::vector<double>> &motor_positions, const std::vector<std::vector<double>> &motor_stiffness, const double &target_time_from_start, const double &previous_time_from_start) {
  std::map<std::string, trajectory_msgs::JointTrajectory> trajectories;
  for (int id=1; id<=3; id++) {
    trajectory_msgs::JointTrajectory trajectory;
    trajectory.header.stamp = ros::Time(0);
    trajectory.header.frame_id = getMotorName(id);
    trajectory.joint_names = getMotorJointNames(id);
    for (int i=0; i<motor_positions.size(); i++) {
      trajectory_msgs::JointTrajectoryPoint point;
      point.positions = {motor_positions.at(i).at(id-1), motor_stiffness.at(i).at(id-1)};
      if (i == 0 || i == motor_positions.size()-1) {
        point.velocities.resize(trajectory.joint_names.size());
        point.accelerations.resize(trajectory.joint_names.size());
      }
      point.time_from_start = ros::Duration(previous_time_from_start + (i+1)*(target_time_from_start-previous_time_from_start)/motor_positions.size());
      trajectory.points.push_back(point);
    }
    trajectories.insert(std::make_pair(getMotorName(id), trajectory));
  }
  return trajectories;
}

bool DeltaKinematicController::deltaStatePublisher(const std::vector<double> &motor_joints, const geometry_msgs::Point &ee_pose) {
  double R_motors = 0.1;  // radius from the center of the motors plane to each motor [meters]
  double R_ee = 0.0455;  // radius from the center of the end-effector plane to each arm [meters]
  double l_motors = 0.09;  // upper arm length [meters]
  double l_ee = 0.156;  // forearm length [meters]
  double phi_1 = 0;  // motor 1 angular position in the motors plane (front/central)
  double phi_2 = 2*M_PI/3;  // motor 2 angular position in the motors plane (behind/left)
  double phi_3 = 4*M_PI/3;  // motor 3 angular position in the motors plane (behind/right)
  double theta_1 = motor_joints.at(0) - 1;
  double theta_2 = motor_joints.at(1) - 1;
  double theta_3 = motor_joints.at(2) - 1;

  sensor_msgs::JointState msg;
  std::vector<double> phi = {phi_1, phi_2, phi_3};
  std::vector<double> theta = {theta_1, theta_2, theta_3};
  for (int i=0; i<3; i++) {
    std::string base_name = "delta_qbmove_" + std::to_string(i+1);

    geometry_msgs::Point upperarm_pose;  // upperarm_poses: [cos(phi_i)*(l_motors*cos(theta_i)+R_motors) -sin(phi_i)*(l_motors*cos(theta_i)+R_motors) -l_motors*sin(theta_i)]
    upperarm_pose.x = cos(phi.at(i))*(l_motors*cos(theta.at(i))+R_motors);
    upperarm_pose.y = sin(phi.at(i))*(l_motors*cos(theta.at(i))+R_motors);
    upperarm_pose.z = -l_motors*sin(theta.at(i));

    geometry_msgs::Point forearm_pose;  // forearm_poses: [cos(phi_i)*l_ee -sin(phi_i)*l_ee 0] + ee_pose
    forearm_pose.x = cos(phi.at(i))*R_ee + ee_pose.x;
    forearm_pose.y = sin(phi.at(i))*R_ee + ee_pose.y;
    forearm_pose.z = 0 + ee_pose.z;

    geometry_msgs::Point diff_pose;
    diff_pose.x = forearm_pose.x - upperarm_pose.x;
    diff_pose.y = forearm_pose.y - upperarm_pose.y;
    diff_pose.z = forearm_pose.z - upperarm_pose.z;

    geometry_msgs::Point forearm_to_upperarm_vector;
    forearm_to_upperarm_vector.x = std::cos(-phi.at(i))*diff_pose.x + -std::sin(-phi.at(i))*diff_pose.y;
    forearm_to_upperarm_vector.y = std::sin(-phi.at(i))*diff_pose.x + std::cos(-phi.at(i))*diff_pose.y;
    forearm_to_upperarm_vector.z = diff_pose.z;

    double pitch = -std::atan2(forearm_to_upperarm_vector.z, -forearm_to_upperarm_vector.x);
    double yaw = std::atan2(forearm_to_upperarm_vector.y, std::hypot(forearm_to_upperarm_vector.x, forearm_to_upperarm_vector.z));
    msg.name.push_back(base_name + "_free_down_joint");
    msg.name.push_back(base_name + "_free_l_joint");
    msg.position.push_back(pitch);
    msg.position.push_back(yaw);
  }
  msg.header.stamp = ros::Time::now();
  free_joint_state_publisher_.publish(msg);

  geometry_msgs::TransformStamped transform;
  transform.header.frame_id = "delta_base_frame_link";
  transform.header.stamp = ros::Time::now();
  transform.child_frame_id = "delta_ee_frame_link";
  transform.transform.translation.x = ee_pose.x;
  transform.transform.translation.y = ee_pose.y;
  transform.transform.translation.z = ee_pose.z;
  transform.transform.rotation.w = 1;  // others are 0
  tf_broadcaster_.sendTransform(transform);
  return true;
}

void DeltaKinematicController::fillMotorJointTrajectories(const std::map<std::string, trajectory_msgs::JointTrajectory> &motor_joint_trajectories) {
  for (auto const &trajectory : motor_joint_trajectories) {
    if (!motor_joint_trajectories_.count(trajectory.first)) {
      motor_joint_trajectories_.insert(std::make_pair(trajectory.first, trajectory.second));
      continue;
    }
    motor_joint_trajectories_.at(trajectory.first).points.insert(std::end(motor_joint_trajectories_.at(trajectory.first).points), std::begin(trajectory.second.points), std::end(trajectory.second.points));
  }
}

void DeltaKinematicController::filter(const std::vector<double> &b, const std::vector<double> &a, const std::vector<double> &x, std::vector<double> &y) {
  if (x.size() < b.size() - 1) {
    ROS_WARN_STREAM_NAMED("delta_controller", "Measurements are shorter than parameters.");
    return;
  }

  // builds first filtered values (to compute aj_yi for all aj in the following)
  if (y.size() < a.size() - 1) {
    if (y.empty()) {
      // y(1) = b1 * x(1) / a1
      y.push_back((b.front() * x.front()) / a.front());
    }

    auto it_a = a.begin();
    auto it_b = b.begin();
    while (y.size() < a.size() - 1) {
      // bj_xi = b1_xk + b2_xk-1 + ... + bnb_xk-nb-1
      double bj_xi = 0;
      for (auto it_x = x.begin(); it_x != x.begin() + y.size(); it_x++) {
        bj_xi += *it_x * *(it_b + y.size() - (it_x - x.begin()));
      }

      // aj_yi = a1_yk-1 + a2_yk-2 + ... + ana_yk-na-1
      double aj_yi = 0;
      for (auto it_y = y.begin(); it_y != y.end(); it_y++) {
        aj_yi += *it_y * *(it_a + y.size() - (it_y - y.begin()));
      }

      y.push_back((bj_xi - aj_yi) / a.front());
    }
  }

  int num_elements_x_exceed_y = x.size() - y.size();
  if (num_elements_x_exceed_y < 0) {  // few measurements
    ROS_WARN_STREAM_NAMED("delta_controller", "Previous filtered data exceeds measurements.");
    return;
  }
  if (num_elements_x_exceed_y == 0 && y.size() != 1) {  // already updated vector
    ROS_WARN_STREAM_NAMED("delta_controller", "There are no new measurements.");
    return;
  }

  // there are new measurements to be evaluated
  for (auto it_x = x.end() - num_elements_x_exceed_y; it_x != x.end(); it_x++) {
    // bj_xi = b1_xk + b2_xk-1 + ... + bnb_xk-nb-1
    double bj_xi = 0;
    for (auto it_b = b.begin(); it_b != b.end(); it_b++) {
      bj_xi += *it_b * *(it_x - (it_b - b.begin()));
    }

    // aj_yi = a1_yk-1 + a2_yk-2 + ... + ana_yk-na-1
    auto it_y = y.begin() + (it_x - x.begin());
    double aj_yi = 0;
    if (a.size() > 1) {
      for (auto it_a = a.begin() + 1; it_a != a.end(); it_a++) {
        aj_yi += *it_a * *(it_y - (it_a - a.begin()));
      }
    }

    y.push_back((bj_xi - aj_yi) / a.front());
  }

  return;
}

void DeltaKinematicController::filterMotorJointTrajectories(const std::vector<double> &b, const std::vector<double> &a) {
  for (int id=1; id<=3; id++) {
    if (!motor_joint_trajectories_.count(getMotorName(id))) {
      return;
    }
    filterMotorJointTrajectory(b, a, motor_joint_trajectories_.at(getMotorName(id)));
  }
}

void DeltaKinematicController::filterMotorJointTrajectory(const std::vector<double> &b, const std::vector<double> &a, trajectory_msgs::JointTrajectory &motor_joint_trajectory) {
  if (motor_joint_trajectory.points.size() <= filter_param_a_.size()) {
    ROS_WARN_STREAM_NAMED("delta_controller", "Trajectory points are less than filter parameters.");
    return;
  }

  std::vector<std::vector<double>> matrix_x(motor_joint_trajectory.joint_names.size()/* *3 */, std::vector<double>());
  std::vector<std::vector<double>> matrix_y(motor_joint_trajectory.joint_names.size()/* *3 */, std::vector<double>());
  for (auto const &point : motor_joint_trajectory.points) {
    for (int i=0; i<point.positions.size(); i++) {
      matrix_x.at(i).push_back(point.positions.at(i));
    }
//    for (int i=0; i<point.velocities.size(); i++) {
//      matrix_x.at(i + motor_joint_trajectory.joint_names.size()).push_back(point.velocities.at(i));
//    }
//    for (int i=0; i<point.accelerations.size(); i++) {
//      matrix_x.at(i + motor_joint_trajectory.joint_names.size()*2).push_back(point.accelerations.at(i));
//    }
  }

  for (int i=0; i<matrix_x.size(); i++) {
    matrix_y.at(i).insert(std::end(matrix_y.at(i)), std::begin(matrix_x.at(i)), std::begin(matrix_x.at(i))+filter_param_a_.size());
    filter(b, a, matrix_x.at(i), matrix_y.at(i));
  }

  for (int j=0; j<matrix_y.at(0).size(); j++) {
    for (int i=0; i<motor_joint_trajectory.joint_names.size(); i++) {
      motor_joint_trajectory.points.at(j).positions.at(i) = matrix_y.at(i).at(j);
    }
//    for (int i=0; i<motor_joint_trajectory.joint_names.size(); i++) {
//      motor_joint_trajectory.points.at(j).velocities.at(i + trajectory_y.joint_names.size()) = matrix_y.at(i).at(j);
//    }
//    for (int i=0; i<motor_joint_trajectory.joint_names.size(); i++) {
//      motor_joint_trajectory.points.at(j).accelerations.at(i + trajectory_y.joint_names.size()*2) = matrix_y.at(i).at(j);
//    }
  }
}

bool DeltaKinematicController::forwardKinematics(const std::vector<double> &motor_joints, geometry_msgs::Point &ee_pose) {
  double R_motors = 0.1;  // radius from the center of the motors plane to each motor [meters]
  double R_ee = 0.0455;  // radius from the center of the end-effector plane to each arm [meters]
  double l_motors = 0.09;  // upper arm length [meters]
  double l_ee = 0.156;  // forearm length [meters]
  double phi_1 = 0;  // motor 1 angular position in the motors plane (front/central)
  double phi_2 = 2*M_PI/3;  // motor 2 angular position in the motors plane (behind/left)
  double phi_3 = 4*M_PI/3;  // motor 3 angular position in the motors plane (behind/right)
  double theta_1 = motor_joints.at(0) - 1;
  double theta_2 = motor_joints.at(1) - 1;
  double theta_3 = motor_joints.at(2) - 1;
  double R = R_motors - R_ee;

  // The forward kinematics for a 3-limb-delta structure comes from the intersection of the three spheres with radius
  // equals to the forearm and centered at the end of each upper limb.
  // Actually several assumptions are taken to simplify the system of equations, e.g. even if it is not true, we can
  // consider all the forearms to be linked in the center of the end-effector plane and use "R = R_motors - R_ee".
  //
  // The generic system is as follows:
  //
  //  { (x-[cos(phi_1)*(l_motors*cos(theta_1)+R)])^2 + (y-[-sin(phi_1)*(l_motors*cos(theta_1)+R)])^2 + (z-[-l_motors*sin(theta_1)])^2 = l_ee^2
  //  { (x-[cos(phi_2)*(l_motors*cos(theta_2)+R)])^2 + (y-[-sin(phi_2)*(l_motors*cos(theta_2)+R)])^2 + (z-[-l_motors*sin(theta_2)])^2 = l_ee^2
  //  { (x-[cos(phi_3)*(l_motors*cos(theta_3)+R)])^2 + (y-[-sin(phi_3)*(l_motors*cos(theta_3)+R)])^2 + (z-[-l_motors*sin(theta_3)])^2 = l_ee^2
  //
  // which can have in general 0, 1 or 2 solutions.
  // Note that if 2 solutions exist, only one is feasible.

  // eq_1: (x-x_1)^2 + (y)^2 + (z-z_1)^2 == l_ee^2  (phi_1 is hardcoded to simplify the computation)
  double x_1 = l_motors*cos(theta_1) + R;
  double z_1 = -l_motors*sin(theta_1);

  // eq_2: (x-x_2)^2 + (y-y_2)^2 + (z-z_2)^2 == l_ee^2
  double x_2 = std::cos(phi_2)*(l_motors*cos(theta_2) + R);
  double y_2 = -std::tan(phi_2)*x_2;
  double z_2 = -l_motors*sin(theta_2);

  // eq_3: (x-x_3)^2 + (y-y_3)^2 + (z-z_3)^2 == l_ee^2
  double x_3 = std::cos(phi_3)*(l_motors*cos(theta_3) + R);
  double y_3 = -std::tan(phi_3)*x_3;
  double z_3 = -l_motors*sin(theta_3);

  // "eq_4 = eq_1-eq2" and "eq_5 = eq1-eq3" produce two linear equations which are more easily to be solved
  //  { eq_4 - k_1*eq_5 --> x = (a_1*z + b_1)/d
  //  { eq_4 - k_2*eq_5 --> y = (a_2*z + b_2)/d
  double w_1 = std::pow(x_1,2) + std::pow(z_1,2);
  double w_2 = std::pow(x_2,2) + std::pow(y_2,2) + std::pow(z_2,2);
  double w_3 = std::pow(x_3,2) + std::pow(y_3,2) + std::pow(z_3,2);

  double d = (x_1-x_2)*y_3 - (x_1-x_3)*y_2;
  if (d == 0) {
    return false;
  }

  double a_1 = y_2*(z_1-z_3) - y_3*(z_1-z_2);
  double b_1 = (y_3*(w_1-w_2) - y_2*(w_1-w_3))/2;
  double a_2 = -(x_1-x_3)*(z_1-z_2)+(x_1-x_2)*(z_1-z_3);
  double b_2 = ((x_1-x_3)*(w_1-w_2)-(x_1-x_2)*(w_1-w_3))/2;

  // Now we can solve for z by substitution, e.g. in eq_1
  double a = std::pow(a_1,2) + std::pow(a_2,2) + std::pow(d,2);
  double b = 2*(a_1*b_1 + a_2*b_2 - z_1*std::pow(d,2) - x_1*a_1*d);
  double c = std::pow(b_1,2) + std::pow(b_2,2) + std::pow(x_1,2)*std::pow(d,2) + std::pow(z_1,2)*std::pow(d,2) - 2*x_1*b_1*d - std::pow(l_ee,2)*std::pow(d,2);
  double delta = std::pow(b,2) - 4*a*c;
  if (delta < 0) {
    return false;
  }

  ee_pose.z = -0.5*(b-std::sqrt(delta))/a;
  ee_pose.x = (a_1*ee_pose.z + b_1)/d;
  ee_pose.y = -(a_2*ee_pose.z + b_2)/d;
  return true;
}

std::string DeltaKinematicController::getMotorName(const int &id) {
  return motor_names_.at(id-1);
}

double DeltaKinematicController::getMotorPosition(const int &id) {
  return motor_joints_.at(getMotorJointNames(id).front()).getPosition();
}

std::vector<double> DeltaKinematicController::getMotorPositions() {
  return {getMotorPosition(1), getMotorPosition(2), getMotorPosition(3)};
}

double DeltaKinematicController::getMotorStiffness(const int &id) {
  return motor_joints_.at(getMotorJointNames(id).back()).getPosition();
}

std::vector<double> DeltaKinematicController::getMotorStiffnesses() {
  return {getMotorStiffness(1), getMotorStiffness(2), getMotorStiffness(3)};
}

std::vector<std::string> DeltaKinematicController::getMotorJointNames(const int &id) {
  return motor_joint_names_.at(getMotorName(id));
}

trajectory_msgs::JointTrajectory DeltaKinematicController::getMotorJointTrajectory(const int &id) {
  return motor_joint_trajectories_.at(getMotorName(id));
}

std::vector<double> DeltaKinematicController::getTrajectoryLastPositions() {
  if (!motor_joint_trajectories_.count(getMotorName(1)) || getMotorJointTrajectory(1).points.empty()) {  // can be used also ID 2 or 3, they are filled and emptied together (ID 4, i.e. the gripper can be different)
    return getMotorPositions();
  }
  return {getMotorJointTrajectory(1).points.back().positions.at(0), getMotorJointTrajectory(2).points.back().positions.at(0), getMotorJointTrajectory(3).points.back().positions.at(0)};
}

std::vector<double> DeltaKinematicController::getTrajectoryLastStiffnesses() {
  if (!motor_joint_trajectories_.count(getMotorName(1)) || getMotorJointTrajectory(1).points.empty()) {  // can be used also ID 2 or 3, they are filled and emptied together (ID 4, i.e. the gripper can be different)
    return getMotorStiffnesses();
  }
  return {getMotorJointTrajectory(1).points.back().positions.at(1), getMotorJointTrajectory(2).points.back().positions.at(1), getMotorJointTrajectory(3).points.back().positions.at(1)};
}

double DeltaKinematicController::getTrajectoryLastTimeFromStart() {
  if (!motor_joint_trajectories_.count(getMotorName(1)) || getMotorJointTrajectory(1).points.empty()) {  // can be used also ID 2 or 3, they are filled and emptied together (ID 4, i.e. the gripper can be different)
    return 0;
  }
  return getMotorJointTrajectory(1).points.back().time_from_start.toSec();
}

bool DeltaKinematicController::init(hardware_interface::PositionJointInterface *robot_hw, ros::NodeHandle &control_nh, ros::NodeHandle &controller_nh) {
  node_handle_ = ros::NodeHandle();
  if (!controller_nh.getParam("device_names", motor_names_)) {
    ROS_ERROR_STREAM_NAMED("device_hw", "[DeviceHW] cannot retrieve 'joint_controllers' from the Parameter Service [" << controller_nh.getNamespace() << "].");
    return false;
  }
  controller_nh.param("max_displacement", max_displacement_, 0.001);  // default is 1mm
  use_waypoints_ = node_handle_.param<bool>("use_waypoints", false);
  use_interactive_markers_ = node_handle_.param<bool>("use_interactive_markers", false) && !use_waypoints_;  // cannot be used together (waypoint is dominant)
  ee_offset_.x = 0.0;
  ee_offset_.y = 0.024;
  ee_offset_.z = 0.142;

  for (auto const &device_name : motor_names_) {
    std::string position_joint = device_name + "_shaft_joint";
    std::string stiffness_joint = device_name + "_stiffness_preset_virtual_joint";
    motor_joints_.insert(std::make_pair(position_joint, robot_hw->getHandle(position_joint)));
    motor_joints_.insert(std::make_pair(stiffness_joint, robot_hw->getHandle(stiffness_joint)));
    motor_joint_names_.insert(std::make_pair(device_name, std::vector<std::string>({position_joint, stiffness_joint})));

    std::string device_action = device_name + "_position_and_preset_trajectory_controller/follow_joint_trajectory";
    motor_action_clients_.insert(std::make_pair(device_name, std::make_unique<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>>(node_handle_control_, device_action, false)));
  }
  robot_hw->clearClaims();  // this controller never set commands directly to the handled devices, i.e. use their Actions

  if (use_interactive_markers_) {
    initMarkers();
    feedback_active_ = false;
  }
  if (use_waypoints_) {
    filter_trajectory_ = controller_nh.param<bool>("filter_trajectory", false);
    if (filter_trajectory_) {
      filter_param_b_ = controller_nh.param<std::vector<double>>("filter_param_b", {0.7});
      filter_param_a_ = controller_nh.param<std::vector<double>>("filter_param_a", {1, -0.3});
    }
    waypoint_namespace_ = node_handle_.param<std::string>("waypoint_namespace", "waypoints");
    waypoint_setup_timer_ = node_handle_.createWallTimer(ros::WallDuration(2.0), [this](auto e){ this->startWaypointTrajectory(); }, true);  // oneshot
  }

  free_joint_state_publisher_ = control_nh.advertise<sensor_msgs::JointState>("joint_states", 1);
  target_poses_sub_ = controller_nh.subscribe("target_poses", 1, &DeltaKinematicController::targetPosesCallback, this);

  return true;
}

void DeltaKinematicController::initMarkers() {
  interactive_commands_server_ = std::make_unique<interactive_markers::InteractiveMarkerServer>("qbdelta_end_effector_interactive_commands");

  controls_.header.frame_id = "delta_base_frame_link";
  controls_.name = "qbdelta_end_effector_position_reference_controls";
  controls_.description = "qbdelta end-effector 3D pose reference.";
  controls_.scale = 0.05;
  buildCube(controls_);
  buildEndEffectorControl(controls_);

  interactive_commands_server_->insert(controls_, std::bind(&DeltaKinematicController::interactiveMarkerCallback, this, std::placeholders::_1));
  interactive_commands_server_->applyChanges();
}

void DeltaKinematicController::interactiveMarkerCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
  if (feedback->event_type == feedback->MOUSE_UP) {
    feedback_active_ = false;
    return;
  }
  if (feedback->event_type == feedback->MOUSE_DOWN) {
    feedback_position_old_ = feedback->pose.position;
    feedback_active_ = true;
    return;
  }

  double feedback_distance = 0;
  if (feedback_active_ && feedback->event_type == feedback->POSE_UPDATE) {
    feedback_distance = computeDistance(feedback->pose.position, feedback_position_old_);
    feedback_position_old_ = feedback->pose.position;
  }
  if (feedback_distance > 0.001) {  // at least 1mm in cartesian space
    geometry_msgs::PointStamped target_pose;
    target_pose.header = feedback->header;
    target_pose.point = feedback->pose.position;
    targetPosesCallback(target_pose);
  }
}

bool DeltaKinematicController::inverseKinematics(const geometry_msgs::Point &ee_pose, std::vector<double> &joint_positions) {
  double R_motors = 0.1;  // radius from the center of the motors plane to each motor [meters]
  double R_ee = 0.0375;  // radius from the center of the end-effector plane to each arm [meters]
  double l_motors = 0.09;  // upper arm length [meters]
  double l_ee = 0.16;  // forearm length [meters]
  double phi_1 = 0;  // motor 1 angular position in the motors plane (behind/right)
  double phi_2 = 2*M_PI/3;  // motor 2 angular position in the motors plane (front/central)
  double phi_3 = 4*M_PI/3;  // motor 3 angular position in the motors plane (behind/left)
  double R = R_motors - R_ee;
  joint_positions.resize(3);

  auto arm_ik = [&](const geometry_msgs::Point &ee_pose, const double &phi, double &joint_position) {
    auto acos_safe = [](const double &num, const double &den, double &angle) {
      angle = std::acos(num/den);  // it may be NaN
      return std::abs(num) < std::abs(den);
    };

    double x = std::cos(phi)*ee_pose.x + std::sin(phi)*ee_pose.y - R;
    double y = -std::sin(phi)*ee_pose.x + std::cos(phi)*ee_pose.y;
    double z = ee_pose.z;
    double theta_1, theta_2;
    if (!acos_safe(y, l_ee, theta_1) || !acos_safe((std::pow(x,2) + std::pow(y,2) + std::pow(z,2) - std::pow(l_motors,2) - std::pow(l_ee,2)), (2*l_motors*l_ee*std::sin(theta_1)), theta_2)) {
      return false;
    }
    double c_1 = l_ee*std::cos(theta_2)*std::sin(theta_1) + l_motors;
    double c_2 = l_ee*std::sin(theta_2)*std::sin(theta_1);
    joint_position = -(std::atan2(-c_2*x + c_1*z, c_1*x + c_2*z) - 1);  // includes motor offset
    return true;
  };

  return arm_ik(ee_pose, phi_1, joint_positions.at(0)) && arm_ik(ee_pose, phi_2, joint_positions.at(1)) && arm_ik(ee_pose, phi_3, joint_positions.at(2));
}

void DeltaKinematicController::move() {
  for (auto const &trajectory : motor_joint_trajectories_) {
    move(trajectory.second, trajectory.first);
  }
}

void DeltaKinematicController::move(const trajectory_msgs::JointTrajectory &joint_trajectory, const std::string &motor) {
  control_msgs::FollowJointTrajectoryGoal goal;
  goal.trajectory = joint_trajectory;
  motor_action_clients_.at(motor)->sendGoal(goal,
                                            std::bind(&DeltaKinematicController::actionDoneCallback, this, std::placeholders::_1, std::placeholders::_2, motor),
                                            std::bind(&DeltaKinematicController::actionActiveCallback, this, motor),
                                            std::bind(&DeltaKinematicController::actionFeedbackCallback, this, std::placeholders::_1, motor));
}

bool DeltaKinematicController::parseVector(const XmlRpc::XmlRpcValue &xml_value, const int &size, std::vector<double> &vector) {
  if (xml_value.size() != size) {
    ROS_ERROR_STREAM_NAMED("delta_controller", "Fails while setting the joint trajectory (joints size mismatch).");
    return false;
  }
  for (int j=0; j<xml_value.size(); j++) {
    vector.push_back(xmlCast<double>(xml_value[j]));
  }
  return true;
}

void DeltaKinematicController::parseWaypointTrajectory(ros::NodeHandle &node_handle) {
  XmlRpc::XmlRpcValue waypoints;
  if (!node_handle.getParam(waypoint_namespace_, waypoints)) {
    ROS_ERROR_STREAM_NAMED("delta_controller", "No waypoints specified in the Parameter Server under [" << node_handle.getNamespace() << "/" + waypoint_namespace_ + "].");
    return;
  }

  // erase all but last point
  for (auto &trajectory : motor_joint_trajectories_) {
    trajectory.second.points.erase(trajectory.second.points.begin(), trajectory.second.points.end()-1);
    trajectory.second.points.front().time_from_start = ros::Duration(0.1);
  }

  trajectory_msgs::JointTrajectory gripper_joint_trajectory;
  for (int i=0; i<waypoints.size(); i++) {
    trajectory_msgs::JointTrajectoryPoint point;

    if (!waypoints[i].hasMember("time") && !waypoints[i].hasMember("duration")) {
      continue;
    }

    if (!waypoints[i].hasMember("gripper") || !parseVector(waypoints[i]["gripper"], 2, point.positions)) {
      continue;
    }
    point.velocities = std::vector<double>(point.positions.size(), 0.0);
    point.accelerations = std::vector<double>(point.positions.size(), 0.0);

    // set joint waypoint for the whole time interval (be aware of joint trajectory interpolation)
    for (int j=0; j<(waypoints[i].hasMember("time") ? waypoints[i]["time"].size() : waypoints[i]["duration"].size()); j++) {
      point.time_from_start = ros::Duration(waypoints[i].hasMember("time") ? static_cast<double>(waypoints[i]["time"][j]) : (gripper_joint_trajectory.points.empty() ? 0 : gripper_joint_trajectory.points.back().time_from_start.toSec()) + static_cast<double>(waypoints[i]["duration"][j]));
      gripper_joint_trajectory.points.push_back(point);
    }
  }
  if (!gripper_joint_trajectory.points.empty()) {
    gripper_joint_trajectory.header.frame_id = getMotorName(4);
    gripper_joint_trajectory.header.stamp = ros::Time(0);
    gripper_joint_trajectory.joint_names = getMotorJointNames(4);
    std::map<std::string, trajectory_msgs::JointTrajectory> gripper_joint_trajectory_map;  //TODO: this is not the best solution
    gripper_joint_trajectory_map.insert(std::make_pair(getMotorName(4), gripper_joint_trajectory));
    fillMotorJointTrajectories(gripper_joint_trajectory_map);
  }

  for (int i=0; i<waypoints.size(); i++) {
    if (!waypoints[i].hasMember("time") && !waypoints[i].hasMember("duration")) {
      continue;
    }

    std::vector<std::vector<double>> joint_positions;
    std::vector<double> end_effector_position;
    if (waypoints[i].hasMember("end_effector") && parseVector(waypoints[i]["end_effector"], 3, end_effector_position)) {
      geometry_msgs::Point target_pose;
      target_pose.x = end_effector_position.at(0);
      target_pose.y = end_effector_position.at(1);
      target_pose.z = end_effector_position.at(2);
      if (!cartesianLinearPlanner(target_pose, joint_positions)) {
        ROS_WARN_STREAM_NAMED("delta_controller", "The Cartesian Linear Planner has produced a solution which cannot be inverted.");
        continue;
      }
    }
    else {
      joint_positions.push_back(getTrajectoryLastPositions());
    }

    std::vector<double> target_joint_stiffnesses;
    if (!waypoints[i].hasMember("joint_stiffness") || !parseVector(waypoints[i]["joint_stiffness"], 3, target_joint_stiffnesses)) {
      target_joint_stiffnesses = getTrajectoryLastStiffnesses();
    }

    std::vector<std::vector<double>> joint_stiffnesses;
    joint_stiffnesses = computeIntermediateStiffnessesTo(target_joint_stiffnesses, joint_positions.size());

    std::map<std::string, trajectory_msgs::JointTrajectory> motor_joint_trajectories;
    double time_from_start = waypoints[i].hasMember("time") ? static_cast<double>(waypoints[i]["time"][0]) : getTrajectoryLastTimeFromStart() + static_cast<double>(waypoints[i]["duration"][0]);
    motor_joint_trajectories = computeJointTrajectories(joint_positions, joint_stiffnesses, time_from_start, getTrajectoryLastTimeFromStart());
    fillMotorJointTrajectories(motor_joint_trajectories);

    // set joint waypoint for the whole time interval (be aware of joint trajectory interpolation)
    for (int j=1; j<(waypoints[i].hasMember("time") ? waypoints[i]["time"].size() : waypoints[i]["duration"].size()); j++) {
      double time_from_start = waypoints[i].hasMember("time") ? static_cast<double>(waypoints[i]["time"][j]) : getTrajectoryLastTimeFromStart() + static_cast<double>(waypoints[i]["duration"][j]);
      motor_joint_trajectories = computeJointTrajectories({joint_positions.back()}, {joint_stiffnesses.back()}, time_from_start, getTrajectoryLastTimeFromStart());
      fillMotorJointTrajectories(motor_joint_trajectories);
    }
  }
}

void DeltaKinematicController::startWaypointTrajectory() {
  parseWaypointTrajectory(node_handle_);
  if (filter_trajectory_) {
    filterMotorJointTrajectories(filter_param_b_, filter_param_a_);
  }
  move();
}

//TODO: add a general callback to set pose together with stiffness values and timing constraints
void DeltaKinematicController::targetPosesCallback(const geometry_msgs::PointStamped &target_pose) {
  std::vector<std::vector<double>> motor_positions;
  if (!cartesianLinearPlanner(target_pose.point, motor_positions)) {
    return;
  }

  // motor stiffnesses are constant for this callback
  motor_joint_trajectories_ = computeJointTrajectories(motor_positions, std::vector<std::vector<double>>(motor_positions.size(), getMotorStiffnesses()), 2.0, 0.5);
  move();
}

void DeltaKinematicController::update(const ros::Time &time, const ros::Duration &period) {
  // update the end-effector pose and the state of all free joints (arm joints)
  std::vector<double> motor_joints(getMotorPositions());
  geometry_msgs::Pose ee_pose;
  if (forwardKinematics(motor_joints, ee_pose.position)) {
    deltaStatePublisher(motor_joints, ee_pose.position);
  }
  if (use_interactive_markers_) {
    interactive_commands_server_->setPose(controls_.name, ee_pose);
    interactive_commands_server_->applyChanges();
  }
}

template<class T>
T DeltaKinematicController::xmlCast(XmlRpc::XmlRpcValue xml_value) {
  // XmlRpcValue does not handle conversion among types but throws an exception if an improper cast is invoked
  if (xml_value.getType() == XmlRpc::XmlRpcValue::TypeBoolean) {
    return static_cast<bool>(xml_value);
  }
  if (xml_value.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
    return static_cast<double>(xml_value);
  }
  if (xml_value.getType() == XmlRpc::XmlRpcValue::TypeInt) {
    return static_cast<int>(xml_value);
  }
  ROS_ERROR_STREAM_NAMED("delta_controller", "Fails while casting the XmlRpcValue [" << xml_value << "].");
  return 0;
}

PLUGINLIB_EXPORT_CLASS(qb_chain_controllers::DeltaKinematicController, controller_interface::ControllerBase)