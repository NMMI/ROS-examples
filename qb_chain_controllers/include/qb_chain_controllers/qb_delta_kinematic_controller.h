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

#ifndef QB_CHAIN_CONTROLLERS_H
#define QB_CHAIN_CONTROLLERS_H

// Standard libraries
#include <regex>

// ROS libraries
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <controller_interface/controller.h>
#include <geometry_msgs/PointStamped.h>
#include <hardware_interface/joint_command_interface.h>
#include <interactive_markers/interactive_marker_server.h>
#include <pluginlib/class_list_macros.hpp>
#include <sensor_msgs/JointState.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <trajectory_msgs/JointTrajectory.h>

// internal libraries

namespace qb_chain_controllers {
/**
 * This controller is made to simplify the usage and the user interaction with the \em delta device. Actually it is an
 * upper level controller which relies on the low-level trajectory controller provided by the \p qbMoveHW interfaces.
 * This controller also compute the forward kinematics to properly visualize the whole kinematic structure in \p rviz.
 * Additionally it provides waypoints control mode (similar to the single device and chained mode), and an interactive
 * marker control mode (similar to the one of the single \em qbmove device).
 */
class DeltaKinematicController : public controller_interface::Controller<hardware_interface::PositionJointInterface> {
 public:
  /**
   * Start the async spinner and do nothing else. The real initialization is done later by \p init().
   * \sa init()
   */
  DeltaKinematicController();

  /**
   * Stop the async spinner.
   */
  ~DeltaKinematicController() override;

  /**
   * The init function is called to initialize the controller from a non-realtime thread with a pointer to the hardware
   * interface itself, instead of a pointer to a RobotHW.
   * \param robot_hw The specific hardware interface used by this controller.
   * \param root_nh A NodeHandle in the root of the controller manager namespace. This is where the ROS interfaces are
   * setup (publishers, subscribers, services).
   * \param controller_nh A NodeHandle in the namespace from which the controller should read its configuration,
   * and where it should set up its ROS interface.
   * \return \p true if initialization was successful and the controller is ready to be started.
   */
  bool init(hardware_interface::PositionJointInterface *robot_hw, ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) override;

  /**
   * This is called from within the realtime thread just before the first call to \ref update.
   * \param time The current time.
   */
  void starting(const ros::Time &time) override {}

  /**
   * This is called from within the realtime thread just after the last update call before the controller is stopped.
   * \param time The current time.
   */
  void stopping(const ros::Time &time) override {}

  /**
   * Update the \em delta state. This is called periodically by the realtime thread when the controller is running.
   * \param time The current time.
   * \param period The time passed since the last call to \ref update.
   */
  void update(const ros::Time &time, const ros::Duration &period) override;

 protected:
  ros::CallbackQueuePtr callback_queue_;
  ros::AsyncSpinner spinner_;
  ros::NodeHandle node_handle_;
  ros::NodeHandle node_handle_control_;
  ros::Publisher free_joint_state_publisher_;
  ros::Subscriber target_poses_sub_;
  ros::WallTimer waypoint_setup_timer_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  std::unique_ptr<interactive_markers::InteractiveMarkerServer> interactive_commands_server_;
  visualization_msgs::InteractiveMarker controls_;
  geometry_msgs::Point feedback_position_old_;
  bool feedback_active_;

  std::vector<std::string> motor_names_;
  std::map<std::string, std::unique_ptr<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>>> motor_action_clients_;
  std::map<std::string, hardware_interface::JointHandle> motor_joints_;
  std::map<std::string, std::vector<std::string>> motor_joint_names_;
  std::map<std::string, trajectory_msgs::JointTrajectory> motor_joint_trajectories_;
  geometry_msgs::Point ee_offset_;

  bool use_interactive_markers_;
  bool use_waypoints_;
  std::string waypoint_namespace_;
  double max_displacement_;
  bool filter_trajectory_;
  std::vector<double> filter_param_a_;
  std::vector<double> filter_param_b_;

  /**
   * Do nothing apart from debug info.
   * \param controller The action controller name.
   */
  void actionActiveCallback(const std::string &controller);

  /**
   * Restart the waypoint trajectory automatically if waypoints have been retrieved during the initialization.
   * \param state The final state of the action.
   * \param result The error code and error message (\p 0 if succeeds).
   * \param controller The action controller name.
   * \sa startWaypointTrajectory()
   */
  void actionDoneCallback(const actionlib::SimpleClientGoalState &state, const control_msgs::FollowJointTrajectoryResultConstPtr &result, const std::string &controller);

  /**
   * Do nothing apart from debug info.
   * \param feedback The action feedback state.
   * \param controller The action controller name.
   */
  void actionFeedbackCallback(const control_msgs::FollowJointTrajectoryFeedbackConstPtr &feedback, const std::string &controller);

  /**
   * Create a cubic marker which can be moved interactively in all the Cartesian space. The marker represent the
   * position of the end-effector of the delta.
   * \param interactive_marker The Interactive Marker structure to be filled.
   * \sa initMarkers()
   */
  void buildCube(visualization_msgs::InteractiveMarker &interactive_marker);

  /**
   * Add six distinct interactive controls to the given interactive marker. They let to move the marker in the Cartesian
   * space by limiting the motion to a single principal axis or plane, e.g. only along x axis or in the yz plane.
   * \param interactive_marker The Interactive Marker structure to be filled.
   * \sa initMarkers()
   */
  void buildEndEffectorControl(visualization_msgs::InteractiveMarker &interactive_marker);

  /**
   * Compute the joint trajectory of all the upper motors from the last computed value to the given target end-effector
   * position in a linearized fashion w.r.t. to the Cartesian space. If there are no last values, current shaft
   * positions of the upper motors are taken.
   * \param target_pose The target end-effector position.
   * \param[out] joint_positions The vector of all the shaft position of all the upper motors.
   * \return \p true on success.
   * \sa computeIntermediatePosesTo(), inverseKinematics()
   */
  bool cartesianLinearPlanner(const geometry_msgs::Point &target_pose, std::vector<std::vector<double>> &joint_positions);

  /**
   * Compute the absolute 3D distance between the given points.
   * \param from_pose The point from which to compute the distance.
   * \param to_pose The point to which to compute the distance.
   * \return The computed distance.
   */
  double computeDistance(const geometry_msgs::Point &from_pose, const geometry_msgs::Point &to_pose);

  /**
   * Compute a linear-in-space interpolation from the last computed value to the given target end-effector position.
   * If there are no last values stored, current joint positions are taken to compute the forward kinematics.
   * \param target_pose The target end-effector position.
   * \return The vector of all the intermediate position of the end-effector.
   * \sa cartesianLinearPlanner(), computeDistance(), forwardKinematics()
   */
  std::vector<geometry_msgs::Point> computeIntermediatePosesTo(const geometry_msgs::Point &target_pose);

  /**
   * Compute the linearized trajectory from the last computed value to the given target joint stiffness of all the
   * upper motors. If there are no last values stored, current stiffness values are taken.
   * \param target_joint_stiffnesses The target stiffness value of all the upper motors.
   * \param size The number of points that needs to be generated.
   * \return The vector of all the intermediate stiffness value of all the upper motors.
   * \sa parseWaypointTrajectory()
   */
  std::vector<std::vector<double>> computeIntermediateStiffnessesTo(const std::vector<double> &target_joint_stiffnesses, const int &size);

  /**
   * Compute a segment of trajectories for all the upper motors from the given parameters.
   * \note Be aware that \p motor_positions must be strictly related to \p motor_stiffness, i.e. same size and meanings.
   * \param motor_positions The vector of shaft positions of all the upper motors.
   * \param motor_stiffness The vector of stiffness values of all the upper motors.
   * \param target_time_from_start The final \p time_from_start of this segment of trajectory.
   * \param previous_time_from_start The initial \p time_from_start of this segment of trajectory.
   * \return The map of joint trajectories of all the upper motors filled with the given segment data.
   */
  std::map<std::string, trajectory_msgs::JointTrajectory> computeJointTrajectories(const std::vector<std::vector<double>> &motor_positions, const std::vector<std::vector<double>> &motor_stiffness, const double &target_time_from_start, const double &previous_time_from_start);

  /**
   * Compute all the free joint positions (i.e. of the arms) of the \em delta device and publish them to the Joint
   * State Publisher. Also update the end-effector floating \p tf frame with the given end-effector position.
   * \param motor_joints The upper motor position.
   * \param ee_pose The end-effector position w.r.t. the upper plate.
   * \return \p true on success.
   * \sa update(), forwardKinematics()
   */
  bool deltaStatePublisher(const std::vector<double> &motor_joints, const geometry_msgs::Point &ee_pose);

  /**
   * Append the given trajectories to the joint trajectories private map.
   * \param motor_joint_trajectories The map with new joint trajectories (either for the gripper and upper motors).
   * \sa parseWaypointTrajectory()
   */
  void fillMotorJointTrajectories(const std::map<std::string, trajectory_msgs::JointTrajectory> &motor_joint_trajectories);

  /**
   * Filter data following a discrete parametrization of the samples.
   * \param b The vector of numerator coefficients, i.e. the ones applied to the new measurements.
   * \param a The vector of denominator coefficients, i.e. the ones applied to the old data.
   * \param x The new measurements.
   * \param[out] y The whole filtered data.
   */
  void filter(const std::vector<double> &b, const std::vector<double> &a, const std::vector<double> &x, std::vector<double> &y);

  /**
   * Filter all the trajectories stored in the joint trajectories private map.
   * \param b The vector of numerator coefficients, i.e. the ones applied to the new measurements.
   * \param a The vector of denominator coefficients, i.e. the ones applied to the old data.
   * \sa filterMotorJointTrajectory()
   */
  void filterMotorJointTrajectories(const std::vector<double> &b, const std::vector<double> &a);

  /**
   * Filter the given joint trajectory, by considering the whole data as a vector of new measurements (each point is
   * a sample).
   * \param b The vector of numerator coefficients, i.e. the ones applied to the new measurements.
   * \param a The vector of denominator coefficients, i.e. the ones applied to the old data.
   * \param[out] motor_joint_trajectory The joint trajectory to be filtered which is modified in place.
   * \sa filterMotorJointTrajectories(), filter()
   */
  void filterMotorJointTrajectory(const std::vector<double> &b, const std::vector<double> &a, trajectory_msgs::JointTrajectory &motor_joint_trajectory);

  /**
   * Compute the forward kinematics of the \em delta device.
   * \param joint_positions The upper motor position.
   * \param[out] ee_pose The computed end-effector position w.r.t. the upper plate.
   * \return \p true on success.
   * \sa deltaStatePublisher(), inverseKinematics()
   */
  bool forwardKinematics(const std::vector<double> &joint_positions, geometry_msgs::Point &ee_pose);

  /**
   * \return The device hardware interface name of the given device ID.
   */
  std::string getMotorName(const int &id);

  /**
   * \return The shaft position of the given device ID.
   */
  double getMotorPosition(const int &id);

  /**
   * \return The shaft position of all the upper motors of the delta.
   */
  std::vector<double> getMotorPositions();

  /**
   * \return The stiffness value of the given device ID.
   */
  double getMotorStiffness(const int &id);

  /**
   * \return The stiffness value of all the upper motors of the delta.
   */
  std::vector<double> getMotorStiffnesses();

  /**
   * \return The joint names of the given device ID.
   */
  std::vector<std::string> getMotorJointNames(const int &id);

  /**
   * \return The stored joint trajectory of the given device ID.
   */
  trajectory_msgs::JointTrajectory getMotorJointTrajectory(const int &id);

  /**
   * \return The last stored shaft position of all the upper motors of the delta.
   */
  std::vector<double> getTrajectoryLastPositions();

  /**
   * \return The last stored stiffness value of all the upper motors of the delta.
   */
  std::vector<double> getTrajectoryLastStiffnesses();

  /**
   * \return The last stored \p time_from_start (\p 0 if empty).
   */
  double getTrajectoryLastTimeFromStart();

  /**
   * Initialize the \p interactive_markers::InteractiveMarkerServer and the marker displayed in \p rviz to control the
   * end-effector position of the \em delta device. This method is called during the controller initialization if
   * required by the user (cannot be started whether waypoints are active).
   * \sa buildCube(), buildEndEffectorControl(), init()
   */
  void initMarkers();

  /**
   * If waypoints are not used, this become the core method of the class, where commands are computed w.r.t. the
   * previous state of the interactive marker and the computed joint trajectory is sent to each low level controllers,
   * e.g. the joint trajectory controller of each \p qbMoveHW.
   * \param feedback The feedback state of the interactive marker, provided by rviz.
   * \sa targetPosesCallback()
   */
  void interactiveMarkerCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

  /**
   * Compute the inverse kinematics of the \em delta device.
   * \param ee_pose The end-effector position w.r.t. the upper plate.
   * \param[out] joint_positions The computed upper motor position.
   * \return \p true on success.
   * \sa deltaStatePublisher(), forwardKinematics()
   */
  bool inverseKinematics(const geometry_msgs::Point &ee_pose, std::vector<double> &joint_positions);

  /**
   * Build a \p control_msgs::FollowJointTrajectoryGoal with the given joint trajectory and make a call to the Action
   * Server relative to the provided motor controller (using the just created goal).
   * \param joint_trajectory The waypoint trajectory properly filled for all the joints of the controller.
   * \param motor The action controller name.
   * \sa move()
   */
  void move(const trajectory_msgs::JointTrajectory &joint_trajectory, const std::string &motor);

  /**
   * Make all the calls to the Action Servers relative to all the trajectories previously parsed (either from waypoint
   * or interactive markers).
   * \sa move(const trajectory_msgs::JointTrajectory &, const std::string &)
   */
  void move();

  /**
   * Parse the given \p XmlRpcValue as a \p std::vector, since the \p XmlRpc::XmlRpcValue class does not handle this
   * conversion yet.
   * \param xml_value The value retrieved from the Parameter Server to be parsed as a \p std::vector.
   * \param controller The action controller name.
   * \param[out] vector The \p std::vector parsed from the \p XmlRpcValue.
   * \return \p true on success.
   * \sa parseWaypointTrajectory(), xmlCast()
   */
  bool parseVector(const XmlRpc::XmlRpcValue &xml_value, const int &size, std::vector<double> &vector);

  /**
   * Parse all the waypoints set up in the Parameter Server at \p <robot_namespace>/waypoints.
   * The waypoints are composed by a vector where each element must contains:
   * - a field named \p time which can be either a single value or an interval for which other values hold;
   * And one or more of the followings:
   * - a list named \p end_effector containing [x, y, z] coordinates for the delta end-effector;
   * - a list named \p joint_stiffness containing the stiffness value for each of the three upper motors;
   * - a list named \p gripper filled with the gripper reference position and its stiffness value;
   * Whenever these are not present in the Parameter Server, it is assigned the last available value.
   * \warning Please note that each of the above list must contain the exact number of required values.
   * Once waypoint are retrieved, the Cartesian Planner is called to compute the relative joint trajectory (linear in
   * the Cartesian space) for each \em qbmove device. This is done through an inverse kinematics processing.
   * \param node_handle The Node Handle in which the waypoints are parsed, under \p .../waypoints (if present).
   * \sa cartesianLinearPlanner(), computeJointTrajectories(), fillMotorJointTrajectories(), parseVector(), xmlCast()
   */
  void parseWaypointTrajectory(ros::NodeHandle &node_handle);

  /**
   * Parse all the waypoints, filter the whole generated joint trajectory and send the commands to the devices through
   * their relative Action server. This can be helpful in a cyclic waypoint behavior to recompute the trajectory from
   * the last reached position (which can be different from the initial one).
   * \sa filterMotorJointTrajectories(), move(), parseWaypointTrajectory()
   */

  void startWaypointTrajectory();
  /**
   * Call the Cartesian Planner to compute the joint trajectory from the current position of the end-effector to the
   * given target. The trajectory is filled in a proper structure which is sent to the Action server of each \em qbmove.
   * \param msg The target end-effector position.
   * \sa cartesianLinearPlanner(), computeJointTrajectories(), move()
   */
  void targetPosesCallback(const geometry_msgs::PointStamped &msg);

  /**
   * Cast an \p XmlRpcValue from \p TypeDouble, \p TypeInt or \p TypeBoolean to the specified template type. This is
   * necessary since XmlRpc does not handle conversion among basic types: it throws an exception if an improper cast is
   * invoked though (e.g. from int to double).
   * \tparam T The type to cast to.
   * \param xml_value The wrong-casted value.
   * \return The casted value.
   * \sa parseWaypointTrajectory(), parseVector()
   */
  template<class T>
  T xmlCast(XmlRpc::XmlRpcValue xml_value);
};
}  // namespace qb_chain_controllers

#endif // QB_CHAIN_CONTROLLERS_H