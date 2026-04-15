/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Sachin Chitta, Michael Lautman*/

// MoveIt
#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit/robot_model/robot_model.hpp>
#include <moveit/robot_state/robot_state.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;

  node_options.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("robot_model_and_state_tutorial", node_options);
  const auto& LOGGER = node->get_logger();

  robot_model_loader::RobotModelLoader robot_model_loader(node);
  const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
  RCLCPP_INFO(LOGGER, "Model frame: %s", kinematic_model->getModelFrame().c_str());

  moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(kinematic_model));
  robot_state->setToDefaultValues();
  const moveit::core::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("arm");

  const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();

  // Get Joint Values
  std::vector<double> joint_values;
  robot_state->copyJointGroupPositions(joint_model_group, joint_values);
  for (std::size_t i = 0; i < joint_names.size(); ++i)
  {
    RCLCPP_INFO(LOGGER, "Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
  }

  // Joint Limits
  joint_values[0] = 5.57;
  robot_state->setJointGroupPositions(joint_model_group, joint_values);

  RCLCPP_INFO_STREAM(LOGGER, "Current state is " << (robot_state->satisfiesBounds() ? "valid" : "not valid"));

  robot_state->enforceBounds();
  RCLCPP_INFO_STREAM(LOGGER, "Current state is " << (robot_state->satisfiesBounds() ? "valid" : "not valid"));

  // Forward Kinematics
  robot_state->setToRandomPositions(joint_model_group);
  const Eigen::Isometry3d& end_effector_state = robot_state->getGlobalLinkTransform("End_Effector");

  /* Print end-effector pose. Remember that this is in the model frame */
  RCLCPP_INFO_STREAM(LOGGER, "Translation: \n" << end_effector_state.translation() << "\n");
  RCLCPP_INFO_STREAM(LOGGER, "Rotation: \n" << end_effector_state.rotation() << "\n");

  // Inverse Kinematics
  double timeout = 0.1;
  bool found_ik = robot_state->setFromIK(joint_model_group, end_effector_state, timeout);

  // Now, we can print out the IK solution (if found):
  if (found_ik)
  {
    robot_state->copyJointGroupPositions(joint_model_group, joint_values);
    for (std::size_t i = 0; i < joint_names.size(); ++i)
    {
      RCLCPP_INFO(LOGGER, "Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
    }
  }
  else
  {
    RCLCPP_INFO(LOGGER, "Did not find IK solution");
  }

  // Get the Jacobian
  Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
  Eigen::MatrixXd jacobian;
  robot_state->getJacobian(joint_model_group, robot_state->getLinkModel(joint_model_group->getLinkModelNames().back()),
                           reference_point_position, jacobian);
  RCLCPP_INFO_STREAM(LOGGER, "Jacobian: \n" << jacobian << "\n");

  rclcpp::shutdown();
  return 0;
}
