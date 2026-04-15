// https://moveit.picknik.ai/main/doc/tutorials/your_first_project/your_first_project.html

#include <chrono>
#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;

  node_options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<rclcpp::Node>( "so_arm_100_moveit", node_options);
  const auto& logger = node->get_logger();

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "arm");

  // Inspect properties of the move_group_interface  
  RCLCPP_INFO_STREAM(logger, "name: " << move_group_interface.getName());

  for (auto && value : move_group_interface.getNamedTargets()) {
    RCLCPP_INFO_STREAM(logger, "named target: " << value);
  }

  RCLCPP_INFO_STREAM(logger, "planning frame: "
      << move_group_interface.getPlanningFrame());

#if 0
  for (auto && value : move_group_interface.getJointModelGroupNames()) {
    RCLCPP_INFO_STREAM(logger, "joint model group name: " << value);
  }
  
  for (auto && value : move_group_interface.getJointNames()) {
    RCLCPP_INFO_STREAM(logger, "joint name: " << value);
  }

  for (auto && value : move_group_interface.getLinkNames()) {
    RCLCPP_INFO_STREAM(logger, "link name: " << value);
  }

  RCLCPP_INFO_STREAM(logger, "variable count: "
      << move_group_interface.getVariableCount());

  RCLCPP_INFO_STREAM(logger, "default planning pipeline id: "
      << move_group_interface.getDefaultPlanningPipelineId());

  RCLCPP_INFO_STREAM(logger, "planning pipeline id: "
      << move_group_interface.getPlanningPipelineId());

  RCLCPP_INFO_STREAM(logger, "default planner id: "
      << move_group_interface.getDefaultPlannerId());

  RCLCPP_INFO_STREAM(logger, "planner id: "
      << move_group_interface.getPlannerId());

  RCLCPP_INFO_STREAM(logger, "max velocity scaling factor: "
      << move_group_interface.getMaxVelocityScalingFactor());

  RCLCPP_INFO_STREAM(logger, "max acceleration scaling factor: "
      << move_group_interface.getMaxAccelerationScalingFactor());

  RCLCPP_INFO_STREAM(logger, "planning time: "
      << move_group_interface.getPlanningTime());
      
  RCLCPP_INFO_STREAM(logger, "goal joint tolerance: "
      << move_group_interface.getGoalJointTolerance());

  RCLCPP_INFO_STREAM(logger, "goal orientation tolerance: "
      << move_group_interface.getGoalOrientationTolerance());

  RCLCPP_INFO_STREAM(logger, "end effector link: "
      << move_group_interface.getEndEffectorLink());

  RCLCPP_INFO_STREAM(logger, "end effector: "
      << move_group_interface.getEndEffector());

  // Robot state
  using namespace std::chrono_literals;
  RCLCPP_INFO_STREAM(logger, "start state monitor");
  move_group_interface.startStateMonitor();
  std::this_thread::sleep_for(100ms);

  for (auto && value : move_group_interface.getCurrentJointValues()) {
    RCLCPP_INFO_STREAM(logger, "joint value: " << value);
  }

  RCLCPP_INFO_STREAM(logger, "current pose: "
      << move_group_interface.getCurrentPose().pose.position.x << ", "
      << move_group_interface.getCurrentPose().pose.position.y << ", "
      << move_group_interface.getCurrentPose().pose.position.z);

  RCLCPP_INFO_STREAM(logger, "current RPY: "
      << move_group_interface.getCurrentRPY()[0] << ", "
      << move_group_interface.getCurrentRPY()[1] << ", "
      << move_group_interface.getCurrentRPY()[2]);


  //
  // pose: 0, -0.2388, 0.0494

  // Set a target pose
  auto const target_pose = []{
    geometry_msgs::msg::Pose msg;
    msg.orientation.w = 1.0;
    msg.position.x = 0.0;
    msg.position.y = -0.3;
    msg.position.z = 0.23;
    return msg;
  }();
  move_group_interface.setPoseTarget(target_pose);

  move_group_interface.setPositionTarget(0.0, -0.3, 0.23);
  RCLCPP_INFO(logger, "set target position");

  // Create a plan to that target pose
  auto const [success, plan] = [&move_group_interface, &logger]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    RCLCPP_INFO(logger, "create plan");
    return std::make_pair(ok, msg);
  }();

  // Execute the plan
  if(success) {
    move_group_interface.execute(plan);
    RCLCPP_INFO(logger, "execute plan");
  } else {
    RCLCPP_ERROR(logger, "Planning failed!");
  }
#endif

  // Shutdown ROS
  RCLCPP_INFO_STREAM(logger, "Shutting down node");
  rclcpp::shutdown();
  return 0;
}