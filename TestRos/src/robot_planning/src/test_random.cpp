//
// Created by wei on 2023/2/23.
//

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "moveit_random_demo", ros::init_options::AnonymousName);
  // 创建一个异步的自旋线程（spinning thread）
  ros::AsyncSpinner spinner(1);
  spinner.start();
  // 连接move_group节点中的机械臂实例组，这里的组名arm是我们之前在setup assistant中设置的
  moveit::planning_interface::MoveGroupInterface group("arm");
  const robot_state::JointModelGroup* joint_model_group =
      group.getCurrentState()->getJointModelGroup("arm");

  const moveit::core::LinkModel* joint_link =
      group.getCurrentState()->getLinkModel("link_6");

  geometry_msgs::Pose target_pose1;
  target_pose1.orientation.w =  0.726282;
  target_pose1.position.x = 0.03;
  target_pose1.position.y = 0.2;
  target_pose1.position.z = 0.5;
  group.setPoseTarget(target_pose1);

  //定义一个plan
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

//用布尔型变量标记运动规划是否成功
  bool success = (group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");

  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("base_link","/rviz_visual_markers");

  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.15;

//  visual_tools.publishAxisLabeled(target_pose1, "pose1");
////markers创建
  visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
//画线
  visual_tools.publishTrajectoryLine(my_plan.trajectory_ , joint_link,joint_model_group,rvt::WHITE);
  visual_tools.trigger();
//完成一次规划，与RViz 界面进行交互，使其点击 next 再继续执行。
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
  if (success)
  {
    group.execute(my_plan);
  }
  auto target = group.getCurrentPose();
  ROS_WARN("================%f",target.pose.position.x);
  ROS_WARN("================%f",target.pose.position.y);
  ROS_WARN("================%f",target.pose.position.z);
  ros::waitForShutdown();
}

