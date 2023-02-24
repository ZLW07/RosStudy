//
// Created by wei on 2023/2/23.
//

#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "moveit_random_demo", ros::init_options::AnonymousName);
  // 创建一个异步的自旋线程（spinning thread）
  ros::AsyncSpinner spinner(1);
  spinner.start();
  // 连接move_group节点中的机械臂实例组，这里的组名arm是我们之前在setup assistant中设置的
  moveit::planning_interface::MoveGroupInterface group("arm");

  ROS_INFO("================================eeeeee===========================");
/*

  geometry_msgs::Pose target_pose1;
  target_pose1.orientation.w =  0.726282;
  target_pose1.orientation.x = 4.04423e-07;
  target_pose1.orientation.y = -0.687396;
  target_pose1.orientation.z = 4.81813e-07;

  target_pose1.position.x = 0.03;
  target_pose1.position.y = 0.2;
  target_pose1.position.z = 0.5;
  group.setPoseTarget(target_pose1);

  //定义一个plan
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

//用布尔型变量标记运动规划是否成功
  bool success = (group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

//打印结果
  ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");
  if (success)
  {
    group.execute(my_plan);
  }
*/

  // 随机产生一个目标位置
  group.setRandomTarget();
  // 开始运动规划，并且让机械臂移动到目标位置
  group.move();

  ros::waitForShutdown();
}

