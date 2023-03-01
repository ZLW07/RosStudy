//
// Created by wei on 2023/3/1.
//
#include "Log/log.h"

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <ros/ros.h>
// 包含moveit的API
#include <eigen_conversions/eigen_msg.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "check_collision");
    // 创建一个异步的自旋线程（spinning thread）
    ros::AsyncSpinner spinner(1);
    spinner.start();
    // 连接move_group节点中的机械臂实例组，这里的组名arm是我们之前在setup
    // assistant中设置的
    moveit::planning_interface::MoveGroupInterface group("arm");
    const robot_state::JointModelGroup *joint_model_group = group.getCurrentState()->getJointModelGroup("arm");

    const moveit::core::LinkModel *joint_link = group.getCurrentState()->getLinkModel("link_6");

    // 加载机器人的运动学模型到情景实例中
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    planning_scene::PlanningScene planning_scene(kinematic_model);

    // 自身碰撞检测
    // 首先需要创建一个碰撞检测的请求对象和响应对象，然后调用碰撞检测的API
    // checkSelfCollision，检测结果会放到collision_result中
    collision_detection::CollisionRequest collision_request;
    collision_detection::CollisionResult collision_result;
    planning_scene.checkSelfCollision(collision_request, collision_result);
    ZLOG << "1. Self collision Test: " << (collision_result.collision ? "in" : "not in") << " self collision";

    // 修改机器人的状态
    // 我们可以使用场景实例的getCurrentStateNonConst()获取当前机器人的状态，然后修改机器人的状态到一个随机的位置，
    // 清零collision_result的结果后再次检测机器人是否发生滋生碰撞
    robot_state::RobotState &current_state = planning_scene.getCurrentStateNonConst();
    current_state.setToRandomPositions();
    collision_result.clear();
    planning_scene.checkSelfCollision(collision_request, collision_result);
    ZLOG << "2. Self collision Test(Change the state): " << (collision_result.collision ? "in" : "not in");

    // 获取碰撞关系
    // 首先，我们先让机器人发生自身碰撞
    std::vector<double> joint_values;
    current_state.copyJointGroupPositions(joint_model_group, joint_values);
    //原来的代码这里是joint_values[0]，并不会导致碰撞，我改成了joint_values[2]，在该状态下机器人会发生碰撞
    joint_values[2] = 3.1416;
    current_state.setJointGroupPositions(joint_model_group, joint_values);
    ZLOG << "4. Collision points " << (current_state.satisfiesBounds(joint_model_group) ? "valid" : "not valid");

    /*
     geometry_msgs::Pose target_pose1;
     target_pose1.orientation.w = 0.726282;
     target_pose1.position.x = 0.03;
     target_pose1.position.y = 0.21;
     target_pose1.position.z = 0.51;
     group.setPoseTarget(target_pose1);

     //定义一个plan
     moveit::planning_interface::MoveGroupInterface::Plan my_plan;

     //用布尔型变量标记运动规划是否成功
     bool success = (group.plan(my_plan) ==
                     moveit::planning_interface::MoveItErrorCode::SUCCESS);

     ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");

     namespace rvt = rviz_visual_tools;
     moveit_visual_tools::MoveItVisualTools visual_tools("base_link",
                                                         "/rviz_visual_markers");

     Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
     text_pose.translation().z() = 1.15;

     //  visual_tools.publishAxisLabeled(target_pose1, "pose1");
     ////markers创建
     visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
     //画线
     visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_link,
                                        joint_model_group);
     visual_tools.trigger();
     //完成一次规划，与RViz 界面进行交互，使其点击 next 再继续执行。
     visual_tools.prompt(
         "Press 'next' in the RvizVisualToolsGui window to continue the demo");
     if (success) {
       group.execute(my_plan);
     }*/

    ros::waitForShutdown();
}