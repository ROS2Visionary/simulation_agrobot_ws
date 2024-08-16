#include <memory>
#include <iostream>
#include "ros2_types.hpp"

using namespace std;

int main(int argc, char const *argv[])
{
  rclcpp::init(argc,argv);

  // 第一个参数是 ROS 将用来命名唯一节点的字符串。由于我们使用 ROS 参数的方式，MoveIt 需要第二个参数。
  auto node = make_shared<Node>("first_moveit",rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  
  const auto logger = rclcpp::get_logger("moveit_log");

  auto move_group_interface = MoveGroupInterface(node,"arm");

  const auto target_pose = []{
    Pose msg;
      msg.position.x = 0.2;
      msg.position.y = 0.2;
      msg.position.z = 0.5;
    return msg;
  }();
  
  move_group_interface.setPoseTarget(target_pose);

  const auto [success,plan] = [&move_group_interface]{
    MoveGroupInterface::Plan msg;
    const auto ok = static_cast<bool>(move_group_interface.plan(msg));
    return make_pair(ok,msg);
  }();

  if (success){
    move_group_interface.execute(plan);
  }else{
    RCLCPP_ERROR(logger,"Planning failed!");
  }
  
  rclcpp::shutdown();
  return 0;
}


