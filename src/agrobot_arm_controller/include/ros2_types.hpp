// types.hpp
#ifndef TYPES_HPP
#define TYPES_HPP

#include <rclcpp/rclcpp.hpp>
using rclcpp::Node;

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
using geometry_msgs::msg::Twist;
using geometry_msgs::msg::Pose;


#include <sensor_msgs/msg/laser_scan.hpp>
using sensor_msgs::msg::LaserScan;

// MoveIt
#include <moveit/move_group_interface/move_group_interface.h>
using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface; // class

// ROS基础数据类型
#include <std_msgs/msg/string.hpp>
using RosString = std_msgs::msg::String;

#endif // TYPES_HPP
