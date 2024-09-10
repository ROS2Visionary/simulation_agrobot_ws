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

// ROS基础数据类型
#include <std_msgs/msg/string.hpp>
using RosString = std_msgs::msg::String;

// nav2
#include <nav2_core/global_planner.hpp>
using GlobalPlanner = nav2_core::GlobalPlanner;

#endif // TYPES_HPP
