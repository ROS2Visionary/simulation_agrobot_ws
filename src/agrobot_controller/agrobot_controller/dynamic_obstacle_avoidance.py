import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from sensor_msgs.msg import LaserScan  
from geometry_msgs.msg import Twist   
from std_msgs.msg import String       

import numpy as np                    
import time                           
import threading                      

class dynamic_obstacle_avoidance(Node):
     
    def __init__(self, node_name: str):
        super().__init__(node_name)

        # 创建订阅激光雷达数据的订阅者
        self.sub_radar = self.create_subscription(LaserScan, "/scan", self.radar_callback, 10)
        
        # 创建发布Twist消息的发布者，用于控制机器人运动
        self.pub_twist = self.create_publisher(Twist, "/cmd_vel", 10)

        
        # 初始化变量
        self.is_stop = False       # 停止标志
        self.is_turn = False       # 转弯标志
        
        # 初始化Twist消息
        self.msg_twist = Twist()
        
    def start_timer(self):
        self.is_turn = True
        self.timer = threading.Timer(2.0,self.turn_end)
        self.timer.start()    
    
    def turn_end(self):
        self.is_turn = False
    
    def radar_callback(self, msg: LaserScan):
        # 激光雷达数据回调函数
        
        if self.is_turn:
            return
        
        # 处理激光雷达数据，避障算法
        self.avoid_obstacles(msg.ranges)
        
    def avoid_obstacles(self, ranges_front):
        # 避障算法函数
        
        ranges_front = np.array(ranges_front)  # 将激光雷达数据转换为NumPy数组
        
        speed = 0.1         # 前进速度
        obstacle_radius = 0.4  # 障碍物半径
        
        self.msg_twist.linear.x = 0.0  # 线速度初始化为0
        self.msg_twist.angular.z = 0.0  # 角速度初始化为0
        
        # 如果前方没有检测到障碍物，则直行
        if not np.any(ranges_front < obstacle_radius):
            self.msg_twist.linear.x = speed
        else:            
            front_subs = np.array_split(ranges_front, 5)  # 将前方数据分为5部分
            
            # 找出最近障碍物的方向索引
            front_min_index = np.argmin([np.min(sub_arr) for sub_arr in front_subs])
            
            # 根据不同方向调整机器人运动
            if np.any(front_subs[2] < 0.2) or np.any(front_subs[1] < 0.2) or np.any(front_subs[3] < 0.2):
                front_min_index = 2
            
            if front_min_index == 0:
                self.msg_twist.linear.x = speed
                self.msg_twist.angular.z = np.radians(15)  # 左转
                
            elif front_min_index == 1:
                self.msg_twist.linear.x = speed
                self.msg_twist.angular.z = np.radians(15)  # 左转
                
            elif front_min_index == 2:
                self.msg_twist.linear.x = -speed
                left_obstacle = np.append(front_subs[4], front_subs[3])
                right_obstacle = np.append(front_subs[1], front_subs[0])
                if np.mean(left_obstacle) > np.mean(right_obstacle):
                    self.msg_twist.angular.z = np.radians(-30)  # 左后退
                else:
                    self.msg_twist.angular.z = np.radians(30)  # 右后退
                
            elif front_min_index == 3:
                self.msg_twist.linear.x = speed
                self.msg_twist.angular.z = np.radians(-15)  # 右转
                
            elif front_min_index == 4:
                self.msg_twist.linear.x = speed
                self.msg_twist.angular.z = np.radians(-15)  # 右转
            
            self.start_timer()
            
        # 发布Twist消息，控制机器人运动
        self.pub_twist.publish(self.msg_twist)
        
def main(args=None):
    rclpy.init(args=args)
    node = dynamic_obstacle_avoidance("motor_node")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # 处理 Ctrl-C 中断
        pass
    finally:
        # 在节点销毁前调用清理方法
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
