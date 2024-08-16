from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import rclpy
import numpy as np
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry


class get_navigation_m1(Node):
 
    def __init__(self):
        super().__init__("get_navigation_point")
        self.topic_sub = self.create_subscription(Odometry,"/odom",self.accept_callback,10)


    def accept_callback(self,msg:Odometry):
        print(f"\nodom - 位置: {msg.pose.pose.position}\n姿态: {msg.pose.pose.orientation}")
        

def main(args=None):
    rclpy.init(args=args)
    node = get_navigation_m1()
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






