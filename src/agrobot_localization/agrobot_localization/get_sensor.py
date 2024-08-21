from rclpy.node import Node
from sensor_msgs.msg import Imu
import rclpy
import numpy as np
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry


class get_navigation_m1(Node):
 
    def __init__(self):
        super().__init__("get_navigation_point")
        self.odom_sub = self.create_subscription(Odometry,"/odom",self.accept_odom,10)
        self.imu_sub = self.create_subscription(Imu,"/imu",self.accept_imu,10)

        self.odom_pub = self.create_publisher(Odometry,"/odom_data",10)
        self.imu_pub = self.create_publisher(Imu,"/imu_data",10)

    def accept_odom(self,msg:Odometry):
        msg.pose.covariance = [1.e-9, 0.0, 0.0, 0.0, 0.0, 0.0,
                                        0.0, 1.e-9, 0.0, 0.0, 0.0, 0.0,
                                        0.0, 0.0, 1.e-9, 0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0, 1.e-9, 0.0, 0.0,
                                        0.0, 0.0, 0.0, 0.0, 1.e-9, 0.0,
                                        0.0, 0.0, 0.0, 0.0, 0.0, 1.e-9]
        
        msg.twist.covariance = [1.e-9, 0.0, 0.0, 0.0, 0.0, 0.0,
                                        0.0, 1.e-9, 0.0, 0.0, 0.0, 0.0,
                                        0.0, 0.0, 1.e-9, 0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0, 1.e-9, 0.0, 0.0,
                                        0.0, 0.0, 0.0, 0.0, 1.e-9, 0.0,
                                        0.0, 0.0, 0.0, 0.0, 0.0, 1.e-9]
        self.odom_pub.publish(msg)

    def accept_imu(self,msg:Imu):
        msg.orientation_covariance = [1.e-9, 0.0, 0.0, 
                                              0.0, 1.e-9, 0.0,
                                              0.0, 0.0, 1.e-9]
        msg.angular_velocity_covariance = [1.e-9, 0.0, 0.0, 
                                                   0.0, 1.e-9, 0.0,
                                                   0.0, 0.0, 1.e-9]
        msg.linear_acceleration_covariance = [1.e-9, 0.0, 0.0, 
                                                      0.0, 1.e-9, 0.0,
                                                      0.0, 0.0, 1.e-9]
        self.imu_pub.publish(msg)


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






