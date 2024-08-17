import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformListener, Buffer
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from tf_transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np

class EKFLocalization(Node):
    def __init__(self):
        super().__init__('ekf_localization_node')

        # 初始化TF2
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 订阅/odom话题
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # 发布融合后的初始位置
        self.pose_publisher = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

        # 初始化状态向量 (x, y, theta)
        self.state = np.zeros(3)
        self.covariance = np.eye(3)  # 状态协方差矩阵

        # 噪声矩阵 (假设)
        self.Q = np.diag([0.1, 0.1, np.deg2rad(1)])  # 过程噪声
        self.R = np.diag([0.1, 0.1, np.deg2rad(5)])  # 观测噪声

    def odom_callback(self, odom_msg):
        try:
            # 获取map到odom的TF变换
            trans = self.tf_buffer.lookup_transform('map', 'odom', rclpy.time.Time())

            # 从odom中提取位置和角度
            odom_position = odom_msg.pose.pose.position
            odom_orientation = odom_msg.pose.pose.orientation
            _, _, odom_theta = euler_from_quaternion([
                odom_orientation.x,
                odom_orientation.y,
                odom_orientation.z,
                odom_orientation.w
            ])

            # 更新步骤 - 使用EKF融合位置信息
            z = np.array([
                trans.transform.translation.x + odom_position.x,
                trans.transform.translation.y + odom_position.y,
                odom_theta + euler_from_quaternion([trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w])[2]
            ])

            self.ekf_update(z)

            # 发布融合后的初始位置
            self.publish_pose()

        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn('TF lookup failed: {}'.format(e))

    def ekf_update(self, z):
        # 预测步骤
        F = np.eye(3)  # 状态转移矩阵
        self.state = F @ self.state
        self.covariance = F @ self.covariance @ F.T + self.Q

        # 观测模型
        H = np.eye(3)  # 观测矩阵
        y = z - H @ self.state  # 观测残差
        S = H @ self.covariance @ H.T + self.R  # 残差协方差
        K = self.covariance @ H.T @ np.linalg.inv(S)  # 卡尔曼增益

        # 更新步骤
        self.state = self.state + K @ y
        self.covariance = (np.eye(3) - K @ H) @ self.covariance

    def publish_pose(self):
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "map"

        # 填充位置和朝向
        pose_msg.pose.pose.position.x = self.state[0]
        pose_msg.pose.pose.position.y = self.state[1]
        pose_msg.pose.pose.position.z = 0.0

        quat = quaternion_from_euler(0, 0, self.state[2])
        pose_msg.pose.pose.orientation.x = quat[0]
        pose_msg.pose.pose.orientation.y = quat[1]
        pose_msg.pose.pose.orientation.z = quat[2]
        pose_msg.pose.pose.orientation.w = quat[3]

        # 填充协方差矩阵
        pose_msg.pose.covariance = np.zeros(36)
        pose_msg.pose.covariance[0] = self.covariance[0, 0]
        pose_msg.pose.covariance[1] = self.covariance[0, 1]
        pose_msg.pose.covariance[5] = self.covariance[0, 2]
        pose_msg.pose.covariance[6] = self.covariance[1, 0]
        pose_msg.pose.covariance[7] = self.covariance[1, 1]
        pose_msg.pose.covariance[11] = self.covariance[1, 2]
        pose_msg.pose.covariance[30] = self.covariance[2, 0]
        pose_msg.pose.covariance[31] = self.covariance[2, 1]
        pose_msg.pose.covariance[35] = self.covariance[2, 2]

        self.pose_publisher.publish(pose_msg)
        self.get_logger().info('发布融合后的初始位置')

def main(args=None):
    rclpy.init(args=args)
    node = EKFLocalization()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
