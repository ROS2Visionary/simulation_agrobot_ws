import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from std_msgs.msg import Header
import time
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformListener, Buffer
from enum import Enum
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
import threading

class TaskResult(Enum):
    UNKNOWN = 0
    SUCCEEDED = 1
    CANCELED = 2
    FAILED = 3

class multi_point_navigator(BasicNavigator):
    def __init__(self):
        super().__init__()
        self.waypoints = []

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.goal_poses = []
        self.make_goal_poses()

        # self.init_robot_pose()
        
        # 阻塞直到Nav2完全在线并且生命周期节点处于活动状态
        # self.waitUntilNav2Active()
        
        # 执行导航
        self.execute_navigation()

    def init_robot_pose(self):

        goal_pose_0 = PoseStamped()
        goal_pose_0.header.frame_id = 'map'
        goal_pose_0.header.stamp = self.get_clock().now().to_msg()
        goal_pose_0.pose.position.x = 0.25619828701019287
        goal_pose_0.pose.position.y = 0.4479288160800934
        goal_pose_0.pose.orientation.z = -0.08135386600978392
        goal_pose_0.pose.orientation.w = 0.9966852805601486
        
        self.setInitialPose(goal_pose_0)


    def execute_navigation(self):
        
        if len(self.goal_poses) > 0:
            self.navigate_to_pose(self.goal_poses.pop(0))


    def navigate_to_pose(self, goal_pose):
        self.goToPose(goal_pose)
        self.monitor_progress()

    def make_goal_poses(self):

        goal_pose_1 = PoseStamped()
        goal_pose_1.header.frame_id = 'map'
        goal_pose_1.pose.position.x = 4.059162139892578
        goal_pose_1.pose.position.y = -1.437158465385437
        goal_pose_1.pose.orientation.z = -0.7065000323557776
        goal_pose_1.pose.orientation.w = 0.7077130098290444

        self.goal_poses = [goal_pose_1]
        # self.goal_poses = [goal_pose_1,goal_pose_2,goal_pose_2,goal_pose_4,goal_pose_5]

    def monitor_progress(self):

        # 监听导航的实时进度和是否成功
        while not self.isTaskComplete():
            feedback = self.getFeedback()
            if feedback:
                self.get_logger().info(f"剩余距离: {feedback.distance_remaining} 米")
            rclpy.spin_once(self, timeout_sec=1.0)  # 每秒监听一次进度

        result = self.getResult()
        if result.value == TaskResult.SUCCEEDED.value:
            self.execute_navigation()
            self.get_logger().info('导航成功')

        elif result.value == TaskResult.CANCELED.value:
            self.get_logger().info('导航取消!')

        elif result.value == TaskResult.FAILED.value:
            self.get_logger().info('导航失败!')

        elif result.value == TaskResult.UNKNOWN.value:
            self.get_logger().info('目标具有无效的返回状态!')


def main(args=None):
    rclpy.init(args=args)
    node = multi_point_navigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # 处理 Ctrl-C 中断
        pass
    finally:
        # 在节点销毁前调用清理方法
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
