import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Odometry
from rclpy.node import Node
import numpy as np
import tf_transformations

from .utils import LineTrajectory


class PurePursuit(Node):
    """ Implements Pure Pursuit trajectory tracking with a fixed lookahead and speed.
    """

    def __init__(self):
        super().__init__("trajectory_follower")
        self.declare_parameter('odom_topic', "default")
        self.declare_parameter('drive_topic', "default")

        self.odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.drive_topic = self.get_parameter('drive_topic').get_parameter_value().string_value

        self.lookahead = 1.0  # FILL IN #
        self.speed = 1.0  # FILL IN #
        self.wheelbase_length = 1.0  # FILL IN #

        self.trajectory = LineTrajectory("/followed_trajectory")

        self.traj_sub = self.create_subscription(Odometry,
                                                 self.odom_topic,
                                                 self.pose_callback,
                                                 1)
        self.traj_sub = self.create_subscription(PoseArray,
                                                 "/trajectory/current",
                                                 self.trajectory_callback,
                                                 1)
        self.drive_pub = self.create_publisher(AckermannDriveStamped,
                                               self.drive_topic,
                                               1)

        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = 0.0
        drive_msg.drive.steering_angle = 0.0
        self.drive_pub.publish(drive_msg)

    def pose_callback(self, odometry_msg):

        # deconstructing pose
        robot_position = odometry_msg.pose.pose.position
        robot_orientation = odometry_msg.pose.pose.orientation

        if self.trajectory.points:
            # x, y, yaw
            robot_x = robot_position.x
            robot_y = robot_position.y
            robot_yaw = tf_transformations.euler_from_quaternion([robot_orientation.x, robot_orientation.y, 
                                                                robot_orientation.z, robot_orientation.w])
            # check if each point is at lookahead distance
            # self.get_logger().info(f"{self.trajectory.points}, {robot_x}, {robot_y}")
            for point in self.trajectory.points:
                dx = point[0] - robot_x
                dy = point[1] - robot_y

                robot_distance_to_point = np.hypot(dx, dy)

                if robot_distance_to_point >= self.lookahead:
                    robot_lookahead_point = point
                    break

            # self.get_logger().info(f"{robot_lookahead_point}")

            # transforming from global frame to robot frame
            # self.get_logger().info(f"{robot_yaw, type(robot_yaw), type(robot_lookahead_point[0]), robot_lookahead_point[0], type(robot_x), robot_x}")
            rframe_lookahead_x = np.cos(-robot_yaw[0]) * (robot_lookahead_point[0] - robot_x) - np.sin(-robot_yaw[0]) * (robot_lookahead_point[1] - robot_y)
            rframe_lookahead_y = np.sin(-robot_yaw[0]) * (robot_lookahead_point[0] - robot_x) + np.cos(-robot_yaw[0]) * (robot_lookahead_point[1] - robot_y)

            # pure pursuit algorithm
            curvature = (2 * rframe_lookahead_y) / (self.lookahead ** 2)
            steering_angle = np.arctan(self.wheelbase_length * curvature)

            # adjusting drive msg and publishing
            drive_msg = AckermannDriveStamped()
            drive_msg.drive.speed = self.speed
            drive_msg.drive.steering_angle = steering_angle
            self.drive_pub.publish(drive_msg)

    def trajectory_callback(self, msg):
        self.get_logger().info(f"Receiving new trajectory {len(msg.poses)} points")

        self.trajectory.clear()
        self.trajectory.fromPoseArray(msg)
        self.trajectory.publish_viz(duration=0.0)

        self.initialized_traj = True


def main(args=None):
    rclpy.init(args=args)
    follower = PurePursuit()
    rclpy.spin(follower)
    rclpy.shutdown()
