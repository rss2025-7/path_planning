import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseArray
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

        self.lookahead = 0  # FILL IN #
        self.speed = 0  # FILL IN #
        self.wheelbase_length = 0  # FILL IN #

        self.trajectory = LineTrajectory("/followed_trajectory")

        self.traj_sub = self.create_subscription(PoseArray,
                                                 "/trajectory/current",
                                                 self.trajectory_callback,
                                                 1)
        self.drive_pub = self.create_publisher(AckermannDriveStamped,
                                               self.drive_topic,
                                               1)

    def pose_callback(self, odometry_msg):

        # deconstructing pose
        robot_position = odometry_msg.pose.pose.position
        robot_orientation = odometry_msg.pose.pose.orientation

        # x, y, yaw
        robot_x = robot_position.x
        robot_y = robot_position.y
        robot_yaw = tf_transformations.euler_from_quaternion([robot_orientation.x, robot_orientation.y, 
                                                              robot_orientation.z, robot_orientation.w])
        # check if each point is at lookahead distance
        for point in self.trajectory.points:
            dx = point[0] - robot_x
            dy = point[1] - robot_y

            robot_distance_to_point = np.hypot(dx, dy)

            if robot_distance_to_point >= self.lookahead:
                robot_lookahead_point = point

        # transforming from global frame to robot frame
        rframe_lookahead_x = np.cos(-robot_yaw) * (robot_lookahead_point[0] - robot_x) - np.sin(-robot_yaw) * (robot_lookahead_point[1] - robot_y)
        rframe_lookahead_y = np.sin(-robot_yaw) * (robot_lookahead_point[0] - robot_x) + np.cos(-robot_yaw) * (robot_lookahead_point[1] - robot_y)

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
