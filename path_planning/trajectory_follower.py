import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Odometry
from rclpy.node import Node
import numpy as np
import tf_transformations

from visualization_msgs.msg import Marker

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

        self.ptf_pub = self.create_publisher(Marker,
                                             "ptf",
                                             1)

        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = 0.0
        drive_msg.drive.steering_angle = 0.0
        self.drive_pub.publish(drive_msg)

    def find_point_along_trajectory(self, r, la, p1, p2):
            """
            r: robot pose (x,y)
            la: lookahead distance (float)
            p1: start point of trajectory segment (x,y)
            p2: end point of trajectory segment (x,y)
            """
            p1 = np.array(p1)
            p2 = np.array(p2)

            self.get_logger().info(f"{p1}, {p2}")

            V = p2 - p1

            a = V.dot(V)
            b = 2 * V.dot(p1 - r)
            c = p1.dot(p1) + r.dot(r) - 2 * p1.dot(r) - la * la

            disc = b**2 - 4 * a * c
            if disc < 0:
                return None

            sqrt_disc = np.sqrt(disc)
            # solutions
            t1 = (-b + sqrt_disc) / (2 * a)
            t2 = (-b - sqrt_disc) / 2 * a

            self.get_logger().info(f"{t1} {t2}")

            # If neither of these is between 0 and 1, then the line segment misses the circle, or hits if extended
            if not (0 <= t1 <= 1 or 0 <= t2 <= 1):
                return None

            t = max(0, min(1, - b / (2 * a)))
            return p1 + t * V

    def pose_callback(self, odometry_msg):
        # deconstructing pose
        robot_position = odometry_msg.pose.pose.position
        robot_orientation = odometry_msg.pose.pose.orientation

        if self.trajectory.points:
            # x, y, yaw
            robot_x = robot_position.x
            robot_y = robot_position.y
            robot_yaw = tf_transformations.euler_from_quaternion([robot_orientation.x, robot_orientation.y,
                                                                robot_orientation.z, robot_orientation.w])[2]

            # Compute closest point on each segment
            curr_pt = np.array([robot_x, robot_y]).reshape(2,1)
            traj_points = np.array(self.trajectory.points).T
            P1 = traj_points[:, :-1]
            P2 = traj_points[:, 1:]

            d = P2 - P1
            norms = np.sum(d**2, axis=0)
            pt_to_traj = curr_pt - P1
            w_dot_v = np.sum(pt_to_traj * d, axis=0) / norms
            projection = np.clip(w_dot_v, 0.0, 1.0)

            closest_pt = P1 + projection * d

            distances = np.sum((curr_pt - closest_pt) ** 2, axis=0)

            # Find segment with closest point
            closest_segment = np.argmin(distances)

            self.get_logger().info(f"{closest_segment}")

            # From that segment onwards, check for circle-line intersections (vectorize with np.roots)
            robot_lookahead_point = None
            for i in range(closest_segment, P1.shape[1]):
                robot_lookahead_point = self.find_point_along_trajectory(curr_pt.flatten(), self.lookahead, P1[:, i].flatten(), P2[:, i].flatten())
                if robot_lookahead_point is None:
                    self.get_logger().info(f"Not found {i}")
                    continue
                else:
                    point_to_follow = robot_lookahead_point[0], robot_lookahead_point[1]
                    self.ptf_pub.publish(self.create_point_marker(point_to_follow, "/map"))
                    break

            # transforming from global frame to robot frame
            # self.get_logger().info(f"{robot_yaw, type(robot_yaw), type(robot_lookahead_point[0]), robot_lookahead_point[0], type(robot_x), robot_x}")
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

        # # deconstructing pose
        # robot_position = odometry_msg.pose.pose.position
        # robot_orientation = odometry_msg.pose.pose.orientation

        # if self.trajectory.points:
        #     # x, y, yaw
        #     robot_x = robot_position.x
        #     robot_y = robot_position.y
        #     robot_yaw = tf_transformations.euler_from_quaternion([robot_orientation.x, robot_orientation.y,
        #                                                         robot_orientation.z, robot_orientation.w])
        #     # check if each point is at lookahead distance
        #     # self.get_logger().info(f"{self.trajectory.points}, {robot_x}, {robot_y}")
        #     for point in self.trajectory.points:
        #         dx = point[0] - robot_x
        #         dy = point[1] - robot_y

        #         robot_distance_to_point = np.hypot(dx, dy)

        #         if robot_distance_to_point >= self.lookahead:
        #             robot_lookahead_point = point
        #             self.ptf_pub.publish(self.create_point_marker(robot_lookahead_point))
        #             break

        #     # self.get_logger().info(f"{robot_lookahead_point}")

        #     # transforming from global frame to robot frame
        #     # self.get_logger().info(f"{robot_yaw, type(robot_yaw), type(robot_lookahead_point[0]), robot_lookahead_point[0], type(robot_x), robot_x}")
        #     rframe_lookahead_x = np.cos(-robot_yaw[0]) * (robot_lookahead_point[0] - robot_x) - np.sin(-robot_yaw[0]) * (robot_lookahead_point[1] - robot_y)
        #     rframe_lookahead_y = np.sin(-robot_yaw[0]) * (robot_lookahead_point[0] - robot_x) + np.cos(-robot_yaw[0]) * (robot_lookahead_point[1] - robot_y)

        #     # pure pursuit algorithm
        #     curvature = (2 * rframe_lookahead_y) / (self.lookahead ** 2)
        #     steering_angle = np.arctan(self.wheelbase_length * curvature)

        #     # adjusting drive msg and publishing
        #     drive_msg = AckermannDriveStamped()
        #     drive_msg.drive.speed = self.speed
        #     drive_msg.drive.steering_angle = steering_angle
        #     self.drive_pub.publish(drive_msg)

    def trajectory_callback(self, msg):
        self.get_logger().info(f"Receiving new trajectory {len(msg.poses)} points")

        self.trajectory.clear()
        self.trajectory.fromPoseArray(msg)
        self.trajectory.publish_viz(duration=0.0)

        self.initialized_traj = True

    def create_point_marker(self, point, frame):
        marker = Marker()
        marker.header.frame_id = frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = point[0]
        marker.pose.position.y = point[1]
        marker.pose.position.z = 0.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        return marker

def main(args=None):
    rclpy.init(args=args)
    follower = PurePursuit()
    rclpy.spin(follower)
    rclpy.shutdown()
