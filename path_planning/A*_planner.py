import rclpy
from rclpy.node import Node

import heapq
assert rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, PoseArray
from nav_msgs.msg import OccupancyGrid
from .utils import LineTrajectory

import numpy as np
from skimage.morphology import disk, dilation

from tf_transformations import euler_from_quaternion

from heapq import heappop, heappush


class PathPlan(Node):
    """ Listens for goal pose published by RViz and uses it to plan a path from
    current car pose.
    """

    def __init__(self):
        super().__init__("trajectory_planner")
        self.declare_parameter('odom_topic', "default")
        self.declare_parameter('map_topic', "default")
        self.declare_parameter('initial_pose_topic', "default")

        self.odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.map_topic = self.get_parameter('map_topic').get_parameter_value().string_value
        self.initial_pose_topic = self.get_parameter('initial_pose_topic').get_parameter_value().string_value

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self.map_cb,
            1)

        self.goal_sub = self.create_subscription(
            PoseStamped,
            "/goal_pose",
            self.goal_cb,
            10
        )

        self.traj_pub = self.create_publisher(
            PoseArray,
            "/trajectory/current",
            10
        )

        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            self.initial_pose_topic,
            self.pose_cb,
            10
        )

        self.trajectory = LineTrajectory(node=self, viz_namespace="/planned_trajectory")
        self.map_data = None
        self.map_info = None
        self.map_width = None
        self.map_height = None
        self.map_resolution = None
        self.map_origin = (0.0, 0.0)
        self.rotation_matrix = np.eye(2)
        self.rotation_matrix_inv = np.eye(2)

        self.robot_radius = 0.25
        self.turning_radius = 1.0

        self.current_pose = None
        self.goal_pose = None

    def map_cb(self, msg):
        self.map_info = msg.info
        self.map_height = msg.info.height
        self.map_width = msg.info.width
        self.map_resolution = msg.info.resolution
        self.map_origin = (msg.info.origin.position.x, msg.info.origin.position.y)

        # occupancy grid has values 0 for free space and -1 for unknown
        self.map_data = np.array(msg.data).astype(np.int8).reshape(msg.info.height, msg.info.width)

        # Alter map
        q = (
            msg.info.origin.orientation.x,
            msg.info.origin.orientation.y,
            msg.info.origin.orientation.z,
            msg.info.origin.orientation.w
        )

        _, _, self.yaw = euler_from_quaternion(q) # quaternion to yaw

        # x_t, y_t = self.map_origin[0], self.map_origin[1]
        # self.rotation_matrix = np.array([
        #     [np.cos(self.yaw), -np.sin(self.yaw), x_t],
        #     [np.sin(self.yaw), np.cos(self.yaw), y_t],
        #     [0, 0, 1]
        # ])

        # self.rotation_matrix_inv = np.array([
        #     [np.cos(self.yaw), np.sin(self.yaw), x_t * np.cos(self.yaw) + y_t * np.sin(self.yaw)],
        #     [-np.sin(self.yaw), np.cos(self.yaw), -x_t * np.sin(self.yaw + y_t * np.cos(self.yaw))],
        #     [0, 0, 1]
        # ])

        self.rotation_matrix = np.array([
            [np.cos(self.yaw), -np.sin(self.yaw)],
            [np.sin(self.yaw), np.cos(self.yaw)],
        ])

        self.rotation_matrix_inv = np.array([
            [np.cos(self.yaw), np.sin(self.yaw)],
            [-np.sin(self.yaw), np.cos(self.yaw)],
        ])

        self.dilate_map(self.robot_radius)

        self.get_logger().info("Initialized map")

    def pose_cb(self, pose):
        # we don't need to run search on orientation
        self.current_pose = (pose.pose.pose.position.x, pose.pose.pose.position.y) # extract x,y
        self.get_logger().info(f"Initialized current pose {self.current_pose}, px {self.convert_world_to_pixel(self.current_pose)}")

        if self.current_pose is not None and self.goal_pose is not None and self.map_data is not None:
            self.plan_path(self.current_pose, self.goal_pose, self.map_data)

    def goal_cb(self, msg):
        # we don't need to run search on orientation
        self.goal_pose = (msg.pose.position.x, msg.pose.position.y)
        self.get_logger().info(f"Initialized goal pose {self.goal_pose}, px {self.convert_world_to_pixel(self.goal_pose)}")

    def plan_path(self, start_point, end_point, map):
        self.get_logger().info("Path planning started")
        self.trajectory.clear()

        start_px = self.convert_world_to_pixel(start_point)
        goal_px = self.convert_world_to_pixel(end_point)
        self.get_logger().info(f"Start pixel: {start_px}, Goal pixel: {goal_px}")

        # Remember start_px and goal_px are (u,v) format but we index map_data as (v,u)
        def get_neighbors(node):
            neighbors = [
                (node[0] + 1, node[1]),
                (node[0] - 1, node[1]),
                (node[0], node[1] + 1),
                (node[0], node[1] - 1),
                (node[0] + 1, node[1] + 1),
                (node[0] - 1, node[1] - 1),
                (node[0] + 1, node[1] - 1),
                (node[0] - 1, node[1] + 1)
            ]
            return [
                (u, v) for u, v in neighbors
                if 0 <= u < self.map_width and 0 <= v < self.map_height
            ]
        def heuristic(a, b):
            # manhattan distance for heuristic
            return np.abs(a[0] - b[0]) + np.abs(a[1] - b[1])

        def euclidean_distance(a, b):
            # euclidean distance for cost
            return np.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

        open_set = []
        heapq.heapify(open_set)
        came_from = {}
        heapq.heappush(open_set, (0 + heuristic(start_px, goal_px), 0, start_px)) # (f_score, g_score, node)
        g_score = {start_px: 0}
        completed = set()
        path = []
        # self.get_logger().info(f"open set {open_set}")
        while open_set:
            _, current_g, current = heappop(open_set)
            self.get_logger().info(f"On node {current}")
            if current == goal_px:
                self.get_logger().info("Goal reached")
                path = self.reconstruct_path(came_from, goal_px)
                break

            completed.add(current)
            # self.get_logger().info(f"{get_neighbors(current)}")
            for neighbor in get_neighbors(current):
                u, v = neighbor
                if neighbor in completed or self.map_data[v,u] == 100:
                    # self.get_logger().info(f"{self.map_data[v,u]}")
                    continue

                tentative_g_score = current_g + euclidean_distance(current, neighbor)

                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    g_score[neighbor] = tentative_g_score
                    f_score = tentative_g_score + heuristic(neighbor, goal_px)
                    heapq.heappush(open_set, (f_score, tentative_g_score, neighbor))
                    came_from[neighbor] = current

        if path:
            for point in path:
                self.trajectory.addPoint(point)
            self.traj_pub.publish(self.trajectory.toPoseArray())
            self.trajectory.publish_viz()
            self.get_logger().info("Path planned successfully")
            self.get_logger().info(f"Path length: {len(path)}")


    def reconstruct_path(self, came_from, end):
        current = end
        path = [self.convert_pixel_to_world(current)]
        self.get_logger().info(f"{came_from}")

        while current in came_from:
            current = came_from[current]
            path.insert(0, self.convert_pixel_to_world(current))
        return path

    def dilate_map(self, r):
        r_px = int(r/self.map_resolution)

        footprint = disk(r_px)

        modified_map = np.zeros_like(self.map_data, dtype = np.int8)
        modified_map[self.map_data != 0] = 1

        dilated_map = dilation(modified_map, footprint) # extends unknown boundary

        scaled_map = np.zeros_like(dilated_map, dtype = np.int8) # scale weights
        scaled_map[dilated_map != 0] = 100

        self.map_data = scaled_map

    def convert_pixel_to_world(self, pixel):
        # pixel_coords = np.array([pixel[0], pixel[1], 1.0])
        # map_coords = np.dot(self.rotation_matrix_inv, pixel_coords) * self.map_resolution
        # return map_coords[0], map_coords[1]
        pixel_coords = np.array([pixel[0], pixel[1]])
        map_coords = np.dot(self.rotation_matrix_inv, pixel_coords) * self.map_resolution
        map_x = map_coords[0] + self.map_origin[0]
        map_y = map_coords[1] + self.map_origin[1]
        return map_x, map_y

    def convert_world_to_pixel(self, point):
        # map_coords = np.array([point[0]/self.map_resolution, point[1]/self.map_resolution, 1.0])
        # pixel_coords = np.dot(self.rotation_matrix, map_coords)
        # return int(pixel_coords[0]), int(pixel_coords[1])
        map_x, map_y = point
        map_coords = np.array([map_x - self.map_origin[0], map_y - self.map_origin[1]])
        pixel_coords = np.dot(self.rotation_matrix, map_coords) / self.map_resolution
        pixel_x = int(pixel_coords[0])
        pixel_y = int(pixel_coords[1])
        return pixel_x, pixel_y

        # # origin map
        # u, v = pixel
        # out = self.map_resolution * (self.t_map_to_world @ (np.array([u,v,1]).reshape(3,-1)))

        # return out[0], out[1]

    # def convert_map_to_pixel(self, point):
        # x, y = point
        # homogeneous_point = np.array([x, y, 1]).reshape(3, 1)
        # transformed_point = self.t_world_to_map @ homogeneous_point
        # pixel_u = transformed_point[0, 0] / self.map_resolution
        # pixel_v = transformed_point[1, 0] / self.map_resolution
        # self.get_logger().info(f"transfomr: {self.t_world_to_map.shape}")
        # self.get_logger().info(f"transfomred {transformed_point}")

        # return int(pixel_u), int(pixel_v)



def main(args=None):
    rclpy.init(args=args)
    planner = PathPlan()
    rclpy.spin(planner)
    rclpy.shutdown()
