import rclpy
from rclpy.node import Node
import numpy as np
from scipy.spatial import KDTree
import random
import math
# import dubins

import heapq
assert rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, PoseArray
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker
from .utils import LineTrajectory

<<<<<<< HEAD
from skimage.morphology import disk, dilation
=======
import numpy as np
from skimage.morphology import disk, dilation

from tf_transformations import euler_from_quaternion

from heapq import heappop, heappush

>>>>>>> refs/remotes/origin/main


# TODO: Add dubins path planner
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

        self.goal_pub = self.create_publisher(
            Marker,
            "/visualization_goal",
            10
        )

        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            self.initial_pose_topic,
            self.pose_cb,
            10
        )

        self.trajectory = LineTrajectory(node=self, viz_namespace="/planned_trajectory")
<<<<<<< HEAD
        self.current_pose = None
        self.goal_pose = None
=======
>>>>>>> refs/remotes/origin/main
        self.map_data = None
        self.map_info = None
        self.map_width = None
        self.map_height = None
        self.map_resolution = None
        self.map_origin = (0.0, 0.0)
<<<<<<< HEAD

        self.num_samples = 1000 #1000 -> 500 -> 300 ->100 // 100 is unstable, 300 is stable
        self.k_neighbors = 10
        self.max_edge_length = 10000.0 # 100

        self.nodes = []
        self.edges = {}
        self.kdtree = None

        self.map_rotation = np.eye(2)
        self.map_rotation_inv = np.eye(2)

        self.robot_radius_m = 0.25
        self.turning_radius = 0.3

    def map_cb(self, msg):
        self.map_data = np.array(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
        self.map_info = msg.info
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_resolution = msg.info.resolution
        self.map_origin = (msg.info.origin.position.x, msg.info.origin.position.y)
        self.get_logger().info("Map received")
        self.get_logger().info(f"Map origin: {self.map_origin}")

        q = msg.info.origin.orientation
        yaw = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)
        cos_theta = math.cos(yaw)
        sin_theta = math.sin(yaw)
        self.map_rotation = np.array([[cos_theta, -sin_theta], [sin_theta, cos_theta]])
        self.map_rotation_inv = np.array([[cos_theta, sin_theta], [-sin_theta, cos_theta]])

        self.dilate_obstacles(self.robot_radius_m)

    def dilate_obstacles(self, robot_radius_m):
        """
        Expand obstacles in self.map_data by robot_radius_m (in meters).
        This ensures the planner leaves sufficient clearance.
        Using skimage.morphology.disk for the structuring element.
        """
        # a) Convert occupancy map to binary
        #    (1 => occupied, 0 => free). We'll treat anything != 0 as occupied
        binary_map = np.zeros_like(self.map_data, dtype=np.uint8)
        binary_map[self.map_data != 0] = 1

        # b) Convert robot radius in meters -> pixels
        radius_px = int(np.ceil(robot_radius_m / self.map_resolution))

        # c) Create disk structuring element
        selem = disk(radius_px)  # disk of radius_px

        # d) Morphological dilation
        dilated_map = dilation(binary_map, selem)

        # e) Convert back to occupancy style: 100 => obstacle, 0 => free
        inflated_occupancy = np.zeros_like(dilated_map, dtype=np.int8)
        inflated_occupancy[dilated_map == 1] = 100

        self.map_data = inflated_occupancy

    def pose_cb(self, pose):
        self.current_pose = (pose.pose.pose.position.x, pose.pose.pose.position.y)
        self.get_logger().info("Current pose received")
=======
        self.rotation_matrix = np.eye(2)
        self.rotation_matrix_inv = np.eye(2)

        self.robot_radius = 0.5
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
>>>>>>> refs/remotes/origin/main

        if self.current_pose is not None and self.goal_pose is not None and self.map_data is not None:
            self.plan_path(self.current_pose, self.goal_pose, self.map_data)

    def goal_cb(self, msg):
<<<<<<< HEAD
        self.goal_pose = (msg.pose.position.x, msg.pose.position.y)
        self.get_logger().info("Goal pose received")

        # Create and publish marker for goal visualization
        marker = Marker()
        marker.header.frame_id = msg.header.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "goal"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose = msg.pose

        self.get_logger().info(f"Goal pose: {self.goal_pose}")

        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        marker.lifetime.sec = 0

        self.goal_pub.publish(marker)

    def plan_path(self, start_point, end_point, map):
        self.get_logger().info("Planning path")
        self.trajectory.clear()
        self.nodes = []
        self.edges = {}
        self.kdtree = None

        self.nodes.append(start_point)
        self.nodes.append(end_point)

        # Build the roadmap if not already built
        if not self.edges:
            self.get_logger().info("Building roadmap")
            self.build_roadmap()

        # Find path using A* algorithm
        path = self.a_star(0, 1)  # 0 is start, 1 is goal
=======
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
>>>>>>> refs/remotes/origin/main

        if path:
            for point in path:
                self.trajectory.addPoint(point)
            self.traj_pub.publish(self.trajectory.toPoseArray())
            self.trajectory.publish_viz()
<<<<<<< HEAD
            self.get_logger().info("Path found and published")
            self.get_logger().info(f"Path length: {len(path)}")
        else:
            self.get_logger().warn("No path found to goal")


    def build_roadmap(self):
        """Build the PRM roadmap"""
        # Sample random points
        while len(self.nodes) < self.num_samples:
            point = (random.uniform(-60.0, 10.0), random.uniform(-10.5, 40.5)) #-61.5, 25.9, -17.0, 48.5
            #self.get_logger().info(f"Sampled point: {len(self.nodes)}")
            if self.is_collision_free(point):
                self.nodes.append(point)

        self.get_logger().info(f"Nodes: {len(self.nodes)}")
        # Build KD-tree for efficient nearest neighbor search
        self.kdtree = KDTree(self.nodes)

        # Connect nodes with edges
        for i, node in enumerate(self.nodes):
            self.edges[i] = []
            distances, indices = self.kdtree.query(node, k=self.k_neighbors + 1)

            for j, idx in enumerate(indices[1:]):  # Skip the first one (self)
                if (self.is_path_collision_free(node, self.nodes[idx]) and
                    distances[j] <= self.max_edge_length):
                    self.edges[i].append(idx)
                # if (self.dubins_is_path_collision_free(node, self.nodes[idx]) and
                #     distances[j] <= self.max_edge_length):
                #     self.edges[i].append(idx)

        self.get_logger().info(f"Edges: {len(self.edges[0])}")

    def a_star(self, start_idx, goal_idx):
        """A* algorithm to find the shortest path"""
        self.get_logger().info("A* algorithm")
        open_set = {start_idx}
        came_from = {}
        g_score = {start_idx: 0}
        f_score = {start_idx: self.heuristic(start_idx, goal_idx)}

        while open_set:
            current = min(open_set, key=lambda x: f_score.get(x, float('inf')))

            if current == goal_idx:
                return self.reconstruct_path(came_from, current)

            open_set.remove(current)

            for neighbor in self.edges.get(current, []):
                tentative_g_score = g_score[current] + self.distance(current, neighbor)

                if tentative_g_score < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, goal_idx)
                    if neighbor not in open_set:
                        open_set.add(neighbor)

        return None

    def is_collision_free(self, point):
        """Check if a point is in free space"""
        x, y = point
        # Convert to map coordinates
        u, v = self.world_to_map(x, y)

        # Check bounds
        # if map_x < 0 or map_x >= self.map_width or map_y < 0 or map_y >= self.map_height:
        #     return False

        if u < 0 or u >= self.map_width or v < 0 or v >= self.map_height:
            return False

        # Check if the point is in free space (0) or unknown (-1)
        return self.map_data[v, u] == 0

    def is_path_collision_free(self, p1, p2, step_size=0.1):
        """Check if the path between two points is collision-free"""
        dist = np.linalg.norm(np.array(p2) - np.array(p1))
        steps = int(dist / step_size) + 1

        for i in range(steps + 1):
            t = i / steps
            point = (1 - t) * np.array(p1) + t * np.array(p2)
            if not self.is_collision_free(point):
                return False
        return True

    def dubins_is_path_collision_free(self, p1, p2):
        p1 = (p1[0], p1[1], 0)
        p2 = (p2[0], p2[1], 0)
        path = dubins.shortest_path(p1, p2, self.turning_radius)
        configurations, _ = path.sample_many(0.1)

        for (x, y, theta) in configurations:
            if not self.is_collision_free((x, y)):
                return False
        return True

    def sample_random_point(self): # TODO: Can be optimized //using world_coordinates
        """Sample a random point in the map"""
        # x = random.uniform(self.map_origin[0],
        #                  self.map_origin[0] + self.map_width * self.map_resolution)
        # y = random.uniform(self.map_origin[1],
        #                  self.map_origin[1] + self.map_height * self.map_resolution)
        x = random.uniform(-61.5, 25.9)
        y = random.uniform(-17.0, 48.5)
        return (x, y)

    def heuristic(self, a, b):
        """Euclidean distance heuristic"""
        return np.linalg.norm(np.array(self.nodes[a]) - np.array(self.nodes[b]))

    def distance(self, a, b):
        """Distance between two nodes"""
        return np.linalg.norm(np.array(self.nodes[a]) - np.array(self.nodes[b]))

    def reconstruct_path(self, came_from, current):
        """Reconstruct the path from start to goal"""
        path = [self.nodes[current]]
        while current in came_from:
            current = came_from[current]
            path.append(self.nodes[current])
        return list(reversed(path))

    def quaternion_to_yaw(self, qx, qy, qz, qw):
        """
        Convert a quaternion into a yaw angle (around Z-axis).
        Assumes the quaternion represents a rotation primarily in the plane (no roll/pitch).
        """
        # Standard formula for yaw (Z-axis) from quaternion
        siny_cosp = 2.0 * (qw*qz + qx*qy)
        cosy_cosp = 1.0 - 2.0 * (qy*qy + qz*qz)
        return math.atan2(siny_cosp, cosy_cosp)

    def world_to_map(self, x, y):
        """
        Convert world coordinates (x, y) into map pixel indices (u, v),
        accounting for the map's origin translation and rotation.
        """
        # Translate by -origin
        dx = x - self.map_origin[0]
        dy = y - self.map_origin[1]
        # Rotate by R^-1 (which is the transpose for a pure rotation)
        rotated = self.map_rotation_inv.dot([dx, dy])
        # Scale by resolution
        u_f = rotated[0] / self.map_resolution
        v_f = rotated[1] / self.map_resolution
        # Round or int
        u = int(round(u_f))
        v = int(round(v_f))
        return (u, v)
=======
            self.get_logger().info("Path planned successfully")
            self.get_logger().info(f"Path length: {len(path)}")


    def reconstruct_path(self, came_from, end):
        current = end
        path = [self.convert_pixel_to_world(current)]
        # self.get_logger().info(f"{came_from}")

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
>>>>>>> refs/remotes/origin/main



def main(args=None):
    rclpy.init(args=args)
    planner = PathPlan()
    rclpy.spin(planner)
    rclpy.shutdown()
