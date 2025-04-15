import rclpy
from rclpy.node import Node
import numpy as np
assert rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, PoseArray,Pose
from nav_msgs.msg import OccupancyGrid
from .utils import LineTrajectory
import dubins
from tf_transformations import quaternion_from_euler

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

        self.min_steer = np.deg2rad(30)
        self.wheelbase = 0.34
        self.map = None

    def pixel_to_world(self, u, v, reverse=False):
        resolution = self.map.info.resolution
        if not reverse:
            u_m = u * resolution
            v_m = v * resolution

            yaw = quaternion_to_yaw(self.map.info.origin.orientation)
            
            x_local = u_m * np.cos(yaw) - v_m * np.sin(yaw)
            y_local = u_m * np.sin(yaw) + v_m * np.cos(yaw)
            
            x_world = self.map.info.origin.position.x + x_local
            y_world = self.map.info.origin.position.y + y_local

            return (x_world, y_world)

        else:
            x,y = u,v
            x_local = x - self.map.info.origin.position.x
            y_local = y  - self.map.info.origin.position.y

            yaw = quaternion_to_yaw(self.map.info.origin.orientation)
            

            u_m = x_local * np.cos(yaw) + y_local * np.sin(yaw)
            v_m = -x_local * np.sin(yaw) + y_local * np.cos(yaw)
            
            u = u_m / resolution
            v = v_m / resolution

            return (int(round(u)),int(round(v)))


    def map_cb(self, msg):
        self.map = msg
        self.get_logger().info("Map received")


    def pose_cb(self, pose):
        x,y,t =  pose.pose.pose.position.x, pose.pose.pose.position.y, quaternion_to_yaw(pose.pose.pose.orientation)
        u,v = self.pixel_to_world(x,y,True)
        self.start  = RRTNode(x, y, u, v, t)

    def goal_cb(self, msg):
        x,y,t =  msg.pose.position.x, msg.pose.position.y, quaternion_to_yaw(msg.pose.orientation)
        u,v = self.pixel_to_world(x,y,True)
        self.goal  = RRTNode(x, y, u, v, t)
        self.plan_path()

    
    def nodes_to_posearray(self, nodes, frame_id="map"):
        pose_array = PoseArray()
        pose_array.header.frame_id = frame_id
        
        for node in nodes:
            p = Pose()
            p.position.x = node.x
            p.position.y = node.y
            p.position.z = 0.0 
            q = quaternion_from_euler(0.0, 0.0, node.theta)
            p.orientation.x = q[0]
            p.orientation.y = q[1]
            p.orientation.z = q[2]
            p.orientation.w = q[3]
            
            pose_array.poses.append(p)
        return pose_array

    def plan_path(self):
        init = RRTStar(self.start, self.goal, self.map)
        tree = init.run(self)
        self.get_logger().info("Tree Done")
        trajectory = init.extract_path(self.goal)
        path = [(n.x,n.y,n.theta) for n in trajectory]
        self.get_logger().info(str(path))

        if path:
            for point in path:
                self.trajectory.addPoint(point)
            self.traj_pub.publish(self.trajectory.toPoseArray())
            self.trajectory.publish_viz()
            self.get_logger().info("Path found and published")
            self.get_logger().info(f"Path length: {len(path)}")
        else:
            self.get_logger().warn("No path found to goal")


        self.traj_pub.publish(self.trajectory.toPoseArray())
        self.trajectory.publish_viz()


class RRTNode:
    def __init__(self, x, y,u, v, theta = 0,children = None,parent=None):
        self.x = x
        self.y = y
        self.theta = theta  
        self.u = u
        self.v = v
        self.parent = parent
        self.children = children if children is not None else []
        self.cost = 0 if parent is None else parent.cost + np.hypot(x - parent.x, y - parent.y) ###change
    def __repr__(self):
        return f"Node({self.x:.2f}, {self.y:.2f}, theta={self.theta:.2f}, cost={self.cost:.2f})"

    def add_child(self, other):
        self.children.append(other)
        return self

    def attach_to(self, parent):
        self.parent = parent
        self.parent.add_child(self)
        return self
        
    

class RRTStar:
    def __init__(self, z_start, z_goal, occupancy_grid, max_iterations=500, step_size=1.0, connect_threshold=0.1):
        self.map = occupancy_grid
        self.grid_data = np.array(self.map.data, dtype=np.int8).reshape((self.map.info.height, self.map.info.width))
        self.step_size = step_size
        self.r_near = 50.0
        self.N = max_iterations
        self.connect_threshold = connect_threshold
        self.tree = [z_start]
        self.z_start = z_start
        self.z_goal = z_goal
        self.free_indices = np.argwhere(self.grid_data == 0)
        self.epsilon = 0.5
        self.wheelbase = 0.34
        self.lookahead = 2.0
        self.M =500
        self.size =1
        self.turn = 0.68

    def pixel_to_world(self, u, v, reverse=False):
        resolution = self.map.info.resolution
        if not reverse:
            u_m = u * resolution
            v_m = v * resolution

            yaw = quaternion_to_yaw(self.map.info.origin.orientation)
            
            x_local = u_m * np.cos(yaw) - v_m * np.sin(yaw)
            y_local = u_m * np.sin(yaw) + v_m * np.cos(yaw)
            
            x_world = self.map.info.origin.position.x + x_local
            y_world = self.map.info.origin.position.y + y_local

            return (x_world, y_world)

        else:
            x,y = u,v
            x_local = x - self.map.info.origin.position.x
            y_local = y  - self.map.info.origin.position.y

            yaw = quaternion_to_yaw(self.map.info.origin.orientation)
            

            u_m = x_local * np.cos(yaw) + y_local * np.sin(yaw)
            v_m = -x_local * np.sin(yaw) + y_local * np.cos(yaw)
            
            u = u_m / resolution
            v = v_m / resolution
            return (int(round(u)),int(round(v)))



    def get_dist(self, node1, node2):
        return np.hypot(node1.x - node2.x, node1.y - node2.y)
    
    def sample(self):
        if np.random.random_sample() <= self.connect_threshold:
            return self.z_goal
        idx = np.random.choice(len(self.free_indices))
        col, row = self.free_indices[idx]
        x,y = self.pixel_to_world(row, col)
        #theta = np.random.uniform(-np.pi, np.pi)
        return RRTNode(x, y, row, col, theta=0)


    def get_nearest(self, z_rand):
        nearest = None
        min_dist = np.inf
        for node in self.tree:
            d = self.get_dist(node, z_rand)
            if d < min_dist:
                nearest = node
                min_dist = d
        return nearest
    
    def normalize_angle(self, angle):
        while angle >= np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle

    def steer(self, z_from, z_to):
        dx = z_to.x - z_from.x
        dy = z_to.y - z_from.y
        target_angle = np.arctan2(dy, dx)
        
        # 2. Compute the heading error (α) relative to z_from's orientation.
        alpha = self.normalize_angle(target_angle - z_from.theta)
        
        # Use the step size as the lookahead distance (l_d); you can adjust this parameter if needed.
        l_d = self.step_size
        
        # 3. If the angle error is negligible, extend in a straight line.
        if abs(alpha) < 1e-3:
            new_x = z_from.x + self.step_size * np.cos(z_from.theta)
            new_y = z_from.y + self.step_size * np.sin(z_from.theta)
            new_theta = z_from.theta
            # Sample a straight-line trajectory.
            n_samples = 10
            trajectory = [(z_from.x + t*(new_x - z_from.x), z_from.y + t*(new_y - z_from.y)) 
                          for t in np.linspace(0, 1, n_samples)]
            # For new pixel coordinates, set as placeholders (or compute via your reverse conversion).
            new_u, new_v = 0, 0
            z_new = RRTNode(new_x, new_y, new_u, new_v, new_theta, parent=z_from)
            z_new.cost = z_from.cost + self.step_size
            return z_new, trajectory
        
        # 4. Compute the required front wheel steering angle using PP:
        #      δ_f = arctan((2 * L * sin(α)) / l_d)
        delta_f = np.arctan2(2 * self.wheelbase * np.sin(alpha), l_d)
        
        # 5. If the steering angle is nearly zero, treat as straight-line motion.
        if abs(delta_f) < 1e-6:
            new_x = z_from.x + self.step_size * np.cos(z_from.theta)
            new_y = z_from.y + self.step_size * np.sin(z_from.theta)
            new_theta = z_from.theta
            n_samples = 10
            trajectory = [(z_from.x + t*(new_x - z_from.x), z_from.y + t*(new_y - z_from.y))
                          for t in np.linspace(0, 1, n_samples)]
            new_u, new_v = 0, 0
            z_new = RRTNode(new_x, new_y, new_u, new_v, new_theta, parent=z_from)
            z_new.cost = z_from.cost + self.step_size
            return z_new, trajectory
        
        # 6. Compute the turning radius:
        # Using the relation: R = L / tan(δ_f)
        R_val = self.wheelbase / np.tan(delta_f)
        # Ensure we use the magnitude of R for distance computations.
        R_val = abs(R_val)
        
        # 7. Compute the change in heading (dθ). For an arc-length equal to step_size:
        dtheta = self.step_size / R_val
        # The sign of the turn is determined by the sign of alpha.
        dtheta = dtheta if alpha > 0 else -dtheta
        
        new_theta = self.normalize_angle(z_from.theta + dtheta)
        
        # 8. Compute the new position using circular arc formulas.
        #     x_new = x_from + R * [ sin(θ_from + dθ) – sin(θ_from) ]
        #     y_new = y_from – R * [ cos(θ_from + dθ) – cos(θ_from) ]
        new_x = z_from.x + R_val * (np.sin(z_from.theta + dtheta) - np.sin(z_from.theta))
        new_y = z_from.y - R_val * (np.cos(z_from.theta + dtheta) - np.cos(z_from.theta))
        
        # 9. Discretize the arc trajectory for collision checking.
        n_samples = 10
        trajectory = []
        for t in np.linspace(0, 1, n_samples):
            interp_theta = z_from.theta + t * dtheta
            interp_x = z_from.x + R_val * (np.sin(z_from.theta + t * dtheta) - np.sin(z_from.theta))
            interp_y = z_from.y - R_val * (np.cos(z_from.theta + t * dtheta) - np.cos(z_from.theta))
            trajectory.append((interp_x, interp_y))
        
        # 10. Create the new node. For pixel coordinates (u, v) you can either compute them using your reverse conversion,
        # here we use placeholders (0,0) or call self.pixel_to_world with reverse=True.
        new_u, new_v = 0, 0  
        z_new = RRTNode(new_x, new_y, new_u, new_v, new_theta, parent=z_from)
        
        # 11. Update cost: here we add the step_size (or you can compute the arc length from the trajectory).
        z_new.cost = z_from.cost + self.step_size  # Alternatively: use the computed arc length.
        
        return z_new, trajectory
    
    def extract_path(self, node):
        path = []
        while node is not None:
            path.append(node)
            node = node.parent
        return path[::-1]
    
    def remove_edge(self, node):
        if node.parent is not None:
            if node in node.parent.children:
                node.parent.children.remove(node)
            node.parent = None

 
    def obstacle_free(self, trajectory):
        height, width = self.map.info.height, self.map.info.width
        for node in trajectory:
            if isinstance(node, RRTNode):
                row, col = node.u, node.v
            else:
                row, col = self.pixel_to_world(node[0], node[1], reverse=True)
            if col < 0 or col >= width or row < 0 or row >= height:
                return False
            if self.grid_data[col, row] != 0:
                return False
        return True


    def get_near(self, z_new):
        return [node for node in self.tree if self.get_dist(node, z_new) <= self.r_near]


    def choose_parent(self, z_new, near_nodes):
        z_min = z_new.parent
        c_min = z_new.cost
        for node in near_nodes:
            if node == z_min:
                continue
            path = self.steer(node, z_new)
            if self.obstacle_free(path):
                new_cost = node.cost + self.traj_cost(path)
                if new_cost < c_min:
                    c_min = new_cost
                    z_min = node
        return z_min, c_min


    def rewire(self, z_new, near_nodes):
        for z_near in near_nodes:
            if z_near == z_new.parent:
                continue
            q0 = (z_new.x, z_new.y, z_new.theta)
            q1 = (z_near.x, z_near.y, z_near.theta)
            path_obj = dubins.shortest_path(q0, q1, self.turn)
            path_length = path_obj.path_length()
            sample_step = self.step_size / 10.0
            configurations, _ = path_obj.sample_many(sample_step)

            traj = [(conf[0], conf[1]) for conf in configurations]
            

            if not self.obstacle_free(traj):
                continue
            
            candidate_cost = z_new.cost + path_length

            if candidate_cost < z_near.cost:
                self.remove_edge(z_near)
                if self.is_leaf(z_near.parent):
                    self.tree.remove(z_near.parent)
                self.extend(z_new, z_near)
                z_near.cost = candidate_cost

        return self.tree

    def traj_cost(self, trajectory, R_c, a=1.0, b=1.5, r_min=0.75):
        if len(trajectory) < 2:
            return 0.0
        curved_length = 0.0
        for i in range(1, len(trajectory)):
            (x1, y1) = trajectory[i - 1]
            (x2, y2) = trajectory[i]
            curved_length += np.hypot(x2 - x1, y2 - y1)
        
        if R_c == float('inf') or R_c > 1e10:
            multiplier = 1.0
        else:
            multiplier = 1.0 + a * np.exp(-b * (R_c - r_min))
        
        total_cost = curved_length * multiplier
        return total_cost


    def is_leaf(self, node):
        for n in self.tree:
            if n.parent == node:
                return False
        return True
    
    def extend(self, parent, child_state):
        child = child_state.attach_to(parent)
        self.tree.append(child)
        return child
    
    # def traverse_rrt_bfs(self, nodes):
    #     queue = list(nodes)
    #     results = []

    #     while queue:
    #         x = queue.pop(0)
    #         results.append(x)
    #         queue.extend(x.children)
    #     return results
    
    def run(self, log):
        for i in range(self.N):
            log.get_logger().info(f"Iteration: {i}")
            z_rand = self.sample()
            if z_rand is None:
                continue
            z_nearest = self.get_nearest(z_rand)

            z_new, traj = self.steer(z_nearest, z_rand)
            if self.obstacle_free(traj):
                Z_near = self.get_near(z_new)
                best_parent, best_cost = self.choose_parent(z_new, Z_near)
                z_new.cost = best_cost
                self.extend(best_parent, z_new)
            
                self.rewire(z_new, Z_near)

                if self.get_dist(z_new, self.z_goal) < self.epsilon:
                    self.z_goal.cost = z_new.cost + self.get_dist(z_new, self.z_goal)
                    self.extend(z_new, self.z_goal)

            if len(self.tree) > self.M:
                idx = np.random.choice(500)
                self.tree.remove(self.tree[idx])
        return self.tree


def quaternion_to_yaw(quaternion):
        qx = quaternion.x
        qy = quaternion.y
        qz = quaternion.z
        qw = quaternion.w

        return np.arctan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy**2 + qz**2))


def main(args=None):
    rclpy.init(args=args)
    planner = PathPlan()
    rclpy.spin(planner)
    rclpy.shutdown()
