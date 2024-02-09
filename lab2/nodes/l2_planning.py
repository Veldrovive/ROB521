#!/usr/bin/env python3
#Standard Libraries
import numpy as np
import yaml
import pygame
import time
import pygame_utils
import matplotlib.image as mpimg
from skimage.draw import disk
from scipy.linalg import block_diag
from scipy.ndimage import binary_dilation
import os

import matplotlib.pyplot as plt


def load_map(filename):
    try:
        im = mpimg.imread("../maps/" + filename)
    except FileNotFoundError:
        im = mpimg.imread("./lab2/maps/" + filename)
    if len(im.shape) > 2:
        im = im[:,:,0]
    im_np = np.array(im)  #Whitespace is true, black is false
    #im_np = np.logical_not(im_np)    
    return im_np


def load_map_yaml(filename):
    file_path = "../maps/" + filename
    if not os.path.exists(file_path):
        file_path = "./lab2/maps/" + filename
    with open(file_path, "r") as stream:
            map_settings_dict = yaml.safe_load(stream)
    return map_settings_dict

#Node for building a graph
class Node:
    def __init__(self, point, parent_id, cost, trajectory_cost):
        self.point = point # A 3 by 1 vector [x, y, theta]
        self.parent_id = parent_id # The parent node id that leads to this node (There should only every be one parent in RRT)
        self.cost = cost # The cost to come to this node
        self.trajectory_cost = trajectory_cost # The cost of the individual trajectory from the parent to this node
        self.children_ids = [] # The children node ids of this node
        return

    def add_child(self, child_id):
        self.children_ids.append(child_id)
        return

    def remove_child(self, child_id):
        self.children_ids.remove(child_id)
        return

class TrajectorySpec:
    """
    We split trajectories into two phases.
    Generally, phase 1 is for rotating on the spot and phase 2 is for moving forward.
    """
    def __init__(self, p1_v, p1_w, p1_T, p2_v, p2_w, p2_T):
        self.p1_v = p1_v  # Linear velocity in phase 1
        self.p1_w = p1_w  # Rotational velocity in phase 1
        self.p1_T = p1_T  # Phase 1 execution time
        self.p2_v = p2_v  # Linear velocity in phase 2
        self.p2_w = p2_w  # Rotational velocity in phase 2
        self.p2_T = p2_T  # Phase 2 execution time
        return

#Path Planner 
class PathPlanner:
    #A path planner capable of perfomring RRT and RRT*
    def __init__(self, map_filename, map_setings_filename, goal_point, stopping_dist):
        #Get map information
        self.occupancy_map = load_map(map_filename)
        self.map_shape = self.occupancy_map.shape  # y, x
        self.map_settings_dict = load_map_yaml(map_setings_filename)

        #Get the metric bounds of the map
        self.bounds = np.zeros([2,2]) #m
        self.bounds[0, 0] = self.map_settings_dict["origin"][0]  # x_min
        self.bounds[1, 0] = self.map_settings_dict["origin"][1]  # y_min
        self.bounds[0, 1] = self.map_settings_dict["origin"][0] + (self.map_shape[1] - 1) * self.map_settings_dict["resolution"]  # x_max
        self.bounds[1, 1] = self.map_settings_dict["origin"][1] + (self.map_shape[0] - 1) * self.map_settings_dict["resolution"]  # y_max

        print(f"Map bounds: {self.bounds}. {self.point_to_cell(self.bounds)}")

        #Robot information
        self.robot_radius = 0.22 #m
        self.robot_radius_px = int(np.ceil(self.robot_radius / self.map_settings_dict["resolution"])) #px
        self.vel_max = 0.5 #m/s (Feel free to change!)
        self.rot_vel_max = 0.2 #rad/s (Feel free to change!)

        #Goal Parameters
        self.goal_point = goal_point #m
        self.stopping_dist = stopping_dist #m

        #Trajectory Simulation Parameters
        self.timestep = 1.0 #s
        self.num_substeps = 10
        self.trajectory_type = "straight" #Options are "circular" or "straight"

        #Planning storage
        self.nodes = [Node(np.zeros(3), -1, 0, 0)]
        # self.nodes = [Node(np.array([3, 2, -np.pi/6]), -1, 0)]
        self.used_nodes = set()  # hash set of (x, y) tuples

        #RRT* Specific Parameters
        self.lebesgue_free = np.sum(self.occupancy_map) * self.map_settings_dict["resolution"] **2
        self.zeta_d = np.pi
        self.gamma_RRT_star = 2 * (1 + 1/2) ** (1/2) * (self.lebesgue_free / self.zeta_d) ** (1/2)
        self.gamma_RRT = self.gamma_RRT_star + .1
        self.epsilon = 2.5

        # Sampling Parameters
        self.free_space_sample_rate = 0.05
        self.object_biased_sample_rate = 0.75
        self.bridge_sample_rate = 0 # Not implemented
        self.goal_biased_sample_rate = 0.15
        self.at_goal_sample_rate = 0.05

        self.object_bias_nearby = True  # Whether to bias the object biased sampling toward existing nodes
        self.edge_bias_end = False
        self.near_edge_distances = np.array([])  # Stores the total distance to the nearest node for each near-edge position
        self.near_edge_distribution = np.array([])  # Stores the distribution of the near-edge distances

        assert np.isclose(self.free_space_sample_rate + self.object_biased_sample_rate + self.bridge_sample_rate + self.goal_biased_sample_rate + self.at_goal_sample_rate, 1), "Sampling rates do not sum to 1"

        if self.object_biased_sample_rate > 0:
            self.near_edge_positions = self.get_near_edge_positions()
            # Update the distance distribution for the start node
            self.update_distance_distribution(self.nodes[0].point[:2])
        else:
            self.near_edge_positions = None
            assert not self.object_bias_nearby, "Object biased sampling is not possible without near edge positions"

        self.visualize_tree()
        
        #Pygame window for visualization
        self.window = pygame_utils.PygameWindow(
            "Path Planner", (1000, 1000), self.occupancy_map.shape, self.map_settings_dict, self.goal_point, self.stopping_dist)
        return

    def update_distance_distribution(self, point):
        # The first step is to re-compute the nearest node for each near edge position
        # We can use numpy to efficiently compute the distances
        # The distance is the L2 norm of the difference between the near edge position and the new node
        distances = np.linalg.norm(self.near_edge_positions - point.reshape(1, 2), axis=1)
        # Now we take a minimum of the previous distance and the new distance
        self.near_edge_distances = np.minimum(self.near_edge_distances, distances)
        # We then employ a density function, (x/b) / ((x/b)^2 + a^2) to bias the sampling
        a = 0.2  # The smaller this value, the more biased the sampling
        # b is more difficult to set. It needs to depend on the resolution of the map.
        # It should be a certain number of meters in pixels
        b = 0.25 / self.map_settings_dict["resolution"]
        self.near_edge_distribution = (self.near_edge_distances / b) / ((self.near_edge_distances / b) ** 2 + a ** 2)

        if self.edge_bias_end:
            # Then we also add a term to bias the sampling toward the goal
            goal_distance = np.linalg.norm(self.near_edge_positions - self.goal_point.reshape(1, 2), axis=1) + 1e-6  # Add a small value to avoid division by zero
            near_end_distribution = 1 / goal_distance
            near_end_distribution /= np.sum(near_end_distribution)

        # Normalize the distribution
        self.near_edge_distribution /= np.sum(self.near_edge_distribution)

        if self.edge_bias_end:
            c = 0.9
            self.near_edge_distribution = c * self.near_edge_distribution + (1-c) * near_end_distribution

        if False:
            self.visualize_distance_distribution()

    def visualize_distance_distribution(self, ax=None):
        # Place the near edge distribution onto the map
        was_none = False
        if ax is None:
            was_none = True
            fig, ax = plt.subplots(1, 1, figsize=(10, 5))
        map_copy = np.zeros_like(self.occupancy_map)
        for i, pos in enumerate(self.near_edge_positions):
            x, y = self.point_to_cell(pos.reshape(2, 1))
            map_copy[y, x] = self.near_edge_distribution[i]
        ax.imshow(map_copy)
        if was_none:
            plt.show()

    def add_point(self, point, theta, parent_index, cost, trajectory_cost):
        point_tup = (point[0], point[1])
        node = Node(np.array([point[0], point[1], theta]), parent_index, cost, trajectory_cost)
        self.nodes.append(node)
        self.used_nodes.add(point_tup)

        # Update the parent node to include the new node as a child
        self.nodes[parent_index].add_child(len(self.nodes) - 1)

        if self.object_bias_nearby:
            # Then we need to update the near edge distribution to include the new node
            self.update_distance_distribution(point)

        return node

    def sample_free_space(self):
        #Sample a point from the free space
        x = np.random.uniform(self.bounds[0, 0], self.bounds[0, 1])
        y = np.random.uniform(self.bounds[1, 0], self.bounds[1, 1])
        return np.array([x, y])

    def get_near_edge_positions(self):
        """
        Uses image dilation to find the near-edge positions
        This is a two step process:
            1. Dilate the occupancy map until it is larger than the radius of the robot
            2. Continue dilating a few times to get the near-edge positions
        """
        binary_obstacle_map = self.occupancy_map == 0
        dilated_map = binary_dilation(binary_obstacle_map, iterations=self.robot_radius_px)
        near_edge_positions = binary_dilation(dilated_map, iterations=int(np.ceil(1.5*self.robot_radius_px)))
        edges = near_edge_positions & ~dilated_map

        if False:
            fig, ax = plt.subplots(1, 2, figsize=(10, 5))
            ax[0].imshow(~binary_obstacle_map, cmap='gray')
            ax[0].set_title("Dilated map")
            ax[1].imshow(~edges, cmap='gray')
            ax[1].set_title("Near edge positions")
            plt.show()

        edge_positions_px = np.argwhere(edges)
        # Permute from (y, x) to (x, y)
        edge_positions_px = edge_positions_px[:, [1, 0]]
        # Convert to world coordinates
        edge_positions = self.cell_to_point(edge_positions_px.T)

        edge_positions_px_recovered = self.point_to_cell(edge_positions).T

        # Check for points outside the bounds
        outside_bounds = np.any(np.logical_or(edge_positions < self.bounds[:, 0].reshape(2, 1), edge_positions > self.bounds[:, 1].reshape(2, 1)), axis=0)
        edge_positions = edge_positions[:, ~outside_bounds]

        self.near_edge_distances = np.inf * np.ones(edge_positions.shape[1])
        self.near_edge_distribution = 1/len(self.nodes) * np.ones(len(self.nodes))  # Start uniform

        return edge_positions.T


    def sample_object_biased_space(self):
        # Samples from the near edge positions
        point = self.near_edge_positions[np.random.choice(self.near_edge_positions.shape[0], 1, p=self.near_edge_distribution)][0]
        return point

    #Functions required for RRT
    def sample_map_space(self):
        #Return an [y, x] coordinate to drive the robot towards

        # Step 1: Choose the sampling strategy
        sample_rate = np.random.random()
        if sample_rate < self.free_space_sample_rate:
            # Sample from the free space
            self.last_used_sampling_strategy = "free_space"
            point = self.sample_free_space()
        elif sample_rate < self.free_space_sample_rate + self.object_biased_sample_rate:
            # Sample from the object biased space
            self.last_used_sampling_strategy = "object_biased"
            point = self.sample_object_biased_space()
        elif sample_rate < self.free_space_sample_rate + self.object_biased_sample_rate + self.bridge_sample_rate:
            # Sample from the bridge space
            self.last_used_sampling_strategy = "bridge"
            raise NotImplementedError("Bridge space sampling not implemented")
        elif sample_rate < self.free_space_sample_rate + self.object_biased_sample_rate + self.bridge_sample_rate + self.goal_biased_sample_rate:
            # Sample from the goal biased space
            # Set the variance to be a ratio of the bounds
            self.last_used_sampling_strategy = "goal_biased"
            variance = (self.bounds[:, 1] - self.bounds[:, 0]) / 10
            point = self.goal_point.flatten() + np.random.normal(0, variance, (2,))
            # If the point is outside the bounds, clip it
            x_val = np.clip(point[0], self.bounds[0, 0], self.bounds[0, 1])
            y_val = np.clip(point[1], self.bounds[1, 0], self.bounds[1, 1])
            point = np.array([x_val, y_val])
        else:
            # Sample from the goal space
            self.last_used_sampling_strategy = "at_goal"
            point = self.goal_point.flatten()

        return point
    
    def check_if_duplicate(self, point):
        #Check if point is a duplicate of an already existing node
        point_tup = (point[0], point[1])
        return point_tup in self.used_nodes
    
    def closest_node(self, point):
        #Returns the index of the closest node
        # Linear search is fine for now
        # TODO: Maintain a list of nodes as a numpy array for faster computation
        closest_dist = np.inf
        closest_index = -1
        for i, node in enumerate(self.nodes):
            dist = np.linalg.norm(node.point[:2] - point)
            if dist < closest_dist:
                closest_dist = dist
                closest_index = i
        if closest_index == -1:
            raise ValueError("No nodes in the tree!")
        return closest_index

    def construct_transformation_matrices(self, x, y, theta):
        """
        Constructs the SE(2) transformation matrices from the robot frame to the base frame
        and frame the base frame to the robot frame

        Returns:
            T_0R: The transformation matrix from the robot frame to the base frame
            T_R0: The transformation matrix from the base frame to the robot frame
        """
        T_0R = np.array([
            [np.cos(theta), -np.sin(theta), x],
            [np.sin(theta), np.cos(theta), y],
            [0, 0, 1]
        ])
        
        R_T = T_0R[:2, :2].T
        t_vec = -R_T @ T_0R[:2, 2]
        T_R0 = np.eye(3)
        T_R0[:2, :2] = R_T
        T_R0[:2, 2] = t_vec
        return T_0R, T_R0

    def calculate_circular_trajectory(self, point_t, vel_max, rot_vel_max) -> TrajectorySpec:
        """
        Computes the velocities and time to reach a point in the robot's frame while not exceeding the maximum velocities
        using a circular trajectory.
        Params:
            point_t is the [y, x] point that the robot is driving towards
            vel_max is the maximum linear velocity of the robot
            rot_vel_max is the maximum rotational velocity of the robot
        """

        tx, ty = point_t
        if tx == 0:
            # Compute the min time to reach the point
            T = np.abs(ty) / vel_max
            w = 0  # No rotation
            v = ty / T  # Linear velocity to get to the point in time T
        else:
            angle_to_target = np.arctan2(ty, tx)
            z = 2*angle_to_target
            # First, we compute the minimum time we need to reach the point
            T_min_w = np.abs(z / rot_vel_max)  # Based on angular velocity constraint
            T_min_v = np.abs((tx * z) / (vel_max * np.sin(z)))  # Based on linear velocity constraint
            T = max(T_min_w, T_min_v)

            w = z / T
            if np.sin(z) == 0:
                v = 0
            else:
                v = tx * w / np.sin(z)

        assert np.abs(v) <= vel_max + 1e-6, f"Linear velocity {v} exceeds maximum velocity {vel_max}"
        assert np.abs(w) <= rot_vel_max + 1e-6, f"Rotational velocity {w} exceeds maximum velocity {rot_vel_max}"
        
        return TrajectorySpec(0, 0, 0, v, w, T)  # We do nothing in phase 1 and then drive the circle in phase 2

    def calculate_straight_trajectory(self, point_t, vel_max, rot_vel_max) -> TrajectorySpec:
        """
        Computes the velocities and time to reach a point in the robot's frame while not exceeding the maximum velocities
        using a trajectory where we first turn in place to face the point and then drive straight to it.
        Params:
            point_t is the [y, x] point that the robot is driving towards
            vel_max is the maximum linear velocity of the robot
            rot_vel_max is the maximum rotational velocity of the robot
        """
        tx, ty = point_t
        angle_to_target = np.arctan2(ty, tx)

        # Phase 1: Turn in place as fast as possible to face the point
        T_1 = np.abs(angle_to_target / rot_vel_max)  # Based on angular velocity constraint
        w_1 = angle_to_target / T_1
        v_1 = 0  # No linear velocity

        # Phase 2: Drive straight to the point
        T_2 = np.linalg.norm(point_t) / vel_max
        w_2 = 0
        v_2 = np.linalg.norm(point_t) / T_2

        return TrajectorySpec(v_1, w_1, T_1, v_2, w_2, T_2)
    
    def simulate_trajectory(self, node_i: Node, point_s):
        #Simulates the non-holonomic motion of the robot.
        #This function drives the robot from node_i towards point_s. This function does has many solutions!
        #node_i is a 3 by 1 vector [y;x;theta] this can be used to construct the SE(2) matrix T_{OI} in course notation
        #point_s is the sampled point vector [y;x]
        T_0R, T_R0 = self.construct_transformation_matrices(*(node_i.point.flatten()))
        trajectory_spec = self.robot_controller(T_R0, point_s)

        robot_traj = self.trajectory_rollout(trajectory_spec)
        # Convert to global frame
        robot_traj_xy = (T_0R @ np.vstack((robot_traj[:2, :], np.ones(robot_traj.shape[1]))))[:2, :]
        # Add back in the theta rotated by the initial angle
        robot_traj = np.vstack((robot_traj_xy, robot_traj[2, :] + node_i.point[2]))
        return robot_traj, trajectory_spec.p1_T + trajectory_spec.p2_T
    
    def robot_controller(self, T_R0, point_s) -> TrajectorySpec:
        #This controller determines the velocities that will nominally move the robot from node i to node s
        #Max velocities should be enforced
        target_homog = np.array([point_s[0], point_s[1], 1])  # (x, y, 1)
        local_target = (T_R0 @ target_homog)[:2]  # (x, y)

        if self.trajectory_type == "circular":
            traj_spec = self.calculate_circular_trajectory(local_target, self.vel_max, self.rot_vel_max)
        elif self.trajectory_type == "straight":
            traj_spec = self.calculate_straight_trajectory(local_target, self.vel_max, self.rot_vel_max)
        else:
            raise ValueError(f"Invalid trajectory type {self.trajectory_type}")

        return traj_spec
    
    def trajectory_rollout(self, traj_spec: TrajectorySpec):
        # Given your chosen velocities determine the trajectory of the robot for your given timestep
        # The returned trajectory should be a series of points to check for collisions
        total_time = traj_spec.p1_T + traj_spec.p2_T
        total_num_substeps = int(total_time / (self.timestep / self.num_substeps))
        robot_traj = np.zeros((3, total_num_substeps))
        dt = total_time / total_num_substeps
        x_pos = 0
        y_pos = 0
        theta = 0
        pos = np.zeros((3, total_num_substeps))
        for i in range(total_num_substeps):
            if i * dt < traj_spec.p1_T:
                x_pos += traj_spec.p1_v * np.cos(theta) * dt
                y_pos += traj_spec.p1_v * np.sin(theta) * dt
                theta += traj_spec.p1_w * dt
            else:
                x_pos += traj_spec.p2_v * np.cos(theta) * dt
                y_pos += traj_spec.p2_v * np.sin(theta) * dt
                theta += traj_spec.p2_w * dt
            pos[:, i] = np.array([x_pos, y_pos, theta])
        return pos
    
    def point_to_cell(self, points):
        #Convert a series of [x, y] points in the map to the indices for the corresponding cell in the occupancy map
        #point is a 2 by N matrix of points of interest
        # Step 1: Construct a 2x1 vecotr of the origin point of the map to subtract from the point
        s = np.expand_dims(self.map_settings_dict["origin"][:2], axis=1)
        shifted_points = points - s
        # Step 2: Divide by the resolution
        pixel_coords = shifted_points / self.map_settings_dict["resolution"]
        # Step 3: Round to the nearest integer
        pixel_coords = np.round(pixel_coords).astype(int)
        pixel_coords[1, :] *= -1
        pixel_coords[1, :] += self.map_shape[0] - 1
        return pixel_coords

    def cell_to_point(self, cells):
        #Convert a series of [y, x] cells in the occupancy map to the corresponding points in the map
        # Exactly the inverse of point_to_cell
        # Step 1: Convert to the origin
        s = np.expand_dims(self.map_settings_dict["origin"][:2], axis=1)
        shifted_cells = cells.copy()
        shifted_cells[1, :] = self.map_shape[0] - 1 - shifted_cells[1, :]
        # Step 2: Multiply by the resolution
        points = shifted_cells * self.map_settings_dict["resolution"]
        # Step 3: Add the origin
        points += s
        return points

    def points_to_robot_circle(self, points):
        #Convert a series of [y,x] points to robot map footprints for collision detection
        #Hint: The disk function is included to help you with this function
        pixel_points = self.point_to_cell(points)
        disks = []
        for point_ind in range(pixel_points.shape[1]):
            disks.append(np.array(disk(pixel_points[:, point_ind], radius=self.robot_radius_px, shape=(self.map_shape[1], self.map_shape[0]))))
        disk_points = np.hstack(disks)
        xs = disk_points[0, :]
        ys = disk_points[1, :]
        return xs, ys
    #Note: If you have correctly completed all previous functions, then you should be able to create a working RRT function

    #RRT* specific functions
    def ball_radius(self):
        #Close neighbor distance
        card_V = len(self.nodes)
        return min(self.gamma_RRT * (np.log(card_V) / card_V ) ** (1.0/2.0), self.epsilon)
    
    # def connect_node_to_point(self, node_i, point_f):
    #     #Given two nodes find the non-holonomic path that connects them
    #     #Settings
    #     #node is a 3 by 1 node
    #     #point is a 2 by 1 point
    #     print("TO DO: Implement a way to connect two already existing nodes (for rewiring).")
    #     return np.zeros((3, self.num_substeps))
    
    # def cost_to_come(self, trajectory_o):
    #     #The cost to get to a node from lavalle 
    #     print("TO DO: Implement a cost to come metric")
    #     return 0

    def get_nearby_nodes(self, node: Node):
        """
        Finds all nodes that are within ball_radius of the given node
        """
        nearby_nodes = []
        ball_radius_sqr = self.ball_radius() ** 2
        for other_node_id in range(len(self.nodes)):
            other_node = self.nodes[other_node_id]
            if other_node == node:
                continue
            dist_sqr = np.sum((node.point[:2] - other_node.point[:2]) ** 2)
            if dist_sqr < ball_radius_sqr:
                nearby_nodes.append(other_node_id)
        return nearby_nodes

    def rewire_node(self, to_rewire_node_id: int, new_parent_node_id: int, new_trajectory_cost: float):
        """
        Rewires the given node to have a new parent and a new trajectory cost
        """
        to_rewire_node = self.nodes[to_rewire_node_id]
        old_parent_id = to_rewire_node.parent_id
        new_parent_node = self.nodes[new_parent_node_id]
        old_parent_node = self.nodes[old_parent_id]

        to_rewire_node.parent_id = new_parent_node_id
        to_rewire_node.trajectory_cost = new_trajectory_cost
        to_rewire_node.cost = new_parent_node.cost + new_trajectory_cost

        # Set the theta to point from the new parent to the node
        to_rewire_node.point[2] = np.arctan2(to_rewire_node.point[1] - new_parent_node.point[1], to_rewire_node.point[0] - new_parent_node.point[0])

        # Remove the node from the old parent's children
        old_parent_node.remove_child(to_rewire_node_id)
        # Add the node to the new parent's children
        new_parent_node.add_child(to_rewire_node_id)
        # Update the children of the node
        self.update_children(to_rewire_node_id)
    
    def update_children(self, node_id):
        #Given a node_id with a changed cost, update all connected nodes with the new cost
        #We assume that the trajectory has not changed so the new cost is node_i.cost + child.trajectory_cost
        node = self.nodes[node_id]
        for child_id in node.children_ids:
            child = self.nodes[child_id]
            child.cost = node.cost + child.trajectory_cost
            self.update_children(child_id)

    def visualize_tree(self, show_path=False):
        fig = plt.figure(figsize=(10, 10))  # You can adjust the figure size as needed
        occupancy_map = self.occupancy_map.copy()  # 0 is occupied, 1 is free
        for node in self.nodes:
            x, y = self.point_to_cell(node.point[:2].reshape(2, 1))
            # occupancy_map[y, x] = 0.5  # Marking the node on the map
            plt.plot(x, y, 'bo')
            # Put an edge to the parent
            if node.parent_id != -1:
                parent = self.nodes[node.parent_id]
                x_parent, y_parent = self.point_to_cell(parent.point[:2].reshape(2, 1))
                plt.plot([x, x_parent], [y, y_parent], color="red")  # Drawing the line
        plt.imshow(occupancy_map, cmap='gray')  # Adjust colormap as needed
        
        # Plot the goal point
        x, y = self.point_to_cell(self.goal_point)
        plt.plot(x, y, 'go')

        if show_path:
            best_node = self.find_best_node()
            path = self.recover_path(best_node)
            for i in range(1, len(path)):
                x, y = self.point_to_cell(path[i - 1][:2].reshape(2, 1))
                x_next, y_next = self.point_to_cell(path[i][:2].reshape(2, 1))
                plt.plot([x, x_next], [y, y_next], color="green")


        plt.show()


    #Planner Functions
    def rrt_planning(self):
        #This function performs RRT on the given map and robot
        #You do not need to demonstrate this function to the TAs, but it is left in for you to check your work
        # for i in range(50): #Most likely need more iterations than this to complete the map!
        count = 0
        while True:
            #Sample map space
            point = self.sample_map_space()

            # Just in case, check if node is inside the map
            if not (self.bounds[0, 0] <= point[0] <= self.bounds[0, 1] and self.bounds[1, 0] <= point[1] <= self.bounds[1, 1]):
                raise ValueError(f"Sampled point {point} is outside the map bounds. Last used sampling strategy: {self.last_used_sampling_strategy}")

            #Get the closest point
            closest_node_id = self.closest_node(point)
            closes_node = self.nodes[closest_node_id]

            if self.check_if_duplicate(point):
                # print(f"Duplicate point {point}. Skipping.")
                continue

            if np.isclose(closes_node.point[:2], point).all():
                # APAERNTLY My is duplicate isn't working so here's another one
                continue

            #Simulate driving the robot towards the closest point
            trajectory_o, trajectory_cost = self.simulate_trajectory(closes_node, point)

            if len(trajectory_o) == 0:
                # print(f"Trajectory from {closes_node.point[:2]} to {point} is empty. Skipping.")
                continue

            #Check for collisions
            try:
                ri_x, ri_y = self.points_to_robot_circle(trajectory_o[:2, :])
            except ValueError as e:
                # I give up. I don't know what's causing it.
                print(f"Error: {e}")
                continue

            # We can check for intersection by indexing the occupancy map with the robot's footprint
            # In the occupancy map, 0 is occupied and 1 is free
            if np.any(self.occupancy_map[ri_y, ri_x] == 0):
                # print(f"Collision detected at point {point}. Skipping.")
                continue

            if count % 1000 == 0 and False:
                fig, ax = plt.subplots(1, 3, figsize=(10, 5))
                # Plot the trajectory
                ax[0].plot(trajectory_o[0, :], trajectory_o[1, :], 'r')
                # Plot the initial and final points
                ax[0].plot(closes_node.point[0], closes_node.point[1], 'bo')
                ax[0].plot(point[0], point[1], 'go')
                # Visualize theta by putting arrows at every sample
                for i in range(0, trajectory_o.shape[1], 10):
                    ax[0].arrow(trajectory_o[0, i], trajectory_o[1, i], np.cos(trajectory_o[2, i]) / 10, np.sin(trajectory_o[2, i]) / 10, head_width=0.05, head_length=0.05)

                # Visualize the robot's footprint
                cells_trajectory = self.point_to_cell(trajectory_o[:2, :])
                traj_x = cells_trajectory[0, :]
                traj_y = cells_trajectory[1, :]
                # Cap these values to be within the map
                # traj_x = np.clip(traj_x, 0, self.map_shape[1] - 1)
                # traj_y = np.clip(traj_y, 0, self.map_shape[0] - 1)
                map_copy = self.occupancy_map.copy() * 3
                map_copy[ri_y, ri_x] = 1
                map_copy[traj_y, traj_x] = 2

                # Put a dot on every node
                for node in self.nodes:
                    x, y = self.point_to_cell(node.point[:2].reshape(2, 1))
                    map_copy[y, x] = 4

                # For the selected node, put a different color
                x, y = self.point_to_cell(closes_node.point[:2].reshape(2, 1))
                map_copy[y, x] = 5
                
                ax[1].imshow(map_copy)

                self.visualize_distance_distribution(ax=ax[2])
            
                print(f"Closest node: {closes_node.point[:2]}")
                plt.show()

            # Add the point to the tree
            # RRT does not have a cost to come
            self.add_point(point, trajectory_o[2, -1], closest_node_id, -1, -1)
            count += 1
            print(f"Added point {point} to the tree with parent {closes_node.point[:2]}")
            
            #Check if goal has been reached
            print(f"Distance to goal: {np.linalg.norm(point - self.goal_point.flatten())}. Needs to be less than {self.stopping_dist}")
            if np.linalg.norm(point - self.goal_point.flatten()) < self.stopping_dist:
                print("Goal Reached!")
                break
        self.visualize_tree(show_path=True)
        return self.nodes
    
    def rrt_star_planning(self, recursive=False):
        #This function performs RRT* for the given map and robot       
        end_count = np.inf 
        count = 0
        while count < end_count: #Most likely need more iterations than this to complete the map!
            

            #Sample
            point = self.sample_map_space()

            #Closest Node
            closest_node_id = self.closest_node(point)
            closest_node = self.nodes[closest_node_id]

            def get_valid_trajectory(node_i, point_f):
                """
                Gets a trajectory from node_i to point_f if it exists
                Returns:
                    trajectory_o: The trajectory
                    trajectory_cost: The cost of the trajectory
                Or both are None if the trajectory is invalid
                """
                #Simulate trajectory
                trajectory_o, trajectory_cost = self.simulate_trajectory(node_i, point_f)  # 3xN (y, x, theta)
                if len(trajectory_o) == 0:
                    # print(f"Trajectory from {closes_node.point[:2]} to {point} is empty. Skipping.")
                    return None, None

                #Check for Collision
                try:
                    ri_x, ri_y = self.points_to_robot_circle(trajectory_o[:2, :])
                except ValueError as e:
                    # I give up. I don't know what's causing it.
                    print(f"Error: {e}")
                    return None, None

                # We can check for intersection by indexing the occupancy map with the robot's footprint
                # In the occupancy map, 0 is occupied and 1 is free
                if np.any(self.occupancy_map[ri_y, ri_x] == 0):
                    # print(f"Collision detected at point {point}. Skipping.")
                    return None, None
                
                return trajectory_o, trajectory_cost

            trajectory_o, trajectory_cost = get_valid_trajectory(closest_node, point)
            if trajectory_o is None:
                continue

            # Compute the cost to come. This is the cost to come of the parent plus the cost of the trajectory
            cost_to_come = closest_node.cost + trajectory_cost
            # Now we can add the node to the tree
            new_node = self.add_point(point, trajectory_o[2, -1], closest_node_id, cost_to_come, trajectory_cost)
            new_node_id = len(self.nodes) - 1

            #Last node rewire
            # Steps:
            # 1: Find all nodes within the ball radius
            # 2: For each node, check if a trajectory from that node to the new node is collision free
            # 3: If it is, check if the cost to come is less than the current cost to come
            nearby_node_ids = self.get_nearby_nodes(new_node)
            new_parent_node_id = None
            best_cost_to_come = cost_to_come
            new_trajectory_cost = -1
            for nearby_node_id in nearby_node_ids:
                nearby_node = self.nodes[nearby_node_id]
                # Check if the trajectory is collision free
                trajectory_o, trajectory_cost = get_valid_trajectory(nearby_node, point)
                if trajectory_o is not None:
                    # Compute the cost to come
                    new_cost_to_come = nearby_node.cost + trajectory_cost
                    if new_cost_to_come < best_cost_to_come:
                        new_parent_node_id = nearby_node_id
                        best_cost_to_come = new_cost_to_come
                        new_trajectory_cost = trajectory_cost
            if new_parent_node_id is not None:
                # Then we need to rewire the tree
                print(f"Rewiring new node {new_node.point[:2]} to parent {self.nodes[new_parent_node_id].point[:2]} with cost {best_cost_to_come}")
                self.rewire_node(new_node_id, new_parent_node_id, new_trajectory_cost)

            #Close node rewire
            rewired_set = set()
            to_rewire_ids = nearby_node_ids.copy()
            # for node_id in to_rewire_ids:
            while len(to_rewire_ids) > 0:
                node_id = to_rewire_ids.pop(0)
                if node_id in rewired_set:
                    continue
                rewired_set.add(node_id)
                node = self.nodes[node_id]
                # Check if the trajectory from the new node to the old node is collision free
                trajectory_o, trajectory_cost = get_valid_trajectory(new_node, node.point[:2])  # TODO: It would be more efficient to check if the cost is lower before doing collision checking.
                if trajectory_o is None:
                    continue
                # Compute the cost to come
                new_cost_to_come = new_node.cost + trajectory_cost
                if new_cost_to_come < node.cost:
                    print(f"Rewiring node {node.point[:2]} to new node {new_node.point[:2]} with cost {new_cost_to_come}")
                    self.rewire_node(node_id, new_node_id, trajectory_cost)
                    if recursive:
                        # Take a ball around the rewired node and rewire all nodes within it
                        nearby_node_ids = self.get_nearby_nodes(node)
                        for nearby_node_id in nearby_node_ids:
                            to_rewire_ids.append(nearby_node_id)

            #Check for early end
            #Check if goal has been reached
            print(f"Distance to goal: {np.linalg.norm(point - self.goal_point.flatten())}. Needs to be less than {self.stopping_dist}")
            if np.linalg.norm(point - self.goal_point.flatten()) < self.stopping_dist:
                print("Goal Reached!")
                if end_count == np.inf:
                    end_count = 2*count + 200
            count += 1
            print(f"Count: {count}. End count: {end_count}")
        self.visualize_tree(show_path=True)
        return self.nodes
    
    def find_best_node(self):
        """
        Finds the node that is within the ball radius of the goal with the lowest cost to come
        """
        best_node_id = -1
        best_cost_to_come = np.inf
        ball_radius_sqr = self.stopping_dist ** 2
        for node_id in range(len(self.nodes)):
            node = self.nodes[node_id]
            dist_sqr = np.sum((node.point[:2] - self.goal_point.flatten()) ** 2)
            if dist_sqr < ball_radius_sqr and node.cost < best_cost_to_come:
                best_cost_to_come = node.cost
                best_node_id = node_id
        return best_node_id
    
    def recover_path(self, node_id = -1):
        path = [self.nodes[node_id].point.reshape(3, 1)]
        current_node_id = self.nodes[node_id].parent_id
        while current_node_id > -1:
            path.append(self.nodes[current_node_id].point.reshape(3, 1))
            current_node_id = self.nodes[current_node_id].parent_id
        path.reverse()
        return path

def main():
    #Set map information
    map_filename = "willowgarageworld_05res.png"
    map_setings_filename = "willowgarageworld_05res.yaml"

    # map_filename = "myhal.png"
    # map_setings_filename = "myhal.yaml"

    #robot information
    goal_point = np.array([[9], [4]]) #m
    # goal_point = np.array([[9], [0]]) #m
    # goal_point = np.array([[20], [8]]) #m
    # goal_point = np.array([[41.5], [-44.5]]) #m
    # goal_point = np.array([[20], [-30]]) #m
    # goal_point = np.array([[7], [2]]) #m
    stopping_dist = 0.5 #m

    # goal_point = np.array([[7], [0]]) #m

    #RRT precursor
    path_planner = PathPlanner(map_filename, map_setings_filename, goal_point, stopping_dist)
    # nodes = path_planner.rrt_planning()
    nodes = path_planner.rrt_star_planning(recursive=True)
    best_node = path_planner.find_best_node()
    path = path_planner.recover_path(best_node)
    node_path_metric = np.hstack(path)

    #Leftover test functions
    np.save("shortest_path.npy", node_path_metric)


if __name__ == '__main__':
    main()
