#!/usr/bin/env python3
from __future__ import division, print_function
import os

import numpy as np
import matplotlib.pyplot as plt
from scipy.linalg import block_diag
from skimage.draw import disk
from scipy.spatial.distance import cityblock
import rospy
import tf2_ros

# msgs
from geometry_msgs.msg import TransformStamped, Twist, PoseStamped
from nav_msgs.msg import Path, Odometry, OccupancyGrid
from visualization_msgs.msg import Marker

# ros and se2 conversion utils
import utils


TRANS_GOAL_TOL = 0.2# .1  # m, tolerance to consider a goal complete
ROT_GOAL_TOL = 0.6  # rad, tolerance to consider a goal complete
TRANS_VEL_OPTS = [0.2] #[0, 0.13, 0.20, 0.23, 0.25, 0.257, 0.26]  # m/s, max of real robot is .26
ROT_VEL_OPTS = [-0.8, -0.4, -0.1, 0, 0.1, 0.4, 0.8] #np.linspace(-1.82, 1.82, 21)  # rad/s, max of real robot is 1.82
CONTROL_RATE = 5  # Hz, how frequently control signals are sent
CONTROL_HORIZON = 3 #5  # seconds. if this is set too high and INTEGRATION_DT is too low, code will take a long time to run!
INTEGRATION_DT = 0.1 #.025  # s, delta t to propagate trajectories forward by
COLLISION_RADIUS = 0.225  # m, radius from base_link to use for collisions, min of 0.2077 based on dimensions of .281 x .306
ROT_DIST_MULT = .1  # multiplier to change effect of rotational distance in choosing correct control
OBS_DIST_MULT = .1  # multiplier to change the effect of low distance to obstacles on a path
MIN_TRANS_DIST_TO_USE_ROT = TRANS_GOAL_TOL  # m, robot has to be within this distance to use rot distance in cost
PATH_NAME = 'path.npy'  # saved path from l2_planning.py, should be in the same directory as this file

# here are some hardcoded paths to use if you want to develop l2_planning and this file in parallel
# TEMP_HARDCODE_PATH = [[2, 0, 0], [2.75, -1, -np.pi/2], [2.75, -4, -np.pi/2], [2, -4.4, np.pi]]  # almost collision-free
TEMP_HARDCODE_PATH = [[2.5, -.5, 0], [2.6, -1, -np.pi/2], [2.45, -3.8, -np.pi/2], [1.5, -4.4, np.pi]]  # some possible collisions
# TEMP_HARDCODE_PATH = [[3, -.5, 0], [3.2, -1, -np.pi/2], [3.3, -3.8, -np.pi/2], [1.5, -4.4, np.pi]]  # some possible collisions
# TEMP_HARDCODE_PATH = [[20, -10, 0]]


#Map Handling Functions
def load_map(filename):
    import matplotlib.image as mpimg
    import cv2 
    im = cv2.imread("../maps/" + filename)
    im = cv2.flip(im, 0)
    # im = mpimg.imread("../maps/" + filename)
    if len(im.shape) > 2:
        im = im[:,:,0]
    im_np = np.array(im)  #Whitespace is true, black is false
    im_np = np.logical_not(im_np)     #for ros
    return im_np

class PathFollower():
    def __init__(self):
        # time full path
        self.path_follow_start_time = rospy.Time.now()

        # use tf2 buffer to access transforms between existing frames in tf tree
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)
        rospy.sleep(1.0)  # time to get buffer running

        # constant transforms
        self.map_odom_tf = self.tf_buffer.lookup_transform('map', 'odom', rospy.Time(0), rospy.Duration(2.0)).transform
        print(f"Map odom tf \n{self.map_odom_tf}")

        # subscribers and publishers
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.global_path_pub = rospy.Publisher('~global_path', Path, queue_size=1, latch=True)
        self.local_path_pub = rospy.Publisher('~local_path', Path, queue_size=1)
        self.collision_marker_pub = rospy.Publisher('~collision_marker', Marker, queue_size=1)

        # map
        map = rospy.wait_for_message('/map', OccupancyGrid)
        self.map_np = np.array(map.data).reshape(map.info.height, map.info.width)
        self.map_resolution = round(map.info.resolution, 5)
        self.map_shape = self.map_np.shape
        self.map_origin = -utils.se2_pose_from_pose(map.info.origin)  # negative because of weird way origin is stored
        print(f"Map origin: {self.map_origin}")
        self.map_nonzero_idxes = np.argwhere(self.map_np)
        print(f"Got map of size {self.map_shape}")
        print(map)

        # map_filename = "myhal.png"
        # occupancy_map = load_map(map_filename)
        # self.map_np = occupancy_map
        # self.map_resolution = 0.05
        # self.map_shape = self.map_np.shape
        # self.map_origin = np.array([ 0.2 , 0.2 ,-0. ])
        # self.map_nonzero_idxes = np.argwhere(self.map_np)


        # collisions
        self.collision_radius_pix = COLLISION_RADIUS / self.map_resolution
        self.collision_marker = Marker()
        self.collision_marker.header.frame_id = '/map'
        self.collision_marker.ns = '/collision_radius'
        self.collision_marker.id = 0
        self.collision_marker.type = Marker.CYLINDER
        self.collision_marker.action = Marker.ADD
        self.collision_marker.scale.x = COLLISION_RADIUS * 2
        self.collision_marker.scale.y = COLLISION_RADIUS * 2
        self.collision_marker.scale.z = 1.0
        self.collision_marker.color.g = 1.0
        self.collision_marker.color.a = 0.5

        # transforms
        self.map_baselink_tf = self.tf_buffer.lookup_transform('map', 'base_link', rospy.Time(0), rospy.Duration(2.0))
        # self.map_baselink_tf = self.tf_buffer.lookup_transform('map', 'base_footprint', rospy.Time(0), rospy.Duration(2.0))
        self.pose_in_map_np = np.zeros(3)
        self.pos_in_map_pix = np.zeros(2)
        self.update_pose()

        # path variables
        cur_dir = os.path.dirname(os.path.realpath(__file__))

        # to use the temp hardcoded paths above, switch the comment on the following two lines
        self.path_tuples = np.load(os.path.join(cur_dir, 'shortest_path_willow_large_rrt_star.npy')).T
        # self.path_tuples = np.array(TEMP_HARDCODE_PATH)

        self.path = utils.se2_pose_list_to_path(self.path_tuples, 'map')
        self.global_path_pub.publish(self.path)

        # goal
        self.cur_goal = np.array(self.path_tuples[0])
        self.cur_path_index = 0

        # trajectory rollout tools
        # self.all_opts is a Nx2 array with all N possible combinations of the t and v vels, scaled by integration dt
        self.all_opts = np.array(np.meshgrid(TRANS_VEL_OPTS, ROT_VEL_OPTS)).T.reshape(-1, 2)

        # if there is a [0, 0] option, remove it
        all_zeros_index = (np.abs(self.all_opts) < [0.001, 0.001]).all(axis=1).nonzero()[0]
        if all_zeros_index.size > 0:
            self.all_opts = np.delete(self.all_opts, all_zeros_index, axis=0)
        self.all_opts_scaled = self.all_opts * INTEGRATION_DT

        self.num_opts = self.all_opts_scaled.shape[0]
        self.horizon_timesteps = int(np.ceil(CONTROL_HORIZON / INTEGRATION_DT))

        self.rate = rospy.Rate(CONTROL_RATE)

        rospy.on_shutdown(self.stop_robot_on_shutdown)
        

        self.local_trajectories = self.compute_local_trajectories(self.all_opts, self.horizon_timesteps, dt=INTEGRATION_DT)
        self.local_colliders_px = self.compute_local_colliders(self.local_trajectories)

        self.follow_path()

    def compute_local_trajectories(self, all_opts, horizon_timesteps, dt):
        """
        Uses a trajectory rollout to compute trajectories as if they are starting at (0, 0, 0)
        """
        trajectories = np.zeros([horizon_timesteps + 1, all_opts.shape[0], 3])
        for opt_ind in range(all_opts.shape[0]):
            x_pos = 0
            y_pos = 0
            theta = 0
            for t in range(1, horizon_timesteps + 1):
                x_pos += all_opts[opt_ind, 0] * np.cos(theta) * dt
                y_pos += all_opts[opt_ind, 0] * np.sin(theta) * dt
                theta += all_opts[opt_ind, 1] * dt
                trajectories[t, opt_ind] = [x_pos, y_pos, theta]
        return trajectories

    def point_to_px(self, points):
        #Convert a series of [x, y] points in the map to the indices for the corresponding cell in the occupancy map
        #point is a 2 by N matrix of points of interest
        # # Step 1: Construct a 2x1 vecotr of the origin point of the map to subtract from the point
        # s = np.expand_dims(self.map_settings_dict["origin"][:2], axis=1)
        # shifted_points = points - s
        # # Step 2: Divide by the resolution
        # pixel_coords = shifted_points / self.map_settings_dict["resolution"]
        # # Step 3: Round to the nearest integer
        # pixel_coords = np.round(pixel_coords).astype(int)
        # pixel_coords[1, :] *= -1
        # pixel_coords[1, :] += self.map_shape[0] - 1
        # return pixel_coords

        # Apparently we are doing this differently here
        # Down below it happens like this (self.map_origin[:2] + self.pose_in_map_np[:2]) / self.map_resolution
        shifted_points = np.expand_dims(self.map_origin[:2], axis=1) + points  # 2xN
        pixel_coords = shifted_points / self.map_resolution
        return pixel_coords

    def generate_trajectory_collider(self, trajectory, unique=False, use_map_shape=True):
        """
        Generates a list of pixel locations that are occupied by the robot at each timestep in the trajectory

        trajectory: np.array of shape (horizon_timesteps + 1, 3). Should be in Meters.
        radius_px: float, radius of the robot in pixels
        """
        # Step 1: Convert the trajectory to pixels
        point_trajectory = trajectory[:, :2]
        # Change to the 2XN shape from the current Nx2 shape
        point_trajectory = point_trajectory.T
        trajectory_px = self.point_to_px(point_trajectory)

        return self.generate_local_collider(trajectory_px, unique=unique, use_map_shape=use_map_shape)
    
    def generate_local_collider(self, trajectory_px, unique=False, use_map_shape=False):
        """
        trajectory_px: np.array of shape (2, horizon_timesteps + 1). Should be in pixels.
        """
        trajectory_colliders = []
        shape = (self.map_shape[1], self.map_shape[0]) if use_map_shape else None
        for point_ind in range(trajectory_px.shape[1]):
            collider = np.array(disk(trajectory_px[:, point_ind], radius=self.collision_radius_pix, shape=shape))
            trajectory_colliders.append(collider.T)
        # VStack the colliders to get the final shape
        trajectory_colliders = np.vstack(trajectory_colliders)
        # The colliders is now an Mx2 array of the pixel locations of the robot's footprint
        if unique:
            # In order to do more efficient computations, we now de-dupe the colliders
            trajectory_colliders = np.unique(trajectory_colliders, axis=0)

        return trajectory_colliders.T  # Return the transposed array to get the 2xM shape

    def compute_local_colliders(self, local_trajectories):
        """
        Computes the pixel locations in the local frame for the colliders of the robot

        local_trajectories: np.array of shape (horizon_timesteps + 1, num_opts, 3). Should be in meters.
        map_resolution: float, meters per pixel
        radius_px: float, radius of the robot in pixels
        map_shape: tuple, shape of the occupancy grid in pixels.
        """
        # Step 1: Generate the disks for each point in each trajectory
        # NOTE: the generate_trajectory_collider function converts to pixels so we do not need to do that here
        trajectory_opt_colliders = []
        for opt in range(local_trajectories.shape[1]):
            # trajectory_opt_colliders.append(self.generate_trajectory_collider(local_trajectories[:, opt], unique=True, use_map_shape=False))
            local_trajectory = local_trajectories[:, opt]  # (horizon_timesteps + 1, 3)
            local_trajectory_px = local_trajectory[:, :2].T / self.map_resolution
            trajectory_opt_colliders.append(self.generate_local_collider(local_trajectory_px, unique=True, use_map_shape=False))

        return trajectory_opt_colliders  # N of 2xM where N is the number of options, M is the number of colliding pixels

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

    def follow_path(self):
        print("Starting path following")
        self.control = [None, None]
        while not rospy.is_shutdown():
            print(f"Pose: x={self.pose_in_map_np[0]}, y={self.pose_in_map_np[1]}, theta={self.pose_in_map_np[2]}, goal: {self.cur_path_index}")
            # timing for debugging...loop time should be less than 1/CONTROL_RATE
            tic = rospy.Time.now()

            self.update_pose()
            self.check_and_update_goal()


            # # start trajectory rollout algorithm
            # local_paths = np.zeros([self.horizon_timesteps + 1, self.num_opts, 3])
            # local_paths[0] = np.atleast_2d(self.pose_in_map_np).repeat(self.num_opts, axis=0)

            # print("TO DO: Propogate the trajectory forward, storing the resulting points in local_paths!")
            # for t in range(1, self.horizon_timesteps + 1):
            #     # propogate trajectory forward, assuming perfect control of velocity and no dynamic effects
            #     pass

            # Compute the local trajectories by transforming the local paths to the robot frame
            T_0R, T_R0 = self.construct_transformation_matrices(self.pose_in_map_np[0], self.pose_in_map_np[1], self.pose_in_map_np[2])
            # self.local_trajectories is a NxMx3 array of the local trajectories in the robot frame (M is the number of timesteps, N is the number of options)
            global_trajectories = np.zeros_like(self.local_trajectories)
            for opt_ind in range(self.num_opts):
                # Extract the theta and convert to homogenous coordinates
                trajectory_theta = self.local_trajectories[:, opt_ind, 2]
                trajectory_xy = self.local_trajectories[:, opt_ind, :2]
                trajectory_hom = np.hstack([trajectory_xy, np.ones((self.local_trajectories.shape[0], 1))])
                global_trajectory_hom = trajectory_hom @ T_0R.T  # Mx3 x 3x3 = Mx3
                global_theta = trajectory_theta + self.pose_in_map_np[2]  # Mx1
                global_trajectories[:, opt_ind] = np.hstack([global_trajectory_hom[:, :2], global_theta[:, None]])

            if self.control[0] == 0:
                # Now we visualize all these trajectories in the global frame using matplotlib
                fig, ax = plt.subplots()
                # Plot each trajectory as a line
                for opt_ind in range(self.num_opts):
                    ax.plot(global_trajectories[:, opt_ind, 0], global_trajectories[:, opt_ind, 1])
                # Show the plot
                plt.show()
                return

            # TODO: I think it would be more efficient to do scoring first and then collision checks since collision checks involve iterating
            # over a lot of points to create the colliders.

            # # check all trajectory points for collisions
            # # first find the closest collision point in the map to each local path point
            # local_paths_pixels = (self.map_origin[:2] + local_paths[:, :, :2]) / self.map_resolution
            # valid_opts = range(self.num_opts)
            # local_paths_lowest_collision_dist = np.ones(self.num_opts) * 50

            # print("TO DO: Check the points in local_path_pixels for collisions")
            # for opt in range(local_paths_pixels.shape[1]):
            #     for timestep in range(local_paths_pixels.shape[0]):
            #         pass

            # Instead of that, we iterate over the global frame trajectories (m) and compute the colliders in the global frame
            # Then we can use numpy indexing to efficiently compute overlap between the colliders and the map
            valid_opts = []
            all_colliders = []
            for opt_ind in range(self.num_opts):
                trajectory = global_trajectories[:, opt_ind]
                colliders = self.generate_trajectory_collider(trajectory, unique=False)
                # all_colliders.append(colliders)
                # Check for collisions
                collision = np.any(self.map_np[colliders[1], colliders[0]])
                if not collision:
                    all_colliders.append(colliders)
                    valid_opts.append(opt_ind)

            # T_0R_px, T_R0_px = self.construct_transformation_matrices(self.pos_in_map_pix[0], self.pos_in_map_pix[1], self.pose_in_map_np[2])

            # valid_opts = []
            # all_colliders = []
            # for opt_ind in range(self.num_opts):
            #     local_colliders = self.local_colliders_px[opt_ind]
            #     local_collider_hom = np.vstack([local_colliders, np.ones(local_colliders.shape[1])])  # 3xM homogeneous coordinates
            #     global_colliders = T_0R_px @ local_collider_hom  # 3x3 @ 3xM

            #     global_colliders = global_colliders[:2]  # 2xM
            #     global_colliders = global_colliders.astype(int)
            #     # Check for collisions
            #     collision = np.any(self.map_np[global_colliders[1], global_colliders[0]])
                
            #     if not collision:
            #         all_colliders.append(global_colliders)
            #         valid_opts.append(opt_ind)

            if len(valid_opts) == 0:
                # If there are no valid options, then we should try backing up and see if that helps.
                self.control = [-1, 0]
            else:
                # remove trajectories that were deemed to have collisions
                # print("TO DO: Remove trajectories with collisions!")
                valid_trajectories = global_trajectories[:, valid_opts]  # NxM'x3 where M' is the number of valid options for the trajectory and N is the number of timesteps

                if self.control[0] == 0 or False:
                    all_colliders = np.hstack(all_colliders)
                    # Visualize the colliders on the map
                    fig, ax = plt.subplots()
                    # To do this, we copy the map and make all the colliders 50
                    map_with_colliders = self.map_np.copy()
                    map_with_colliders[all_colliders[1], all_colliders[0]] = 50
                    ax.imshow(map_with_colliders)
                    print(f"Current pixel position: {self.pos_in_map_pix}")
                    print(f"Num colliders: {len(self.local_colliders_px)}")
                    # Plot the current position
                    ax.plot(self.pos_in_map_pix[0], self.pos_in_map_pix[1], 'ro')
                    # Plot the colliders as dots
                    # ax.plot(colliders[0], colliders[1], 'bo')

                    # #plot the valid trajectories
                    # for opt_ind in valid_opts:
                    #     trajectory = global_trajectories[:, opt_ind]
                    #     trajectory_px = self.point_to_px(trajectory[:, :2].T)
                    #     ax.plot(trajectory_px[0], trajectory_px[1], 'g')

                    # Print out the valid controls
                    print(f"Valid controls: {self.all_opts[valid_opts]}")

                    # Plot the valid trajectories
                    for opt_ind in range(valid_trajectories.shape[1]):
                        trajectory_px = self.point_to_px(valid_trajectories[:, opt_ind, :2].T)
                        ax.plot(trajectory_px[0], trajectory_px[1], 'g')

                    plt.show()
                    return

                # # calculate final cost and choose best option
                # print("TO DO: Calculate the final cost and choose the best control option!")
                # final_cost = np.zeros(self.num_opts)
                # if final_cost.size == 0:  # hardcoded recovery if all options have collision
                #     control = [-.1, 0]
                # else:
                #     best_opt = valid_opts[final_cost.argmin()]
                #     control = self.all_opts[best_opt]
                #     self.local_path_pub.publish(utils.se2_pose_list_to_path(local_paths[:, best_opt], 'map'))

                final_positions = valid_trajectories[-1, :, :2]  # M'x2
                # # TODO: This is a bad way of generating the cost function. It assumes that the only way the robot can be at the goal is if it is at the goal at the last timestep.
                # # The correct way to do this would be to select the point in each trajectory that we are closest to the goal and use that as the "final position".

                # Improvement: Instead of selecting -1 as the final position timestep, compute the timestep that is closest to the goal for each trajectory
                valid_trajectories_xy = valid_trajectories[:, :, :2]  # NxM'x2 where N is the number of timesteps and M' is the number of valid options
                # Subtract off the goal position prepare to get the distance. self.cur_goal is a 3x1 array
                goal_diff = valid_trajectories_xy - self.cur_goal[:2]  # NxM'x2
                # Square and sum over the last axis to get the square distance to the goal
                goal_squared_distance = np.sum(goal_diff ** 2, axis=2)  # NxM'
                # Take the argmin over axis=0 to get the index of the closest point to the goal for each trajectory
                closest_goal_index = goal_squared_distance.argmin(axis=0)  # M'
                # Select the position of the closest point to the goal for each trajectory
                # Why doesn't : work for dimension 1? It should be the same as np.arange(valid_trajectories.shape[1])?
                closest_positions = valid_trajectories[closest_goal_index, np.arange(valid_trajectories.shape[1]), :2]  # M'x2

                
                cost_positions = final_positions

                # Now we can compute the cost function
                # First, we compute a euclidian distance cost
                goal_diff = cost_positions - self.cur_goal[:2]  # M'x2
                goal_distance = np.linalg.norm(goal_diff, axis=1)  # Measures similarity to the goal in meters

                # Then we compute an angle similarity cost
                theta_cosine_similarity = np.cos(valid_trajectories[-1, :, 2] - self.cur_goal[2])  # Measures similarity to the goal in a unitless way
                # We want a angle cost that varies from 0 to 1. It is 0 when the angle is exactly correct and 1 when the angle is 180 degrees off
                angle_distance = 1 - (theta_cosine_similarity + 1) / 2

                # And finally, we compute a cost for being far from the line extending from the goal given the goal angle
                # The first step is to compute a unit vector in the direction of the goal angle
                goal_angle_unit_vector = np.array([np.cos(self.cur_goal[2]), np.sin(self.cur_goal[2])])  # 2
                # We can then compute the orthogonal distance as sqrt(||v||^2 - (v . u)^2) where u is the unit vector in the direction of the goal angle and v is the vector from the goal to the final position
                # Note that we already have ||v|| from the goal distance computation
                goal_diff_orthogonal_distance = np.sqrt(goal_distance ** 2 - np.sum(goal_diff * goal_angle_unit_vector, axis=1) ** 2)  # Measures similarity to the goal in meters

                alpha = 1.0  # Weighting for the distance to the Goal. Unitless
                beta = 0.0  # Weighting for the angle distance. Expressed in meters. Can be understood as the corresponding distance for being completely off angle
                cappa = 1.0  # Weighting for the distance to the line. Unitless.

                # An even better approach would be to compute the cost function at each timestep and then to the min over those, but I can't be bothered to figure out
                # how to do that right now. This also allows us to make the cost function more complex without sacrificing efficiency.

                # The cost is then the sum of the distance to the goal and the angle distance
                final_cost = alpha*goal_distance + beta*angle_distance + cappa*goal_diff_orthogonal_distance
                final_cost -= TRANS_GOAL_TOL - 0.05
                final_cost[goal_distance < 0] = 0
                if np.any(final_cost == 0):
                    print(final_cost.shape)
                    zero_cost_indices = np.argwhere(final_cost == 0).flatten()
                    print(f"Zero cost indices: {zero_cost_indices}")
                    zero_cost_controls = [self.all_opts[valid_opts[i]] for i in zero_cost_indices]
                    print(f"Zero cost controls: {zero_cost_controls}")
                    # Add a small penalty for rotatinal velocity
                    final_cost += np.abs(self.all_opts[valid_opts, 1])
                    print(f"Final cost after adding rotational velocity: {final_cost}")

                
                best_opt = final_cost.argmin()  # M'
                
                self.control = self.all_opts[valid_opts][best_opt]  # M -> M' -> 2
                self.local_path_pub.publish(utils.se2_pose_list_to_path(valid_trajectories[:, best_opt], 'map'))
                print(f"Best option: {best_opt}, Final cost: {final_cost[best_opt]}. Control: {self.control}")

                if False:
                    
                    # Now we visualize all these trajectories in the global frame using matplotlib
                    fig, ax = plt.subplots()
                    # Plot each trajectory as a line
                    for opt_ind in range(valid_trajectories.shape[1]):
                        # plot the best option in red
                        if opt_ind == best_opt:
                            ax.plot(valid_trajectories[:, opt_ind, 0], valid_trajectories[:, opt_ind, 1], 'r')
                        else:
                            ax.plot(valid_trajectories[:, opt_ind, 0], valid_trajectories[:, opt_ind, 1], 'b')
                    # Plot the final positions as red dots
                    ax.plot(final_positions[:, 0], final_positions[:, 1], 'ro')
                    # Show the plot
                    plt.show()
                    return

            # send command to robot
            self.cmd_pub.publish(utils.unicyle_vel_to_twist(self.control))

            # uncomment out for debugging if necessary
            # print("Selected control: {control}, Loop time: {time}, Max time: {max_time}".format(
            #     control=control, time=(rospy.Time.now() - tic).to_sec(), max_time=1/CONTROL_RATE))

            self.rate.sleep()
        print("Shut Down")

    def update_pose(self):
        # Update numpy poses with current pose using the tf_buffer
        self.map_baselink_tf = self.tf_buffer.lookup_transform('map', 'base_link', rospy.Time(0)).transform
        # self.map_baselink_tf = self.tf_buffer.lookup_transform('map', 'base_footprint', rospy.Time(0)).transform
        self.pose_in_map_np[:] = [self.map_baselink_tf.translation.x, self.map_baselink_tf.translation.y,
                                  utils.euler_from_ros_quat(self.map_baselink_tf.rotation)[2]]
        self.pos_in_map_pix = (self.map_origin[:2] + self.pose_in_map_np[:2]) / self.map_resolution  # TODO: I think this is incorrect
        # Pose is now (x, y) with shape (2,)
        self.collision_marker.header.stamp = rospy.Time.now()
        self.collision_marker.pose = utils.pose_from_se2_pose(self.pose_in_map_np)
        self.collision_marker_pub.publish(self.collision_marker)

    def check_and_update_goal(self):
        # iterate the goal if necessary
        dist_from_goal = np.linalg.norm(self.pose_in_map_np[:2] - self.cur_goal[:2])
        abs_angle_diff = np.abs(self.pose_in_map_np[2] - self.cur_goal[2])
        rot_dist_from_goal = min(np.pi * 2 - abs_angle_diff, abs_angle_diff)
        if dist_from_goal < TRANS_GOAL_TOL and rot_dist_from_goal < ROT_GOAL_TOL:
            rospy.loginfo("Goal {goal} at {pose} complete.".format(
                    goal=self.cur_path_index, pose=self.cur_goal))
            if self.cur_path_index == len(self.path_tuples) - 1:
                rospy.loginfo("Full path complete in {time}s! Path Follower node shutting down.".format(
                    time=(rospy.Time.now() - self.path_follow_start_time).to_sec()))
                rospy.signal_shutdown("Full path complete! Path Follower node shutting down.")
            else:
                self.cur_path_index += 1
                self.cur_goal = np.array(self.path_tuples[self.cur_path_index])
        else:
            # rospy.logdebug("Goal {goal} at {pose}, trans error: {t_err}, rot error: {r_err}.".format(
            #     goal=self.cur_path_index, pose=self.cur_goal, t_err=dist_from_goal, r_err=rot_dist_from_goal
            # ))
            pass

    def stop_robot_on_shutdown(self):
        self.cmd_pub.publish(Twist())
        rospy.loginfo("Published zero vel on shutdown.")


if __name__ == '__main__':
    try:
        rospy.init_node('path_follower', log_level=rospy.DEBUG)
        pf = PathFollower()
    except rospy.ROSInterruptException:
        pass