#!/usr/bin/env python3
from __future__ import division, print_function

import numpy as np
import rospy
import tf2_ros
from skimage.draw import line as ray_trace
import rospkg
import matplotlib.pyplot as plt

# msgs
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import LaserScan

from utils import convert_pose_to_tf, convert_tf_to_pose, euler_from_ros_quat, \
     tf_to_tf_mat, tf_mat_to_tf


ALPHA = 1
BETA = 1
MAP_DIM = (4, 4)
CELL_SIZE = .01
NUM_PTS_OBSTACLE = 3
SCAN_DOWNSAMPLE = 1

class OccupancyGripMap:
    def __init__(self):
        # use tf2 buffer to access transforms between existing frames in tf tree
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_br = tf2_ros.TransformBroadcaster()

        # subscribers and publishers
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_cb, queue_size=1)
        self.map_pub = rospy.Publisher('/map', OccupancyGrid, queue_size=1)

        # attributes
        width = int(MAP_DIM[0] / CELL_SIZE); height = int(MAP_DIM[1] / CELL_SIZE)
        self.log_odds = np.zeros((width, height))
        self.np_map = np.ones((width, height), dtype=np.uint8) * -1  # -1 for unknown
        self.map_msg = OccupancyGrid()
        self.map_msg.info = MapMetaData()
        self.map_msg.info.resolution = CELL_SIZE
        self.map_msg.info.width = width
        self.map_msg.info.height = height

        # transforms
        self.base_link_scan_tf = self.tf_buffer.lookup_transform('base_link', 'base_scan', rospy.Time(0),
                                                            rospy.Duration(2.0))
        odom_tf = self.tf_buffer.lookup_transform('odom', 'base_link', rospy.Time(0), rospy.Duration(2.0)).transform

        # set origin to center of map
        rob_to_mid_origin_tf_mat = np.eye(4)
        rob_to_mid_origin_tf_mat[0, 3] = -width / 2 * CELL_SIZE
        rob_to_mid_origin_tf_mat[1, 3] = -height / 2 * CELL_SIZE
        odom_tf_mat = tf_to_tf_mat(odom_tf)
        self.map_msg.info.origin = convert_tf_to_pose(tf_mat_to_tf(odom_tf_mat.dot(rob_to_mid_origin_tf_mat)))

        # map to odom broadcaster
        self.map_odom_timer = rospy.Timer(rospy.Duration(0.1), self.broadcast_map_odom)
        self.map_odom_tf = TransformStamped()
        self.map_odom_tf.header.frame_id = 'map'
        self.map_odom_tf.child_frame_id = 'odom'
        self.map_odom_tf.transform.rotation.w = 1.0

        rospy.spin()
        plt.imshow(100-self.np_map, cmap='gray', vmin=0, vmax=100)
        rospack = rospkg.RosPack()
        path = rospack.get_path("rob521_lab3")
        plt.savefig(path+"/map.png")

    def broadcast_map_odom(self, e):
        self.map_odom_tf.header.stamp = rospy.Time.now()
        self.tf_br.sendTransform(self.map_odom_tf)

    def scan_cb(self, scan_msg):
        # read new laser data and populate map
        # get current odometry robot pose
        try:
            odom_tf = self.tf_buffer.lookup_transform('odom', 'base_scan', rospy.Time(0)).transform
        except tf2_ros.TransformException:
            rospy.logwarn('Pose from odom lookup failed. Using origin as odom.')
            odom_tf = convert_pose_to_tf(self.map_msg.info.origin)

        # get odom in frame of map
        odom_map_tf = tf_mat_to_tf(
            np.linalg.inv(tf_to_tf_mat(convert_pose_to_tf(self.map_msg.info.origin))).dot(tf_to_tf_mat(odom_tf))
        )
        odom_map = np.zeros(3)
        odom_map[0] = odom_map_tf.translation.x
        odom_map[1] = odom_map_tf.translation.y
        odom_map[2] = euler_from_ros_quat(odom_map_tf.rotation)[2]
        # Is this the position of the lidar? We don't have any other calibration information about the lidar position with respect
        # to the robot so I guess I'll make that assumption.
        # That means our x_start and y_start are always just odom_map[0] and odom_map[1] which makes it weird that they're
        # talking about them as if we need to compute them though. Whatever.

        # Loop through each measurement in scan_msg to get the correct angle and
        # x_start and y_start to send to your ray_trace_update function.
        measurements = scan_msg.ranges
        relative_angles = scan_msg.angle_increment * np.arange(len(measurements)) + scan_msg.angle_min
        # We should have that relative_angles[-1] is close to scan_msg.angle_max
        assert np.isclose(relative_angles[-1], scan_msg.angle_max), "The last relative angle is not close to the max angle"
        downsampled_measurements = measurements[::SCAN_DOWNSAMPLE]
        downsampled_relative_angles = relative_angles[::SCAN_DOWNSAMPLE]

        for s_range, s_angle in zip(downsampled_measurements, downsampled_relative_angles):
            # get x and y start
            x_start = odom_map[0] / CELL_SIZE
            y_start = odom_map[1] / CELL_SIZE
            angle = s_angle + odom_map[2]
            self.np_map, self.log_odds = self.ray_trace_update(self.np_map, self.log_odds, x_start, y_start, angle, s_range)

        # publish the message
        self.map_msg.info.map_load_time = rospy.Time.now()
        self.map_msg.data = self.np_map.flatten()
        self.map_pub.publish(self.map_msg)

    def ray_trace_update(self, map, log_odds, x_start, y_start, angle, range_mes):
        """
        A ray tracing grid update as described in the lab document.

        :param map: The numpy map.
        :param log_odds: The map of log odds values.
        :param x_start: The x starting point in the map coordinate frame (i.e. the x 'pixel' that the robot is in).
        :param y_start: The y starting point in the map coordinate frame (i.e. the y 'pixel' that the robot is in).
        :param angle: The ray angle relative to the x axis of the map.
        :param range_mes: The range of the measurement along the ray.
        :return: The numpy map and the log odds updated along a single ray.
        """
        # You should modify the log_odds object and the numpy map based on the outputs from
        # ray_trace and the equations from class. Your numpy map must be an array of int8s with 0 to 100 representing
        # probability of occupancy, and -1 representing unknown.

        # # Step 1: Compute all pixels that lie on the ray
        # # We can just sample at small even intervals and then round to the nearest pixel and take the unique ones
        # sample_density = 3  # n samples per pixel. Since we are in pixel coordinates this is also the number of samples per unit distance
        # x_component = np.cos(angle)
        # y_component = np.sin(angle)
        # sample_distances = np.arange(0, range_mes, 1 / sample_density)
        # x_samples = x_start + sample_distances * x_component
        # y_samples = y_start + sample_distances * y_component

        # # Step 2: Round the samples to the nearest pixel
        # sample_points = np.stack((x_samples, y_samples), axis=-1)
        # sample_points = np.round(sample_points).astype(int)
        # sample_points = np.unique(sample_points, axis=0)

        # Step 1 but not dumb
        final_x = int(round(x_start + range_mes * np.cos(angle)))
        final_y = int(round(y_start + range_mes * np.sin(angle)))
        x_start = int(round(x_start))
        y_start = int(round(y_start))
        y_samples, x_samples = ray_trace(x_start, y_start, final_x, final_y)
        sample_points = np.stack((x_samples, y_samples), axis=-1)

        # Step 3: Generate labels. Empty is 0, occupied is 1
        # All pixels are empty except for the last one which is occupied
        labels = np.zeros(len(sample_points), dtype=int)
        labels[-1] = 1

        # Step 4: Compute the logit domain update for each pixel
        # This is ALPHA for occupied and -BETA for empty
        logit_updates = np.zeros(len(sample_points))
        logit_updates[labels == 1] = ALPHA
        logit_updates[labels == 0] = -BETA

        # Step 5: Use advanced indexing to update the log odds
        indexes = tuple(sample_points.T)
        log_odds[indexes] += logit_updates

        # Step 6: Compute the probability map
        # However, we only update the pixels that were observed to preserve the unknowns
        sample_log_odds = log_odds[indexes]
        sample_probabilities = self.log_odds_to_probability(sample_log_odds)
        map[indexes] = (1 - sample_probabilities) * 100

        return map, log_odds

    def log_odds_to_probability(self, values):
        # print(values)
        return np.exp(values) / (1 + np.exp(values))


if __name__ == '__main__':
    try:
        rospy.init_node('mapping')
        ogm = OccupancyGripMap()
    except rospy.ROSInterruptException:
        pass