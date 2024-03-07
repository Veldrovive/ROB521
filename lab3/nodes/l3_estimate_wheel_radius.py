#!/usr/bin/env python3

import rospy
import numpy as np
import threading
from turtlebot3_msgs.msg import SensorState
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist

INT32_MAX = 2**31
DRIVEN_DISTANCE = 0.75 #in meters
TICKS_PER_ROTATION = 4096

class wheelRadiusEstimator():
    def __init__(self):
        rospy.init_node('encoder_data', anonymous=True) # Initialize node

        #Subscriber bank
        rospy.Subscriber("cmd_vel", Twist, self.startStopCallback)
        rospy.Subscriber("sensor_state", SensorState, self.sensorCallback) #Subscribe to the sensor state msg

        #Publisher bank
        self.reset_pub = rospy.Publisher('reset', Empty, queue_size=1)

        #Initialize variables
        self.left_encoder_prev = None
        self.right_encoder_prev = None
        self.del_left_encoder = 0
        self.del_right_encoder = 0
        self.isMoving = False #Moving or not moving
        self.lock = threading.Lock()

        #Reset the robot 
        reset_msg = Empty()
        self.reset_pub.publish(reset_msg)
        print('Ready to start wheel radius calibration!')
        return

    def safeDelPhi(self, a, b):
        #Need to check if the encoder storage variable has overflowed
        diff = np.int64(b) - np.int64(a)
        if diff < -np.int64(INT32_MAX): #Overflowed
            delPhi = (INT32_MAX - 1 - a) + (INT32_MAX + b) + 1
        elif diff > np.int64(INT32_MAX) - 1: #Underflowed
            delPhi = (INT32_MAX + a) + (INT32_MAX - 1 - b) + 1
        else:
            delPhi = b - a  
        return delPhi

    def sensorCallback(self, msg):
        #Retrieve the encoder data form the sensor state msg
        print(msg.left_encoder, msg.right_encoder)
        self.lock.acquire()
        if self.left_encoder_prev is None or self.left_encoder_prev is None: 
            self.left_encoder_prev = msg.left_encoder #int32
            self.right_encoder_prev = msg.right_encoder #int32
        else:
            #Calculate and integrate the change in encoder value
            left_diff = self.safeDelPhi(self.left_encoder_prev, msg.left_encoder)
            right_diff = self.safeDelPhi(self.right_encoder_prev, msg.right_encoder)
            self.del_left_encoder += left_diff
            self.del_right_encoder += right_diff

            #Store the new encoder values
            self.left_encoder_prev = msg.left_encoder #int32
            self.right_encoder_prev = msg.right_encoder #int32
        self.lock.release()
        print(f'Left: {self.del_left_encoder}, Right: {self.del_right_encoder}')
        print(f'Difference: {left_diff}, {right_diff}')
        return

    def startStopCallback(self, msg):
        input_velocity_mag = np.linalg.norm(np.array([msg.linear.x, msg.linear.y, msg.linear.z]))
        if self.isMoving is False and np.absolute(input_velocity_mag) > 0:
            self.isMoving = True #Set state to moving
            print('Starting Calibration Procedure')

        elif self.isMoving is True and np.isclose(input_velocity_mag, 0):
            self.isMoving = False #Set the state to stopped

            # Calculate the radius of the wheel based on encoder measurements
            num_left_rotations = self.del_left_encoder / TICKS_PER_ROTATION
            num_right_rotations = self.del_right_encoder / TICKS_PER_ROTATION

            estimated_circumference = (2 * DRIVEN_DISTANCE) / (num_left_rotations + num_right_rotations)
            estimated_radius = estimated_circumference / (2 * np.pi)

            radius = estimated_radius
            print('Calibrated Radius: {} m'.format(radius))

            print(f'Encoder Data: Left: {self.del_left_encoder}, Right: {self.del_right_encoder}')
            print(f'Num Rotations: Left: {num_left_rotations}, Right: {num_right_rotations}')
            print(f'Distance Driven: {DRIVEN_DISTANCE} m')
            print(f'Estimated Circumference: {estimated_circumference} m')

            #Reset the robot and calibration routine
            self.lock.acquire()
            self.left_encoder_prev = None
            self.right_encoder_prev = None
            self.del_left_encoder = 0
            self.del_right_encoder = 0
            self.lock.release()
            reset_msg = Empty()
            self.reset_pub.publish(reset_msg)
            print('Resetted the robot to calibrate again!')

        return


if __name__ == '__main__':
    Estimator = wheelRadiusEstimator() #create instance
    rospy.spin()
