#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

class Driver:
    def __init__(self):
        self.rate = rospy.Rate(10)
        self.cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    def drive_line(self, distance, velocity):
        twist = Twist()
        twist.linear.x = velocity
        twist.angular.z = 0
        start_time = rospy.Time.now().to_sec()
        total_time = distance / velocity
        print(f"total_time: {total_time}")
        while rospy.Time.now().to_sec() - start_time < distance / velocity:
            self.cmd_vel.publish(twist)
            self.rate.sleep()
        twist.linear.x = 0
        self.cmd_vel.publish(twist)
        print(f"Finished driving {distance} meters")

    def drive_rotate(self, angle, velocity):
        twist = Twist()
        twist.linear.x = 0
        twist.angular.z = velocity
        start_time = rospy.Time.now().to_sec()
        while rospy.Time.now().to_sec() - start_time < angle / velocity:
            self.cmd_vel.publish(twist)
            self.rate.sleep()
        twist.angular.z = 0
        self.cmd_vel.publish(twist)

    def drive_arc(self, radius, angle, velocity):
        twist = Twist()
        twist.linear.x = velocity
        twist.angular.z = velocity / radius
        start_time = rospy.Time.now().to_sec()
        while rospy.Time.now().to_sec() - start_time < angle / velocity:
            self.cmd_vel.publish(twist)
            self.rate.sleep()
        twist.linear.x = 0
        twist.angular.z = 0
        self.cmd_vel.publish(twist)

if __name__ == '__main__':
    rospy.init_node('drive_cal')
    driver = Driver()
    # driver.drive_line(1, 0.1)
    # driver.drive_rotate(3.14, 0.1)
    driver.drive_arc(0.5, 3.14, 0.1)