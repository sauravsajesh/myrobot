#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist

class ImuSlopeMonitor:
    def __init__(self):
        rospy.init_node('imu_slope_monitor')
        self.slope_threshold = rospy.get_param('~slope_threshold', 0.5)  # in radians, ~28.6 degrees
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('imu/data', Imu, self.imu_callback)
        self.stop_robot()

    def imu_callback(self, data):
        # Calculate the slope
        orientation = data.orientation
        # Extract the pitch from quaternion (simplified calculation)
        pitch = abs(2 * (orientation.x * orientation.z - orientation.w * orientation.y))

        rospy.loginfo(f'Pitch: {pitch:.4f} radians')

        if pitch > self.slope_threshold:
            rospy.logwarn('Slope threshold exceeded! Stopping the robot.')
            self.stop_robot()

    def stop_robot(self):
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)

if __name__ == '__main__':
    try:
        ImuSlopeMonitor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
