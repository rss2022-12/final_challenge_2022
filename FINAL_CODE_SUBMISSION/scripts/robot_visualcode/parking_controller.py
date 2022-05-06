#!/usr/bin/env python

import rospy
import numpy as np

from visual_servoing.msg import ConeLocation, ParkingError
from ackermann_msgs.msg import AckermannDriveStamped

class ParkingController():
    """
    A controller for parking in front of a cone.
    Listens for a relative cone location and publishes control commands.
    Can be used in the simulator and on the real robot.
    """
    def __init__(self):
        rospy.Subscriber("/relative_cone", ConeLocation,
            self.relative_cone_callback)

        DRIVE_TOPIC = rospy.get_param("visual_servoing/drive_topic","/vesc/ackermann_cmd_mux/input/navigation") # set in launch file; different for simulator vs racecar
        self.drive_pub = rospy.Publisher(DRIVE_TOPIC,
            AckermannDriveStamped, queue_size=10)
        self.error_pub = rospy.Publisher("/parking_error",
            ParkingError, queue_size=10)
	self.DESIRED_SPEED = 0.5

	self.parking_distance = 0.0 # meters; try playing with this number!
        # self.parking_distance = 0.0
	self.relative_x = 0
        self.relative_y = 0

        # My additions
        self.close_to_cone = False

    def relative_cone_callback(self, msg):
        self.relative_x = msg.x_pos
        self.relative_y = msg.y_pos
        drive_cmd = AckermannDriveStamped()

        #################################

        # YOUR CODE HERE
        # Use relative position and your control law to set drive_cmd

        K_p_steer = 1.0

        absolute_distance = np.sqrt(self.relative_x**2.0 + self.relative_y**2.0)
        print("Distance:", absolute_distance)

        # Set drive speed based on distance
        if absolute_distance - self.parking_distance > 0.1:
            drive_cmd.drive.speed = self.DESIRED_SPEED
        elif (absolute_distance - self.parking_distance < -0.1):
            drive_cmd.drive.speed = -self.DESIRED_SPEED
        else:
            self.close_to_cone = True

        # Override speed if steering correction is necessary
        if abs(self.relative_y) > 0.3 and self.close_to_cone:
            drive_cmd.drive.speed = -self.DESIRED_SPEED
            print("Making angle correction")
        else:
            self.close_to_cone = False

        # Set steering angle based on relative cone position
        if drive_cmd.drive.speed < 0:
            steer_error = -self.relative_y
        else:
            steer_error = self.relative_y
        drive_cmd.drive.steering_angle = K_p_steer*steer_error

        #################################

        self.drive_pub.publish(drive_cmd)
        self.error_publisher()

    def error_publisher(self):
        """
        Publish the error between the car and the cone. We will view this
        with rqt_plot to plot the success of the controller
        """
        error_msg = ParkingError()

        #################################

        # YOUR CODE HERE
        # Populate error_msg with relative_x, relative_y, sqrt(x^2+y^2)

        error_msg.x_error = self.relative_x
        error_msg.y_error = self.relative_y
        error_msg.distance_error = np.sqrt(self.relative_x**2.0 + self.relative_y**2.0)

        #################################
        
        self.error_pub.publish(error_msg)

if __name__ == '__main__':
    try:
        rospy.init_node('ParkingController', anonymous=True)
        ParkingController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
