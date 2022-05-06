#!/usr/bin/env python

import rospy
import numpy as np
import time
from cv_bridge import CvBridge, CvBridgeError

from visual_servoing.msg import ConeLocation, ConeLocationPixel, ParkingError
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image

class ParkingController():
    """
    A controller for parking in front of a cone.
    Listens for a relative cone location and publishes control commands.
    Can be used in the simulator and on the real robot.
    """
    def __init__(self):
        rospy.Subscriber("/relative_cone", ConeLocation,
            self.Pursuiter)

        DRIVE_TOPIC = rospy.get_param("visual_servoing/drive_topic","/vesc/ackermann_cmd_mux/input/navigation") # set in launch file; different for simulator vs racecar
        self.drive_pub = rospy.Publisher(DRIVE_TOPIC,
            AckermannDriveStamped, queue_size=10)
        self.error_pub = rospy.Publisher("/parking_error",
            ParkingError, queue_size=10)
	self.speed = 0.55
        self.max_speed = 0.6

	self.parking_distance = 0.0 # meters; try playing with this number!
        # self.parking_distance = 0.0
	self.relative_x = 0
        self.relative_y = 0

        # My additions
        self.close_to_cone = False
        self.wheelbase_length = .3

        # Stop sign vars
        self.stop_sign_detected = False
        self.time_detected = None
        self.prev_stop_in_range = False
        self.stop_in_range = True
        self.stop_timer = False
        self.ignore_timer = False

        self.MIN_DEPTH = 0.5 # based on experimental data
        self.MAX_DEPTH = 0.75 # based on experimental data
        self.depths = None
	self.depths_received = False

        self.prev_angle=0
        self.prev_time=time.clock()
        # Stop sign subs/pubs
        rospy.Subscriber("/stop_sign_bbox", Float32MultiArray, self.stop_sign_callback)
        self.depth_sub = rospy.Subscriber("zed/zed_node/depth/depth_registered", Image, self.depth_callback)
	self.pixel_sub = rospy.Subscriber("relative_cone_px", ConeLocationPixel, self.pixel_cb) 
	self.bad_pixel = False
	self.last_turns = np.zeros(40)
        # self.lost_counter = 0

    def p_controller(self, msg):
	x, y = msg.x_pos, msg.y_pos

	ack=AckermannDriveStamped()
	ack.drive.speed = self.max_speed
	ack.drive.steering_angle = 1.5*y

	self.drive_pub.publish(ack)
    
    def pixel_cb(self, msg):
	if msg.u == 0.0 and msg.v == 0.0:
	    # print("BACKTRACK")
	    self.bad_pixel = True
	    ack = AckermannDriveStamped()
	    ack.drive.speed = -0.5 * self.max_speed
	    ack.drive.steering_angle = -1.0 * self.last_turns[len(self.last_turns)-1] # -1.0 * self.last_turns[np.argmax(np.abs(self.last_turns))]
	    self.drive_pub.publish(ack)
	    # rospy.sleep(0.1)
	else:
	    self.bad_pixel = False
	# print("BAD PIXEL:", self.bad_pixel)

    def Pursuiter(self,msg):
	# print("ENTERING PERSUITER")
        ack=AckermannDriveStamped()

        if self.stop_sign_detected:
	    ack = AckermannDriveStamped()
            ack.drive.speed = 0.0
	    self.drive_pub.publish(ack)
            print("Stoped at sign")
        elif not self.bad_pixel:
	    ack = AckermannDriveStamped()
            # print("line seen")
            rot_point = (msg.x_pos, msg.y_pos) #  - 0.127)
	    # if msg.y_pos > 0:
		# print("TURN LEFT")
	    # elif msg.y_pos < 0:
		# print("TURN RIGHT")
            L1 = np.linalg.norm(rot_point)
                
            alpha= np.arctan2(rot_point[1],rot_point[0])

            steer_angle= np.arctan2((2*self.wheelbase_length * np.sin(alpha)),L1)

                #new_vel=3.06*(-steer_angle+1.4)*(steer_angle+1.4) # the 3.06 is the speed constant (assuming 6 m/s max speed), and the 1.4 is max turning angle (assuming 1.4)
            abs_steer=min(abs(steer_angle),0.5)

                #print("angle is : ",steer_angle)
                #print("goal is :" , goal)
	    self.speed=self.max_speed
	    # print("STEER ANGLE:", steer_angle)
	    if abs_steer > 0.2:
            	#self.speed= (1-1.25*abs_steer)*self.max_speed
		self.speed=(1-0.8*abs_steer)*self.max_speed
	    else:
		steer_angle *= 0.2

            d_error=0.07*(steer_angle-self.prev_angle)
            # print(d_error)
            # print("time is",time.clock())
            #steer_angle-=d_error
            ack.drive.steering_angle=steer_angle
            ack.drive.speed=self.speed #new_vel
            self.prev_angle=steer_angle
	    self.last_turns[:self.last_turns.size-1] = self.last_turns[1:]
	    self.last_turns[self.last_turns.size-1] = steer_angle
            self.prev_time=time.clock()
	    self.drive_pub.publish(ack)

    def time_to_secs(self, time):
        return (time.nsecs / 10.0**9.0)

    # Receive image depths
    def depth_callback(self, msg):
        bridge=CvBridge()  
        img = np.asarray(bridge.imgmsg_to_cv2(msg, msg.encoding))
        self.depths = np.reshape(np.array(img), (msg.height, msg.width))
	self.depths_received = True

    # State machine: Handles stopping at stop signs
    def stop_sign_callback(self, msg):
        if self.depths_received and msg.data != (0.0, 0.0, 0.0, 0.0) and msg.data != ():
	    print(msg.data)
            x_min, y_min, x_max, y_max = msg.data
	    x_min, y_min, x_max, y_max = int(x_min), int(y_min), int(x_max), int(y_max)
            row_min, row_max = y_min + (y_max - y_min)//3, y_min + 2*(y_max - y_min)//3
            col_min, col_max = x_min + (x_max - x_min)//3, x_min + 2*(x_max - x_min)//3
	   
	    #print(self.depths[row_min:row_max, col_min:col_max])
            sign_depths = self.depths[row_min:row_max, col_min:col_max]
	    sign_depth = np.mean(sign_depths[~np.isnan(sign_depths)])
	    print("SIGN DEPTH:", sign_depth)
            # return
            # self.stop_in_range = self.MIN_DEPTH <= sign_depth <= self.MAX_DEPTH
	    self.stop_in_range = True
            # If we're stopped at a stop sign, stay for 0.5s
            if self.stop_timer:
                if time.clock() - self.time_detected < 0.20:
                    self.stop_sign_detected = True
                else:
                    self.stop_sign_detected = False
                    self.stop_timer = False
                    self.ignore_timer = True
            # If we're finished stopping, ignore stop signs for 1s
            elif self.ignore_timer:
                if time.clock() - self.time_detected < 1.0:
		    print("IGNORING STOPS")
                    self.stop_sign_detected = False
                else:
                    self.ignore_timer = False
                    self.prev_stop_in_range = False
                    self.stop_in_range = False
            # If we just detected a stop sign, start the timer and stop
            elif self.stop_in_range and not self.prev_stop_in_range:
                self.time_detected = time.clock()
                self.stop_sign_detected = True
                self.stop_timer = True


            self.prev_stop_in_range = self.stop_in_range
	else:
	    self.stop_sign_detected = False
	    self.stop_timer = False
            self.ignore_timer = False
	    self.stop_in_range = False
 	    self.prev_stop_in_range = False
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
