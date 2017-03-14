#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist
import tf
from std_msgs.msg import Float32MultiArray
import numpy as np

class Controller:

    def __init__(self):
        rospy.init_node('turtlebot_controller', anonymous=True)
        # rospy.Subscriber('/gazebo/model_states', ModelStates, self.callback)
        self.trans_listener = tf.TransformListener()
        rospy.Subscriber('/turtlebot_control/position_goal', Float32MultiArray, self.callback_desired)
        self.pub = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.xg = None



    def callback_desired(self, data):
        self.xg = data.data

    def get_ctrl_output(self):

        try:
          (self.position, self.orientation) = self.trans_listener.lookupTransform("/map", "/base_footprint", rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
          self.position = (0, 0, 0)
          self.orientation = (0, 0, 0, 1)

        if self.xg is not None:
	        self.x = self.position[0]
	        self.y = self.position[1]  
	        self.theta = tf.transformations.euler_from_quaternion(self.orientation)[2]  

	        # use self.x self.y and self.theta to compute the right control input here
	        # xg = np.array([1.0, 1.0])
	        # th_g = 0.0
	        x = np.array([self.x, self.y])
	        rho = np.linalg.norm(x - self.xg[:2])

	        alpha = np.arctan2(self.xg[1] - self.y, self.xg[0] - self.x) - self.theta
	        delta = np.arctan2(self.xg[1] - self.y, self.xg[0] - self.x) - self.xg[2]

	        if alpha < np.pi or alpha > np.pi:
	            alpha = ( (alpha + np.pi) % (2 * np.pi)) - np.pi
	        if delta < np.pi or delta > np.pi:
	            delta = ( (delta + np.pi) % (2 * np.pi)) - np.pi

	        k1 = 0.5
	        k2 = 0.5
	        k3 = 0.5

	        if rho > .001:
	            cmd_x_dot = k1 * rho * np.cos(alpha)
	            cmd_theta_dot = k2 * alpha + k1 * (np.sinc(alpha/np.pi) * np.cos(alpha)) * (alpha + k3 * delta)
	        else:
	            cmd_x_dot = 0.
	            cmd_theta_dot = k2 * (self.xg[2] - self.theta)


	        # Apply saturation limits
	        cmd_x_dot = np.sign(cmd_x_dot)*min(0.5, np.abs(cmd_x_dot))
	        cmd_theta_dot = np.sign(cmd_theta_dot)*min(1, np.abs(cmd_theta_dot))

	        cmd = Twist()
	        cmd.linear.x = cmd_x_dot
	        cmd.angular.z = cmd_theta_dot
	        return cmd

	    cmd = Twist()
        cmd.linear.x = 0
        cmd.angular.z = 0
        return cmd

    def run(self):
        rate = rospy.Rate(10) # 10 Hz
        while not rospy.is_shutdown():
            ctrl_output = self.get_ctrl_output()
            self.pub.publish(ctrl_output)
            rate.sleep()

if __name__ == '__main__':
    ctrl = Controller()
    ctrl.run()



pass