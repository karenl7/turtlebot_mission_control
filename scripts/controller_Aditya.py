#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String
import tf
import numpy as np
import pdb

class Controller:

    def __init__(self):
        rospy.init_node('turtlebot_controller', anonymous=True)

        # Commented out subscriber for HW3 Q3
        # rospy.Subscriber('/gazebo/model_states', ModelStates, self.callback)
        self.trans_listener = tf.TransformListener()

        self.pub = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size = 10)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        rospy.Subscriber('/turtlebot_controller/position_goal',Float32MultiArray, self.callback_g)
        self.x_g = 0
        self.y_g = 0
        self.theta_g = 0

        rospy.Subscriber('/turtlebot_control/velocity_goal',Float32MultiArray, self.callback_v)
        self.V_g = 0
        self.om_g = 0

        rospy.Subscriber('/turtlebot_control/control_mode',String, self.callback_m)
        self.mode = 'Position'

    def callback_g(self, data):
        self.x_g = data.data[0]
        self.y_g = data.data[1]
        self.theta_g = data.data[2]

    def callback_v(self, data):
        self.V_g = data.data[0]
        self.om_g = data.data[1]

    def callback_m(self, data):
        self.mode = data.data

    '''
    def callback(self, data):
        pose = data.pose[data.name.index("mobile_base")]
        twist = data.twist[data.name.index("mobile_base")]
        self.x = pose.position.x
        self.y = pose.position.y
        quaternion = (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.theta = euler[2]
    '''

    @staticmethod
    def wrapToPiScalar(a):
        b = a
        if a < -np.pi or a > np.pi:
            b = ((a+np.pi) % (2*np.pi)) - np.pi
        return b

    def get_ctrl_output(self):
        # use self.x self.y and self.theta to compute the right control input here
        # cmd_x_dot = 0.0 # forward velocity
        # cmd_theta_dot = 0.0

        try:
            # tf knows where the tag is
            (translation,rotation) = self.trans_listener.lookupTransform("/map", "/base_footprint", rospy.Time(0))

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            # tf doesn't know where the tag is
            translation = (0,0,0)
            rotation = (0,0,0,1)

        euler = tf.transformations.euler_from_quaternion(rotation)
        self.x = translation[0]
        self.y = translation[1]
        self.theta = euler[2]

        if self.mode == 'Position':
            x_g = self.x_g
            y_g = self.y_g
            th_g = self.theta_g

            k1 = 0.5
            k2 = 1
            k3 = 1.5

            rho = np.sqrt((x_g-self.x)**2 + (y_g-self.y)**2)
            alpha = np.arctan2(y_g-self.y,x_g-self.x) - self.theta
            d = np.arctan2(y_g-self.y,x_g-self.x) - th_g

            alpha = Controller.wrapToPiScalar(alpha)
            d = Controller.wrapToPiScalar(d)

            #Define control inputs (V,om) - without saturation constraints
            V = k1 * rho * np.cos(alpha)
            om = k2 * alpha + k1 * np.sinc(2*alpha/np.pi) * (alpha + k3*d)

        elif self.mode == 'Stop':
            V = 0
            om = 0
            
        else:
            V = self.V_g
            om = self.om_g


        # Apply saturation limits
        cmd_x_dot = np.sign(V)*min(0.5, np.abs(V))
        cmd_theta_dot = np.sign(om)*min(1, np.abs(om))

        # end of what you need to modify
        cmd = Twist()
        cmd.linear.x = cmd_x_dot
        cmd.angular.z = cmd_theta_dot
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
