#!/usr/bin/env python

import rospy
from nav_msgs.msg import MapMetaData, OccupancyGrid
import numpy as np
import matplotlib.pyplot as plt
import tf
from std_msgs.msg import Float32MultiArray, Bool
from astar import AStar, DetOccupancyGrid2D, StochOccupancyGrid2D
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class Navigator:

    def __init__(self):
        rospy.init_node('navigator', anonymous=True)

        self.plan_resolution = 0.1
        self.plan_horizon = 15

        self.map_width = 0
        self.map_height = 0
        self.map_resolution = 0
        self.map_origin = [0,0]
        self.map_probs = []
        self.occupancy = None
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.nav_sp = None
        self.pose_sp = (0.0,0.0,0.0)

        self.trans_listener = tf.TransformListener()

        rospy.Subscriber("map", OccupancyGrid, self.map_callback)
        rospy.Subscriber("map_metadata", MapMetaData, self.map_md_callback)
        rospy.Subscriber("/turtlebot_controller/nav_goal", Float32MultiArray, self.nav_sp_callback)

        self.pose_sp_pub = rospy.Publisher('/turtlebot_controller/position_goal', Float32MultiArray, queue_size=10)
        self.nav_path_pub = rospy.Publisher('/turtlebot_controller/path_goal', Path, queue_size=10)
        # publish status of astar
        self.astar_status = rospy.Publisher('/turtlebot_controller/astar_status', Bool, queue_size=10)
        # waypoint done
        self.waypoint_done = rospy.Publisher('/turtlebot_control/waypoint_done', Bool, queue_size=10)

    def map_md_callback(self,msg):
        self.map_width = msg.width
        self.map_height = msg.height
        self.map_resolution = msg.resolution
        self.map_origin = (msg.origin.position.x,msg.origin.position.y)

    def map_callback(self,msg):
        self.map_probs = msg.data
        if self.map_width>0 and self.map_height>0 and len(self.map_probs)>0:
            self.occupancy = StochOccupancyGrid2D(self.map_resolution,
                                                  self.map_width,
                                                  self.map_height,
                                                  self.map_origin[0],
                                                  self.map_origin[1],
                                                  int(self.plan_resolution / self.map_resolution) * 2,
                                                  self.map_probs)
        self.send_pose_sp()   # every time the map changes, we need to update our astar path. (also updates the position goal)

    def nav_sp_callback(self,msg):
        self.nav_sp = (msg.data[0],msg.data[1],msg.data[2])
        self.send_pose_sp()

    def send_pose_sp(self):
        try:
            (robot_translation,robot_rotation) = self.trans_listener.lookupTransform("/map", "/base_footprint", rospy.Time(0))
            self.has_robot_location = True
            self.x = robot_translation[0]
            self.y = robot_translation[1]  
            self.theta = tf.transformations.euler_from_quaternion(robot_rotation)[2]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            robot_translation = (0,0,0)
            robot_rotation = (0,0,0,1)
            self.has_robot_location = False

        if self.occupancy and self.has_robot_location and self.nav_sp:
            state_min = (-int(round(self.plan_horizon)), -int(round(self.plan_horizon)))
            state_max = (int(round(self.plan_horizon)), int(round(self.plan_horizon)))
            x_init = (int(round(robot_translation[0])), int(round(robot_translation[1])))
            x_goal = (int(round(self.nav_sp[0])), int(round(self.nav_sp[1])))
            astar = AStar(state_min,state_max,x_init,x_goal,self.occupancy,self.plan_resolution)

            rospy.loginfo("Computing navigation plan")
            if astar.solve():
            	self.astar_status.publish(True)
                
                if len(astar.path) > 1:
                # a naive path follower we could use
	                self.pose_sp = (astar.path[1][0],astar.path[1][1],self.nav_sp[2])
	                msg = Float32MultiArray()
	                msg.data = self.pose_sp
	                self.pose_sp_pub.publish(msg)
	                # astar.plot_path()
            	else:
            		# when astar gives a single point,  position goal is tag position
	                msg.data = self.nav_sp
	                self.pose_sp_pub.publish(msg)



                # needed for rviz
                path_msg = Path()   # path message for rviz
                path_msg.header.frame_id = 'map'   # path message for rviz 
                for state in astar.path:    # for the astar solution found
                    pose_st = PoseStamped()  # path message for rviz
                    pose_st.pose.position.x = state[0]
                    pose_st.pose.position.y = state[1]
                    pose_st.header.frame_id = 'map'
                    path_msg.poses.append(pose_st)
                self.nav_path_pub.publish(path_msg)

            else:
            	self.astar_status.publish(False)
                rospy.logwarn("Could not find path")

    def run(self):
    	rate = rospy.Rate(10) # 10 Hz
    	while not rospy.is_shutdown():
            if np.linalg.norm(np.array([self.x, self.y]) - np.array([self.pose_sp[0], self.pose_sp[1]])) < self.plan_resolution*0.5:
      		    self.send_pose_sp()
            '''is_waypoint_done = Bool()
            is_waypoint_done.data = False

            if self.nav_sp:
                if np.linalg.norm(np.array([self.x, self.y, self.theta]) - np.array(self.nav_sp)) < self.plan_resolution*0.1:
                    is_waypoint_done.data = True
            self.waypoint_done.publish(is_waypoint_done)'''
            rate.sleep()



#  to do: publish waypoint_done

if __name__ == '__main__':
    nav = Navigator()
    nav.run()

pass