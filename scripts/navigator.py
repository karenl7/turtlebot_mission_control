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
import pdb

class Navigator:

    def __init__(self):
        rospy.init_node('navigator', anonymous=True)

        self.plan_resolution = .15
        self.plan_horizon = 15
        self.replan_threshold = self.plan_resolution*0.3

        self.map_width = 0
        self.map_height = 0
        self.map_resolution = 0
        self.map_origin = [0,0]
        self.map_probs = []
        self.occupancy = None
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_path = None
        self.has_robot_location = False
        self.tries = 0

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
                                                  int(self.plan_resolution / self.map_resolution*2.3),
                                                  self.map_probs)
        if self.has_robot_location and self.nav_sp:
            state_min = (-int(round(self.plan_horizon)), -int(round(self.plan_horizon)))
            state_max = (int(round(self.plan_horizon)), int(round(self.plan_horizon)))
            x_init = (round(self.x/self.plan_resolution)*self.plan_resolution, round(self.y/self.plan_resolution)*self.plan_resolution)
            x_goal = (round(self.nav_sp[0]/self.plan_resolution)*self.plan_resolution, round(self.nav_sp[1]/self.plan_resolution)*self.plan_resolution)
            astar = AStar(state_min,state_max,x_init,x_goal,self.occupancy,self.plan_resolution)

            if self.last_path and astar.check_path(self.last_path):
                rospy.logwarn("Last computed path is still valid")
                self.send_controller_goal()
            else:
                self.send_pose_sp()   # every time the map changes, we need to update our astar path. (also updates the position goal)

    def nav_sp_callback(self,msg):
        new_nav_sp = (msg.data[0],msg.data[1],msg.data[2])
        if not self.nav_sp:
            self.nav_sp = (msg.data[0],msg.data[1],msg.data[2])
            self.send_pose_sp()
            rospy.logwarn('Initializing target (should only happen at bootup)')
        elif ((new_nav_sp[2]-self.nav_sp[2])**2 + (new_nav_sp[1]-self.nav_sp[1])**2 + (new_nav_sp[0]-self.nav_sp[0])**2)>self.replan_threshold**2:
            self.nav_sp = (msg.data[0],msg.data[1],msg.data[2])
            rospy.logwarn('New target received, replanning')
            self.send_pose_sp()
        else:
            rospy.logwarn('Old target is close enough, will not replan')

    def send_controller_goal(self):
        path_to_robot_dist = np.array(map(lambda x: (x[1]-self.y)**2 + (x[0]-self.x)**2, self.last_path))
        track_lookahead = 4
        closest_pt = np.argmin(path_to_robot_dist)
        track_astar_step_no = min(closest_pt + track_lookahead, len(self.last_path) - 1) # we tell the controller to track the track_astar_step_no-th point in the A* path

        if len(self.last_path) > track_astar_step_no:
            # a naive path follower we could use
            # final_orientation_ctrl=np.arctan2(astar.path[track_astar_step_no][1]-astar.path[track_astar_step_no-1][1],astar.path[track_astar_step_no][1]-astar.path[track_astar_step_no-1][1])
            final_orientation_ctrl = np.arctan2(self.last_path[track_astar_step_no][1] - self.last_path[closest_pt][1],self.last_path[track_astar_step_no][1] - self.last_path[closest_pt][1])
            self.pose_sp = (self.last_path[track_astar_step_no][0], self.last_path[track_astar_step_no][1], final_orientation_ctrl)
            msg = Float32MultiArray()
            msg.data = self.pose_sp
            self.pose_sp_pub.publish(msg)
            # astar.plot_path()
        else:
            # when astar gives a single point,  position goal is tag position
            msg = Float32MultiArray()
            msg.data = self.nav_sp
            self.pose_sp_pub.publish(msg)

        # needed for rviz
        path_msg = Path()   # path message for rviz
        path_msg.header.frame_id = 'map'   # path message for rviz
        for state in self.last_path:    # for the astar solution found
            pose_st = PoseStamped()  # path message for rviz
            pose_st.pose.position.x = state[0]
            pose_st.pose.position.y = state[1]
            pose_st.header.frame_id = 'map'
            path_msg.poses.append(pose_st)
        self.nav_path_pub.publish(path_msg)

    def update_location(self):
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
    def send_pose_sp(self):


        if self.occupancy and self.has_robot_location and self.nav_sp:
            state_min = (-int(round(self.plan_horizon)), -int(round(self.plan_horizon)))
            state_max = (int(round(self.plan_horizon)), int(round(self.plan_horizon)))
            x_init = (round(self.x/self.plan_resolution)*self.plan_resolution, round(self.y/self.plan_resolution)*self.plan_resolution)
            x_goal = (round(self.nav_sp[0]/self.plan_resolution)*self.plan_resolution, round(self.nav_sp[1]/self.plan_resolution)*self.plan_resolution)
            astar = AStar(state_min,state_max,x_init,x_goal,self.occupancy,self.plan_resolution)

            rospy.logwarn("Computing navigation plan")
            if astar.solve():
                self.tries = 0
                self.astar_status.publish(True)
                rospy.logwarn("A* solved")
                self.last_path = astar.path
                self.send_controller_goal()

            else:
                rospy.logwarn("Could not find path")




                self.tries += 1
                msg = Float32MultiArray()
                if self.tries % 2 == 1:
                    rospy.logwarn("Could not find path: spin a bit to try free itself")
                    msg.data = (self.x, self.y, self.theta + np.pi/2)
                    self.pose_sp_pub.publish(msg)
                elif astar.is_free((self.x + self.plan_resolution*np.cos(self.theta), self.y + self.plan_resolution*np.sin(self.theta))):
                    rospy.logwarn("Could not find path: after spinning a bit, move forward")
                    msg.data = (self.x + self.plan_resolution*np.cos(self.theta), self.y + self.plan_resolution*np.sin(self.theta), self.theta)
                    self.pose_sp_pub.publish(msg)

                self.astar_status.publish(False)
                
                self.last_path = None

    def run(self):
        rate = rospy.Rate(10) # 10 Hz
        while not rospy.is_shutdown():
            self.update_location()
            #if (np.linalg.norm(np.array([self.x, self.y]) - np.array([self.pose_sp[0], self.pose_sp[1]])) < self.plan_resolution*0.7) :
      	#	    self.send_pose_sp()
                #rospy.logwarn("Arrived at pose_sp recomputing path")
            rate.sleep()



#  to do: publish waypoint_done

if __name__ == '__main__':
    nav = Navigator()
    nav.run()

pass

