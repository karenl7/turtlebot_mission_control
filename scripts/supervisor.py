#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32MultiArray, Float32MultiArray, String, Bool
from geometry_msgs.msg import PoseStamped
import tf
import numpy as np

def pose_to_xyth(pose):
    th = tf.transformations.euler_from_quaternion((pose.orientation.x,
                                                   pose.orientation.y,
                                                   pose.orientation.z,
                                                   pose.orientation.w))[2]
    return [pose.position.x, pose.position.y, th]


class Supervisor:

    def __init__(self):
        rospy.init_node('turtlebot_supervisor', anonymous=True)
        self.trans_listener = tf.TransformListener()
        self.trans_broad = tf.TransformBroadcaster()

        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.rviz_goal_callback)    # rviz "2D Nav Goal"

        # Read mission
        rospy.Subscriber('/mission', Int32MultiArray, self.mission_callback)

        # Broadcast location of tags
        self.loc_broadcast = rospy.Publisher('/turtlebot_controller/nav_goal', Float32MultiArray, queue_size=10)

        # Read in if has reached waypoint
        rospy.Subscriber('/turtlebot_control/waypoint_done', Bool, self.get_done)

        self.waypoint_locations = {}    # dictionary that caches the most updated locations of each mission waypoint
        self.waypoint_offset = PoseStamped()
        self.waypoint_offset.pose.position.z = .4    # waypoint is located 40cm in front of the AprilTag, facing it
        quat = tf.transformations.quaternion_from_euler(0., np.pi/2, np.pi/2)
        self.waypoint_offset.pose.orientation.x = quat[0]
        self.waypoint_offset.pose.orientation.y = quat[1]
        self.waypoint_offset.pose.orientation.z = quat[2]
        self.waypoint_offset.pose.orientation.w = quat[3]

        # Variables for FSM
        self.state = "explore"
        self.mission = [-1]
        self.waypoint_number = 0
        self.waypoint_done = False
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.resolution = 0.1
        self.goal = None

    # Check if done moving to waypoint
    def get_done(self, data):
        self.waypoint_done = data.data

    # Read in mission tags
    def mission_callback(self, data):
        self.mission = data.data

    def rviz_goal_callback(self, msg):
        self.goal = pose_to_xyth(msg.pose)    # example usage of the function pose_to_xyth (defined above)
        # this callback does nothing... yet!

    def update_waypoints(self):
        for tag_number in self.mission:
            try:
                self.waypoint_offset.header.frame_id = "/tag_{0}".format(tag_number)
                self.waypoint_locations[tag_number] = self.trans_listener.transformPose("/map", self.waypoint_offset)
                #rospy.loginfo(self.waypoint_locations[tag_number])
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pass

    def update_position(self):
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

    def check_close(self):
        wp_x, wp_y, wp_th = pose_to_xyth(self.waypoint_locations[self.mission[self.waypoint_number]].pose)
        #wp_th = tf.transformations.euler_from_quaternion(wp_quat)[2]
        return np.linalg.norm(np.array([self.x, self.y, 0.01*self.theta]) - \
                    np.array([wp_x,wp_y,0.01*wp_th])) < self.resolution
        #return np.linalg.norm(np.array([self.x, self.y]) - \
        #            np.array([wp_x,wp_y])) < self.resolution


    def run(self):
        rate = rospy.Rate(1) # 1 Hz, change this to whatever you like
        while not rospy.is_shutdown():
            self.update_waypoints()   # updates location of tags
            self.update_position()

            # explore
            if self.state == "explore":
                rospy.loginfo("Exploring")
                # select points on rviz
                    # self.goal = self.rviz_goal_callback
                # Broadcast next goal state
                if self.goal is not None:
                    data = Float32MultiArray()
                    data.data = self.goal
                    self.loc_broadcast.publish(data)

                # Check if we've seen all tags
                if set(self.waypoint_locations.keys()).issuperset(set(self.mission)):
                    self.state = "mission"
                else:
                    notSeen = set(self.mission) - set(self.waypoint_locations.keys())
                    message = str(len(notSeen)) + "/" + str(len(set(self.mission))) + " waypoints to go."
                    rospy.loginfo(message)



            # go to tag locations
            elif self.state == "mission":

                rospy.loginfo("Collecting relics")
                self.waypoint_done = self.check_close()
                if self.waypoint_done:
                    self.waypoint_number += 1

                if self.waypoint_done and (self.waypoint_number == len(self.mission)):
                    self.state = "home"
                else:
                # Check if arrived at desired waypoint

                    # Broadcast next goal state
                    tag_x, tag_y, tag_th = pose_to_xyth(self.waypoint_locations[self.mission[self.waypoint_number]].pose)
                    data = Float32MultiArray()
                    data.data = np.array([tag_x, tag_y, tag_th])
                    self.loc_broadcast.publish(data)

                # Check if all waypoints done


            # go home
            else:
                # Broadcast state 0 to go home
                tag_x, tag_y, tag_th = pose_to_xyth(self.waypoint_locations[self.mission[0]].pose)
                data = Float32MultiArray()
                data.data = np.array([tag_x, tag_y, tag_th])
                self.loc_broadcast.publish(data)


            # FILL ME IN!

            rate.sleep()

if __name__ == '__main__':
    sup = Supervisor()
    sup.run()


# waypoints == tags
