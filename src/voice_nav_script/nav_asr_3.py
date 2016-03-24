#!/usr/bin/env python

import rospy
import actionlib
import copy
import os

from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String
from visualization_msgs.msg import Marker

class NavTest():
    def __init__(self):
        rospy.init_node('nav_asr', anonymous = True)
        rospy.Subscriber("/recognizer/output", String, self.SpeechUpdateGoal)
        
        rospy.on_shutdown(self.shutdown) # @@@@
        
        # How long in seconds should the robot pause at each location?
        self.rest_time = rospy.get_param("~rest_time", 2)
        
        # Are we running in the fake simulator?
        self.fake_test = rospy.get_param("~fake_test", True)
        
        # Goal state return values
        
        self.locations = dict()
        
        # self.locations['foyer']   = Pose(Point(1.841, -1.735, 0.000),  Quaternion(0.000, 0.000, -0.525, 0.851))
        # self.locations['kitchen'] = Pose(Point(0.257, 1.119, 0.000),   Quaternion(0.000, 0.000, 0.818, 0.576))
        # self.locations['bedroom'] = Pose(Point(-1.488, -0.895, 0.000), Quaternion(0.000, 0.000, 0.924, -0.383))
        # self.locations['origin']  = Pose(Point(-0.000, -0.000, 0.000), Quaternion(0.000, 0.000, 0.000, 1.000))

        self.IsWPListOK = False
        rospy.Subscriber("/WPsOK", String, self.GetWayPoints)

    	# #Rviz Marker
     #    self.init_markers()

        # Publisher to manually control the robot (e.g. to stop it, queue_size=5)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        
        # Subscribe to the move_base action server
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        
        # Wait 60 seconds for the action server to become available
        self.move_base.wait_for_server(rospy.Duration(60))
        rospy.loginfo("Connected to move base server")
        
        while not rospy.is_shutdown():
        	while self.IsWPListOK:
				i = 0
				for marker_pub in self.marker_pub_list:
					marker_pub.publish(self.marker_list[i])
					i = i + 1

        	rospy.sleep(self.rest_time)

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.move_base.cancel_goal()
        rospy.sleep(2)
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)
    
    def SpeechUpdateGoal(self, SpeechGoal):
        rospy.loginfo("input = %s", str(SpeechGoal.data))
        if self.locations.has_key(str(SpeechGoal.data)):

            rospy.loginfo("location = %s", str(SpeechGoal.data))
            self.goal = MoveBaseGoal()
            self.goal.target_pose.pose = self.locations[str(SpeechGoal.data)]
            self.goal.target_pose.header.frame_id = 'map'
            self.goal.target_pose.header.stamp = rospy.Time.now()

            # Let the user know where the robot is going next
            rospy.loginfo("Going to: " + str(SpeechGoal.data))
            
            # Start the robot toward the next location
            self.move_base.send_goal(self.goal)
        else:
            rospy.loginfo("No such waypoint")

    def init_markers(self):
       # Set up our waypoint markers
        marker_scale 		= 0.6
        marker_lifetime 	= 0 # 0 is forever
        marker_ns 			= 'waypoints'
        marker_id 			= 0
        marker_color 		= {'r': 1.0, 'g': 0.0, 'b': 0.0, 'a': 1.0}
        
        # Define a marker publisher list.
        location_list 		= list()
        location_list		= self.locations.keys()
        self.marker_pub_list = list()
        for i in location_list:
            self.marker_pub_list.append(rospy.Publisher('waypoint_' + i, Marker, queue_size=5))

        # Define a marker list.
        self.marker_list = list()
        for location in self.locations:

            marker                  = Marker()
            marker.ns               = marker_ns
            marker.id               = marker_id
            marker.type             = Marker.TEXT_VIEW_FACING
            marker.action           = Marker.ADD
            marker.lifetime         = rospy.Duration(marker_lifetime)
            marker.scale.x          = marker_scale
            marker.scale.y          = marker_scale
            marker.scale.z          = marker_scale

            marker.color.r          = marker_color['r']
            marker.color.g          = marker_color['g']
            marker.color.b          = marker_color['b']
            marker.color.a          = marker_color['a']

            marker.header.frame_id  = 'map'
            marker.header.stamp     = rospy.Time.now()

            # rospy.loginfo(self.locations['foyer'])
            marker.pose             = copy.deepcopy(self.locations[location])
            marker.pose.position.z  = 0.65
            marker.text             = location

            # rospy.loginfo(self.locations['foyer'])
            self.marker_list.append(marker)
    
    # Create the waypoints list from txt
    def GetWayPoints(self,Data):
    	# dir = os.path.dirname(__file__)
    	# filename = dir+'/locationPoint.txt'
    	filename = Data.data
    	rospy.loginfo(str(filename))
    	rospy.loginfo(self.locations)
    	self.locations.clear()
    	rospy.loginfo(self.locations)

    	f = open(filename,'r')

    	for i in f.readlines():
    		j = i.split(",")
    		current_wp_name 	= str(j[0])
    		rospy.loginfo(current_wp_name)    		
    		
    		current_point 		= Point(float(j[1]),float(j[2]),float(j[3]))
    		current_quaternion 	= Quaternion(float(j[4]),float(j[5]),float(j[6]),float(j[7]))
    		
    		self.locations[current_wp_name] = Pose(current_point,current_quaternion)

    	f.close()
    	rospy.loginfo(self.locations)
    	
    	#Rviz Marker
        self.init_markers()

    	self.IsWPListOK = True

def trunc(f, n):
    # Truncates/pads a float f to n decimal places without rounding
    slen = len('%.*f' % (n, f))
    return float(str(f)[:slen])

if __name__ == '__main__':
    try:
        NavTest()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("AMCL navigation test finished.")