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
from visualization_msgs.msg import MarkerArray

class NavTest():
    def __init__(self):
        rospy.init_node('nav_asr', anonymous = True)
        rospy.Subscriber("/recognizer/output", String, self.SpeechUpdateGoal)
        rospy.Subscriber("/WPsOK", String, self.GetWayPoints)

        self.waypoint_list     = dict()
        self.marker_list       = list()
        self.makerArray        = MarkerArray()

        self.makerArray_pub    = rospy.Publisher("Waypoint_Array",MarkerArray,queue_size=5)

        rospy.on_shutdown(self.shutdown) # @@@@
        # How long in seconds should the robot pause at each location?
        self.rest_time = rospy.get_param("~rest_time", 2)
        # Are we running in the fake simulator?
        self.fake_test = rospy.get_param("~fake_test", True)
        # Publisher to manually control the robot (e.g. to stop it, queue_size=5)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        # Subscribe to the move_base action server
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        # Wait 60 seconds for the action server to become available
        self.move_base.wait_for_server(rospy.Duration(60))
        rospy.loginfo("Connected to move base server")

    # Create the waypoints list from txt
    def GetWayPoints(self,Data):
        filename = Data.data
        # clear former lists
        self.waypoint_list.clear()
        self.marker_list[:] = []
        self.makerArray.markers[:] = []

        f = open(filename,'r')
        for line in f.readlines():
            j = line.split(",")
            current_wp_name     = str(j[0])        
            current_point       = Point(float(j[1]),float(j[2]),float(j[3]))
            current_quaternion  = Quaternion(float(j[4]),float(j[5]),float(j[6]),float(j[7]))
            
            self.waypoint_list[current_wp_name] = Pose(current_point,current_quaternion)

        f.close()
        
        self.create_markers()
        self.makerArray_pub.publish(self.makerArray)

    def create_markers(self):
       # Set up our waypoint markers
        marker_scale        = 0.6
        marker_lifetime     = 0 # 0 is forever
        marker_ns           = 'waypoints'
        marker_id           = 0
        marker_color        = {'r': 0.2, 'g': 0.8, 'b': 1.0, 'a': 1.0}
        
        # Define a marker publisher list.
        waypoint_name_list       = list()
        waypoint_name_list       = self.waypoint_list.keys()

        for waypoint in self.waypoint_list:

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
            marker.pose             = copy.deepcopy(self.waypoint_list[waypoint])
            marker.pose.position.z  = 0.65
            marker.text             = waypoint

            self.makerArray.markers.append(marker)

            marker_id               = marker_id + 1

        rospy.loginfo(self.makerArray)

#thread -> publish marks

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.move_base.cancel_goal()
        rospy.sleep(2)
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)
    
    def SpeechUpdateGoal(self, SpeechGoal):
        rospy.loginfo("input = %s", str(SpeechGoal.data))

      	for i in self.waypoint_list:
      		if str(SpeechGoal.data).find(i) != -1:
      			self.goal = MoveBaseGoal()
      			self.goal.target_pose.pose = self.waypoint_list[i]
            	self.goal.target_pose.header.frame_id = 'map'
            	self.goal.target_pose.header.stamp = rospy.Time.now()
            	rospy.loginfo("Going to: " + str(SpeechGoal.data))
            
            # Start the robot toward the next location
            	self.move_base.send_goal(self.goal)

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