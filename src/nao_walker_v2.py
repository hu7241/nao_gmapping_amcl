#!/usr/bin/env python

#
# ROS node to control Nao's walking engine (omniwalk and footsteps)
# This code is currently compatible to NaoQI version 1.6 or newer (latest
# tested: 1.12)
#
# Copyright 2009-2011 Armin Hornung & Stefan Osswald, University of Freiburg
# http://www.ros.org/wiki/nao
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    # Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#    # Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#    # Neither the name of the University of Freiburg nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#

import rospy

from naoqi_driver.naoqi_node import NaoqiNode

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D

from std_srvs.srv import Empty, EmptyResponse
from naoqi_bridge_msgs.srv import CmdPoseService, CmdVelService, CmdPoseServiceResponse, CmdVelServiceResponse, SetArmsEnabled, SetArmsEnabledResponse
from humanoid_nav_msgs.msg import StepTarget
from humanoid_nav_msgs.srv import StepTargetService, StepTargetServiceResponse

from nao_apps import startWalkPose

import time

class NaoWalker(NaoqiNode):
    def __init__(self):
        NaoqiNode.__init__(self, 'nao_walker')

        self.connectNaoQi()

        # walking pattern params:
        self.headPitchAngle = rospy.get_param('~head_pitch_angle', 0.0)
        self.headPitchAngleBack = rospy.get_param('~head_pitch_angle_back', 0.0)
        self.headPitchAngleLeft = rospy.get_param('~head_pitch_angle_left', 0.0)
        self.headPitchAngleRight = rospy.get_param('~head_pitch_angle_right', 0.0)

        self.stepFrequency = rospy.get_param('~step_frequency', 0.5)
        self.stepFrequencyBack = rospy.get_param('~step_frequency_back', 0.5)
        self.stepFrequencyLeft = rospy.get_param('~step_frequency_left', 0.5)
        self.stepFrequencyRight = rospy.get_param('~step_frequency_right', 0.5)

        self.useStartWalkPose = rospy.get_param('~use_walk_pose', False)
        self.needsStartWalkPose = True

        # other params
        self.maxHeadSpeed = rospy.get_param('~max_head_speed', 0.2)
        # initial stiffness (defaults to 0 so it doesn't strain the robot when no teleoperation is running)
        # set to 1.0 if you want to control the robot immediately
        initStiffness = rospy.get_param('~init_stiffness', 0.0)

        self.useFootGaitConfig = rospy.get_param('~use_foot_gait_config', False)
        rospy.loginfo("useFootGaitConfig = %d" % self.useFootGaitConfig)
        if self.useFootGaitConfig:
            self.footGaitConfig = rospy.get_param('~foot_gait_config', self.motionProxy.getMoveConfig("Default"))
            self.footGaitConfigBack = rospy.get_param('~foot_gait_config_back', self.motionProxy.getMoveConfig("Default"))
            self.footGaitConfigLeft = rospy.get_param('~foot_gait_config_left', self.motionProxy.getMoveConfig("Default"))
            self.footGaitConfigRight = rospy.get_param('~foot_gait_config_right', self.motionProxy.getMoveConfig("Default"))
        else:
            self.footGaitConfig = self.motionProxy.getMoveConfig("Default")
            self.footGaitConfigBack = self.motionProxy.getMoveConfig("Default")
            self.footGaitConfigLeft = self.motionProxy.getMoveConfig("Default")
            self.footGaitConfigRight = self.motionProxy.getMoveConfig("Default")

        rospy.loginfo("stepFrequency = %f" % self.stepFrequency)
        rospy.loginfo("footGaitConfig = %s" % self.footGaitConfig)

        rospy.loginfo("stepFrequencyBack = %f" % self.stepFrequencyBack)
        rospy.loginfo("footGaitConfigBack = %s" % self.footGaitConfigBack)

        rospy.loginfo("stepFrequencyLeft = %f" % self.stepFrequencyLeft)
        rospy.loginfo("footGaitConfigLeft = %s" % self.footGaitConfigLeft)

        rospy.loginfo("stepFrequencyRight = %f" % self.stepFrequencyRight)
        rospy.loginfo("footGaitConfigRight = %s" % self.footGaitConfigRight)

        # TODO: parameterize
        if initStiffness > 0.0 and initStiffness <= 1.0:
            self.motionProxy.stiffnessInterpolation('Body', initStiffness, 0.5)

        try:
            enableFootContactProtection = rospy.get_param('~enable_foot_contact_protection')
            self.motionProxy.setMotionConfig([["ENABLE_FOOT_CONTACT_PROTECTION", enableFootContactProtection]])
            if enableFootContactProtection:
                rospy.loginfo("Enabled foot contact protection")
            else:
                rospy.loginfo("Disabled foot contact protection")
        except KeyError:
            # do not change foot contact protection
            pass

        # last: ROS subscriptions (after all vars are initialized)
        rospy.Subscriber("cmd_vel", Twist, self.handleCmdVel, queue_size=1)
        rospy.Subscriber("cmd_pose", Pose2D, self.handleTargetPose, queue_size=1)
        rospy.Subscriber("cmd_step", StepTarget, self.handleStep, queue_size=50)

        # Create ROS publisher for speech
        #self.pub = rospy.Publisher("speech", String, latch = True)

        # ROS services (blocking functions)
        self.cmdPoseSrv = rospy.Service("cmd_pose_srv", CmdPoseService, self.handleTargetPoseService)
        self.cmdVelSrv = rospy.Service("cmd_vel_srv", CmdVelService, self.handleCmdVelService)
        self.stepToSrv = rospy.Service("cmd_step_srv", StepTargetService, self.handleStepSrv)
        self.stopWalkSrv = rospy.Service("stop_walk_srv", Empty, self.handleStopWalkSrv)
        self.needsStartWalkPoseSrv = rospy.Service("needs_start_walk_pose_srv", Empty, self.handleNeedsStartWalkPoseSrv)
        self.readFootGaitConfigSrv = rospy.Service("read_foot_gait_config_srv", Empty, self.handleReadFootGaitConfigSrv)
        self.setArmsEnabledSrv = rospy.Service("enable_arms_walking_srv", SetArmsEnabled, self.handleSetArmsEnabledSrv)
        
        self.logSrv = rospy.Service("log_srv", Empty, self.handleLogService)

        #self.say("Walker online")

        rospy.loginfo("nao_walker initialized")

    def connectNaoQi(self):
        '''(re-) connect to NaoQI'''
        rospy.loginfo("Connecting to NaoQi at %s:%d", self.pip, self.pport)

        self.motionProxy = self.get_proxy("ALMotion")
        if self.motionProxy is None:
            exit(1)
            
        self.memoryProxy = self.get_proxy("ALMemory")
        if self.memoryProxy is None:
            exit(2)
        else:
            rospy.loginfo("Memory Proxy OK")

    def stopMove(self):
        """ Stops the current walking bahavior and blocks until the clearing is complete. """
        try:
            self.motionProxy.moveToward(0.0, 0.0, 0.0, [["Frequency", self.stepFrequency]])
            self.motionProxy.waitUntilMoveIsFinished()


        except RuntimeError,e:
            print "An error has been caught"
            print e
            return False

        return True


    def say(self, text):
        self.pub.publish(text)

    def handleCmdVel(self, data):
        rospy.logdebug("Walk cmd_vel: %f %f %f, frequency %f", data.linear.x, data.linear.y, data.angular.z, self.stepFrequency)
        if data.linear.x != 0 or data.linear.y != 0 or data.angular.z != 0:
            self.gotoStartWalkPose()
        try:
            eps = 1e-3 # maybe 0,0,0 is a special command in motionProxy...
            if abs(data.linear.x)<eps and abs(data.linear.y)<eps and abs(data.angular.z)<eps:
                self.motionProxy.moveToward(0,0,0,[["Frequency",0.5]])
            else:
                if data.linear.x>=eps and abs(data.linear.y)<eps and abs(data.angular.z)<eps:
                    self.motionProxy.angleInterpolationWithSpeed("HeadPitch", self.headPitchAngle, 0.5)
                    self.motionProxy.moveToward(data.linear.x, data.linear.y, data.angular.z, [["Frequency", self.stepFrequency]] + self.footGaitConfig)
                    rospy.loginfo('case forward')
                elif data.linear.x<=-eps and abs(data.linear.y)<eps and abs(data.angular.z)<eps:
                    self.motionProxy.angleInterpolationWithSpeed("HeadPitch", self.headPitchAngleBack, 0.5)
                    self.motionProxy.moveToward(data.linear.x, data.linear.y, data.angular.z, [["Frequency", self.stepFrequencyBack]] + self.footGaitConfigBack)
                    rospy.loginfo('case backward')
                elif abs(data.linear.x)<eps and abs(data.linear.y)<eps and data.angular.z>=-eps:
                    self.motionProxy.angleInterpolationWithSpeed("HeadPitch", self.headPitchAngleLeft, 0.5)
                    self.motionProxy.moveToward(data.linear.x, data.linear.y, data.angular.z, [["Frequency", self.stepFrequencyLeft]] + self.footGaitConfigLeft)
                    rospy.loginfo('case left')
                elif abs(data.linear.x)<eps and abs(data.linear.y)<eps and data.angular.z<=eps:
                    self.motionProxy.angleInterpolationWithSpeed("HeadPitch", self.headPitchAngleRight, 0.5)
                    self.motionProxy.moveToward(data.linear.x, data.linear.y, data.angular.z, [["Frequency", self.stepFrequencyRight]] + self.footGaitConfigRight)
                    rospy.loginfo('case right')
                else:
                    self.motionProxy.angleInterpolationWithSpeed("HeadPitch", self.headPitchAngle, 0.5)
                    self.motionProxy.moveToward(data.linear.x, data.linear.y, data.angular.z, [["Frequency", self.stepFrequency]] + self.footGaitConfig)
                    rospy.loginfo('case else')
        except RuntimeError,e:
            # We have to assume there's no NaoQI running anymore => exit!
            rospy.logerr("Exception caught in handleCmdVel:\n%s", e)
            rospy.signal_shutdown("No NaoQI available anymore")
        
        
            

    def handleCmdVelService(self, req):
        self.handleCmdVel(req.twist)
        return CmdVelServiceResponse()

    def handleTargetPose(self, data, post=True):
        """handles cmd_pose requests, walks to (x,y,theta) in robot coordinate system"""

        rospy.logdebug("Walk target_pose: %f %f %f", data.x,
                data.y, data.theta)

        self.gotoStartWalkPose()

        try:
            if post:
                self.motionProxy.post.moveTo(data.x, data.y, data.theta, self.footGaitConfig)
            else:
                self.motionProxy.moveTo(data.x, data.y, data.theta, self.footGaitConfig)
            return True
        except RuntimeError,e:
            rospy.logerr("Exception caught in handleTargetPose:\n%s", e)
            return False


    def handleStep(self, data):
        rospy.logdebug("Step leg: %d; target: %f %f %f", data.leg, data.pose.x,
                data.pose.y, data.pose.theta)
        try:
            if data.leg == StepTarget.right:
                leg = "RLeg"
            elif data.leg == StepTarget.left:
                leg = "LLeg"
            else:
                rospy.logerr("Received a wrong leg constant: %d, ignoring step command", data.leg)
                return
            self.motionProxy.stepTo(leg, data.pose.x, data.pose.y, data.pose.theta)
            return True
        except RuntimeError, e:
            rospy.logerr("Exception caught in handleStep:\n%s", e)
            return False

    def handleStepSrv(self, req):
        if self.handleStep(req.step):
            return StepTargetServiceResponse()
        else:
            return None

    def handleTargetPoseService(self, req):
        """ do NOT use post"""
        if self.handleTargetPose(req.pose, False):
            return CmdPoseServiceResponse()
        else:
            return None

    def handleStopWalkSrv(self, req):
        if self.stopMove():
            return EmptyResponse()
        else:
            return None

    def gotoStartWalkPose(self):
        if self.useStartWalkPose and self.needsStartWalkPose:
            startWalkPose(self.motionProxy)
            self.needsStartWalkPose = False

    def handleNeedsStartWalkPoseSrv(self, data):
        self.needsStartWalkPose = True
        return EmptyResponse()

    def handleReadFootGaitConfigSrv(self, data):
        self.useFootGaitConfig = rospy.get_param('~use_foot_gait_config', False)
        rospy.loginfo("useFootGaitConfig = %d" % self.useFootGaitConfig)
        if self.useFootGaitConfig:
            self.footGaitConfig = rospy.get_param('~foot_gait_config', self.motionProxy.getMoveConfig("Default"))
        else:
            self.footGaitConfig = self.motionProxy.getMoveConfig("Default")
        return EmptyResponse()

    def handleSetArmsEnabledSrv(self, req):
        self.motionProxy.setWalkArmsEnable(req.left_arm, req.right_arm)
        rospy.loginfo("Arms enabled during walk: left(%s) right(%s)" % (req.left_arm, req.right_arm))
        return SetArmsEnabledResponse()
        
    def handleLog(self):
        for x in self.footGaitConfig:
            rospy.loginfo("{0};{1}".format(x[0],x[1]))
            
        rospy.loginfo("StepFrequency;%f" % self.stepFrequency)
        
        for x in range(100):
            ticker = time.time()
            gyroscopeX = self.getGyroscopeX()
            gyroscopeY = self.getGyroscopeY()
            gyroscopeZ = self.getGyroscopeZ()
            accelerometerX = self.getAccelerometerX()
            accelerometerY = self.getAccelerometerY()
            accelerometerZ = self.getAccelerometerZ()
            angleX = self.getAngleX()
            angleY = self.getAngleY()
            fsrLFootFrontLeft = self.getFsrLFootFrontLeft()
            fsrLFootFrontRight = self.getFsrLFootFrontRight()
            fsrLFootRearLeft = self.getFsrLFootRearLeft()
            fsrLFootRearRight = self.getFsrLFootRearRight()
            fsrRFootFrontLeft = self.getFsrRFootFrontLeft()
            fsrRFootFrontRight = self.getFsrRFootFrontRight()
            fsrRFootRearLeft = self.getFsrRFootRearLeft()
            fsrRFootRearRight = self.getFsrRFootRearRight()
            fsrLFootTotalWeight = self.getFsrLFootTotalWeight()
            fsrRFootTotalWeight = self.getFsrRFootTotalWeight()
            fsrLFootCenterOfPressureX = self.getFsrLFootCenterOfPressureX()
            fsrLFootCenterOfPressureY = self.getFsrLFootCenterOfPressureY()
            fsrRFootCenterOfPressureX = self.getFsrRFootCenterOfPressureX()
            fsrRFootCenterOfPressureY = self.getFsrRFootCenterOfPressureY()
            headYaw = self.getJointPositionHeadYaw()
            headPitch = self.getJointPositionHeadPitch()
            lShoulderPitch = self.getJointPositionLShoulderPitch()
            lShoulderRoll = self.getJointPositionLShoulderRoll()
            lElbowYaw = self.getJointPositionLElbowYaw()
            lElbowRoll = self.getJointPositionLElbowRoll()
            lWristYaw = self.getJointPositionLWristYaw()
            lHand = self.getJointPositionLHand()
            rShoulderPitch = self.getJointPositionRShoulderPitch()
            rShoulderRoll = self.getJointPositionRShoulderRoll()       
            rElbowYaw = self.getJointPositionRElbowYaw()
            rElbowRoll = self.getJointPositionRElbowRoll()
            rWristYaw = self.getJointPositionRWristYaw()
            rHand = self.getJointPositionRHand()
            lHipYawPitch = self.getJointPositionLHipYawPitch()     
            lHipRoll = self.getJointPositionLHipRoll()
            lHipPitch = self.getJointPositionLHipPitch()
            lKnee = self.getJointPositionLKneePitch()
            lAnklePitch = self.getJointPositionLAnklePitch()     
            lAnkleRoll = self.getJointPositionLAnkleRoll()        
            rHipRoll = self.getJointPositionRHipRoll()
            rHipPitch = self.getJointPositionRHipPitch()   
            rKneePitch = self.getJointPositionRKneePitch()     
            rAnklePitch = self.getJointPositionRAnklePitch()
            rAnkleRoll = self.getJointPositionRAnkleRoll()
                
            rospy.loginfo("GyroscopeX;%f;%f" % (gyroscopeX, ticker))
            rospy.loginfo("GyroscopeY;%f;%f" % (gyroscopeY, ticker))
            rospy.loginfo("GyroscopeZ;%f;%f" % (gyroscopeZ, ticker))
            rospy.loginfo("AccelerometerX;%f;%f" % (accelerometerX, ticker))
            rospy.loginfo("AccelerometerY;%f;%f" % (accelerometerY, ticker))
            rospy.loginfo("AccelerometerZ;%f;%f" % (accelerometerZ, ticker))
            rospy.loginfo("AngleX;%f;%f" % (angleX, ticker))
            rospy.loginfo("AngleY;%f;%f" % (angleY, ticker))
            rospy.loginfo("FsrLFootFrontLeft;%f;%f" % (fsrLFootFrontLeft, ticker))
            rospy.loginfo("FsrLFootFrontRight;%f;%f" % (fsrLFootFrontRight, ticker))
            rospy.loginfo("FsrLFootRearLeft;%f;%f" % (fsrLFootRearLeft, ticker))
            rospy.loginfo("FsrLFootRearRight;%f;%f" % (fsrLFootRearRight, ticker))
            rospy.loginfo("FsrRFootFrontLeft;%f;%f" % (fsrRFootFrontLeft, ticker))
            rospy.loginfo("FsrRFootFrontRight;%f;%f" % (fsrRFootFrontRight, ticker))
            rospy.loginfo("FsrRFootRearLeft;%f;%f" % (fsrRFootRearLeft, ticker))
            rospy.loginfo("FsrRFootRearRight;%f;%f" % (fsrRFootRearRight, ticker))
            rospy.loginfo("FsrLFootTotalWeight;%f;%f" % (fsrLFootTotalWeight, ticker))
            rospy.loginfo("FsrRFootTotalWeight;%f;%f" % (fsrRFootTotalWeight, ticker))
            rospy.loginfo("FsrLFootCenterOfPressureX;%f;%f" % (fsrLFootCenterOfPressureX, ticker))
            rospy.loginfo("FsrLFootCenterOfPressureY;%f;%f" % (fsrLFootCenterOfPressureY, ticker))
            rospy.loginfo("FsrRFootCenterOfPressureX;%f;%f" % (fsrRFootCenterOfPressureX, ticker))
            rospy.loginfo("FsrRFootCenterOfPressureY;%f;%f" % (fsrRFootCenterOfPressureY, ticker))
            rospy.loginfo("HeadYaw;%f;%f" % (headYaw, ticker))
            rospy.loginfo("HeadPitch;%f;%f" % (headPitch, ticker))
            rospy.loginfo("LShoulderPitch;%f;%f" % (lShoulderPitch, ticker))
            rospy.loginfo("LShoulderRoll;%f;%f" % (lShoulderRoll, ticker))
            rospy.loginfo("LElbowYaw;%f;%f" % (lElbowYaw, ticker))
            rospy.loginfo("LElbowRoll;%f;%f" % (lElbowRoll, ticker))
            rospy.loginfo("LWristYaw;%f;%f" % (lWristYaw, ticker))
            rospy.loginfo("LHand;%f;%f" % (lHand, ticker))
            rospy.loginfo("RShoulderPitch;%f;%f" % (rShoulderPitch, ticker))
            rospy.loginfo("RShoulderRoll;%f;%f" % (rShoulderRoll, ticker))
            rospy.loginfo("RElbowYaw;%f;%f" % (rElbowYaw, ticker))
            rospy.loginfo("RElbowRoll;%f;%f" % (rElbowRoll, ticker))
            rospy.loginfo("RWristYaw;%f;%f" % (rWristYaw, ticker))
            rospy.loginfo("RHand;%f;%f" % (rHand, ticker))
            rospy.loginfo("LHipYawPitch;%f;%f" % (lHipYawPitch, ticker))
            rospy.loginfo("LHipRoll;%f;%f" % (lHipRoll, ticker))
            rospy.loginfo("LHipPitch;%f;%f" % (lHipPitch, ticker))
            rospy.loginfo("LKnee;%f;%f" % (lKnee, ticker))
            rospy.loginfo("LAnklePitch;%f;%f" % (lAnklePitch, ticker))
            rospy.loginfo("LAnkleRoll;%f;%f" % (lAnkleRoll, ticker))
            rospy.loginfo("RHipRoll;%f;%f" % (rHipRoll, ticker))
            rospy.loginfo("RHipPitch;%f;%f" % (rHipPitch, ticker))
            rospy.loginfo("RKneePitch;%f;%f" % (rKneePitch, ticker))
            rospy.loginfo("RAnklePitch;%f;%f" % (rAnklePitch, ticker))
            rospy.loginfo("RAnkleRoll;%f;%f" % (rAnkleRoll, ticker))
        
    def handleLogService(self, req):
        self.handleLog()
        return EmptyResponse()
        
    def getGyroscopeX(self):
        return self.memoryProxy.getData("Device/SubDeviceList/InertialSensor/GyroscopeX/Sensor/Value")

    def getGyroscopeY(self):
        return self.memoryProxy.getData("Device/SubDeviceList/InertialSensor/GyroscopeY/Sensor/Value")
        
    def getGyroscopeZ(self):
        return self.memoryProxy.getData("Device/SubDeviceList/InertialSensor/GyroscopeZ/Sensor/Value")
        
    def getAccelerometerX(self):
        return self.memoryProxy.getData("Device/SubDeviceList/InertialSensor/AccelerometerX/Sensor/Value")
        
    def getAccelerometerY(self):
        return self.memoryProxy.getData("Device/SubDeviceList/InertialSensor/AccelerometerY/Sensor/Value")
        
    def getAccelerometerZ(self):
        return self.memoryProxy.getData("Device/SubDeviceList/InertialSensor/AccelerometerZ/Sensor/Value")
        
    def getAngleX(self):
        return self.memoryProxy.getData("Device/SubDeviceList/InertialSensor/AngleX/Sensor/Value")
        
    def getAngleY(self):
        return self.memoryProxy.getData("Device/SubDeviceList/InertialSensor/AngleY/Sensor/Value")

    def getFsrLFootFrontLeft(self):
        return self.memoryProxy.getData("Device/SubDeviceList/LFoot/FSR/FrontLeft/Sensor/Value")
        
    def getFsrLFootFrontRight(self):
        return self.memoryProxy.getData("Device/SubDeviceList/LFoot/FSR/FrontRight/Sensor/Value")
        
    def getFsrLFootRearLeft(self):
        return self.memoryProxy.getData("Device/SubDeviceList/LFoot/FSR/RearLeft/Sensor/Value")
        
    def getFsrLFootRearRight(self):
        return self.memoryProxy.getData("Device/SubDeviceList/LFoot/FSR/RearRight/Sensor/Value")
        
    def getFsrRFootFrontLeft(self):
        return self.memoryProxy.getData("Device/SubDeviceList/RFoot/FSR/FrontLeft/Sensor/Value")
        
    def getFsrRFootFrontRight(self):
        return self.memoryProxy.getData("Device/SubDeviceList/RFoot/FSR/FrontRight/Sensor/Value")
        
    def getFsrRFootRearLeft(self):
        return self.memoryProxy.getData("Device/SubDeviceList/RFoot/FSR/RearLeft/Sensor/Value")
        
    def getFsrRFootRearRight(self):
        return self.memoryProxy.getData("Device/SubDeviceList/RFoot/FSR/RearRight/Sensor/Value")
        
    def getFsrLFootTotalWeight(self):
        return self.memoryProxy.getData("Device/SubDeviceList/LFoot/FSR/TotalWeight/Sensor/Value")
        
    def getFsrRFootTotalWeight(self):
        return self.memoryProxy.getData("Device/SubDeviceList/RFoot/FSR/TotalWeight/Sensor/Value")
        
    def getFsrLFootCenterOfPressureX(self):
        return self.memoryProxy.getData("Device/SubDeviceList/LFoot/FSR/CenterOfPressure/X/Sensor/Value")
        
    def getFsrLFootCenterOfPressureY(self):
        return self.memoryProxy.getData("Device/SubDeviceList/LFoot/FSR/CenterOfPressure/Y/Sensor/Value")
        
    def getFsrRFootCenterOfPressureX(self):
        return self.memoryProxy.getData("Device/SubDeviceList/RFoot/FSR/CenterOfPressure/X/Sensor/Value")
        
    def getFsrRFootCenterOfPressureY(self):
        return self.memoryProxy.getData("Device/SubDeviceList/RFoot/FSR/CenterOfPressure/Y/Sensor/Value")
        
    def getJointPositionHeadYaw(self):
        return self.memoryProxy.getData("Device/SubDeviceList/HeadYaw/Position/Sensor/Value")
        
    def getJointPositionHeadPitch(self):
        return self.memoryProxy.getData("Device/SubDeviceList/HeadPitch/Position/Sensor/Value")
        
    def getJointPositionLShoulderPitch(self):
        return self.memoryProxy.getData("Device/SubDeviceList/LShoulderPitch/Position/Sensor/Value")
        
    def getJointPositionLShoulderRoll(self):
        return self.memoryProxy.getData("Device/SubDeviceList/LShoulderRoll/Position/Sensor/Value")
        
    def getJointPositionLElbowYaw(self):
        return self.memoryProxy.getData("Device/SubDeviceList/LElbowYaw/Position/Sensor/Value")
        
    def getJointPositionLElbowRoll(self):
        return self.memoryProxy.getData("Device/SubDeviceList/LElbowRoll/Position/Sensor/Value")
    
    def getJointPositionLWristYaw(self):
        return self.memoryProxy.getData("Device/SubDeviceList/LWristYaw/Position/Sensor/Value")
        
    def getJointPositionLHand(self):
        return self.memoryProxy.getData("Device/SubDeviceList/LHand/Position/Sensor/Value")
        
    def getJointPositionRShoulderPitch(self):
        return self.memoryProxy.getData("Device/SubDeviceList/RShoulderPitch/Position/Sensor/Value")
        
    def getJointPositionRShoulderRoll(self):
        return self.memoryProxy.getData("Device/SubDeviceList/RShoulderRoll/Position/Sensor/Value")
        
    def getJointPositionRElbowYaw(self):
        return self.memoryProxy.getData("Device/SubDeviceList/RElbowYaw/Position/Sensor/Value")
        
    def getJointPositionRElbowRoll(self):
        return self.memoryProxy.getData("Device/SubDeviceList/RElbowRoll/Position/Sensor/Value")
        
    def getJointPositionRWristYaw(self):
        return self.memoryProxy.getData("Device/SubDeviceList/RWristYaw/Position/Sensor/Value")
        
    def getJointPositionRHand(self):
        return self.memoryProxy.getData("Device/SubDeviceList/RHand/Position/Sensor/Value")
        
    def getJointPositionLHipYawPitch(self):
        return self.memoryProxy.getData("Device/SubDeviceList/LHipYawPitch/Position/Sensor/Value")
        
    def getJointPositionLHipRoll(self):
        return self.memoryProxy.getData("Device/SubDeviceList/LHipRoll/Position/Sensor/Value")
        
    def getJointPositionLHipPitch(self):
        return self.memoryProxy.getData("Device/SubDeviceList/LHipPitch/Position/Sensor/Value")
        
    def getJointPositionLKneePitch(self):
        return self.memoryProxy.getData("Device/SubDeviceList/LKneePitch/Position/Sensor/Value")
        
    def getJointPositionLAnklePitch(self):
        return self.memoryProxy.getData("Device/SubDeviceList/LAnklePitch/Position/Sensor/Value")
        
    def getJointPositionLAnkleRoll(self):
        return self.memoryProxy.getData("Device/SubDeviceList/LAnkleRoll/Position/Sensor/Value")
        
    def getJointPositionRHipRoll(self):
        return self.memoryProxy.getData("Device/SubDeviceList/RHipRoll/Position/Sensor/Value")
        
    def getJointPositionRHipPitch(self):
        return self.memoryProxy.getData("Device/SubDeviceList/RHipPitch/Position/Sensor/Value")
        
    def getJointPositionRKneePitch(self):
        return self.memoryProxy.getData("Device/SubDeviceList/RKneePitch/Position/Sensor/Value")
        
    def getJointPositionRAnklePitch(self):
        return self.memoryProxy.getData("Device/SubDeviceList/RAnklePitch/Position/Sensor/Value")
        
    def getJointPositionRAnkleRoll(self):
        return self.memoryProxy.getData("Device/SubDeviceList/RAnkleRoll/Position/Sensor/Value")

if __name__ == '__main__':
    walker = NaoWalker()
    rospy.loginfo("nao_walker running...")
    rospy.spin()
    rospy.loginfo("nao_walker stopping...")
    walker.stopMove()

    rospy.loginfo("nao_walker stopped.")
    exit(0)
