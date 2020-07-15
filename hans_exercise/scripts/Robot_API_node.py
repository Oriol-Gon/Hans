#!/usr/bin/env python



import tf
import sys
import math
import rospy
import copy
import random
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

from hans_exercise.msg import xyz_RPY,speed_scale
from tf2_msgs.msg import TFMessage

from tf.transformations import *

from DistanceTool import *
from GroupLinearClass import *


class HansSolution():

    def __init__(self,group,robot,scale=0.10):

	# Publishers
        self._pub=rospy.Publisher('current_cartesian',xyz_RPY,queue_size=10);
	self._display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                               moveit_msgs.msg.DisplayTrajectory,
                                               queue_size=20)

        self._listenertf = tf.TransformListener()

	self._scale=scale
        self._group = group
	self._robot = robot
		

	self._group.stop()
        self._group.clear_pose_targets()
	self.GoCustomHome()

    def GoCustomHome(self):

	pose_Home = geometry_msgs.msg.Pose()
	pose_Home.orientation.w = 1.0
	pose_Home.position.x = 0.0
	pose_Home.position.y = 0.0
	pose_Home.position.z = 0.8

	self._TargetPoint=pose_Home
	group.set_pose_target(pose_Home)
        plan_Home = self._group.plan()
        self._group.execute(plan_Home,wait=False);

    def Start(self):
	rospy.loginfo('Node started: HansSolution')
        self._timer=rospy.Timer(rospy.Duration(nsecs=50000000),self.PubEuler)

    def PubEuler(self, evt):
        while not rospy.is_shutdown():
            try:
                self._listenertf.waitForTransform( self._group.get_planning_frame(),
                                                self._group.get_end_effector_link(),
                                                rospy.Time(0),rospy.Duration(100))
                (xyz,qua) = self._listenertf.lookupTransform( self._group.get_planning_frame(),
                                                            self._group.get_end_effector_link(),
                                                            rospy.Time(0))
                break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
	
	rpy= tf.transformations.euler_from_quaternion(qua)

        cc = xyz_RPY()
        cc.x=round(xyz[0]*1000,2)
        cc.y=round(xyz[1]*1000,2)
        cc.z=round(xyz[2]*1000,2)
	cc.R=round(rpy[0]*180/math.pi,2)
	cc.P=round(rpy[1]*180/math.pi,2)
	cc.Y=round(rpy[2]*180/math.pi,2)

        self._pub.publish(cc)
	
	if(Distance(xyz,self._TargetPoint)<0.0001):
	    self.MoveNext()

    def MoveNext(self):
	self._group.stop()
	
	#get new random point
	(npoint, plan, fraction) = self._group.Return_valid_linear_plan(scale=self._scale)

	if (fraction==1):

	     self._TargetPoint=copy.deepcopy(npoint)

	     #publish the plan
	     display_trajectory = moveit_msgs.msg.DisplayTrajectory()
	     display_trajectory.trajectory_start = self._robot.get_current_state()
	     display_trajectory.trajectory.append(plan)
	     #print(plan)
	     self._display_trajectory_publisher.publish(display_trajectory);

	     #execute the plan
	     self._group.execute(plan,wait=False)
	else:
	    rospy.loginfo("Warning: No new point found")


    def UpdateSpeed_cb(self,msg):
	self._scale=msg.scale


if __name__ == '__main__':

    
    rospy.init_node('Hans_exercise_node')

    #init
    moveit_commander.roscpp_initialize(sys.argv)
    scene = moveit_commander.PlanningSceneInterface()
    robot = moveit_commander.RobotCommander()

    group = GroupLinear('elfin_arm') # GroupLinear Extends from moveit_commander.move_group.MoveGroupCommander

    hanssolution = HansSolution(group,robot)
    hanssolution.Start()  

    rospy.Publisher('/new_speed_factor',speed_scale,queue_size=10);
    rospy.Subscriber("/new_speed_factor",speed_scale,hanssolution.UpdateSpeed_cb)

    rospy.spin()
