#!/usr/bin/env python

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

class GroupLinear(moveit_commander.move_group.MoveGroupCommander):

    def __init__(self,name,robot_description="robot_description",ns=""):
	"""Constructor form the new child class"""
        super(GroupLinear,self).__init__(name)
	self.allow_replanning(True)

    def planLinear(self, wpose, eef_step = 0.01, jump_threshold = 1.5, avoid_collisions=True, path_constraints = None):
	"""Creates a Linear movement plan from compute_cartesian_path"""
	waypoints =[]
	waypoints.append(wpose)
	self.set_pose_target(wpose)
 	(plan,fraction) = self.compute_cartesian_path (waypoints, eef_step, jump_threshold)
	
	return (plan,fraction)


    def Return_valid_linear_plan(self,attempts=100,scale=1.0):
	""" Function to find random positions that can be achived in linear path """
	self.set_max_velocity_scaling_factor(0.01)
	while(attempts>0):
	    self.npose = self.get_random_pose().pose
	    while(self.npose.position.z<0.8):
		self.npose = self.get_random_pose().pose
	    (plan,fraction) = self.planLinear(self.npose,eef_step = 0.01, jump_threshold = 1.5)
	    if (fraction==1):
		plan = self.ModifySpeed(plan,scale)
	        return (self.npose, plan, fraction)
	    attempts=attempts-1
	return (self.npose, plan, fraction)

    def ModifySpeed(self,plan,scale=1.0):
	""" Modify the speed factor of a plan """

	#print info from the current plan
	d=Distance2Points(self.get_current_pose().pose,self.npose)
	rospy.loginfo('Distance to target {:f} m'.format(d))
	
	list_points=list(plan.joint_trajectory.points)
	total_time =list_points[-1].time_from_start.secs+ float(list_points[-1].time_from_start.nsecs)/1000000000
	rospy.loginfo('Previous time: {:f} s, at an average speed:  {:f} m/s'.format(total_time,d/total_time))
	
	if(scale<1 and scale>0):
	    for p in list_points:
		#Calculating new start time for each point
		p_time =(p.time_from_start.secs + float(p.time_from_start.nsecs)/1000000000)*(1/scale)
		p.time_from_start.secs = int(p_time)
		p.time_from_start.nsecs =(abs(p_time)-abs(int(p_time)))*1000000000

		# Recalculating velocities and accelerations
	        v=list(p.velocities)
		a=list(p.accelerations)

		for vi in v:
		    vi=vi*scale
		for ai in a:
		    ai=ai*scale*scale
		p.velocities=tuple(v)
		p.accelerations=tuple(a)

	    #print info from the new plan
	    rospy.loginfo('New path, adjusting speed to {:f} %'.format(100*scale))
	    total_time =list_points[-1].time_from_start.secs + float(list_points[-1].time_from_start.nsecs)/1000000000
	    rospy.loginfo('New time: {:f} s, at an average speed:  {:f} m/s'.format(total_time,d/total_time))
	
	    plan.joint_trajectory.points=tuple(list_points)
	return plan


