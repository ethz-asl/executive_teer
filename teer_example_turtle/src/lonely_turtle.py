#!/usr/bin/env python
import math
import roslib; roslib.load_manifest('teer_example_turtle')
import rospy
import numpy as np
from turtlesim.msg import Velocity
from turtlesim.msg import Pose
from turtlesim.srv import TeleportAbsolute
from turtlesim.srv import SetPen
from std_srvs.srv import Empty as EmptyServiceCall
from turtle_math import *
import rosteer
from teer import *

turtle1_velocity = None
turtle1_teleport = None
clear_background = None

class TurtleScheduler(rosteer.ROSScheduler):
	''' A teer scheduler working with ROS '''
	
	turtle1_pose = rosteer.ROSConditionVariable(None)
	
	def __init__(self):
		''' Init the ROS scheduler '''
		super(TurtleScheduler,self).__init__()

def turtle1_go(sched, target):
	''' Make turtle1 go to target, giving new speed command every second '''
	while True:
		# set new speed commands
		turtle1_velocity.publish(control_command(sched.turtle1_pose, target))
		# wait for 1 s
		yield WaitDuration(1)

def turtle1_task(sched):
	''' Make turtle1 do a square in the environment '''
	yield WaitCondition(lambda: sched.turtle1_pose is not None)
	print 'Invoking teleport'
	turtle1_teleport(2,2,0)
	clear_background()
	
	targets = [(2,2), (8,2), (8,8), (2,8)]
	target_id = 0
	while True:
		print ('Going to ' + str(targets[target_id]))
		target = targets[target_id]
		go_tid =  yield NewTask(turtle1_go(sched, target))
		yield WaitCondition(lambda: dist(sched.turtle1_pose, target) < 0.1)
		yield KillTask(go_tid)
		target_id = (target_id + 1) % len(targets)
		
def turtle1_pose_updated(new_pose):
	''' We received a new pose of turtle1 from turtlesim, update condition variable in scheduler '''
	global sched
	sched.turtle1_pose = new_pose

if __name__ == '__main__':
	# create scheduler
	global sched
	sched = TurtleScheduler()
	sched.new(turtle1_task(sched))
	
	# connect to turtlesim
	rospy.init_node('teer_example_turtle')
	# subscriber/publisher
	rospy.Subscriber('turtle1/pose', Pose, turtle1_pose_updated)
	turtle1_velocity = rospy.Publisher('turtle1/command_velocity', Velocity)
	# services
	rospy.wait_for_service('reset')
	reset_simulator = rospy.ServiceProxy('reset', EmptyServiceCall)
	reset_simulator()
	rospy.wait_for_service('turtle1/set_pen')
	turtle1_set_pen = rospy.ServiceProxy('turtle1/set_pen', SetPen)
	turtle1_set_pen(0,0,0,0,1)
	rospy.wait_for_service('turtle1/teleport_absolute')
	turtle1_teleport = rospy.ServiceProxy('turtle1/teleport_absolute', TeleportAbsolute)
	rospy.wait_for_service('clear')
	clear_background = rospy.ServiceProxy('clear', EmptyServiceCall)
	
	# run scheduler
	sched.run()