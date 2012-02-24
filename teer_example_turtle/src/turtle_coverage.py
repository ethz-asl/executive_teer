#!/usr/bin/env python
import math
import roslib; roslib.load_manifest('teer_example_turtle')
import rospy
import numpy as np
from turtlesim.msg import Velocity
from turtlesim.msg import Pose
from turtlesim.srv import TeleportAbsolute
from turtlesim.srv import SetPen
from turtlesim.srv import Spawn
from std_srvs.srv import Empty as EmptyServiceCall
from turtle_math import *
import rosteer
from teer import *

turtle1_velocity = None
turtle2_velocity = None
turtle1_set_pen = None
turtle2_set_pen = None
sched = None

class TurtleScheduler(rosteer.ROSScheduler):
	""" A teer scheduler working with ROS """
	
	turtle1_pose = rosteer.ROSConditionVariable(None)
	turtle2_pose = rosteer.ROSConditionVariable(None)
	
	def __init__(self):
		""" Init the ROS scheduler """
		super(TurtleScheduler,self).__init__()

def turtle1_go(target):
	""" Make turtle1 go to target, giving new speed command every 0.2 s """
	while True:
		# set new speed commands
		turtle1_velocity.publish(control_command(sched.turtle1_pose, target, 1.0))
		# wait for 1 s
		yield WaitDuration(0.2)

def turtle1_align(target_angle):
	"""  Make turtle1 align to angle, giving new speed command every second """
	diff = angle_diff(sched.turtle1_pose.theta, target_angle)
	sign_diff = math.copysign(1, diff)
	while True:
		# set new speed commands
		turtle1_velocity.publish(Velocity(0,-sign_diff))
		# wait for 1 s
		yield WaitDuration(1)

def turtle2_orbit(target):
	""" Make turtle2 orbit around target, giving new speed command every second """
	while True:
		# set new speed commands
		turtle2_velocity.publish(orbit_command(sched.turtle2_pose, target, 1.0))
		# wait for 1 s
		yield WaitDuration(0.2)

def turtle1_coverage():
	""" Make turtle1 do a square in the environment """
	yield WaitCondition(lambda: sched.turtle1_pose is not None)
	try:
		l = round(sched.turtle1_pose.y/2.)*2
		if l > 8:
			l = 2
		sched.printd('Restarting at ' + str(l))
		while True:
			targets = [(2,l), (8,l), (9,l+0.5), (8,l+1), (2,l+1), (1,l+1.5)]
			for target in targets:
				go_tid =  sched.new_task(turtle1_go(target))
				yield WaitCondition(lambda: dist(sched.turtle1_pose, target) < 0.1)
				sched.kill_task(go_tid)
			l += 2
			if l > 8:
				l = 2
	except GeneratorExit:
		sched.kill_task(go_tid)
		raise

def turtle2_orbiting():
	""" Make turtle2 do a square in the environment, reverse direction as turtle1 """
	yield WaitCondition(lambda: sched.turtle2_pose is not None)
	coverage_tid = sched.new_task(turtle1_coverage())
	orbit_tid = sched.new_task(turtle2_orbit((5, 5.5)))
	while True:
		yield WaitCondition(lambda: dist(sched.turtle1_pose, sched.turtle2_pose) < 1)
		sched.printd('Met friend, exchange data')
		sched.pause_task(orbit_tid)
		turtle2_velocity.publish(Velocity(0,0))
		sched.kill_task(coverage_tid)
		if sched.turtle1_pose.y < 5.5:
			align_tid = sched.new_task(turtle1_align(math.pi/2))
			yield WaitCondition(lambda: abs(angle_diff(sched.turtle1_pose.theta, math.pi/2)) < 0.1)
			sched.kill_task(align_tid)
			target = (sched.turtle1_pose.x, sched.turtle1_pose.y + 3)
		else:
			align_tid = sched.new_task(turtle1_align(-math.pi/2))
			yield WaitCondition(lambda: abs(angle_diff(sched.turtle1_pose.theta, -math.pi/2)) < 0.1)
			sched.kill_task(align_tid)
			target = (sched.turtle1_pose.x, sched.turtle1_pose.y - 3)
		go_tid = sched.new_task(turtle1_go(target))
		yield WaitCondition(lambda: dist(sched.turtle1_pose, target) < 0.1)
		sched.kill_task(go_tid)
		sched.printd('Meeting done')
		sched.resume_task(orbit_tid)
		coverage_tid = sched.new_task(turtle1_coverage())
		yield WaitDuration(6)

def turtle1_pose_updated(new_pose):
	""" We received a new pose of turtle1 from turtlesim, update condition variable in scheduler """
	sched.turtle1_pose = new_pose

def turtle2_pose_updated(new_pose):
	""" We received a new pose of turtle2 from turtlesim, update condition variable in scheduler """
	sched.turtle2_pose = new_pose

if __name__ == '__main__':
	# create scheduler
	sched = TurtleScheduler()
	sched.new_task(turtle2_orbiting())
	
	# connect to turtlesim
	rospy.init_node('teer_example_turtle')
	# services
	rospy.wait_for_service('reset')
	reset_simulator = rospy.ServiceProxy('reset', EmptyServiceCall)
	reset_simulator()
	rospy.wait_for_service('clear')
	clear_background = rospy.ServiceProxy('clear', EmptyServiceCall)
	spawn_turtle = rospy.ServiceProxy('spawn', Spawn)
	spawn_turtle(0,0,0, "turtle2")
	rospy.wait_for_service('turtle1/set_pen')
	turtle1_set_pen = rospy.ServiceProxy('turtle1/set_pen', SetPen)
	rospy.wait_for_service('turtle1/teleport_absolute')
	turtle1_teleport = rospy.ServiceProxy('turtle1/teleport_absolute', TeleportAbsolute)
	rospy.wait_for_service('turtle2/set_pen')
	turtle2_set_pen = rospy.ServiceProxy('turtle2/set_pen', SetPen)
	rospy.wait_for_service('turtle2/teleport_absolute')
	turtle2_teleport = rospy.ServiceProxy('turtle2/teleport_absolute', TeleportAbsolute)
	# subscriber/publisher
	rospy.Subscriber('turtle1/pose', Pose, turtle1_pose_updated)
	turtle1_velocity = rospy.Publisher('turtle1/command_velocity', Velocity)
	rospy.Subscriber('turtle2/pose', Pose, turtle2_pose_updated)
	turtle2_velocity = rospy.Publisher('turtle2/command_velocity', Velocity)
	
	# setup environment
	turtle1_set_pen(0,0,0,0,1)
	turtle2_set_pen(0,0,0,0,1)
	turtle1_teleport(2,2,0)
	turtle2_teleport(5.5,9,0)
	clear_background()
	
	# run scheduler
	sched.run()