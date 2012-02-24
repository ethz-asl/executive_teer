# -*- coding: utf-8 -*-
# kate: replace-tabs off; indent-width 4; indent-mode normal
# vim: ts=4:sw=4:noexpandtab

from teer import *
import threading
import rospy

# ------------------------------------------------------------
#		=== Scheduler working with ROS' time and threading ===
# ------------------------------------------------------------
class ROSScheduler(Scheduler):
	""" A scheduler supporting multi-threading access and ROS time """
	def __init__(self):
		super(ROSScheduler, self).__init__()
		self.wake_cond = threading.Condition()
		self.running = True
		self.in_timer_count = 0
		def stop_run():
			self.wake_cond.acquire()
			self.running = False
			self.wake_cond.notify()
			self.wake_cond.release()
		rospy.on_shutdown(stop_run)
	
	# Public API, these functions are safe to be called from within a task or from outside
	
	def current_time(self):
		return rospy.Time.now().to_sec()
	
	# Public API, these funtions must be called outside a task
	
	def run(self):
		self.wake_cond.acquire()
		self.step()
		while not rospy.is_shutdown() and self.running and ((self.in_timer_count != 0) or self.cond_waiting or self.ready):
			self.wake_cond.wait()
			self.step()
	
	# Protected implementations, these functions can only be called by functions from this object
	
	def _sleep(self, duration):
		rospy.sleep(duration)
	
	def _set_timer_callback(self, t, f):
		def timer_callback(event):
			self.wake_cond.acquire()
			self.in_timer_count -= 1
			f()
			self.wake_cond.notify()
			self.wake_cond.release()
		self.in_timer_count += 1
		rospy.Timer(rospy.Duration(t - self.current_time()), timer_callback, True)

# ------------------------------------------------------------
#	 === Conditional Variables working with ROS' threading ===
# ------------------------------------------------------------
class ROSConditionVariable(ConditionVariable):
	""" A conditional variable working with ROSScheduler """
	def __init__(self, initval=None):
		super(ROSConditionVariable, self).__init__(initval)
	def __get__(self, obj, objtype):
		return self.val
	def __set__(self, obj, val):
		obj.wake_cond.acquire()
		self.val = val
		self._set_name(type(obj))
		obj._test_conditions(self.myname)
		obj.wake_cond.notify()
		obj.wake_cond.release()
