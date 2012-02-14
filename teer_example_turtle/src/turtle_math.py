import math
import numpy as np
from turtlesim.msg import Velocity

def unit(v):
	''' Return unitary vector of direction v '''
	return v / np.linalg.norm(v)

def dist(current_pose, dest_pos):
	''' Return distance between current position and a target '''
	dx = current_pose.x - dest_pos[0]
	dy = current_pose.y - dest_pos[1]
	return math.hypot(dx, dy)

def control_command(pose, dest_pos):
	''' Compute velocity given current pose and target '''
	p = np.array((pose.x, pose.y))
	o = np.array((math.cos(pose.theta), math.sin(pose.theta)))
	t_g = np.array(dest_pos)
	d_t = unit(t_g - p)
	v_forward = np.dot(d_t, o)
	v_rot = np.dot(d_t, np.array((-o[1], o[0])))
	return Velocity(v_forward, v_rot)
