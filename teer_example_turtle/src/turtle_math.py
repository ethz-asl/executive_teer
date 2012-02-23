import math
import numpy as np
from turtlesim.msg import Velocity
from turtlesim.msg import Pose

def unit(v):
	''' Return unitary vector of direction v '''
	n = np.linalg.norm(v)
	if n != 0:
		return v / n
	else:
		return v

def perp(v):
	''' Return the perpendiculary vector to v '''
	return np.array((-v[1], v[0]))

def dist(current_pose, dest_pos):
	''' Return distance between current position and a target '''
	if hasattr(dest_pos, 'x') and hasattr(dest_pos, 'y'):
		dx = current_pose.x - dest_pos.x
		dy = current_pose.y - dest_pos.y
	else:
		dx = current_pose.x - dest_pos[0]
		dy = current_pose.y - dest_pos[1]
	return math.hypot(dx, dy)

def control_command(pose, dest_pos, speed = 1.0):
	''' Compute velocity given current pose and target '''
	p = np.array((pose.x, pose.y))
	o = np.array((math.cos(pose.theta), math.sin(pose.theta)))
	t_g = np.array(dest_pos)
	d_t = unit(t_g - p)
	v_forward = np.dot(d_t, o)
	v_rot = np.dot(d_t, perp(o))
	return Velocity(v_forward * speed, v_rot * speed * 2)

def orbit_command(pose, center, speed = 1.0, d = 2.5):
	''' Give a command such as orbiting around center '''
	c_d = dist(pose, center)
	d_d = c_d - d
	turn = max(0, d_d)
	return Velocity(speed, -turn*turn*0.1)

def angle_diff(a, b):
	pi = math.pi
	diff = a - b
	while diff > pi:
		diff -= 2*pi
	while diff < -pi:
		diff += 2*pi
	return diff