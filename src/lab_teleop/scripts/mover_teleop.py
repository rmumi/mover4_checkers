#!/usr/bin/env python
import rospy
from math import sin, cos
from sensor_msgs.msg import Joy, JointState
from std_msgs.msg import String

JOY_STATE = None
JOINT_STATE = JointState()
JOINT_STATE.position.extend([0.0] * 4)

def callback(v):
	global JOY_STATE
	JOY_STATE = v

def listener():
	rospy.Subscriber('joy', Joy, callback)

def apply_deadzone(v):
	if (v < 0):
		return min(0, v + 0.2)
	else:
		return max(0, v - 0.2)

def transform_axes(joints, axes):
	lz0 = 130.0
	lz1 = 62.5
	lz2 = 190.0
	lz3 = 220.0
	lz4 = 48.0
	s0 = sin(joints[0])
	c0 = cos(joints[0])
	s1 = sin(joints[1])
	c1 = cos(joints[1])
	s12 = sin(joints[1] + joints[2])
	c12 = cos(joints[1] + joints[2])
	s123 = sin(joints[1] + joints[2] + joints[3])
	c123 = cos(joints[1] + joints[2] + joints[3])
	x = axes[0]
	y = axes[1]
	z = axes[2]
	old_sum = abs(x) + abs(y) + abs(z)

	new_axes = [
		(lz1 + c1 * lz2 + c12 * lz3 + c123 * lz4) * (c0 * x - s0 * z),
		(s1 * lz2 + s12 * lz3 + s123 * lz4) * (-s0 * x - c0 * z) - (c1 * lz2 + c12 * lz3 + c123 * lz4) * y,
		(s12 * lz3 + s123 * lz4) * (-s0 * x - c0 * z) - (c12 * lz3 + c123 * lz4) * y,
		(s123 * lz4) * (-s0 * x - c0 * z) - (c123 * lz4) * y
	]
	multiplier = old_sum / max(0.0001, sum([abs(v) for v in new_axes]))
	return [v * multiplier for v in new_axes]

def dk(joints):
	lz0 = 130.0
	lz1 = 62.5
	lz2 = 190.0
	lz3 = 220.0
	lz4 = 48.0
	s0 = sin(joints[0])
	c0 = cos(joints[0])
	s1 = sin(joints[1])
	c1 = cos(joints[1])
	s12 = sin(joints[1] + joints[2])
	c12 = cos(joints[1] + joints[2])
	s123 = sin(joints[1] + joints[2] + joints[3])
	c123 = cos(joints[1] + joints[2] + joints[3])
	A = lz1 + c1 * lz2 + c12 * lz3 + c123 * lz4
	B = lz1 + s1 * lz2 + s12 * lz3 + s123 * lz4
	return [s0 * A, -B, c0 * A + lz0]

def talker():
	global JOY_STATE
	global JOINT_STATE
	pub = rospy.Publisher('/CPRMoverJointPos', JointState, queue_size=10)
	pub_cmd = rospy.Publisher('/CPRMoverCommands', String, queue_size=10)
	rate = rospy.Rate(20)
	while not rospy.is_shutdown():
		if JOY_STATE is None:
			rate.sleep()
			continue
		axes = JOY_STATE.axes
		axes = [-apply_deadzone(axes[i]) for i in [0, 1, 4, 3]] 
		if JOY_STATE.buttons[3]:
			axes = transform_axes(JOINT_STATE.position, axes)
		joints = zip(JOINT_STATE.position, axes)
		JOINT_STATE = JointState()
		JOINT_STATE.position = [max(-1.0, min(1.5, angle + speed * 0.02)) for angle, speed in joints]
		for i in range(1,3):
			JOINT_STATE.position[i] = max(0.0, JOINT_STATE.position[i])
		buttonsa = JOY_STATE.buttons
		print buttonsa
		if buttonsa[4]:
			pub_cmd.publish('GripperOpen')
		if buttonsa[5]:
			pub_cmd.publish('GripperClose')
		print(dk(JOINT_STATE.position))
		pub.publish(JOINT_STATE)
		rate.sleep()



if __name__ == '__main__':
	rospy.init_node('mover_teleop')
	listener()
	talker()
