#!/usr/bin/env python
import rospy
from math import sin, cos
from sensor_msgs.msg import Joy, JointState

JOY_STATE = None
JOINT_STATE = JointState()
JOINT_STATE.position.extend([0.0] * 6)

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

def talker():
	global JOY_STATE
	global JOINT_STATE
	pub = rospy.Publisher('joints', JointState, queue_size=10)
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		if JOY_STATE is None:
			rate.sleep()
			continue
		axes = JOY_STATE.axes
		axes = [apply_deadzone(axes[i]) for i in [0, 1]]
		if JOY_STATE.buttons[0]:
			axes = axes + [0, 0, 0, 0]
		elif JOY_STATE.buttons[2]:
			axes = [0, 0] + axes + [0, 0]
		elif JOY_STATE.buttons[3]:
			axes = [0, 0, 0, 0] + axes
		else:
			axes = [0, 0, 0, 0, 0, 0]
		joints = zip(JOINT_STATE.position, axes)
		JOINT_STATE = JointState()
		JOINT_STATE.position = [max(-1.0, min(1.0, angle + speed * 0.02)) for angle, speed in joints]
		pub.publish(JOINT_STATE)
		rate.sleep()



if __name__ == '__main__':
	rospy.init_node('mover_teleop')
	listener()
	talker()
