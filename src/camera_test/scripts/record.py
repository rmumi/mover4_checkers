# from __future__ import print_function
import rospy
from std_msgs.msg import String
import copy
from collections import deque
import sys
from time import sleep

record = False

f = open('recordings/' + sys.argv[2], 'w')


def robot_callback(msg):
    global record
    if msg.data[6] == 'G':
        record = True
    elif msg.data[6] == 'F':
        record = False
        print "Finished"


def moment_callback(msg):
    if record:
        f.write(msg.data + '\n')

if __name__ == "__main__":
    rospy.init_node("checkers_record")

    rospy.Subscriber("/checkers_moment", String, moment_callback, queue_size=50)

    rospy.Subscriber("/checkers/robot_sig", String, robot_callback, queue_size=50)

    pub_sig = rospy.Publisher("/checkers/robot_sig", String, queue_size=50)

    sleep(0.4)

    pub_sig.publish("ROBOT_GO" + str(sys.argv[1]))

    rospy.spin()

    f.close()
