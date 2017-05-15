#!/usr/bin/env python

import roslib
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
from cv_bridge import CvBridge, CvBridgeError
from math import sqrt
import numpy as np

import PySide
import cv2
from PySide.QtCore import *
from PySide.QtGui import *
import sys
import copy
import time
import platform

from ui_main import Ui_MainWindow

__version__ = '0.0.2'
PI = 3.14159265359


class GUI(QMainWindow, Ui_MainWindow):
    def __init__(self, parent=None):
        super(GUI, self).__init__(parent)
        self.setupUi(self)

        # All buttons
        self.startWhiteButton.clicked.connect(self.startWhiteButton_callback)
        self.startBlackButton.clicked.connect(self.startBlackButton_callback)
        self.stopButton.clicked.connect(self.stopButton_callback)
        self.connectButton.clicked.connect(self.connectButton_callback)
        self.resetButton.clicked.connect(self.resetButton_callback)
        self.enableMotorsButton.clicked.connect(self.enableMotorsButton_callback)
        self.disableMotorsButton.clicked.connect(self.disableMotorsButton_callback)
        self.initWhiteButton.clicked.connect(self.initWhiteButton_callback)
        self.initBlackButton.clicked.connect(self.initBlackButton_callback)

        # All subscriptions
        self.camera_sig_sub = rospy.Subscriber('/checkers/camera_sig', String, self.camera_sig_sub_callback, queue_size=50)
        self.ai_sig_sub = rospy.Subscriber('/checkers/ai_sig', String, self.ai_sig_sub_callback, queue_size=50)
        self.robot_sig_sub = rospy.Subscriber('/checkers/robot_sig', String, self.robot_sig_sub_callback, queue_size=50)
        self.image_msg_sub = rospy.Subscriber('/checkers/image_msg', Image, self.image_msg_sub_callback, queue_size=50)
        self.moves_msg_sub = rospy.Subscriber('/checkers/moves_msg', String, self.moves_msg_sub_callback, queue_size=50)
        self.robot_state_msg_sub = rospy.Subscriber('/checkers/robot_state_msg', Float64MultiArray, self.robot_state_msg_sub_callback, queue_size=50)

        # All publishers
        self.camera_sig_pub = rospy.Publisher('/checkers/camera_sig', String, queue_size=50)
        self.ai_sig_pub = rospy.Publisher('/checkers/ai_sig', String, queue_size=50)
        self.robot_sig_pub = rospy.Publisher('/checkers/robot_sig', String, queue_size=50)
        self.robot_action_msg_pub = rospy.Publisher('/checkers/robot_action_msg', Float64MultiArray, queue_size=50)

        # All helpers
        self.count = 0
        self.moving = 0  # if the robot is moving
        self.white = 1  # if the board is turned to the robot like he is white
        self.flags = {
            "gripper_open": 1,
            "gripper_close": 2,
            "to_point": 4,
            "mid_point": 8,
            "wait": 16,
        }
        # Physical board description, and constants
        self.wait_from_init = 5
        self.wait_p2mid = 3
        self.wait_mid2p = 3
        self.init_pos = (0, 300)

        self.board_h = 50.
        self.board_w = 195.
        self.board_x_dr = 370.
        self.board_y_dr = 90.
        self.board_x_ul = 125.
        self.board_y_ul = -102.
        self.board_v_x = (1 / 14. * (+(self.board_x_ul - self.board_x_dr) - (self.board_y_ul - self.board_y_dr)),
                          1 / 14. * (+(self.board_x_ul - self.board_x_dr) + (self.board_y_ul - self.board_y_dr)))
        self.board_v_y = (1 / 14. * (+(self.board_x_ul - self.board_x_dr) + (self.board_y_ul - self.board_y_dr)),
                          1 / 14. * (-(self.board_x_ul - self.board_x_dr) + (self.board_y_ul - self.board_y_dr)))
        self.board_xy = [(self.board_x_dr + x * self.board_v_x[0] + y * self.board_v_y[0],
                          self.board_y_dr + x * self.board_v_x[1] + y * self.board_v_y[1])
                         for y, x in np.ndindex((8, 8))]
        print self.board_xy

        time.sleep(1)  # to start node for publishing

        # print self.from_numbering_to_xy(1), self.from_numbering_to_xy(5)
        self.moves_msg_sub_callback(String("1-10"))

        frame = self.imageGraphics
        label_Image = QLabel(frame)
        image_path = '/home/rijad/Pictures/Webcam/image1.jpg'  # path to your image file
        image_profile = QImage(image_path)  # QImage object
        image_profile = image_profile.scaled(400, 400, aspectRatioMode=Qt.KeepAspectRatio,
                                             transformMode=Qt.SmoothTransformation)  # To scale image for example and keep its Aspect Ration
        label_Image.setPixmap(QPixmap.fromImage(image_profile))


    def initWhiteButton_callback(self):
        self.white = 1
        self.ai_sig_pub.publish("AI_INIT_WHITE")
        # todo move to init pos

    def initBlackButton_callback(self):
        self.white = 0
        self.ai_sig_pub.publish("AI_INIT_BLACK")
        # todo move to init pos

    def startWhiteButton_callback(self):
        self.ai_sig_pub.publish("AI_GO_WHITE")
        # todo start counter

    def startBlackButton_callback(self):
        self.ai_sig_pub.publish("AI_GO_BLACK")
        # todo start counter

    def stopButton_callback(self):
        self.robot_sig_pub.publish("ROBOT_STOP")

    def connectButton_callback(self):
        self.robot_sig_pub.publish("ROBOT_CONNECT")

    def enableMotorsButton_callback(self):
        self.robot_sig_pub.publish("ROBOT_ENABLE")

    def disableMotorsButton_callback(self):
        self.robot_sig_pub.publish("ROBOT_DISABLE")

    def resetButton_callback(self):
        self.robot_sig_pub.publish("ROBOT_STOP")
        self.robot_sig_pub.publish("ROBOT_RESET")

    def camera_sig_sub_callback(self, msg):
        if msg.data == "CAMERA_GO":
            pass
        elif msg.data == "CAMERA_FIN":
            # todo approve ai call
            pass
        else:
            print "Ilegal CAMERA signal"

    def ai_sig_sub_callback(self, msg):
        if msg.data == "AI_GO":
            pass
        elif msg.data == "AI_FIN":
            # todo approve moves generation
            pass
        else:
            print "Illegal AI signal"

    def robot_sig_sub_callback(self, msg):
        if msg.data == "ROBOT_GO":
            pass
        elif msg.data == "ROBOT_FIN":
            self.moving = 0
            # todo stop robot counter, start human coutner
            # todo myb something more
        else:
            pass

    # x
    # y
    # z
    # fi
    # flags
    #     gripper_open = flags & 1;
    #     gripper_close = flags & 2;
    #     to_point = flags & 4;
    #     mid_point = flags & 8;
    #     wait = flags & 16;
    # orient
    # wait

    def moves_msg_sub_callback(self, msg):
        # the best part
        moves = msg.data.split(';')
        self.moving = 1
        all_actions = []
        if moves[0].find('-') != -1:
            a, b = map(int, moves[0].split('-'))
            a_xy = self.from_numbering_to_xy(a)
            b_xy = self.from_numbering_to_xy(b)
            p = Float64MultiArray()
            p.layout.dim.append(MultiArrayDimension())
            p.layout.dim[0].size = 7
            p.layout.dim[0].stride = 1
            p.layout.dim[0].label = "AAA"
            # open the gripper
            p.data = [0, 0, 0, 0, self.flags['gripper_open'], 0, 0]
            all_actions.append(copy.copy(p))
            # go to figure
            p.data = [a_xy[0], a_xy[1], 100, PI, self.flags['mid_point'], 0, 7]
            all_actions.append(copy.copy(p))
            p.data = [a_xy[0], a_xy[1], 50, PI, self.flags['to_point'], 0, 5]
            all_actions.append(copy.copy(p))
            # close gripper
            p.data = [0, 0, 0, 0, self.flags['gripper_close'], 0, 0]
            all_actions.append(copy.copy(p))
            p.data = [0, 0, 0, 0, self.flags['wait'], 0, 1]
            all_actions.append(copy.copy(p))
            # go to end-location
            p.data = [(a_xy[0]+b_xy[0])/2, (a_xy[1]+b_xy[1])/2, 70, PI, self.flags['mid_point'], 0, 2]
            all_actions.append(copy.copy(p))
            p.data = [b_xy[0], b_xy[1], 50, PI, self.flags['to_point'], 0, 2]
            all_actions.append(copy.copy(p))
            # open gripper
            p.data = [0, 0, 0, 0, self.flags['gripper_open'], 0, 0]
            all_actions.append(copy.copy(p))
            p.data = [0, 0, 0, 0, self.flags['wait'], 0, 1]
            all_actions.append(copy.copy(p))
            # back to init pos
            p.data = [b_xy[0], b_xy[1], 100, PI, self.flags['mid_point'], 0, 5]
            all_actions.append(copy.copy(p))
            p.data = [self.init_pos[0], self.init_pos[1], 150, PI, self.flags['to_point'], 0, 7]
            all_actions.append(copy.copy(p))
            # publish them all
            for x in all_actions:
                print str(x)
                self.robot_action_msg_pub.publish(x)
                print "AAA"
        else:

            pass

    def image_msg_sub_callback(self, msg):
        pass

    def robot_state_msg_sub_callback(self, msg):
        pass

    def from_numbering_to_xy(self, num):
        if self.white:
            return self.board_xy[num * 2 - (1 if (num / 4) % 2 == 0 else 2)]
        else:
            return self.board_xy[63 - (num * 2 - (1 if (num / 4) % 2 == 0 else 2))]
        pass



if __name__ == '__main__':
    # Initialize the node
    rospy.init_node('checkers_main')

    # Initialize Qt
    app = QApplication(sys.argv)

    gui = GUI()
    gui.show()

    sys.exit(app.exec_())
