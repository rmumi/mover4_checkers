#!/usr/bin/env python

import roslib
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray
from cv_bridge import CvBridge, CvBridgeError
from math import sqrt
import numpy as np

import PySide
import cv2
from PySide.QtCore import *
from PySide.QtGui import *
import sys
import platform

from ui_main import Ui_MainWindow

__version__ = '0.0.2'


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
        self.robot_point_msg_pub = rospy.Publisher('/checkers/robot_point_msg', Float64MultiArray, queue_size=50)

        # All helpers
        self.count = 0
        self.moving = 0

        # Physical board description
        self.board_h = 20.
        self.board_w = 195.
        self.board_x_dr = 250.
        self.board_y_dr = 250.
        self.board_x_ul = 250. + 195.
        self.board_y_ul = 250. - 195.
        self.board_v_x = (1 / 14. * (+(self.board_x_ul - self.board_x_dr) - (self.board_y_ul - self.board_y_dr)),
                          1 / 14. * (+(self.board_x_ul - self.board_x_dr) + (self.board_y_ul - self.board_y_dr)))
        self.board_v_y = (1 / 14. * (+(self.board_x_ul - self.board_x_dr) + (self.board_y_ul - self.board_y_dr)),
                          1 / 14. * (-(self.board_x_ul - self.board_x_dr) + (self.board_y_ul - self.board_y_dr)))
        self.board_xy = [(self.board_x_dr + x * self.board_v_x[0] + y * self.board_v_y[0],
                          self.board_y_dr + x * self.board_v_x[1] + y * self.board_v_y[1])
                         for y, x in np.ndindex((8, 8))]
        print self.board_xy




        frame = self.imageGraphics
        label_Image = QLabel(frame)
        image_path = '/home/rijad/Pictures/Webcam/image1.jpg'  # path to your image file
        image_profile = QImage(image_path)  # QImage object
        image_profile = image_profile.scaled(400, 400, aspectRatioMode=Qt.KeepAspectRatio,
                                             transformMode=Qt.SmoothTransformation)  # To scale image for example and keep its Aspect Ration
        label_Image.setPixmap(QPixmap.fromImage(image_profile))


    def initWhiteButton_callback(self):
        self.ai_sig_pub.publish("AI_INIT_WHITE")
        pass

    def initBlackButton_callback(self):
        self.ai_sig_pub.publish("AI_INIT_BLACK")
        pass

    def startWhiteButton_callback(self):
        self.ai_sig_pub.publish("AI_GO_WHITE")
        pass

    def startBlackButton_callback(self):
        self.ai_sig_pub.publish("AI_GO_BLACK")
        pass

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
            print "Ilegal AI signal"

    def robot_sig_sub_callback(self, msg):
        if msg.data == "ROBOT_GO":
            pass
        elif msg.data == "ROBOT_FIN":
            self.moving = 0
            # todo myb something more
        else:
            pass

    def moves_msg_sub_callback(self, msg):
        # the best part
        pass

    def image_msg_sub_callback(self, msg):
        pass

    def robot_state_msg_sub_callback(self, msg):
        pass



if __name__ == '__main__':
    # Initialize the node
    rospy.init_node('checkers_main')

    # Initialize Qt
    app = QApplication(sys.argv)

    gui = GUI()
    gui.show()

    sys.exit(app.exec_())
