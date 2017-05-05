#!/usr/bin/env python
# about.py - display about box with info on platform etc.

import sys
import platform

import PySide
from PySide.QtGui import QApplication, QMainWindow, QTextEdit, QPushButton, QMessageBox

import cv2
import rospy
from std_msgs.msg import String

from ui_main import Ui_MainWindow

__version__ = '0.0.1'


class MainWindow(QMainWindow, Ui_MainWindow):

    def __init__(self, parent=None):
        super(MainWindow, self).__init__(parent)
        self.setupUi(self)
        self.startButton.clicked.connect(self.about)

    def about(self):
        # cam = cv2.VideoCapture(0)
        # ret, frame2 = cam.read()
        frame2 = cv2.imread("/home/rijad/Pictures/Webcam/image1.jpg", )
        cv2.imshow('frame', frame2)
        # cam.release()
        '''Popup a box with about message.'''
        QMessageBox.about(self, "About PySide, Platform and the like",
        """<b>Platform Details</b> v {}
        <p>Copyright &copy; 2010 Joe Bloggs.
        All rights reserved in accordance with
        GPL v2 or later - NO WARRANTIES!
        <p>This application can be used for
        displaying platform details.
        <p>Python {} - PySide version {} - Qt version {} on {}""".format(__version__,
        platform.python_version(), PySide.__version__, PySide.QtCore.__version__,
        platform.system()))



if __name__ == '__main__':
    app = QApplication(sys.argv)
    frame = MainWindow()

    frame.show()
    sys.exit(app.exec_())
    # pub = rospy.Publisher('chatter', String, queue_size=10)
    # rospy.init_node('talker', anonymous=True)
    # rate = rospy.Rate(10)  # 10hz
    # while not rospy.is_shutdown():
    #     hello_str = "hello world %s" % rospy.get_time()
    #     rospy.loginfo(hello_str)
    #     pub.publish(hello_str)
    #     rate.sleep()

