#!/usr/bin/env python

import sys
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge

from PyQt4 import QtGui
from PyQt4.QtGui import QLabel, QVBoxLayout, QHBoxLayout, QSlider, QPushButton
from PyQt4.QtCore import Qt

class OperatorGui(QtGui.QWidget):
    def __init__(self):
        super(OperatorGui, self).__init__()
        self.setObjectName('OperatorGui')

        # Listen to joystick commands and publish them as twists
        # if the emergency operator mode is on.
        self.isOperator = False
        self.velPub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.imgSub = rospy.Subscriber("/camera/color/image_raw", Image, self.updateStreamCallback)
        rospy.init_node('station_gui')

        self.layout = QHBoxLayout()

        self.setOperatorButton = QPushButton()
        self.setOperatorButton.setText("Set Emergency Controls ON")
        self.setOperatorButton.clicked.connect(self.toggleMode)

        self.imageStream = QLabel()
        self.bridge = CvBridge()

        self.layout.addWidget(self.imageStream)
        self.layout.addWidget(self.setOperatorButton)
        self.setLayout(self.layout)

    # When the setOperatorButton is pressed, change the status
    # of the emergency operator controls.
    def toggleMode(self):
        self.isOperator = not self.isOperator

        if self.isOperator:
            rospy.loginfo("Emergency controls on")
            self.setOperatorButton.setText("Emergency Controls are ON")
        else:
            rospy.loginfo("Emergency controls off")
            self.setOperatorButton.setText("Emergency Controls are OFF")

    # Update the image stream whenever a new ROS image comes in.
    def updateStreamCallback(self, img_msg):
        cv_img = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='passthrough')
        image = QtGui.QImage(cv_img.data, cv_img.shape[1]. cv_img.shape[0])
        self.imageStream.setPixmap(QtGui.QPixmap.fromImage(image))


if __name__ == "__main__":
    app = QtGui.QApplication(sys.argv)
    opShow = OperatorGui()
    opShow.show()
    sys.exit(app.exec_())

