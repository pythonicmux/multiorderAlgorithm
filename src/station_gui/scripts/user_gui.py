#!/usr/bin/env python

import sys
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from multiorder_alg.msg import order

import cv2
from cv_bridge import CvBridge

from PyQt4 import QtGui
from PyQt4.QtGui import QLabel, QVBoxLayout, QHBoxLayout, QLineEdit, QPushButton
from PyQt4.QtCore import Qt

class UserGui(QtGui.QWidget):
    def __init__(self):
        super(UserGui, self).__init__()
        self.setObjectName('UserGui')

        self.currentOrderID = 0
        self.orderPub = rospy.Publisher("/incoming_orders", order, queue_size=10)
        rospy.init_node('user_gui')

        self.layout = QHBoxLayout()

        self.sendOrderButton = QPushButton()
        self.sendOrderButton.setText("Send Order")
        self.sendOrderButton.clicked.connect(self.sendOrder)

        self.SText = QLineEdit()
        self.DText = QLineEdit()
        self.weightText = QLineEdit()

        self.layout.addWidget(self.SText)
        self.layout.addWidget(self.DText)
        self.layout.addWidget(self.weightText)
        self.layout.addWidget(self.sendOrderButton)
        self.setLayout(self.layout)

    def sendOrder(self):
        order_msg = order()
        self.currentOrderID += 1
        order_msg.id = self.currentOrderID
        order_msg.startNode = int(self.SText.text())
        order_msg.destNode = int(self.DText.text())
        order_msg.weight = float(self.weightText.text())
        self.orderPub.publish(order_msg)


if __name__ == "__main__":
    app = QtGui.QApplication(sys.argv)
    userShow = UserGui()
    userShow.show()
    sys.exit(app.exec_())

