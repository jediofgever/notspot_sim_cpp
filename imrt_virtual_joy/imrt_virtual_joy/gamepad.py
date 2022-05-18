#!/usr/bin/env python
# Copyright 2022 NMBU Robotics
#
# Use of this source code is governed by an MIT
# license that can be found at
# https://opensource.org/licenses/MIT.
#
# Author: Lars Grimstad (lars.grimstad@nmbu.no)


from __future__ import print_function
from PyQt5 import QtCore, QtGui, QtWidgets
from imrt_virtual_joy.joystick import Joystick
import rclpy
import rclpy.qos
from sensor_msgs.msg import Joy
import imrt_virtual_joy.resources
import sys


class GamePad(QtWidgets.QMainWindow):

    def __init__(self, parent=None):
        super(GamePad, self).__init__(parent)

        # Autorepeat
        self._autorepeat = True
        autorepeat_rate = 50  # Hz

        # Axes and button members
        self._axes = []
        self._buttons = []

        # Button setup
        button_rows = 2
        button_cols = 3

        # Window setup
        self.setWindowTitle('NMBU Virtual Gamepad')
        self.setWindowIcon(QtGui.QIcon(':/icons/nmbu_gamepad_3.png'))
        self.setFixedSize(400, 195)

        # Logo
        logo_label = QtWidgets.QLabel()
        pixmap = QtGui.QPixmap(':/icons/nmbu_robotics.png')
        logo_label.setPixmap(pixmap)
        logo_layout = QtWidgets.QHBoxLayout()
        logo_layout.addWidget(logo_label, alignment=QtCore.Qt.AlignHCenter)
        # Joysticks
        self._joys = []
        for i in range(2):
            self._joys.append(Joystick())
            self._joys[i].set_id(i)
            self._joys[i].stick_change.connect(self._joy_event)
            self._axes.extend([0.0, 0.0])

        # Normal buttons
        middle_layout = QtWidgets.QVBoxLayout()
        middle_layout.addStretch()
        middle_layout.addLayout(logo_layout)
        self._button_group = QtWidgets.QButtonGroup()
        button_layout = QtWidgets.QGridLayout()

        for i in range(button_rows):
            for j in range(button_cols):
                button = QtWidgets.QPushButton(
                    'B{}'.format(i*button_cols + j + 1))
                button.setFont(QtGui.QFont('Arial', 9))
                self._button_group.addButton(button, i*button_cols + j)
                button_layout.addWidget(button, i, j)
                self._buttons.append(False)

        middle_layout.addLayout(button_layout)
        self._button_group.buttonPressed.connect(self._button_event)
        self._button_group.buttonReleased.connect(self._button_event)

        # Special buttons
        s_button_layout = QtWidgets.QHBoxLayout()

        self._link_button = QtWidgets.QPushButton('Link\nSticks')
        self._link_button.setCheckable(True)
        self._link_button.setFont(QtGui.QFont('Arial', 7))
        self._link_button.setStyleSheet('background-color: #D08770')
        self._buttons.append(False)
        self._link_button.clicked.connect(self._link_event)
        s_button_layout.addWidget(self._link_button)

        self._sticky_button = QtWidgets.QPushButton('Sticky\nSticks')
        self._sticky_button.setCheckable(True)
        self._sticky_button.setFont(QtGui.QFont('Arial', 7))
        self._sticky_button.setStyleSheet('background-color: #D08770')
        self._buttons.append(False)
        self._sticky_button.clicked.connect(self._sticky_axes_event)
        s_button_layout.addWidget(self._sticky_button)

        middle_layout.addLayout(s_button_layout)
        middle_layout.addStretch()

        # Layout
        layout = QtWidgets.QHBoxLayout()
        layout.addStretch()
        layout.addWidget(self._joys[0])
        layout.addLayout(middle_layout)
        layout.addWidget(self._joys[1])
        layout.addStretch()

        self.window = QtWidgets.QWidget()
        self.setCentralWidget(self.window)
        self.window.setLayout(layout)

        # Timer for loop
        if self._autorepeat:
            self._timer = QtCore.QTimer()
            self._timer.timeout.connect(self._loop_event)
            self._timer.setInterval(autorepeat_rate)
            self._timer.start()

        # Royfy
        rclpy.init(args=None)
        node = rclpy.create_node('virtual_gamepad')
        node.get_logger().info('Virtual gamepad started')
        self.joy_publisher = node.create_publisher(
            Joy, 'joy', qos_profile=rclpy.qos.qos_profile_sensor_data)

        # Show window
        self.show()

    def _joy_event(self, up_value, side_value):
        id = self.sender().get_id()
        if self._link_button.isChecked():
            self._axes[0::2] = [up_value for value in self._axes[0::2]]
            self._axes[1::2] = [side_value for value in self._axes[1::2]]
            for joy in self._joys:
                joy.set_value(up_value, side_value)
        else:
            self._axes[id*2] = up_value
            self._axes[id*2 + 1] = side_value
        if not self._autorepeat:
            self._output_data()

    def _button_event(self, button):
        id = self.sender().id(button)
        self._buttons[id] = button.isDown()
        if not self._autorepeat:
            self._output_data()

    def _sticky_axes_event(self, value):
        self._buttons[-1] = value
        for joy in self._joys:
            joy.set_sticky(value)
        if not self._autorepeat:
            self._output_data()

    def _link_event(self, value):
        self._buttons[-2] = value
        if self._sticky_button.isChecked():
            self._sticky_button.click()
        if not self._autorepeat:
            self._output_data()

    def _loop_event(self):
        self._output_data()

    def _output_data(self):
        joy_msg = Joy()
        joy_msg.axes = self._axes
        joy_msg.buttons = self._buttons

        self.joy_publisher.publish(joy_msg)

        sys.stdout.flush()


def main():
    app = QtWidgets.QApplication([])
    app.setStyleSheet(
        """
        QMainWindow{background-color: #ECEFF4; 
                    border: 8px double #4C566A;
                    border-radius: 20px;}
        QPushButton{background-color: #88C0D0;
                    color: #4C566A}
        """)

    gamepad = GamePad()

    app.exec_()
