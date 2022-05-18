#!/usr/bin/env python

# Copyright 2022 NMBU Robotics

# Use of this source code is governed by an MIT
# license that can be found at
# https://opensource.org/licenses/MIT.
#
# Author: Lars Grimstad (lars.grimstad@nmbu.no)


from PyQt5 import QtCore, QtGui, QtWidgets

class Joystick(QtWidgets.QWidget):

    stick_change = QtCore.pyqtSignal(float, float)

    def __init__(self, *args, **kwargs):
        super(Joystick, self).__init__(*args, **kwargs)

        self.setAttribute(QtCore.Qt.WA_AcceptTouchEvents, True)
        self.setAttribute(QtCore.Qt.WA_MouseTracking, True)
        self.setSizePolicy(
            QtWidgets.QSizePolicy.Fixed,
            QtWidgets.QSizePolicy.Fixed
        )
        
        self._stick_radius = 20
        self._bounds_radius = 40
        self._max_stick_disp = self._bounds_radius - self._stick_radius/2.
        self._snap_stick_disp = self._max_stick_disp/4.
        self._value_x = 0
        self._value_y = 0
        self._touch_id = None
        self._touch_pos = None
        self._id = None
        self._stick_pos = None
        self._touch_grabbed = False
        self._mouse_grabbed = False
        self._sticky = False


    def sizeHint(self):
        return QtCore.QSize(120,120)


    def paintEvent(self, e):
        painter = QtGui.QPainter(self)
        painter.setBrush(QtGui.QColor('#D8DEE9'))
        painter.drawEllipse(self._get_bounds_rect())
        painter.setBrush(QtGui.QColor('#2E3440'))
        painter.drawEllipse(self._get_stick_rect())


    def event(self, event):
        event_caught = False
        touch_event = event.type() == QtCore.QEvent.TouchBegin
        touch_event = touch_event or event.type() == QtCore.QEvent.TouchEnd
        touch_event = touch_event or event.type() == QtCore.QEvent.TouchUpdate
        
        if touch_event:
            for point in event.touchPoints():
                if point.state() == QtCore.Qt.TouchPointReleased:
                    if point.id() == self._touch_id:
                        self._touch_grabbed = False
                        event_caught = True

                elif point.state() == QtCore.Qt.TouchPointPressed:
                    if self._get_stick_rect().contains(point.pos()):
                        self._touch_grabbed = True
                        self._touch_pos = point.pos()
                        self._touch_id = point.id()
                        event_caught = True

                elif self._touch_id == point.id():
                    self._touch_pos = point.pos()
                    event_caught = True

        elif event.type() == QtCore.QEvent.MouseButtonPress:
            if self._get_stick_rect().contains(event.pos()):
                self._mouse_grabbed = True
                self._touch_pos = event.pos()
                event_caught = True

        elif event.type() == QtCore.QEvent.MouseMove:
            if self._mouse_grabbed:
                self._touch_pos = event.pos()
                event_caught = True

        elif event.type() == QtCore.QEvent.MouseButtonRelease:
            if self._mouse_grabbed:
                self._mouse_grabbed = False
                event_caught = True

        if event_caught:
            self._update_stick()
            self.stick_change.emit(self._value_y, self._value_x)
            return True
        else:
            return super(Joystick, self).event(event)


    def set_id(self, id):
        self._id = id


    def set_sticky(self, value):
        self._sticky = value
        self._update_stick()
        self.stick_change.emit(self._value_y, self._value_x)


    def set_value(self, value_y, value_x):
        self._stick_pos = QtCore.QPointF(value_x * 30.+self.width()/2., -value_y * 30.+self.height()/2.)
        self._value_x = value_x
        self._value_y = value_y
        self._update_stick(visuals_only=True)


    def get_id(self):
        return self._id


    def get_values(self):
        return self._value_y, self._value_x


    def _get_stick_rect(self):
        if self._stick_pos is None:
            self._stick_pos = QtCore.QPointF(self.width()/2., self.height()/2.)
        return QtCore.QRectF(-self._stick_radius, -self._stick_radius, 2*self._stick_radius, 2*self._stick_radius).translated(self._stick_pos)


    def _get_bounds_rect(self):
        side_len = 2 * self._bounds_radius
        return QtCore.QRectF(-side_len/2, -side_len/2, side_len, side_len).translated(self.width()/2., self.height()/2.)


    def _is_grabbed(self):
        return self._mouse_grabbed or self._touch_grabbed


    def _update_stick(self, visuals_only=False):
        stick_offset = QtCore.QLineF(QtCore.QPointF(self.width()/2., self.height()/2.), self._stick_pos).length()
        if visuals_only:
            pass
        elif self._is_grabbed():
            dir_line = QtCore.QLineF(QtCore.QPointF(self.width()/2., self.height()/2.), self._touch_pos)
            if dir_line.length() > self._max_stick_disp:
                dir_line.setLength(self._max_stick_disp)
            self._value_x = (dir_line.p2().x() - self.width()/2.) / self._max_stick_disp
            self._value_y = (-dir_line.p2().y() + self.height()/2.) / self._max_stick_disp
            self._stick_pos = dir_line.p2()
        elif not self._sticky or stick_offset < self._snap_stick_disp:
            self._value_x = 0
            self._value_y = 0
            self._stick_pos = QtCore.QPointF(self.width()/2., self.height()/2.)
        self.update()


