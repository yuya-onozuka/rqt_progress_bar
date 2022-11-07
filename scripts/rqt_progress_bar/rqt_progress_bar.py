#! /usr/bin/env python
# -*- coding: utf-8 -*-

import roslib
roslib.load_manifest('rqt_progress_bar')
import os
import rospy
import roslib.packages

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from PyQt5.QtGui import QMovie
from PyQt5.QtCore import QByteArray
from std_msgs.msg import Int8, Float32


class RqtProgressBar(Plugin):
    
    def __init__(self, context):

        super(RqtProgressBar, self).__init__(context)

        self.setObjectName('RqtProgressBar')

        self._widget = QWidget()

        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'rqt_progress_bar.ui')
        loadUi(ui_file, self._widget)

        self._widget.setObjectName('RqtProgressBarUi')

        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))  

        context.add_widget(self._widget)
        
        # DEFAULT GIF
        self.gif_file = roslib.packages.get_pkg_dir('rqt_progress_bar') + "/images/test.gif"
        self.movie = QMovie(self.gif_file)  # This is the "wrong" gif
        self.movie.start()
        self._widget.gif.setMovie(self.movie)
        self._widget.gif.hide()

        # DEFAULT GIF
        self.gif_file = roslib.packages.get_pkg_dir('rqt_progress_bar') + "/images/180half_f.gif"
        self.movie = QMovie(self.gif_file)  # This is the "wrong" gif
        self.movie.start()
        self._widget.gif2.setMovie(self.movie)
        self._widget.gif2.hide()



        self.__sub = rospy.Subscriber('percentage', Int8, self.callback)
        self.__sub_current = rospy.Subscriber('bldc_current', Float32, self.current_callback)

    def callback(self, data):
        x="%d" %(data.data)
        self._widget.percentage.setValue(data.data)
        # print(data)
    
    def current_callback(self, data):
        current = data.data
        if current > 2 and current < 3:
            self._widget.gif2.hide()
            self._widget.gif.show()
        elif current > 3:
            self._widget.gif.hide()
            self._widget.gif2.show()
        # print(data)


    def shutdown_plugin(self):
        pass

    def save_settings(self, plugin_settings, instance_settings):
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        pass