#! /usr/bin/env python
# -*- coding: utf-8 -*-

import roslib
roslib.load_manifest('rqt_progress_bar')
import os
import rospy
import roslib.packages
import numpy as np

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
        self.gif_file = roslib.packages.get_pkg_dir('rqt_progress_bar') + "/images/cool.gif" # 使用するgifファイルを指定
        self.movie = QMovie(self.gif_file)
        self.movie.start()
        self._widget.gif.setMovie(self.movie)
        self._widget.gif.hide()

        # DEFAULT GIF
        self.gif_file = roslib.packages.get_pkg_dir('rqt_progress_bar') + "/images/good.gif" #使用するgifファイルを指定
        self.movie = QMovie(self.gif_file)
        self.movie.start()
        self._widget.gif2.setMovie(self.movie)
        self._widget.gif2.hide()

        # DEFAULT GIF
        self.gif_file = roslib.packages.get_pkg_dir('rqt_progress_bar') + "/images/great.gif" #使用するgifファイルを指定
        self.movie = QMovie(self.gif_file)
        self.movie.start()
        self._widget.gif3.setMovie(self.movie)
        self._widget.gif3.hide()


        self.__sub = rospy.Subscriber('percentage', Int8, self.callback)
        self.__sub_current = rospy.Subscriber('bldc_current', Float32, self.current_callback)

        self.last_time = rospy.Time.now().to_sec()
        self.max_current = 0.0
        self.current_data_array = np.zeros(10) # 最大値を探索するデータ数指定（電流値が10Hzで10個のデータを取るので、1秒間のデータを保持するイメージ）

    def callback(self, data):
        x="%d" %(data.data)
        self._widget.percentage.setValue(data.data)
        # print(data)
    
    def current_callback(self, data):
        current = data.data
        self.current_data_array[len(self.current_data_array)-1] = current
        for i in range(len(self.current_data_array)-1):
            self.current_data_array[i] = self.current_data_array[i+1]

        max_current = self.current_data_array.max()
            
        # rospy.loginfo(self.current_data_array)

        gif_display_time = 1.0 # gifを表示する時間 [s]
        if max_current < 1:
            self.hide_all_gif()
        elif max_current >= 1 and max_current < 2:
            self._widget.gif2.hide()
            self._widget.gif3.hide()
            self._widget.gif.show()
            # rospy.sleep(gif_display_time)
        elif max_current >= 2 and max_current < 3:
            self._widget.gif.hide()
            self._widget.gif3.hide()
            self._widget.gif2.show()
            # rospy.sleep(gif_display_time)
        elif max_current >= 3:
            self._widget.gif.hide()
            self._widget.gif2.hide()
            self._widget.gif3.show()
            # rospy.sleep(gif_display_time)

    def hide_all_gif(self):
        self._widget.gif.hide()
        self._widget.gif2.hide()
        self._widget.gif3.hide()

    def shutdown_plugin(self):
        pass

    def save_settings(self, plugin_settings, instance_settings):
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        pass