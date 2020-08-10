#! /usr/bin/env python

import rospy
import sys
import os
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (QApplication, QCheckBox, QGridLayout, QGroupBox, QMenu, QPushButton, QRadioButton, QVBoxLayout, QWidget, QSlider, QLabel)
from wisc_msgs.msg import DCPoseGoals
from std_msgs.msg import Float32
from RelaxedIK.Utils.yaml_utils import get_relaxedIK_yaml_obj

class SliderWidget(object):
    def __init__(self,parent,index,name,min,max):
        self.parent = parent
        self.index =  index
        self.name = name
        self.min = min
        self.max = max
        self.label = QLabel(str(min))

        self.groupBox = QGroupBox(name)

        self.slider = QSlider(Qt.Horizontal)

        self.slider.setFocusPolicy(Qt.StrongFocus)
        self.slider.setTickInterval(10)
        self.slider.setSingleStep(1)


        vbox = QVBoxLayout()
        vbox.addWidget(self.slider)
        vbox.addWidget(self.label)
        vbox.addStretch(1)
        self.groupBox.setLayout(vbox)

        self.slider.valueChanged.connect(self.value_changed)

    def value_changed(self):
        range = self.max - self.min
        scale = range / 100
        slider_value = (self.slider.value() * scale) - self.min
        self.label.setText(str(slider_value))
        self.parent.values[self.index] = slider_value
        dcpg = DCPoseGoals()
        for v in self.parent.values:
            dcpg.dc_values.append(v)
        dcpg.header.seq = self.parent.seq
        dcpg.header.stamp = rospy.Time.now()
        self.parent.seq += 1

        self.parent.dc_pose_goal_pub.publish(dcpg)

class Window(QWidget):
    def __init__(self, parent=None):
        super(Window, self).__init__(parent)
        self.sliders = []
        self.labels = []
        self.rospy_node = rospy.init_node('direct_control_driver')
        self.dc_pose_goal_pub = rospy.Publisher('/relaxed_ik/dc_pose_goals', DCPoseGoals, queue_size=3)
        self.seq = 0
        path_to_src = os.path.dirname(__file__)
        y = get_relaxedIK_yaml_obj(path_to_src)

        grid = QGridLayout()

        joint_weights = y["dc_joint_weight"]
        joint_ordering = y["joint_ordering"]
        joint_limits = y["joint_limits"]


        self.values = [0 for joint in joint_ordering]
        slider_count = 0
        for i, joint in enumerate(joint_ordering):
            if joint_weights[i] > 0:
                slider_widget = SliderWidget(self, i, joint, joint_limits[i][0], joint_limits[i][1])
                self.sliders.append(slider_widget)
                grid.addWidget(slider_widget.groupBox, slider_count, 0)
                slider_count += 1

        self.setLayout(grid)

        self.setWindowTitle("Direct Control Joints")
        self.resize(400, 0)

if __name__ == "__main__":
    rospy.set_param("direct_control/value", 5)

    app = QApplication(sys.argv)
    clock = Window()
    clock.show()
    sys.exit(app.exec_())
