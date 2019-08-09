import rospy
import sys
import os
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (QApplication, QCheckBox, QGridLayout, QGroupBox, QMenu, QPushButton, QRadioButton, QVBoxLayout, QWidget, QSlider, QLabel)
from lively_ik.msg import EEPoseGoals, DCPoseGoals
from std_msgs.msg import Float32

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
            dcpg.dc_values.append(Float32(v))

        self.parent.dc_pose_goal_pub.publish(dcpg)

class Window(QWidget):
    def __init__(self, parent=None):
        super(Window, self).__init__(parent)
        self.sliders = []
        self.labels = []
        self.rospy_node = rospy.init_node('direct_control_driver')
        self.dc_pose_goal_pub = rospy.Publisher('/relaxed_ik/dc_pose_goals', DCPoseGoals, queue_size=3)

        grid = QGridLayout()
        path = os.path.abspath(__file__)[:-18] + '/RelaxedIK/Config/info_files/nao_v4_info.yaml'

        with open(path,'r') as file:
            lines = file.readlines()

            joint_list = []
            joint_ordering = []
            joint_limits = []

            for line in lines:
                if 'dc_joint_names' in line:
                    list = line[:-2].split('[')
                    list = list[len(list) - 1].split(',')
                    for item in list:
                        joint_list.append(item.strip())
                elif 'joint_ordering' in line:
                    list = line[:-3].split('[')
                    list = list[len(list) - 1].split(',')
                    for item in list:
                        joint_ordering.append(item.strip().strip('"'))
                elif 'joint_limits' in line:
                    list = line[:-3].split('[')
                    for i, item in enumerate(list):
                        if i < 2:
                            continue
                        if i == len(list) - 1:
                            pair = item.split(',')
                            pair[1] = pair[1][1:]
                            pair[0] = float(pair[0])
                            pair[1] = float(pair[1])
                            joint_limits.append(pair)
                        else:
                            pair = item.split(',')[:-1]
                            pair[1] = pair[1][1:-1]
                            pair[0] = float(pair[0])
                            pair[1] = float(pair[1])
                            joint_limits.append(pair)

            slider_count = 0
            self.values = []
            for i, joint in enumerate(joint_ordering):
                if joint in joint_list:
                    slider_widget = SliderWidget(self, slider_count, joint, joint_limits[i][0], joint_limits[i][1])
                    self.sliders.append(slider_widget)
                    grid.addWidget(slider_widget.groupBox, slider_count, 0)
                    self.values.append(0)
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
