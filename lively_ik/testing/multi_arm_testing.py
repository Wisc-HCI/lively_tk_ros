from lively_ik.configuration.config_manager import ConfigManager
from lively_ik_core import parse_config_data, LivelyIK
import yaml
import time
import rclpy
import math
from rclpy.node import Node
from std_msgs.msg import Header, String
from sensor_msgs.msg import JointState
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped, Transform, Vector3, Quaternion
import pandas as pd
from matplotlib import pyplot as plt
from datetime import datetime
from pprint import PrettyPrinter
import networkx as nx

URDF = '''<?xml version="1.0" ?><robot name="NaoH25V40" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <joint name="HeadYaw" type="revolute">
    <parent link="torso"/>
    <child link="Neck"/>
    <origin rpy="0 0 0" xyz="0 0 0.1265"/>
    <axis xyz="0 0 1.0"/>
    <limit effort="1.547" lower="-2.08567" upper="2.08567" velocity="8.26797"/>
  </joint>
  <link name="Neck">
    <inertial>
      <mass value="0.06442"/>
      <inertia ixx="2.65584e-05" ixy="1.57e-09" ixz="0" iyy="2.75654e-05" iyz="-5.295e-08" izz="5.53372e-06"/>
      <origin rpy="0 0 0" xyz="-1e-05 0 -0.02742"/>
    </inertial>
    <visual>
      <geometry>
        <mesh filename="package://nao_meshes/meshes/V40/HeadYaw.dae" scale="0.1 0.1 0.1"/>
      </geometry>
      <origin rpy="0 0 0" xyz="0 0 0"/>
    </visual>
    <collision>
      <geometry>
        <mesh filename="package://nao_meshes/meshes/V40/HeadYaw_0.10.stl" scale="0.1 0.1 0.1"/>
      </geometry>
      <origin rpy="0 0 0" xyz="0 0 0"/>
    </collision>
  </link>
  <joint name="HeadPitch" type="revolute">
    <parent link="Neck"/>
    <child link="Head"/>
    <origin rpy="0 0 0" xyz="0 0 0"/>
    <axis xyz="0 1.0 0"/>
    <limit effort="1.532" lower="-0.671952" upper="0.514872" velocity="7.19407"/>
  </joint>
  <link name="Head">
    <inertial>
      <mass value="0.60533"/>
      <inertia ixx="0.000957766" ixy="8.78814e-06" ixz="5.33702e-06" iyy="0.000816836" iyz="-2.99579e-05" izz="0.000984976"/>
      <origin rpy="0 0 0" xyz="-0.00112 0 0.05258"/>
    </inertial>
    <visual>
      <geometry>
        <mesh filename="package://nao_meshes/meshes/V40/HeadPitch.dae" scale="0.1 0.1 0.1"/>
      </geometry>
      <origin rpy="0 0 0" xyz="0 0 0"/>
    </visual>
    <collision>
      <geometry>
        <mesh filename="package://nao_meshes/meshes/V40/HeadPitch_0.10.stl" scale="0.1 0.1 0.1"/>
      </geometry>
      <origin rpy="0 0 0" xyz="0 0 0"/>
    </collision>
  </link>
  <joint name="gaze_joint" type="fixed">
    <parent link="Head"/>
    <child link="gaze"/>
    <origin rpy="0 0 0" xyz="0.05871 0 0.06364"/>
  </joint>
  <link name="gaze"/>
  <joint name="LHipYawPitch" type="revolute">
    <parent link="torso"/>
    <child link="LPelvis"/>
    <origin rpy="0 0 0" xyz="0 0.05 -0.085"/>
    <axis xyz="1 0 0"/>
    <limit effort="3.348" lower="-1.14529" upper="0.740718" velocity="4.16174"/>
  </joint>
  <link name="LPelvis">
    <inertial>
      <mass value="0.06981"/>
      <inertia ixx="2.3407e-05" ixy="1.07922e-06" ixz="-1.76003e-06" iyy="4.76355e-05" iyz="2.76058e-06" izz="4.97021e-05"/>
      <origin rpy="0 0 0" xyz="-0.00781 -0.01114 0.02661"/>
    </inertial>
    <visual>
      <geometry>
        <mesh filename="package://nao_meshes/meshes/V40/LHipYawPitch.dae" scale="0.1 0.1 0.1"/>
      </geometry>
      <origin rpy="0 0 0" xyz="0 0 0"/>
    </visual>
    <collision>
      <geometry>
        <mesh filename="package://nao_meshes/meshes/V40/LHipYawPitch_0.10.stl" scale="0.1 0.1 0.1"/>
      </geometry>
      <origin rpy="0 0 0" xyz="0 0 0"/>
    </collision>
  </link>
  <joint name="LHipRoll" type="revolute">
    <parent link="LPelvis"/>
    <child link="LHip"/>
    <origin rpy="0 0 0" xyz="0 0 0"/>
    <axis xyz="1.0 0 0"/>
    <limit effort="3.348" lower="-0.379435" upper="0.79046" velocity="4.16174"/>
  </joint>
  <link name="LHip">
    <inertial>
      <mass value="0.13053"/>
      <inertia ixx="2.41106e-05" ixy="-6.08684e-07" ixz="6.33119e-06" iyy="6.34892e-05" iyz="-1.99137e-07" izz="5.67694e-05"/>
      <origin rpy="0 0 0" xyz="-0.01549 0.00029 -0.00515"/>
    </inertial>
    <visual>
      <geometry>
        <mesh filename="package://nao_meshes/meshes/V40/LHipRoll.dae" scale="0.1 0.1 0.1"/>
      </geometry>
      <origin rpy="0 0 0" xyz="0 0 0"/>
    </visual>
    <collision>
      <geometry>
        <mesh filename="package://nao_meshes/meshes/V40/LHipRoll_0.10.stl" scale="0.1 0.1 0.1"/>
      </geometry>
      <origin rpy="0 0 0" xyz="0 0 0"/>
    </collision>
  </link>
  <joint name="LHipPitch" type="revolute">
    <parent link="LHip"/>
    <child link="LThigh"/>
    <origin rpy="0 0 0" xyz="0 0 0"/>
    <axis xyz="0 1.0 0"/>
    <limit effort="3.023" lower="-1.53589" upper="0.48398" velocity="6.40239"/>
  </joint>
  <link name="LThigh">
    <inertial>
      <mass value="0.38968"/>
      <inertia ixx="0.000509844" ixy="2.11296e-06" ixz="5.64129e-05" iyy="0.000465358" iyz="-7.91029e-06" izz="0.000301098"/>
      <origin rpy="0 0 0" xyz="0.00138 0.00221 -0.05373"/>
    </inertial>
    <visual>
      <geometry>
        <mesh filename="package://nao_meshes/meshes/V40/LHipPitch.dae" scale="0.1 0.1 0.1"/>
      </geometry>
      <origin rpy="0 0 0" xyz="0 0 0"/>
    </visual>
    <collision>
      <geometry>
        <mesh filename="package://nao_meshes/meshes/V40/LHipPitch_0.10.stl" scale="0.1 0.1 0.1"/>
      </geometry>
      <origin rpy="0 0 0" xyz="0 0 0"/>
    </collision>
  </link>
  <joint name="LKneePitch" type="revolute">
    <parent link="LThigh"/>
    <child link="LTibia"/>
    <origin rpy="0 0 0" xyz="0 0 -0.1"/>
    <axis xyz="0 1.0 0"/>
    <limit effort="3.023" lower="-0.0923279" upper="2.11255" velocity="6.40239"/>
  </joint>
  <link name="LTibia">
    <inertial>
      <mass value="0.29142"/>
      <inertia ixx="0.000470586" ixy="3.60392e-06" ixz="-2.86648e-05" iyy="0.000412654" iyz="7.13013e-06" izz="0.000185772"/>
      <origin rpy="0 0 0" xyz="0.00453 0.00225 -0.04936"/>
    </inertial>
    <visual>
      <geometry>
        <mesh filename="package://nao_meshes/meshes/V40/LKneePitch.dae" scale="0.1 0.1 0.1"/>
      </geometry>
      <origin rpy="0 0 0" xyz="0 0 0"/>
    </visual>
    <collision>
      <geometry>
        <mesh filename="package://nao_meshes/meshes/V40/LKneePitch_0.10.stl" scale="0.1 0.1 0.1"/>
      </geometry>
      <origin rpy="0 0 0" xyz="0 0 0"/>
    </collision>
  </link>
  <joint name="LAnklePitch" type="revolute">
    <parent link="LTibia"/>
    <child link="LAnklePitch"/>
    <origin rpy="0 0 0" xyz="0 0 -0.1029"/>
    <axis xyz="0 1.0 0"/>
    <limit effort="3.023" lower="-1.18944" upper="0.922581" velocity="6.40239"/>
  </joint>
  <link name="LAnklePitch">
    <inertial>
      <mass value="0.13416"/>
      <inertia ixx="3.22034e-05" ixy="-8.83212e-09" ixz="4.27549e-06" iyy="6.7943e-05" iyz="2.84849e-07" izz="5.48269e-05"/>
      <origin rpy="0 0 0" xyz="0.00045 0.00029 0.00685"/>
    </inertial>
    <visual>
      <geometry>
        <mesh filename="package://nao_meshes/meshes/V40/LAnklePitch.dae" scale="0.1 0.1 0.1"/>
      </geometry>
      <origin rpy="0 0 0" xyz="0 0 0"/>
    </visual>
    <collision>
      <geometry>
        <mesh filename="package://nao_meshes/meshes/V40/LAnklePitch_0.10.stl" scale="0.1 0.1 0.1"/>
      </geometry>
      <origin rpy="0 0 0" xyz="0 0 0"/>
    </collision>
  </link>
  <joint name="LAnkleRoll" type="revolute">
    <parent link="LAnklePitch"/>
    <child link="l_ankle"/>
    <origin rpy="0 0 0" xyz="0 0 0"/>
    <axis xyz="1.0 0 0"/>
    <limit effort="3.348" lower="-0.397761" upper="0.768992" velocity="4.16174"/>
  </joint>
  <link name="l_ankle">
    <inertial>
      <mass value="0.16184"/>
      <inertia ixx="9.78911e-05" ixy="7.88039e-06" ixz="6.1279e-06" iyy="0.000369977" iyz="1.44233e-06" izz="0.000419417"/>
      <origin rpy="0 0 0" xyz="0.02542 0.0033 -0.03239"/>
    </inertial>
    <visual>
      <geometry>
        <mesh filename="package://nao_meshes/meshes/V40/LAnkleRoll.dae" scale="0.1 0.1 0.1"/>
      </geometry>
      <origin rpy="0 0 0" xyz="0 0 0"/>
    </visual>
    <collision>
      <geometry>
        <mesh filename="package://nao_meshes/meshes/V40/LAnkleRoll_0.10.stl" scale="0.1 0.1 0.1"/>
      </geometry>
      <origin rpy="0 0 0" xyz="0 0 0"/>
    </collision>
  </link>
  <joint name="LLeg_effector_fixedjoint" type="fixed">
    <parent link="l_ankle"/>
    <child link="l_sole"/>
    <origin rpy="0 0 0" xyz="0 0 -0.04511"/>
    <axis xyz="0 0 0"/>
  </joint>
  <link name="l_sole"/>
  <joint name="RHipYawPitch" type="revolute">
    <parent link="torso"/>
    <child link="RPelvis"/>
    <origin rpy="0 0 0" xyz="0 -0.05 -0.085"/>
    <axis xyz="1 0 0"/>
    <limit effort="3.348" lower="-1.14529" upper="0.740718" velocity="4.16174"/>
    <mimic joint="LHipYawPitch" multiplier="1.0" offset="0"/>
  </joint>
  <link name="RPelvis">
    <inertial>
      <mass value="0.06981"/>
      <inertia ixx="3.18766e-05" ixy="-1.07152e-06" ixz="-1.77295e-06" iyy="5.18361e-05" iyz="-7.00664e-06" izz="5.39657e-05"/>
      <origin rpy="0 0 0" xyz="-0.00781 0.01114 0.02661"/>
    </inertial>
    <visual>
      <geometry>
        <mesh filename="package://nao_meshes/meshes/V40/RHipYawPitch.dae" scale="0.1 0.1 0.1"/>
      </geometry>
      <origin rpy="0 0 0" xyz="0 0 0"/>
    </visual>
    <collision>
      <geometry>
        <mesh filename="package://nao_meshes/meshes/V40/RHipYawPitch_0.10.stl" scale="0.1 0.1 0.1"/>
      </geometry>
      <origin rpy="0 0 0" xyz="0 0 0"/>
    </collision>
  </link>
  <joint name="RHipRoll" type="revolute">
    <parent link="RPelvis"/>
    <child link="RHip"/>
    <origin rpy="0 0 0" xyz="0 0 0"/>
    <axis xyz="1.0 0 0"/>
    <limit effort="3.348" lower="-0.79046" upper="0.379435" velocity="4.16174"/>
  </joint>
  <link name="RHip">
    <inertial>
      <mass value="0.13053"/>
      <inertia ixx="2.41136e-05" ixy="5.67164e-07" ixz="6.30461e-06" iyy="6.34886e-05" iyz="1.97457e-07" izz="5.6773e-05"/>
      <origin rpy="0 0 0" xyz="-0.01549 -0.00029 -0.00515"/>
    </inertial>
    <visual>
      <geometry>
        <mesh filename="package://nao_meshes/meshes/V40/RHipRoll.dae" scale="0.1 0.1 0.1"/>
      </geometry>
      <origin rpy="0 0 0" xyz="0 0 0"/>
    </visual>
    <collision>
      <geometry>
        <mesh filename="package://nao_meshes/meshes/V40/RHipRoll_0.10.stl" scale="0.1 0.1 0.1"/>
      </geometry>
      <origin rpy="0 0 0" xyz="0 0 0"/>
    </collision>
  </link>
  <joint name="RHipPitch" type="revolute">
    <parent link="RHip"/>
    <child link="RThigh"/>
    <origin rpy="0 0 0" xyz="0 0 0"/>
    <axis xyz="0 1.0 0"/>
    <limit effort="3.0226" lower="-1.53589" upper="0.48398" velocity="6.40239"/>
  </joint>
  <link name="RThigh">
    <inertial>
      <mass value="0.38968"/>
      <inertia ixx="0.000510607" ixy="-2.02799e-06" ixz="5.69893e-05" iyy="0.0004665" iyz="7.09563e-06" izz="0.000301333"/>
      <origin rpy="0 0 0" xyz="0.00138 -0.00221 -0.05373"/>
    </inertial>
    <visual>
      <geometry>
        <mesh filename="package://nao_meshes/meshes/V40/RHipPitch.dae" scale="0.1 0.1 0.1"/>
      </geometry>
      <origin rpy="0 0 0" xyz="0 0 0"/>
    </visual>
    <collision>
      <geometry>
        <mesh filename="package://nao_meshes/meshes/V40/RHipPitch_0.10.stl" scale="0.1 0.1 0.1"/>
      </geometry>
      <origin rpy="0 0 0" xyz="0 0 0"/>
    </collision>
  </link>
  <joint name="RKneePitch" type="revolute">
    <parent link="RThigh"/>
    <child link="RTibia"/>
    <origin rpy="0 0 0" xyz="0 0 -0.1"/>
    <axis xyz="0 1.0 0"/>
    <limit effort="3.0226" lower="-0.0923279" upper="2.11255" velocity="6.40239"/>
  </joint>
  <link name="RTibia">
    <inertial>
      <mass value="0.29142"/>
      <inertia ixx="0.000471336" ixy="-3.8668e-06" ixz="-3.71648e-05" iyy="0.00041228" iyz="-6.11093e-06" izz="0.000183997"/>
      <origin rpy="0 0 0" xyz="0.00453 -0.00225 -0.04936"/>
    </inertial>
    <visual>
      <geometry>
        <mesh filename="package://nao_meshes/meshes/V40/RKneePitch.dae" scale="0.1 0.1 0.1"/>
      </geometry>
      <origin rpy="0 0 0" xyz="0 0 0"/>
    </visual>
    <collision>
      <geometry>
        <mesh filename="package://nao_meshes/meshes/V40/RKneePitch_0.10.stl" scale="0.1 0.1 0.1"/>
      </geometry>
      <origin rpy="0 0 0" xyz="0 0 0"/>
    </collision>
  </link>
  <joint name="RAnklePitch" type="revolute">
    <parent link="RTibia"/>
    <child link="RAnklePitch"/>
    <origin rpy="0 0 0" xyz="0 0 -0.1029"/>
    <axis xyz="0 1.0 0"/>
    <limit effort="3.0226" lower="-1.1863" upper="0.932006" velocity="6.40239"/>
  </joint>
  <link name="RAnklePitch">
    <inertial>
      <mass value="0.13416"/>
      <inertia ixx="3.22017e-05" ixy="4.68321e-08" ixz="4.28821e-06" iyy="6.79885e-05" iyz="-2.71089e-07" izz="5.48747e-05"/>
      <origin rpy="0 0 0" xyz="0.00045 -0.00029 0.00685"/>
    </inertial>
    <visual>
      <geometry>
        <mesh filename="package://nao_meshes/meshes/V40/RAnklePitch.dae" scale="0.1 0.1 0.1"/>
      </geometry>
      <origin rpy="0 0 0" xyz="0 0 0"/>
    </visual>
    <collision>
      <geometry>
        <mesh filename="package://nao_meshes/meshes/V40/RAnklePitch_0.10.stl" scale="0.1 0.1 0.1"/>
      </geometry>
      <origin rpy="0 0 0" xyz="0 0 0"/>
    </collision>
  </link>
  <joint name="RAnkleRoll" type="revolute">
    <parent link="RAnklePitch"/>
    <child link="r_ankle"/>
    <origin rpy="0 0 0" xyz="0 0 0"/>
    <axis xyz="1.0 0 0"/>
    <limit effort="3.348" lower="-0.768992" upper="0.397761" velocity="4.16174"/>
  </joint>
  <link name="r_ankle">
    <inertial>
      <mass value="0.16184"/>
      <inertia ixx="9.77513e-05" ixy="-7.70106e-06" ixz="5.88169e-06" iyy="0.000369108" iyz="-1.55058e-06" izz="0.000418695"/>
      <origin rpy="0 0 0" xyz="0.02542 -0.0033 -0.03239"/>
    </inertial>
    <visual>
      <geometry>
        <mesh filename="package://nao_meshes/meshes/V40/RAnkleRoll.dae" scale="0.1 0.1 0.1"/>
      </geometry>
      <origin rpy="0 0 0" xyz="0 0 0"/>
    </visual>
    <collision>
      <geometry>
        <mesh filename="package://nao_meshes/meshes/V40/RAnkleRoll_0.10.stl" scale="0.1 0.1 0.1"/>
      </geometry>
      <origin rpy="0 0 0" xyz="0 0 0"/>
    </collision>
  </link>
  <joint name="RLeg_effector_fixedjoint" type="fixed">
    <parent link="r_ankle"/>
    <child link="r_sole"/>
    <origin rpy="0 0 0" xyz="0 0 -0.04511"/>
    <axis xyz="0 0 0"/>
  </joint>
  <link name="r_sole"/>
  <link name="base_link"/>
  <joint name="base_link_fixedjoint" type="fixed">
    <parent link="base_link"/>
    <child link="torso"/>
    <origin rpy="0 0 0" xyz="0 0 0"/>
    <axis xyz="0 0 0"/>
  </joint>
  <link name="torso">
    <inertial>
      <mass value="1.04956"/>
      <inertia ixx="0.00308361" ixy="1.43116e-05" ixz="-3.30211e-05" iyy="0.0028835" iyz="-2.70793e-05" izz="0.0015924"/>
      <origin rpy="0 0 0" xyz="-0.00413 0 0.04342"/>
    </inertial>
    <visual>
      <geometry>
        <mesh filename="package://nao_meshes/meshes/V40/Torso.dae" scale="0.1 0.1 0.1"/>
      </geometry>
      <origin rpy="0 0 0" xyz="0 0 0"/>
    </visual>
    <collision>
      <geometry>
        <mesh filename="package://nao_meshes/meshes/V40/Torso_0.10.stl" scale="0.1 0.1 0.1"/>
      </geometry>
      <origin rpy="0 0 0" xyz="0 0 0"/>
    </collision>
  </link>
  <joint name="LShoulderPitch" type="revolute">
    <parent link="torso"/>
    <child link="LShoulder"/>
    <origin rpy="0 0 0" xyz="0 0.098 0.1"/>
    <axis xyz="0 1.0 0"/>
    <limit effort="1.329" lower="-2.08567" upper="2.08567" velocity="8.26797"/>
  </joint>
  <link name="LShoulder">
    <inertial>
      <mass value="0.07504"/>
      <inertia ixx="3.10677e-05" ixy="1.2692e-06" ixz="6.04576e-09" iyy="1.39498e-05" iyz="-2.99484e-07" izz="3.30001e-05"/>
      <origin rpy="0 0 0" xyz="-0.00165 -0.02663 0.00014"/>
    </inertial>
    <visual>
      <geometry>
        <mesh filename="package://nao_meshes/meshes/V40/LShoulderPitch.dae" scale="0.1 0.1 0.1"/>
      </geometry>
      <origin rpy="0 0 0" xyz="0 0 0"/>
    </visual>
    <collision>
      <geometry>
        <mesh filename="package://nao_meshes/meshes/V40/LShoulderPitch_0.10.stl" scale="0.1 0.1 0.1"/>
      </geometry>
      <origin rpy="0 0 0" xyz="0 0 0"/>
    </collision>
  </link>
  <joint name="LShoulderRoll" type="revolute">
    <parent link="LShoulder"/>
    <child link="LBicep"/>
    <origin rpy="0 0 0" xyz="0 0 0"/>
    <axis xyz="0 0 1.0"/>
    <limit effort="1.7835" lower="-0.314159" upper="1.32645" velocity="7.19407"/>
  </joint>
  <link name="LBicep">
    <inertial>
      <mass value="0.15777"/>
      <inertia ixx="8.7181e-05" ixy="-2.53381e-05" ixz="-1.4213e-05" iyy="0.000274712" iyz="4.712439e-07" izz="0.000241812"/>
      <origin rpy="0 0 0" xyz="0.02455 0.00563 0.0033"/>
    </inertial>
    <visual>
      <geometry>
        <mesh filename="package://nao_meshes/meshes/V40/LShoulderRoll.dae" scale="0.1 0.1 0.1"/>
      </geometry>
      <origin rpy="0 0 0" xyz="0 0 0"/>
    </visual>
    <collision>
      <geometry>
        <mesh filename="package://nao_meshes/meshes/V40/LShoulderRoll_0.10.stl" scale="0.1 0.1 0.1"/>
      </geometry>
      <origin rpy="0 0 0" xyz="0 0 0"/>
    </collision>
  </link>
  <joint name="LElbowYaw" type="revolute">
    <parent link="LBicep"/>
    <child link="LElbow"/>
    <origin rpy="0 0 0" xyz="0.105 0.015 0"/>
    <axis xyz="1.0 0 0"/>
    <limit effort="1.547" lower="-2.08567" upper="2.08567" velocity="8.26797"/>
  </joint>
  <link name="LElbow">
    <inertial>
      <mass value="0.06483"/>
      <inertia ixx="5.59588e-06" ixy="4.21e-09" ixz="2.92241e-07" iyy="2.66179e-05" iyz="-1.84e-09" izz="2.76294e-05"/>
      <origin rpy="0 0 0" xyz="-0.02744 0 -0.00014"/>
    </inertial>
  </link>
  <joint name="LElbowRoll" type="revolute">
    <parent link="LElbow"/>
    <child link="LForeArm"/>
    <origin rpy="0 0 0" xyz="0 0 0"/>
    <axis xyz="0 0 1.0"/>
    <limit effort="1.532" lower="-1.54462" upper="-0.0349066" velocity="7.19407"/>
  </joint>
  <link name="LForeArm">
    <inertial>
      <mass value="0.07761"/>
      <inertia ixx="2.46746e-05" ixy="3.23152e-06" ixz="1.58221e-06" iyy="3.83837e-05" iyz="1.39194e-07" izz="3.59708e-05"/>
      <origin rpy="0 0 0" xyz="0.02556 0.00281 0.00076"/>
    </inertial>
    <visual>
      <geometry>
        <mesh filename="package://nao_meshes/meshes/V40/LElbowRoll.dae" scale="0.1 0.1 0.1"/>
      </geometry>
      <origin rpy="0 0 0" xyz="0 0 0"/>
    </visual>
    <collision>
      <geometry>
        <mesh filename="package://nao_meshes/meshes/V40/LElbowRoll_0.10.stl" scale="0.1 0.1 0.1"/>
      </geometry>
      <origin rpy="0 0 0" xyz="0 0 0"/>
    </collision>
  </link>
  <joint name="LWristYaw" type="revolute">
    <parent link="LForeArm"/>
    <child link="l_wrist"/>
    <origin rpy="0 0 0" xyz="0.05595 0 0"/>
    <axis xyz="1.0 0 0"/>
    <limit effort="0.4075" lower="-1.82387" upper="1.82387" velocity="24.6229"/>
  </joint>
  <link name="l_wrist">
    <inertial>
      <mass value="0.18533"/>
      <inertia ixx="6.86477e-05" ixy="1.15465e-07" ixz="-2.87254e-06" iyy="0.000135756" iyz="2.67539e-06" izz="0.000133228"/>
      <origin rpy="0 0 0" xyz="0.03434 -0.00088 0.00308"/>
    </inertial>
    <visual>
      <geometry>
        <mesh filename="package://nao_meshes/meshes/V40/LWristYaw.dae" scale="0.1 0.1 0.1"/>
      </geometry>
      <origin rpy="0 0 0" xyz="0 0 0"/>
    </visual>
    <collision>
      <geometry>
        <mesh filename="package://nao_meshes/meshes/V40/LWristYaw_0.10.stl" scale="0.1 0.1 0.1"/>
      </geometry>
      <origin rpy="0 0 0" xyz="0 0 0"/>
    </collision>
  </link>
  <joint name="LArm_effector_fixedjoint" type="fixed">
    <parent link="l_wrist"/>
    <child link="l_palm"/>
    <origin rpy="0 0 0" xyz="0 0 0"/>
    <axis xyz="-0.4017 -0.2659 -0.1617"/>
  </joint>
  <link name="l_palm"/>
  <joint name="LHand" type="revolute">
    <parent link="l_wrist"/>
    <child link="l_gripper"/>
    <origin rpy="0 0 0" xyz="0.05775 0 -0.01231"/>
    <axis xyz="1.0 0 0"/>
    <limit effort="0.292" lower="0" upper="1.0" velocity="8.33"/>
  </joint>
  <link name="l_gripper">
    <inertial>
      <mass value="2e-06"/>
      <inertia ixx="1.1e-09" ixy="0" ixz="0" iyy="1.1e-09" iyz="0" izz="1.1e-09"/>
    </inertial>
  </link>
  <joint name="RShoulderPitch" type="revolute">
    <parent link="torso"/>
    <child link="RShoulder"/>
    <origin rpy="0 0 0" xyz="0 -0.098 0.1"/>
    <axis xyz="0 1.0 0"/>
    <limit effort="1.329" lower="-2.08567" upper="2.08567" velocity="8.26797"/>
  </joint>
  <link name="RShoulder">
    <inertial>
      <mass value="0.07504"/>
      <inertia ixx="3.10677e-05" ixy="-1.2692e-06" ixz="6.04576e-09" iyy="1.39498e-05" iyz="2.99484e-07" izz="3.30001e-05"/>
      <origin rpy="0 0 0" xyz="-0.00165 0.02663 0.00014"/>
    </inertial>
    <visual>
      <geometry>
        <mesh filename="package://nao_meshes/meshes/V40/RShoulderPitch.dae" scale="0.1 0.1 0.1"/>
      </geometry>
      <origin rpy="0 0 0" xyz="0 0 0"/>
    </visual>
    <collision>
      <geometry>
        <mesh filename="package://nao_meshes/meshes/V40/RShoulderPitch_0.10.stl" scale="0.1 0.1 0.1"/>
      </geometry>
      <origin rpy="0 0 0" xyz="0 0 0"/>
    </collision>
  </link>
  <joint name="RShoulderRoll" type="revolute">
    <parent link="RShoulder"/>
    <child link="RBicep"/>
    <origin rpy="0 0 0" xyz="0 0 0"/>
    <axis xyz="0 0 1.0"/>
    <limit effort="1.783" lower="-1.32645" upper="0.314159" velocity="7.19407"/>
  </joint>
  <link name="RBicep">
    <inertial>
      <mass value="0.15777"/>
      <inertia ixx="0.000103401" ixy="5.48849e-05" ixz="-1.32643e-05" iyy="0.00027077" iyz="9.16707e-06" izz="0.000254529"/>
      <origin rpy="0 0 0" xyz="0.02455 -0.00563 0.0033"/>
    </inertial>
    <visual>
      <geometry>
        <mesh filename="package://nao_meshes/meshes/V40/RShoulderRoll.dae" scale="0.1 0.1 0.1"/>
      </geometry>
      <origin rpy="0 0 0" xyz="0 0 0"/>
    </visual>
    <collision>
      <geometry>
        <mesh filename="package://nao_meshes/meshes/V40/RShoulderRoll_0.10.stl" scale="0.1 0.1 0.1"/>
      </geometry>
      <origin rpy="0 0 0" xyz="0 0 0"/>
    </collision>
  </link>
  <joint name="RElbowYaw" type="revolute">
    <parent link="RBicep"/>
    <child link="RElbow"/>
    <origin rpy="0 0 0" xyz="0.105 -0.015 0"/>
    <axis xyz="1.0 0 0"/>
    <limit effort="1.547" lower="-2.08567" upper="2.08567" velocity="8.26797"/>
  </joint>
  <link name="RElbow">
    <inertial>
      <mass value="0.06483"/>
      <inertia ixx="5.59588e-06" ixy="4.21e-09" ixz="2.92241e-07" iyy="2.66179e-05" iyz="-1.84e-09" izz="2.76294e-05"/>
      <origin rpy="0 0 0" xyz="-0.02744 0 -0.00014"/>
    </inertial>
  </link>
  <joint name="RElbowRoll" type="revolute">
    <parent link="RElbow"/>
    <child link="RForeArm"/>
    <origin rpy="0 0 0" xyz="0 0 0"/>
    <axis xyz="0 0 1.0"/>
    <limit effort="1.532" lower="0.0349066" upper="1.54462" velocity="7.19407"/>
  </joint>
  <link name="RForeArm">
    <inertial>
      <mass value="0.07761"/>
      <inertia ixx="2.47331e-05" ixy="-3.2418e-06" ixz="9.06451e-07" iyy="3.84719e-05" iyz="-1.38804e-07" izz="3.59319e-05"/>
      <origin rpy="0 0 0" xyz="0.02556 -0.00281 0.00076"/>
    </inertial>
    <visual>
      <geometry>
        <mesh filename="package://nao_meshes/meshes/V40/RElbowRoll.dae" scale="0.1 0.1 0.1"/>
      </geometry>
      <origin rpy="0 0 0" xyz="0 0 0"/>
    </visual>
    <collision>
      <geometry>
        <mesh filename="package://nao_meshes/meshes/V40/RElbowRoll_0.10.stl" scale="0.1 0.1 0.1"/>
      </geometry>
      <origin rpy="0 0 0" xyz="0 0 0"/>
    </collision>
  </link>
  <joint name="RWristYaw" type="revolute">
    <parent link="RForeArm"/>
    <child link="r_wrist"/>
    <origin rpy="0 0 0" xyz="0.05595 0 0"/>
    <axis xyz="1.0 0 0"/>
    <limit effort="0.4075" lower="-1.82387" upper="1.82387" velocity="24.6229"/>
  </joint>
  <link name="r_wrist">
    <inertial>
      <mass value="0.18533"/>
      <inertia ixx="6.86477e-05" ixy="1.13165e-05" ixz="-2.87254e-06" iyy="0.000135756" iyz="3.68003e-06" izz="0.000133228"/>
      <origin rpy="0 0 0" xyz="0.03434 0.00088 0.00308"/>
    </inertial>
    <visual>
      <geometry>
        <mesh filename="package://nao_meshes/meshes/V40/RWristYaw.dae" scale="0.1 0.1 0.1"/>
      </geometry>
      <origin rpy="0 0 0" xyz="0 0 0"/>
    </visual>
    <collision>
      <geometry>
        <mesh filename="package://nao_meshes/meshes/V40/RWristYaw_0.10.stl" scale="0.1 0.1 0.1"/>
      </geometry>
      <origin rpy="0 0 0" xyz="0 0 0"/>
    </collision>
  </link>
  <joint name="RArm_effector_fixedjoint" type="fixed">
    <parent link="r_wrist"/>
    <child link="r_palm"/>
    <origin rpy="0 0 0" xyz="0 0 0"/>
    <axis xyz="0.4017 -0.2659 0.1617"/>
  </joint>
  <link name="r_palm"/>
  <joint name="RHand" type="revolute">
    <parent link="r_wrist"/>
    <child link="r_gripper"/>
    <origin rpy="0 0 0" xyz="0.05775 0 -0.01213"/>
    <axis xyz="1.0 0 0"/>
    <limit effort="0.292" lower="0" upper="1.0" velocity="8.33"/>
  </joint>
  <link name="r_gripper">
    <inertial>
      <mass value="2e-06"/>
      <inertia ixx="1.1e-09" ixy="0" ixz="0" iyy="1.1e-09" iyz="0" izz="1.1e-09"/>
    </inertial>
  </link>
  <link name="RFinger23_link">
    <inertial>
      <mass value="2e-06"/>
      <inertia ixx="1.1e-09" ixy="0" ixz="0" iyy="1.1e-09" iyz="0" izz="1.1e-09"/>
    </inertial>
    <visual>
      <geometry>
        <mesh filename="package://nao_meshes/meshes/V40/RFinger23.dae" scale="0.1 0.1 0.1"/>
      </geometry>
      <origin rpy="-1.578 0 0" xyz="0 0 0"/>
    </visual>
    <collision>
      <geometry>
        <mesh filename="package://nao_meshes/meshes/V40/RFinger23_0.10.stl" scale="0.1 0.1 0.1"/>
      </geometry>
      <origin rpy="0 0 0" xyz="0 0 0"/>
    </collision>
  </link>
  <joint name="RFinger23" type="continuous">
    <parent link="RFinger22_link"/>
    <child link="RFinger23_link"/>
    <origin rpy="0 0 -0.999899" xyz="0.01436 0 0"/>
    <axis xyz="0 0 1.0"/>
    <mimic joint="RHand" multiplier="0.999899" offset="0"/>
  </joint>
  <link name="RFinger13_link">
    <inertial>
      <mass value="2e-06"/>
      <inertia ixx="1.1e-09" ixy="0" ixz="0" iyy="1.1e-09" iyz="0" izz="1.1e-09"/>
    </inertial>
    <visual>
      <geometry>
        <mesh filename="package://nao_meshes/meshes/V40/RFinger13.dae" scale="0.1 0.1 0.1"/>
      </geometry>
      <origin rpy="-1.578 0 0" xyz="0 0 0"/>
    </visual>
    <collision>
      <geometry>
        <mesh filename="package://nao_meshes/meshes/V40/RFinger13_0.10.stl" scale="0.1 0.1 0.1"/>
      </geometry>
      <origin rpy="-1.578 0 0" xyz="0 0 0"/>
    </collision>
  </link>
  <joint name="RFinger13" type="continuous">
    <parent link="RFinger12_link"/>
    <child link="RFinger13_link"/>
    <origin rpy="0 0 -0.999899" xyz="0.01436 0 0"/>
    <axis xyz="0 0 1.0"/>
    <mimic joint="RHand" multiplier="0.999899" offset="0"/>
  </joint>
  <link name="RFinger12_link">
    <inertial>
      <mass value="2e-06"/>
      <inertia ixx="1.1e-09" ixy="0" ixz="0" iyy="1.1e-09" iyz="0" izz="1.1e-09"/>
    </inertial>
    <visual>
      <geometry>
        <mesh filename="package://nao_meshes/meshes/V40/RFinger12.dae" scale="0.1 0.1 0.1"/>
      </geometry>
      <origin rpy="-1.578 0 0" xyz="0 0 0"/>
    </visual>
    <collision>
      <geometry>
        <mesh filename="package://nao_meshes/meshes/V40/RFinger12_0.10.stl" scale="0.1 0.1 0.1"/>
      </geometry>
      <origin rpy="-1.578 0 0" xyz="0 0 0"/>
    </collision>
  </link>
  <joint name="RFinger12" type="continuous">
    <parent link="RFinger11_link"/>
    <child link="RFinger12_link"/>
    <origin rpy="0 0 -0.999899" xyz="0.01436 0 0"/>
    <axis xyz="0 0 1.0"/>
    <mimic joint="RHand" multiplier="0.999899" offset="0"/>
  </joint>
  <link name="LFinger21_link">
    <inertial>
      <mass value="2e-06"/>
      <inertia ixx="1.1e-09" ixy="0" ixz="0" iyy="1.1e-09" iyz="0" izz="1.1e-09"/>
    </inertial>
    <visual>
      <geometry>
        <mesh filename="package://nao_meshes/meshes/V40/LFinger21.dae" scale="0.1 0.1 0.1"/>
      </geometry>
      <origin rpy="-1.578 0 0" xyz="0 0 0"/>
    </visual>
    <collision>
      <geometry>
        <mesh filename="package://nao_meshes/meshes/V40/LFinger21_0.10.stl" scale="0.1 0.1 0.1"/>
      </geometry>
      <origin rpy="-1.578 0 0" xyz="0 0 0"/>
    </collision>
  </link>
  <joint name="LFinger21" type="continuous">
    <parent link="l_wrist"/>
    <child link="LFinger21_link"/>
    <origin rpy="1.25539 0.976662 -0.264067" xyz="0.06907 -0.01157 -0.00304"/>
    <axis xyz="0 0 1.0"/>
    <mimic joint="LHand" multiplier="0.999899" offset="0"/>
  </joint>
  <link name="LFinger13_link">
    <inertial>
      <mass value="2e-06"/>
      <inertia ixx="1.1e-09" ixy="0" ixz="0" iyy="1.1e-09" iyz="0" izz="1.1e-09"/>
    </inertial>
    <visual>
      <geometry>
        <mesh filename="package://nao_meshes/meshes/V40/LFinger13.dae" scale="0.1 0.1 0.1"/>
      </geometry>
      <origin rpy="-1.578 0 0" xyz="0 0 0"/>
    </visual>
    <collision>
      <geometry>
        <mesh filename="package://nao_meshes/meshes/V40/LFinger13_0.10.stl" scale="0.1 0.1 0.1"/>
      </geometry>
      <origin rpy="-1.578 0 0" xyz="0 0 0"/>
    </collision>
  </link>
  <joint name="LFinger13" type="continuous">
    <parent link="LFinger12_link"/>
    <child link="LFinger13_link"/>
    <origin rpy="0 0 -0.999899" xyz="0.01436 0 0"/>
    <axis xyz="0 0 1.0"/>
    <mimic joint="LHand" multiplier="0.999899" offset="0"/>
  </joint>
  <link name="LFinger11_link">
    <inertial>
      <mass value="2e-06"/>
      <inertia ixx="1.1e-09" ixy="0" ixz="0" iyy="1.1e-09" iyz="0" izz="1.1e-09"/>
    </inertial>
    <visual>
      <geometry>
        <mesh filename="package://nao_meshes/meshes/V40/LFinger11.dae" scale="0.1 0.1 0.1"/>
      </geometry>
      <origin rpy="4.712 0.0 0.0" xyz="0 0 0"/>
    </visual>
    <collision>
      <geometry>
        <mesh filename="package://nao_meshes/meshes/V40/LFinger11_0.10.stl" scale="0.1 0.1 0.1"/>
      </geometry>
      <origin rpy="4.712 0.0 0.0" xyz="0 0 0"/>
    </collision>
  </link>
  <joint name="LFinger11" type="continuous">
    <parent link="l_wrist"/>
    <child link="LFinger11_link"/>
    <origin rpy="1.8862 0.976662 0.264067" xyz="0.06907 0.01157 -0.00304"/>
    <axis xyz="0 0 1.0"/>
    <mimic joint="LHand" multiplier="0.999899" offset="0"/>
  </joint>
  <link name="RFinger22_link">-1.578 0 0
    <inertial>
      <mass value="2e-06"/>
      <inertia ixx="1.1e-09" ixy="0" ixz="0" iyy="1.1e-09" iyz="0" izz="1.1e-09"/>
    </inertial>
    <visual>
      <geometry>
        <mesh filename="package://nao_meshes/meshes/V40/RFinger22.dae" scale="0.1 0.1 0.1"/>
      </geometry>
      <origin rpy="-1.578 0 0" xyz="0 0 0"/>
    </visual>
    <collision>
      <geometry>
        <mesh filename="package://nao_meshes/meshes/V40/RFinger22_0.10.stl" scale="0.1 0.1 0.1"/>
      </geometry>
      <origin rpy="-1.578 0 0" xyz="0 0 0"/>
    </collision>
  </link>
  <joint name="RFinger22" type="continuous">
    <parent link="RFinger21_link"/>
    <child link="RFinger22_link"/>
    <origin rpy="0 0 -0.999899" xyz="0.01436 0 0"/>
    <axis xyz="0 0 1.0"/>
    <mimic joint="RHand" multiplier="0.999899" offset="0"/>
  </joint>
  <link name="LFinger22_link">
    <inertial>
      <mass value="2e-06"/>
      <inertia ixx="1.1e-09" ixy="0" ixz="0" iyy="1.1e-09" iyz="0" izz="1.1e-09"/>
    </inertial>
    <visual>
      <geometry>
        <mesh filename="package://nao_meshes/meshes/V40/LFinger22.dae" scale="0.1 0.1 0.1"/>
      </geometry>
      <origin rpy="-1.578 0 0" xyz="0 0 0"/>
    </visual>
    <collision>
      <geometry>
        <mesh filename="package://nao_meshes/meshes/V40/LFinger22_0.10.stl" scale="0.1 0.1 0.1"/>
      </geometry>
      <origin rpy="-1.578 0 0" xyz="0 0 0"/>
    </collision>
  </link>
  <joint name="LFinger22" type="continuous">
    <parent link="LFinger21_link"/>
    <child link="LFinger22_link"/>
    <origin rpy="0 0 -0.999899" xyz="0.01436 0 0"/>
    <axis xyz="0 0 1.0"/>
    <mimic joint="LHand" multiplier="0.999899" offset="0"/>
  </joint>
  <link name="RFinger21_link">
    <inertial>
      <mass value="2e-06"/>
      <inertia ixx="1.1e-09" ixy="0" ixz="0" iyy="1.1e-09" iyz="0" izz="1.1e-09"/>
    </inertial>
    <visual>
      <geometry>
        <mesh filename="package://nao_meshes/meshes/V40/RFinger21.dae" scale="0.1 0.1 0.1"/>
      </geometry>
      <origin rpy="-1.578 0 0" xyz="0 0 0"/>
    </visual>
    <collision>
      <geometry>
        <mesh filename="package://nao_meshes/meshes/V40/RFinger21_0.10.stl" scale="0.1 0.1 0.1"/>
      </geometry>
      <origin rpy="-1.578 0 0" xyz="0 0 0"/>
    </collision>
  </link>
  <joint name="RFinger21" type="continuous">
    <parent link="r_wrist"/>
    <child link="RFinger21_link"/>
    <origin rpy="1.25539 0.976662 -0.264067" xyz="0.06907 -0.01157 -0.00304"/>
    <axis xyz="0 0 1.0"/>
    <mimic joint="RHand" multiplier="0.999899" offset="0"/>
  </joint>
  <link name="LFinger12_link">
    <inertial>
      <mass value="2e-06"/>
      <inertia ixx="1.1e-09" ixy="0" ixz="0" iyy="1.1e-09" iyz="0" izz="1.1e-09"/>
    </inertial>
    <visual>
      <geometry>
        <mesh filename="package://nao_meshes/meshes/V40/LFinger12.dae" scale="0.1 0.1 0.1"/>
      </geometry>
      <origin rpy="-1.578 0 0" xyz="0 0 0"/>
    </visual>
    <collision>
      <geometry>
        <mesh filename="package://nao_meshes/meshes/V40/LFinger12_0.10.stl" scale="0.1 0.1 0.1"/>
      </geometry>
      <origin rpy="-1.578 0 0" xyz="0 0 0"/>
    </collision>
  </link>
  <joint name="LFinger12" type="continuous">
    <parent link="LFinger11_link"/>
    <child link="LFinger12_link"/>
    <origin rpy="0 0 -0.999899" xyz="0.01436 0 0"/>
    <axis xyz="0 0 1.0"/>
    <mimic joint="LHand" multiplier="0.999899" offset="0"/>
  </joint>
  <link name="RFinger11_link">
    <inertial>
      <mass value="2e-06"/>
      <inertia ixx="1.1e-09" ixy="0" ixz="0" iyy="1.1e-09" iyz="0" izz="1.1e-09"/>
    </inertial>
    <visual>
      <geometry>
        <mesh filename="package://nao_meshes/meshes/V40/RFinger11.dae" scale="0.1 0.1 0.1"/>
      </geometry>
      <origin rpy="-1.578 0 0" xyz="0 0 0"/>
    </visual>
    <collision>
      <geometry>
        <mesh filename="package://nao_meshes/meshes/V40/RFinger11_0.10.stl" scale="0.1 0.1 0.1"/>
      </geometry>
      <origin rpy="-1.578 0 0" xyz="0 0 0"/>
    </collision>
  </link>
  <joint name="RFinger11" type="continuous">
    <parent link="r_wrist"/>
    <child link="RFinger11_link"/>
    <origin rpy="1.8862 0.976662 0.264067" xyz="0.06907 0.01157 -0.00304"/>
    <axis xyz="0 0 1.0"/>
    <mimic joint="RHand" multiplier="0.999899" offset="0"/>
  </joint>
  <link name="LFinger23_link">
    <inertial>
      <mass value="2e-06"/>
      <inertia ixx="1.1e-09" ixy="0" ixz="0" iyy="1.1e-09" iyz="0" izz="1.1e-09"/>
    </inertial>
    <visual>
      <geometry>
        <mesh filename="package://nao_meshes/meshes/V40/LFinger23.dae" scale="0.1 0.1 0.1"/>
      </geometry>
      <origin rpy="-4.712 0 0" xyz="0 0 0"/>
    </visual>
    <collision>
      <geometry>
        <mesh filename="package://nao_meshes/meshes/V40/LFinger23_0.10.stl" scale="0.1 0.1 0.1"/>
      </geometry>
      <origin rpy="-4.712 0 0" xyz="0 0 0"/>
    </collision>
  </link>
  <joint name="LFinger23" type="continuous">
    <parent link="LFinger22_link"/>
    <child link="LFinger23_link"/>
    <origin rpy="0 0 -0.999899" xyz="0.01436 0 0"/>
    <axis xyz="0 0 1.0"/>
    <mimic joint="LHand" multiplier="0.999899" offset="0"/>
  </joint>
  <link name="LThumb1_link">
    <inertial>
      <mass value="2e-06"/>
      <inertia ixx="1.1e-09" ixy="0" ixz="0" iyy="1.1e-09" iyz="0" izz="1.1e-09"/>
    </inertial>
    <visual>
      <geometry>
        <mesh filename="package://nao_meshes/meshes/V40/LThumb1.dae" scale="0.1 0.1 0.1"/>
      </geometry>
      <origin rpy="4.712 0 1.578" xyz="0.002 -0.007 0"/>
    </visual>
    <collision>
      <geometry>
        <mesh filename="package://nao_meshes/meshes/V40/LThumb1_0.10.stl" scale="0.1 0.1 0.1"/>
      </geometry>
      <origin rpy="-4.712 0 1.578" xyz="-0.002 0.007 0"/>
    </collision>
  </link>
  <joint name="LThumb1" type="continuous">
    <parent link="l_wrist"/>
    <child link="LThumb1_link"/>
    <origin rpy="-1.5708 0.0472984 -3.26826e-08" xyz="0.04895 0 -0.02638"/>
    <axis xyz="0 0 1.0"/>
    <mimic joint="LHand" multiplier="0.999899" offset="0"/>
  </joint>
  <link name="RThumb1_link">
    <inertial>
      <mass value="2e-06"/>
      <inertia ixx="1.1e-09" ixy="0" ixz="0" iyy="1.1e-09" iyz="0" izz="1.1e-09"/>
    </inertial>
    <visual>
      <geometry>
        <mesh filename="package://nao_meshes/meshes/V40/RThumb1.dae" scale="0.1 0.1 0.1"/>
      </geometry>
      <origin rpy="4.712 0 1.578" xyz="0.002 -0.007 0"/>
    </visual>
    <collision>
      <geometry>
        <mesh filename="package://nao_meshes/meshes/V40/RThumb1_0.10.stl" scale="0.1 0.1 0.1"/>
      </geometry>
      <origin rpy="4.712 0 1.578" xyz="0.002 -0.007 0"/>
    </collision>
  </link>
  <joint name="RThumb1" type="continuous">
    <parent link="r_wrist"/>
    <child link="RThumb1_link"/>
    <origin rpy="-1.5708 0.0472984 -3.26826e-08" xyz="0.04895 0 -0.02638"/>
    <axis xyz="0 0 1.0"/>
    <mimic joint="RHand" multiplier="0.999899" offset="0"/>
  </joint>
  <link name="RThumb2_link">
    <inertial>
      <mass value="2e-06"/>
      <inertia ixx="1.1e-09" ixy="0" ixz="0" iyy="1.1e-09" iyz="0" izz="1.1e-09"/>
    </inertial>
    <visual>
      <geometry>
        <mesh filename="package://nao_meshes/meshes/V40/RThumb2.dae" scale="0.1 0.1 0.1"/>
      </geometry>
      <origin rpy="-4.712 0 0" xyz="0 0 0"/>
    </visual>
    <collision>
      <geometry>
        <mesh filename="package://nao_meshes/meshes/V40/RThumb2_0.10.stl" scale="0.1 0.1 0.1"/>
      </geometry>
      <origin rpy="-4.712 0 0" xyz="0 0 0"/>
    </collision>
  </link>
  <joint name="RThumb2" type="continuous">
    <parent link="RThumb1_link"/>
    <child link="RThumb2_link"/>
    <origin rpy="0 0 -0.999899" xyz="0.01436 0 0"/>
    <axis xyz="0 0 1.0"/>
    <mimic joint="RHand" multiplier="0.999899" offset="0"/>
  </joint>
  <link name="LThumb2_link">
    <inertial>
      <mass value="2e-06"/>
      <inertia ixx="1.1e-09" ixy="0" ixz="0" iyy="1.1e-09" iyz="0" izz="1.1e-09"/>
    </inertial>
    <visual>
      <geometry>
        <mesh filename="package://nao_meshes/meshes/V40/LThumb2.dae" scale="0.1 0.1 0.1"/>
      </geometry>
      <origin rpy="1.578 0 0" xyz="0 0 0"/>
    </visual>
    <collision>
      <geometry>
        <mesh filename="package://nao_meshes/meshes/V40/LThumb2_0.10.stl" scale="0.1 0.1 0.1"/>
      </geometry>
      <origin rpy="-1.578 0 0" xyz="0 0 0"/>
    </collision>
  </link>
  <joint name="LThumb2" type="continuous">
    <parent link="LThumb1_link"/>
    <child link="LThumb2_link"/>
    <origin rpy="0 0 -0.999899" xyz="0.01436 0 0"/>
    <axis xyz="0 0 1.0"/>
    <mimic joint="LHand" multiplier="0.999899" offset="0"/>
  </joint>
</robot>
'''

# <parent link="RThigh"/>
# <child link="RTibia"/>

pprinter = PrettyPrinter()
pprint = lambda content: pprinter.pprint(content)
cm = ConfigManager()
cm.load({'urdf':URDF,'fixed_frame':'base_link'})
# pprint(cm.meta['robot_tree'])
pprint(cm.data['joint_names'])
pprint([name for name,info in cm.meta['robot_tree']['joints'].items() if info['parent']=='RTibia' and info['dynamic']])
pprint(cm.meta['robot_tree']['joints']['RAnklePitch'])
exit()
