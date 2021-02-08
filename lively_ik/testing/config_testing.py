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

yaml_string = '''
axis_types:
- [z, z, z, z, z, z, z]
base_link_motion_bounds:
- [0, 0]
- [0, 0]
- [0, 0]
static_environment:
    spheres: []
    cuboids: []
    pcs: []
disp_offsets:
- [0.0, 0.0, 0.333]
displacements:
- - [0.0, 0.0, 0.0]
  - [0.0, -0.316, 0.0]
  - [0.0825, 0.0, 0.0]
  - [-0.0825, 0.384, 0.0]
  - [0.0, 0.0, 0.0]
  - [0.088, 0.0, 0.0]
  - [0.0, 0.0, 0.107]
fixed_frame: panda_link0
ee_fixed_joints: [panda_joint8]
goals:
- name: default
  goals:
  - vector: [1.0,0.5,0.25]
    weight: 5
  - quaternion: [1,0,0,0]
    weight: 5
  - scalar: 0.03
    weight: 20
  - weight: 30
  - weight: 10
  - weight: 20
  - weight: 100
  - weight: 1
  - weight: 1
  - weight: 0.1
  - weight: 2.0
  - weight: 5.0
joint_limits:
- [-2.9671, 2.9671]
- [-1.8326, 1.8326]
- [-2.9671, 2.9671]
- [-3.1416, 0.0873]
- [-2.9671, 2.9671]
- [-0.0873, 3.8223]
- [-2.9671, 2.9671]
- [0.0, 0.04]
- [0.0, 0.04]
joint_names:
- [panda_joint1, panda_joint2, panda_joint3, panda_joint4, panda_joint5, panda_joint6,panda_joint7]
joint_ordering: [panda_joint1, panda_joint2, panda_joint3, panda_joint4, panda_joint5, panda_joint6, panda_joint7, panda_finger_joint1, panda_finger_joint2]
joint_types:
- [revolute, revolute, revolute, revolute, revolute, revolute, revolute]
mode_control: absolute
mode_environment: ECAA
nn_main:
    coefs:
        - - [ 0.003985830013447632, 0.006725205820258482, 0.0029430774802702018, -0.0180975079318873, -0.008740493235075752, 0.009681578407093315, 0.013176887259211434, 0.019552203319950008, 0.007340945898508483, -0.01976900036452251, -0.00037313810141402894, 0.0015464459975115697, -0.005243660561450487, -0.0030195482703034696, -0.016839154818828608 ]
          - [ -0.03953382844997415, -0.4493063510098432, 1.1425623965176515, -0.4772486311043159, -0.02680082220029155, 0.3132717399074815, -0.42120224606445184, -0.0762941303967958, -1.0647040619753165, -0.6763981534703989, 0.011421728258359028, 0.04361734575324665, 0.10884856946075061, -0.9746974395670763, 0.4657616790041838 ]
          - [ 0.03161979548224097, 0.7231587851045483, -0.0028110319736152894, 0.5881942651876335, -0.00856106786930193, 1.067444988094614, -0.5658882883310457, -0.20549832857371386, 0.007514262317741042, -0.01675111246237722, -0.005999860558633437, -0.027222912627611576, 0.6873293442930538, 0.03179277022590048, -0.04161233768352949 ]
    intercepts:
        - [ -0.9137286925169599, 1.052737850989435, -0.041441299686975804, 0.5953145295524237, 0.6250385158509061, 0.1448818202963769, 1.3819921357959233, -1.2754704887777293, -0.406457920008248, -0.8190155875480795, 0.3257077627705829, -0.3425333300374119, -0.45770881638596844, 0.9571895372397862, -0.20233238758621963 ]
        - [ -0.012912593776023006, 0.04201748501978764, 0.24329158536857462, 1.3614340792316262, -0.6651836619264188, -0.9759335946785755, 0.9628469838544244, -0.709000754149039, 0.7149627646642095, 1.3758402986640188, -0.7099859619585349, 0.8244932254588426, -0.656711318096124, 0.7438390166867931, -0.6876822075381197 ]
        - [ -0.018767074497088473, 1.3271115658835928, -0.05472535314596514, 1.1185848114825305, 1.3256154602917924, -0.012622258554159929, -0.1612064682666347, -0.34278023688528775, -0.012875888208829504, -1.2424249048627218, 1.858459828275621, 0.8679577801928821, -0.018679379378624357, -0.013314873339754963, 1.0066214851270665 ]
        - [ -0.04303836378972717, 2.282855843074993, -0.24301006710615364, 3.3125986876106066, -0.24703653201249792, -0.03409405056594006, 0.9971049885234112, -0.3934707032164864, -0.4070127213058031, 0.4806589282135768, -0.23275502701403838, -0.0037685418150549617, -0.4198185943388448, -0.13192285174730137, -0.4423074021093166 ]
        - [ 0.7750750826862081, -0.11695119257076615, 0.3812102285480506, -0.11471575486836305, 0.42722274588545495, -0.098271127916013, -0.444220456740873, -0.0697964614272632, -0.19779276942378723, -0.02963812122536514, 0.8929098858704745, 0.9976919138893771, -0.2896500729574561, -0.006551949691720627, -0.2483537250720149 ]
        - [ 0.43176422855265884 ]
    split_point: 4.89
nn_jointpoint:
    coefs:
        - - [ 1.1078950379750274e-12, 2.8784241827743017e-06, -1.7768236747083063e-09, -3.044871121930853e-06, 1.3039845643771108e-12, 0.0001540429627508986, 2.765233958146012e-08, -2.1539412206429114e-13, -1.0135817520528143e-12, -1.7242019896254512e-09, -1.7684757709717013e-07, -4.4908725177452956e-07, -2.2029406089714117e-06, -1.8542219783535501e-07, -1.3226610035795642e-11 ]
          - [ 1.1294443397930104e-05, -1.548818662393586e-09, 1.1858127713396543e-07, 4.279200047386968e-12, 1.8856468674960752e-11, -0.00010113262483078852, 3.3682615503053044e-14, 1.2063667585096615e-05, 4.1285e-319, -2.3095506861346532e-11, -2.8855578417607474e-08, 1.4885088669003848e-11, 8.89518660809176e-06, 2.3447505616892414e-06, -2.1188171253704032e-05 ]
          - [ 0.1446385096545734, 3.268008543081875e-186, 0.05813065670054119, -8.647159570159481e-148, -0.05819773976164197, 0.06292058955703948, 0.10742187914543538, 0.07071466546463004, 0.045651852113121304, 5.076385010604459e-293, -0.011385596664808383, -0.07837384068939049, 0.14189197277739019, 0.02254320769333527, 4.302922676062015e-62 ]
          - [ 2.2918697716206403e-07, 5.059021561218665e-10, 2.4632e-319, -2.751358997034878e-06, -2.151125990509931e-06, -2.875279367259269e-12, 0.00013187196811153743, 4.173255984419567e-11, 5.852500107242325e-06, 1.448640344724711e-12, 3.9267931002505744e-13, -3.1175506452297096e-13, -1.0527821976465252e-09, 2.7924154190532574e-07, 1.0781850988452419e-08 ]
          - [ 7.214465785459145e-13, 2.8975316413494524e-06, 6.419394018519474e-11, -7.347348384648939e-11, -7.087003351651472e-08, -3.05515e-319, 1.6852834443018348e-06, 3.162165214390654e-10, 8.162085360158558e-12, 8.26296866990947e-12, -1.8117552484994145e-07, 2.2790494652857478e-10, 1.5508100671171408e-10, -2.870333490438286e-09, 7.710936745466818e-06 ]
          - [ 0.13109717382930658, 3.2654637742405614e-186, 0.05993041569997088, -8.637178545509506e-148, -0.058053681938631, 0.13569361516228323, 0.13712700973649866, 0.03820168263151696, 0.031147271187494635, 5.02130979042465e-293, -0.01138586744551199, -0.08770253014742735, 0.1723283741461906, 0.04250657520071422, 4.302922693298448e-62 ]
          - [ 0.0004416245449008482, 1.5248439941030267e-215, -0.11892661174282275, 3.521506481983556e-166, -0.018604692085767083, 0.0024938782854790503, -0.14096213868260507, 0.17740360698546073, 0.15188531939111788, 4.33734787e-315, 0.0103688461218697, -0.031326804732518476, -0.003850505520959452, 0.0285330129695488, 1.4414720407193935e-76 ]
          - [ -0.005508293353934522, -5.797901746895952e-208, -0.12277219259327532, 2.0198334826007345e-164, -0.006725529543465857, -0.00781780844996289, 0.18279129178660342, -0.15354885663258092, 0.10638302958454483, 1.93384117958e-312, 0.026553262733034492, 0.012082049257363708, -0.008742859811795664, -0.03869821229845684, 4.42766116954603e-71 ]
          - [ 0.022162330520806305, 3.6863843609324933e-171, -0.03642505198522418, -1.1434689669653097e-133, 0.49128085745753697, 0.20325761345193635, -0.2776435493412891, -0.09805835623044262, -0.10338176214209834, -4.91475545759572e-280, -0.07138411740935327, 0.8552356615042821, -1.2157485884981782, 0.5105528362416679, 8.575797853196608e-54 ]
    intercepts:
        - [ 0.4424729580496465, -0.007484933936992695, -0.0169959398698863, -9.628227113782902e-06, -0.11015082732942602, 0.6069440909681623, -0.08907824471319171, -0.0734960832612773, -0.0039379930899484435, -0.0001473786359513204, -0.027023319219268063, -0.1944185224967221, 0.6679964936255317, 0.4500537536208132, -0.008678931023701915 ]
        - [ -0.15179389945867028, 0.3081417304530315, 0.38504765593227946, 0.5406833379989318, 0.20192405773134395, 0.6210806129696388, 0.5384071558706596, 0.12946345659719868, 0.03650556531833388, 0.2149614468936943, -0.06962193600902189, 0.33397547609977796, -0.05579921022583934, 0.2739047955830142, -0.012520255767678262 ]
        - [ 0.3005070722277035, -0.0016778574643300306, -0.07942085890730087, 0.3331105506104369, -0.035232908162506606, -0.285132320978239, -0.02487762519728127, -0.00012049770129009652, -0.0007407690070151807, -0.31525425429170506, 2.1696495477664657, 0.321599032600653, -0.08525362361761976, -0.10641959767069427, -0.061277558538595914 ]
        - [ 0.04590013827169419, -0.3940437121987022, -0.4286874850844137, 0.782596139924694, -0.2849953747378892, 0.49182297087667926, 0.45785928810008836, 0.9594525300260861, -0.44500534339014375, -0.03011030432008525, -0.23079544675282518, 0.8840186905603491, -0.009381469667688129, 0.5170844293940459, -0.03328640420899679 ]
        - [ -0.5737281038948252, 0.6350666333740317, -0.7538788267432469, 0.5325909413775315, 0.4216349590013047, -0.09365028709290363, -0.014579854057239973, -0.151969878599525, -0.4170902384308002, 0.6260916532930153, 0.519540955903691, 0.36707703631303096, -0.09520295898981662, 0.7184435364040391, 0.4858265407912395 ]
        - [ 0.14211476672033146 ]
    split_point: 4.97
objectives:
- {index: 0, variant: ee_position_match, weight: 30}
- {index: 0, variant: ee_orientation_match, weight: 25}
- {index: 7, variant: joint_match, weight: 20}
- {frequency:  5, index: 0, scale: 0.15, variant: ee_position_liveliness,weight: 30}
- {frequency:  5, index: 0, scale: 1.0, variant: ee_orientation_liveliness,weight: 10}
- {frequency:  5, index: 7, scale: 0.01, variant: joint_liveliness, weight: 20}
- {index: 7, secondary_index: 8, variant: joint_mirroring, weight: 100}
- {variant: min_velocity, weight: 1}
- {variant: min_acceleration, weight: 1}
- {variant: min_jerk, weight: 0.1}
- {variant: joint_limits, weight: 2.0}
- {variant: nn_collision, weight: 5.0}
robot_link_radius: 0.045
robot_name: panda
rot_offsets:
- - [0.0, 0.0, 0.0]
  - [-1.57079632679, 0.0, 0.0]
  - [1.57079632679, 0.0, 0.0]
  - [1.57079632679, 0.0, 0.0]
  - [-1.57079632679, 0.0, 0.0]
  - [1.57079632679, 0.0, 0.0]
  - [1.57079632679, 0.0, 0.0]
  - [0.0, 0.0, 0.0]
starting_config: [0, 0, 0, -1.52715, 0, 1.8675, 0.8, 0.03, 0.02]
states:
- [0, 0, 0, -1.52715, 0, 1.8675, 0, 0.02, 0.02]
- [-1.74, -0.05, 0, -1.71, 0, 1.8675, 0, 0.02, 0.03]
urdf: '<?xml version="1.0" ?><!-- =================================================================================== --><!-- | This document was autogenerated by xacro from /opt/ros/kinetic/share/moveit_resources/panda_description/urdf/panda.urdf | --><!-- | EDITING THIS FILE BY HAND IS NOT RECOMMENDED | --><!-- =================================================================================== --><robot name="panda" xmlns:xacro="http://www.ros.org/wiki/xacro"><link name="panda_link0"><visual><geometry><mesh filename="package://moveit_resources_panda_description/meshes/visual/link0.dae"/></geometry></visual><collision><geometry><mesh filename="package://moveit_resources_panda_description/meshes/collision/link0.stl"/></geometry></collision></link><link name="panda_link1"><visual><geometry><mesh filename="package://moveit_resources_panda_description/meshes/visual/link1.dae"/></geometry></visual><collision><geometry><mesh filename="package://moveit_resources_panda_description/meshes/collision/link1.stl"/></geometry></collision></link><joint name="panda_joint1" type="revolute"><safety_controller k_position="100.0" k_velocity="40.0" soft_lower_limit="-2.8973" soft_upper_limit="2.8973"/><origin rpy="0 0 0" xyz="0 0 0.333"/><parent link="panda_link0"/><child link="panda_link1"/><axis xyz="0 0 1"/><limit effort="87" lower="-2.9671" upper="2.9671" velocity="2.3925"/></joint><link name="panda_link2"><visual><geometry><mesh filename="package://moveit_resources_panda_description/meshes/visual/link2.dae"/></geometry></visual><collision><geometry><mesh filename="package://moveit_resources_panda_description/meshes/collision/link2.stl"/></geometry></collision></link><joint name="panda_joint2" type="revolute"><safety_controller k_position="100.0" k_velocity="40.0" soft_lower_limit="-1.7628" soft_upper_limit="1.7628"/><origin rpy="-1.57079632679 0 0" xyz="0 0 0"/><parent link="panda_link1"/><child link="panda_link2"/><axis xyz="0   0 1"/><limit effort="87" lower="-1.8326" upper="1.8326" velocity="  2.3925"/></joint><link name="panda_link3"><visual><geometry><mesh filename="package://moveit_resources_panda_description/meshes/visual/link3.dae"/></geometry></visual><collision><geometry><mesh filename="package://moveit_resources_panda_description/meshes/collision/link3.stl"/></geometry></collision></link><joint name="panda_joint3" type="revolute"><safety_controller k_position="100.0" k_velocity="40.0" soft_lower_limit="-2.8973" soft_upper_limit="2.8973"/><origin rpy="1.57079632679 0 0" xyz="0 -0.316 0"/><parent link="panda_link2"/><child link="panda_link3"/><axis xyz="0 0 1"/><limit effort="87" lower="-2.9671" upper="2.9671" velocity="2.3925"/></joint><link name="panda_link4"><visual><geometry><mesh filename="package://moveit_resources_panda_description/meshes/visual/link4.dae"/></geometry></visual><collision><geometry><mesh filename="package://moveit_resources_panda_description/meshes/collision/link4.stl"/></geometry></collision></link><joint name="panda_joint4" type="revolute"><safety_controller k_position="100.0" k_velocity="40.0" soft_lower_limit="-3.0718" soft_upper_limit="0.0175"/><origin rpy="1.57079632679 0 0" xyz="0.0825 0 0"/><parent link="panda_link3"/><child link="panda_link4"/><axis xyz="0 0 1"/><limit effort="87" lower="-3.1416" upper="0.0873" velocity="2.3925"/></joint><link name="panda_link5"><visual><geometry><mesh filename="package://moveit_resources_panda_description/meshes/visual/link5.dae"/></geometry></visual><collision><geometry><mesh filename="package://moveit_resources_panda_description/meshes/collision/link5.stl"/></geometry></collision></link><joint name="panda_joint5" type="revolute"><safety_controller k_position="100.0" k_velocity="40.0" soft_lower_limit="-2.8973" soft_upper_limit="2.8973"/><origin rpy="-1.57079632679 0 0" xyz="-0.0825 0.384 0"/><parent link="panda_link4"/><child link="panda_link5"/><axis xyz="0 0 1"/><limit effort="12" lower="-2.9671" upper="  2.9671" velocity="2.8710"/></joint><link name="panda_link6"><visual><geometry><mesh filename="package://moveit_resources_panda_description/meshes/visual/link6.dae"/></geometry></visual><collision><geometry><mesh filename="package://moveit_resources_panda_description/meshes/collision/link6.stl"/></geometry></collision></link><joint name="panda_joint6" type="revolute"><safety_controller k_position="100.0" k_velocity="40.0" soft_lower_limit="-0.0175" soft_upper_limit="3.7525"/><origin rpy="1.57079632679 0 0" xyz="0 0 0"/><parent link="panda_link5"/><child link="panda_link6"/><axis xyz="0   0 1"/><limit effort="12" lower="-0.0873" upper="3.8223" velocity="  2.8710"/></joint><link name="panda_link7"><visual><geometry><mesh filename="package://moveit_resources_panda_description/meshes/visual/link7.dae"/></geometry></visual><collision><geometry><mesh filename="package://moveit_resources_panda_description/meshes/collision/link7.stl"/></geometry></collision></link><joint name="panda_joint7" type="revolute"><safety_controller k_position="100.0" k_velocity="40.0" soft_lower_limit="-2.8973" soft_upper_limit="2.8973"/><origin rpy="1.57079632679 0 0" xyz="0.088 0 0"/><parent link="panda_link6"/><child link="panda_link7"/><axis xyz="0 0 1"/><limit effort="12" lower="-2.9671" upper="2.9671" velocity="2.8710"/></joint><link name="panda_link8"/><joint name="panda_joint8" type="fixed"><origin rpy="0 0 0" xyz="0 0   0.107"/><parent link="panda_link7"/><child link="panda_link8"/><axis xyz="0 0 0"/></joint><joint name="panda_hand_joint" type="fixed"><parent link="panda_link8"/><child link="panda_hand"/><origin rpy="0 0 -0.785398163397" xyz="0 0 0"/></joint><link name="panda_hand"><visual><geometry><mesh filename="package://moveit_resources_panda_description/meshes/visual/hand.dae"/></geometry></visual><collision><geometry><mesh filename="package://moveit_resources_panda_description/meshes/collision/hand.stl"/></geometry></collision></link><link name="panda_leftfinger"><visual><geometry><mesh filename="package://moveit_resources_panda_description/meshes/visual/finger.dae"/></geometry></visual><collision><geometry><mesh filename="package://moveit_resources_panda_description/meshes/collision/finger.stl"/></geometry></collision></link><link name="panda_rightfinger"><visual><origin rpy="0 0 3.14159265359" xyz="0 0 0"/><geometry><mesh filename="package://moveit_resources_panda_description/meshes/visual/finger.dae"/></geometry></visual><collision><origin rpy="0 0 3.14159265359" xyz="0 0 0"/><geometry><mesh filename="package://moveit_resources_panda_description/meshes/collision/finger.stl"/></geometry></collision></link><joint name="panda_finger_joint1" type="prismatic"><parent link="panda_hand"/><child link="panda_leftfinger"/><origin rpy="0 0 0" xyz="  0 0 0.0584"/><axis xyz="0 1 0"/><limit effort="20" lower="  0.0" upper="0.04" velocity="0.2"/></joint><joint name="panda_finger_joint2" type="prismatic"><parent link="panda_hand"/><child link="panda_rightfinger"/><origin rpy="0 0 0" xyz="0 0 0.0584"/><axis xyz="0 -1 0"/><limit effort="20" lower="0.0" upper="  0.04" velocity="0.2"/><mimic joint="panda_finger_joint1"/></joint></robot>'
velocity_limits: [2.3925, 2.3925, 2.3925, 2.3925, 2.871, 2.871, 2.871, 10, 10]
'''


pprinter = PrettyPrinter()
pprint = lambda content: pprinter.pprint(content)
data = yaml.safe_load(yaml_string)
cm = ConfigManager()

# elements = []
#
# for field in cm._fields.keys():
#     elements.append({'data':{'id': field, 'label': field}})
#     for derivation in cm._fields[field]['dependencies']:
#         elements.append({'data':{'source': field, 'target': derivation},'classes':'dep'})
#     # for guard in cm._fields[field]['guards']:
#     #     elements.append({'data':{'source': field, 'target': guard},'classes':'guard'})
#
# import dash
# import dash_core_components as dcc
# import dash_html_components as html
# import dash_cytoscape as cyto
# from dash.dependencies import Input, Output
# import plotly.express as px
#
# app = dash.Dash(__name__)
#
# stylesheet = [
#     {
#         "selector": 'node', #For all nodes
#         'style': {
#             "opacity": 0.9,
#             "label": "data(label)", #Label of node to display
#             "background-color": "#07ABA0", #node color
#             "color": "#008B80" #node label color
#         }
#     },
#     {
#         "selector": 'edge.dep', #For all edges
#         "style": {
#             "target-arrow-color": "#C5D3E2", #Arrow color
#             "target-arrow-shape": "triangle", #Arrow shape
#             "line-color": "#C5D3E2", #edge color
#             'arrow-scale': 2, #Arrow size
#             'curve-style': 'bezier' #Default curve-If it is style, the arrow will not be displayed, so specify it
#         }
#     },
#     {
#         "selector": 'edge.guard', #For all guard edges
#         "style": {
#             "target-arrow-color": "##F8C0CF", #Arrow color
#             "target-arrow-shape": "triangle", #Arrow shape
#             "line-color": "##F8C0CF", #edge color
#             'arrow-scale': 2, #Arrow size
#             'curve-style': 'bezier' #Default curve-If it is style, the arrow will not be displayed, so specify it
#         }
#     }
# ]
#
# app.layout = html.Div([
#     html.P("Dash Cytoscape:"),
#     cyto.Cytoscape(
#         id='cytoscape',
#         elements=elements,
#         layout={'name': 'cose'},#{'name': 'breadthfirst', 'roots':'#urdf, #mode_control, #mode_environment, #robot_link_radius, #static_environment, #base_link_motion_bounds'},
#         style={'width': '1280px', 'height': '800px'},
#         stylesheet=stylesheet
#     )
# ])
#
# app.run_server(debug=True)
# exit()
#
# cm.load({'urdf':data['urdf']})
# pprint(cm.simplified())
# pprint(cm.meta)

cm.load(data)
pprint(cm.simplified())
pprint(cm.meta)
exit()
config = parse_config_data(data)
solver = LivelyIK(config)
print(solver.solve(config.default_goals,9.0))




dfdata = {joint:[] for joint in cm.joint_ordering}

num_solves = 1000

solver_goals = parse_config_data(cm.goals[0]['goals'])

print('running with liveliness')
# for i in range(num_solves):
while True:
    try:
        time = datetime.utcnow().timestamp()
        # print(time)
        trans,sol = cm.solver.solve(solver_goals,time)
        print(sol)
        for j,v in enumerate(sol):
            dfdata[cm.joint_ordering[j]].append(sol[j])
    except (KeyboardInterrupt, SystemExit):
        break

df = pd.DataFrame(dfdata,index=list(range(len(dfdata['panda_joint1']))))
df.plot.line()
show()
