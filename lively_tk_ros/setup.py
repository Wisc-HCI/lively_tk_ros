from setuptools import setup
import glob
import os

package_name = 'lively_tk_ros'
config_files = glob.glob(os.path.join('config', '*'))
launch_files = glob.glob(os.path.join('launch', '*'))

data_files = [
    ('share/ament_index/resource_index/packages',['resource/' + package_name]),
    ('share/' + package_name, ['package.xml'])
]
for config_file in config_files:
    base = os.path.dirname(config_file)
    data_files.append(('share/' + package_name + '/' + base,[config_file]))

for launch_file in launch_files:
    data_files.append(('share/' + package_name + '/',[launch_file]))

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools','scikit-learn','numpy','python-fcl','urdf-parser-py'],
    zip_safe=True,
    maintainer='schoen',
    maintainer_email='schoen.andrewj@gmail.com',
    description='The LivelyTK ROS2 Package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'interface = lively_tk_ros.interface:main',
            'solver = lively_tk_ros.solver:main'
        ],
    },
)
