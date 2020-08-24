from setuptools import setup
import glob
import os
from julia import Pkg

package_name = 'lively_ik'
config_files = glob.glob(os.path.join('config', '*', '*'))
launch_files = glob.glob(os.path.join('launch', '*'))
rviz_files = glob.glob(os.path.join('rviz', '*'))

data_files = [
    ('share/ament_index/resource_index/packages',['resource/' + package_name]),
    ('share/' + package_name, ['package.xml'])
]
for config_file in config_files:
    base = os.path.dirname(config_file)
    data_files.append(('share/' + package_name + '/' + base,[config_file]))

for launch_file in launch_files:
    data_files.append(('share/' + package_name + '/',[launch_file]))

for rviz_file in rviz_files:
    data_files.append(('share/' + package_name + '/',[rviz_file]))

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='schoen',
    maintainer_email='schoen.andrewj@gmail.com',
    description='The LivelyIK Package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    scripts=['lively_ik/Node.jl'],
    entry_points={
        'console_scripts': [
            'manager = lively_ik.manager:main',
            'visualize = lively_ik.visualize:main',
            'control = lively_ik.control:main',
            'param = lively_ik.param:main',
            'simple = lively_ik.simple_pub:main'
        ],
    },
)

# TODO figure out Julia stuff
Pkg.add(PackageSpec(path="lively_ik"))
J.call()
