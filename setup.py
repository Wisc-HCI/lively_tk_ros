from setuptools import setup
import glob
import os

package_name = 'lively_ik'
config_files = glob.glob(os.path.join('config', '*', '*'))
launch_files = glob.glob(os.path.join('launch', '*'))

data_files = [
    ('share/ament_index/resource_index/packages',['resource/' + package_name]),
    ('share/' + package_name, ['package.xml'])
]
for config_file in config_files:
    data_files.append(('share/' + package_name + '/' + config_file,[config_file]))

for launch_file in launch_files:
    data_files.append(('share/' + package_name + '/' + launch_file,[launch_file]))

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='schoen',
    maintainer_email='schoen.andrewj@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    scripts=['src/Node.jl'],
    entry_points={
        'console_scripts': [
            'manager = lively_ik.manager:main'
        ],
    },
)

# TODO figure out Julia stuff
