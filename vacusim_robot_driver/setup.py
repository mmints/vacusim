'''
Author: Mark O. Mints (mmints@uni-koblenz.de)
'''

from setuptools import setup

package_name = 'vacusim_robot_driver'
data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name + '/launch', ['launch/simple_arena_launch.py']))
data_files.append(('share/' + package_name + '/launch', ['launch/apartment_launch.py']))
data_files.append(('share/' + package_name + '/launch', ['launch/empty_apartment_launch.py']))
data_files.append(('share/' + package_name + '/launch', ['launch/large_apartment_launch.py']))
data_files.append(('share/' + package_name + '/launch', ['launch/empty_large_apartment_launch.py']))
data_files.append(('share/' + package_name + '/launch', ['launch/robot_driver_launch.py']))

data_files.append(('share/' + package_name + '/worlds', ['worlds/simple_arena.wbt']))
data_files.append(('share/' + package_name + '/worlds', ['worlds/apartment.wbt']))
data_files.append(('share/' + package_name + '/worlds', ['worlds/empty_apartment.wbt']))
data_files.append(('share/' + package_name + '/worlds', ['worlds/large_apartment.wbt']))
data_files.append(('share/' + package_name + '/worlds', ['worlds/empty_large_apartment.wbt']))

data_files.append(('share/' + package_name + '/controllers/ground', ['controllers/ground/dust.jpg']))
data_files.append(('share/' + package_name + '/controllers/ground/', ['controllers/ground/build/release/ground']))

data_files.append(('share/' + package_name + '/resource', ['resource/robot.urdf']))
data_files.append(('share/' + package_name + '/protos', ['protos/Custom_iRobot_Create.proto']))
data_files.append(('share/' + package_name + '/protos/textures', ['protos/textures/create_base_color.jpg']))
data_files.append(('share/' + package_name + '/protos/textures', ['protos/textures/create_roughness.jpg']))
data_files.append(('share/' + package_name + '/protos/textures', ['protos/textures/create_normal.jpg']))
data_files.append(('share/' + package_name + '/protos/textures', ['protos/textures/create_occlusion.jpg']))
data_files.append(('share/' + package_name, ['package.xml']))

setup(
    name=package_name,
    version='0.2.0',
    packages=[package_name],
        data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mark O. Mints',
    maintainer_email='mmints@uni-koblenz.de',
    description='Assignment for ECV (B): Programming Robots Using ROS 2',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vacusim_robot_driver = vacusim_robot_driver.vacusim_robot_driver:main'
        ],
    },
)
