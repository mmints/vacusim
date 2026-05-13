from setuptools import setup

package_name = 'vacusim_robot_planning'


data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name + '/launch', ['launch/planner_launch.py']))
data_files.append(('share/' + package_name + '/maps', ['maps/default_map.txt']))
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
    description='A* planning node for VacuSim',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'planner_node = vacusim_robot_planning.planner_node:main',
        ],
    },
)
