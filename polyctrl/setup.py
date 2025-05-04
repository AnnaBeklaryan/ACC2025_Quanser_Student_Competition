from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'polyctrl'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'utils'), glob('utils/*.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Package for MPC and PID nodes for QCar2 control',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'MPC_node = polyctrl.MPC_node:main',
            'PID_node = polyctrl.PID_node:main',
            'Detection_node = polyctrl.Detection_node:main',
        ],
    },
)
