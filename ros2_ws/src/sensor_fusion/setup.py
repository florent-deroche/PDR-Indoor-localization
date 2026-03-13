from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'sensor_fusion'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='Lois',
    maintainer_email='loisgonce.pro@gmail.com',
    description='Sensor Fusion with 2D Kalman Filter of wheel odometry and BLE trilateration',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fusion_node = sensor_fusion.fusion_node:main',
        ],
    },
)