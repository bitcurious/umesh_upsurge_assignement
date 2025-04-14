from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'champ_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='umesh',
    maintainer_email='umeshmane280@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'raycaster_node = champ_perception.raycaster_node:main',
        'pointcloud_processor = champ_perception.pointcloud_processor:main',
        'occupancy_mapper = champ_perception.occupancy_mapper:main',
    ],
},
)
