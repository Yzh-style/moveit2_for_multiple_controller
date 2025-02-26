import glob
import os
from setuptools import find_packages, setup

package_name = 'welding_kinematic_controller'

# Helper function to create relative paths
def get_relative_paths(base_dir, pattern):
    files = glob.glob(os.path.join(base_dir, pattern))
    return [os.path.relpath(f, base_dir) for f in files]

# Define base directory
base_dir = os.path.abspath(os.path.dirname(__file__))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'urdf/meshes/welder'), get_relative_paths(base_dir, 'urdf/meshes/welder/*.stl')),
        (os.path.join('share', package_name, 'urdf/meshes/positioner'), get_relative_paths(base_dir, 'urdf/meshes/positioner/*.stl')),
        (os.path.join('share', package_name, 'urdf'), get_relative_paths(base_dir, 'urdf/*.xacro')),
        (os.path.join('share', package_name, 'launch'), get_relative_paths(base_dir, 'launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), get_relative_paths(base_dir, 'config/config.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Zeynep Sertkaya, Zonghai Yang, Jiani Zhu',
    maintainer_email='zeynep.sertkaya@rwth-aachen.de',
    description='Welding Robot Kinematic Controller',
    license='RWTH Aachen',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'line_marker_publisher = welding_kinematic_controller.line_marker:main',
            'lookup_transform_publisher = welding_kinematic_controller.lookup_transform:main',
        ],
    },
)
