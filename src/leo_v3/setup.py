from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'leo_v3'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        # Include model files
        (os.path.join('share', package_name, 'models'), glob(os.path.join('models', '*.[dae|stl]*'))),
        (os.path.join('share', package_name, 'meshes'), glob(os.path.join('meshes', '*.[dae|stl]*'))),
        (os.path.join('share', package_name, 'px150_meshes'), glob(os.path.join('px150_meshes', '*.[dae|stl]*'))),
        # Include URDF (.urdf) files
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*.urdf.xacro*'))),
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*.gazebo.xacro*'))),
        # Include rviz (.rviz) files
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*.rviz'))),
        # Include world (.sdf or dae) files
        (os.path.join('share', package_name, 'worlds'), glob(os.path.join('worlds', '*/*.[sd][da][fe]'), recursive=True)),
        (os.path.join('share', package_name, 'worlds'), glob(os.path.join('worlds', '*.sdf'))),
        # Include config (.yaml) files
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
	# Include map (.yaml and .pgm) files
        (os.path.join('share', package_name, 'maps'), glob(os.path.join('maps', '*.[yp][ag][m]'))),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nathan',
    maintainer_email='nathanshankar465@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'manipulator_control_gui = leo_v3.manipulator_control_gui:main',
            'stretch_ignition_control_action_server = leo_v3.stretch_ignition_control_action_server:main',
        ],
    },
)
