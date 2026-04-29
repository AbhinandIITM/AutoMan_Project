from glob import glob

from setuptools import find_packages, setup
import os
package_name = 'dual_ur5'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files = [
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),

        ('share/' + package_name, ['package.xml']),

        # Launch files
        ('share/' + package_name + '/launch', glob('launch/*.py')),

        # URDF / Xacro
        ('share/' + package_name + '/urdf', glob('urdf/*.urdf')),
        ('share/' + package_name + '/urdf', glob('urdf/*.xacro')),

        # ✅ CONFIG (CRITICAL for ros2_control)
        ('share/' + package_name + '/config', glob('config/*.yaml')),

        # ✅ RVIZ
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
        ('share/' + package_name + '/worlds', glob('worlds/*.world')),
        # Meshes
        ('share/' + package_name + '/urdf/meshes/visual',
            glob('urdf/meshes/visual/*')),

        ('share/' + package_name + '/urdf/meshes/collision',
            glob('urdf/meshes/collision/*')),
    ] + [(os.path.join('share', package_name, root), [os.path.join(root, f) for f in files]) for root, _, files in os.walk('models') if files],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='abhinand',
    maintainer_email='153114742+AbhinandIITM@users.noreply.github.com',
    description='Dual-arm AutoMan simulation package',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'gui_bridge = dual_ur5.gui_bridge:main',
            'set_joint_angles = dual_ur5.set_joint_angles:main',
            'inv_kin = dual_ur5.inv_kin:main',
        ],
    },
)
