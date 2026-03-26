from setuptools import find_packages, setup
import os

package_name = 'ros2_studio'

# Collect all image files in the package directory
data_files = [
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    ('share/' + package_name, ['README.md']),
]

# Include any images in the package directory
image_files = [
    f for f in os.listdir(package_name)
    if f.endswith(('.png', '.jpg', '.jpeg', '.gif'))
] if os.path.isdir(package_name) else []

if image_files:
    data_files.append(
        ('share/' + package_name + '/images',
         [os.path.join(package_name, f) for f in image_files])
    )

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sourav Hawaldar',
    maintainer_email='sourav.hawaldar@gmail.com',
    description='Comprehensive ROS2 monitoring and management tool with GUI',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'ros2cli.command': [
            'studio = ros2_studio.command.studio:StudioCommand',
        ],
        'console_scripts': [
            'ros2_studio_gui = ros2_studio.main:main',
        ],
    },
)