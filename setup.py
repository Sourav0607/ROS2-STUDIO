from setuptools import find_packages, setup

package_name = 'ros2_studio'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'PyQt5',
        'psutil',
        'matplotlib',
    ],
    zip_safe=True,
    maintainer='sourav',
    maintainer_email='sourav.hawaldar@gmail.com',
    description='Comprehensive ROS2 monitoring and management tool with GUI',
    license='Apache License 2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'ros2cli.command': [
            'studio = ros2_studio.command.studio:StudioCommand',
        ],
        'console_scripts': [
            'ros2_studio_gui = ros2_studio.main:main',
        ],
    },
)
