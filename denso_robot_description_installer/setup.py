from setuptools import setup

package_name = 'denso_robot_description_installer'

setup(
    name=package_name,
    version='1.2.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='DENSO WAVE INCORPORATED',
    maintainer_email='fa-support@denso-wave.com',
    description='The denso_robot_description_installer package contains utilities for'\
    + ' installing robot description and configuration files for DENSO robots'\
    + ' (as created by the DENSO ROS2 Converter Tool).',
    license='MIT',
    entry_points={
        'console_scripts': [
            'install_robot_description = denso_robot_description_installer.install_robot_description:main',
            'update_joint_limits = denso_robot_description_installer.update_joint_limits:main',
        ],
    },
)
