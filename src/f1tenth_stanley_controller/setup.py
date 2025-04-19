from setuptools import setup

package_name = 'f1tenth_stanley_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Manually list the Python files in the launch directory
        ('share/' + package_name + '/launch', [
            'launch/controller_launch.py'
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Rohan Singh',
    maintainer_email='singh.rohan@kgpian.iitkgp.ac.in',
    description='F1Tenth ICRA 2025 code for Autonomous Ground Vehicles, Indian Institute of Technology, Kharagpur',
    license='License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'controller_node = f1tenth_stanley_controller.controller_node:main',
            'wheel_odom_node = f1tenth_stanley_controller.wheel_odom_node:main',
        ],
    },
)

