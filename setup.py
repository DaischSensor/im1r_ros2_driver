from setuptools import find_packages, setup

package_name = 'im1r_ros2_driver'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    py_modules=[
        'im1r_ros2_driver.daisch_im1r_node',
        'im1r_ros2_driver.parser',
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='daisch',
    maintainer_email='daisch@todo.todo',
    description='IM1R ROS2 Driver package for ROS 2',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "im1r_node = im1r_ros2_driver.daisch_im1r_node:main"
        ],
    },
)
