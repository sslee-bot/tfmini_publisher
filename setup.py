from setuptools import setup

package_name = 'tfmini_publisher'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='sslee-bot',
    maintainer_email='physism@gmail.com',
    description='ROS 2 node to read TFmini Plus LiDAR data via UART',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tfmini_publisher_node = tfmini_publisher.tfmini_publisher_node:main',
        ],
    },
)

