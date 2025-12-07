from setuptools import find_packages, setup

package_name = 'basic_nodes'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Basic ROS 2 nodes for humanoid robotics',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'basic_publisher = basic_nodes.publisher_member_function:main',
            'basic_subscriber = basic_nodes.subscriber_member_function:main',
        ],
    },
)