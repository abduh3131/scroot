from setuptools import setup

package_name = 'kuksa_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Malyka Sardar',
    maintainer_email='malyka.sardar@ontariotechu.ca',
    description='ROS2 node bridging vehicle data to Kuksa Databroker',
    license='MIT',
    entry_points={
        'console_scripts': [
            'kuksa_bridge_node = kuksa_bridge.kuksa_bridge_node:main',
        ],
    },
)

