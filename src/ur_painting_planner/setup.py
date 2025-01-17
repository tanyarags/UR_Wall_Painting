from setuptools import setup

package_name = 'ur_painting_planner'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your@email.com',
    description='Wall painting planner for UR10e',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'painting_node2 = ur_painting_planner.painting_node2:main',
        ],
    },
)