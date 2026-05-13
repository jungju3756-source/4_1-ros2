from setuptools import find_packages, setup

package_name = 'py_launch_example'

setup(
    name=package_name,
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/my_launch.py']),
    ],
    install_requires=['setuptools'],
    entry_points={
        'console_scripts': [
            'pub = py_launch_example.pub:main',
            'sub = py_launch_example.sub:main',
        ],
    },
)
