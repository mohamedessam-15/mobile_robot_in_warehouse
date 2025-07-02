from setuptools import find_packages, setup

package_name = 'control'

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
    maintainer='essam',
    maintainer_email='essam@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            "fake_odom_publisher = control.fake_odom_publisher:main",
            "encoder_speed_node = control.encoder_speed_node:main",
            "odom_node = control.odom_node:main",
            "logic = control.logic:main",
            "functions = control.functions:main"
        ],
    },
)
