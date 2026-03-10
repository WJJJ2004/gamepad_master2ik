from setuptools import setup

package_name = 'gamepad_master2ik'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/gamepad_master2ik.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lwj',
    maintainer_email='lwj@example.com',
    description='Gamepad to Master2Ik bridge node',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gamepad_master2ik_node = gamepad_master2ik.gamepad_master2ik_node:main',
        ],
    },
)
