from glob import glob
import os

from setuptools import setup


package_name = 'imrt_virtual_joy'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer=' ',
    maintainer_email=' ',
    description=' ',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'imrt_virtual_joy = imrt_virtual_joy.gamepad:main',
            'imrt_teleop_node = imrt_virtual_joy.imrt_teleop_node:main'
        ],
    },
)
