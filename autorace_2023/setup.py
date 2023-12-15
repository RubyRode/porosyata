from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'autorace_2023'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'msg'), glob(os.path.join('msg', '*.msg')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ruby',
    maintainer_email='dimaskrut71@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "detect_lane = autorace_2023.detect_lane:main",
            "mov_ctrl = autorace_2023.mov_ctrl:main",
            "pid_lane = autorace_2023.pid_lane:main"
        ],
    },
)
