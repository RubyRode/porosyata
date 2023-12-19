from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'detect_signs'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'images/templates'), glob(os.path.join('images/templates', '*.png'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='apim',
    maintainer_email='a.pimonov@g.nsu.ru',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'detect = detect_signs.detect:main',
            'pid = detect_signs.pid:main',
        ],
    },
)
