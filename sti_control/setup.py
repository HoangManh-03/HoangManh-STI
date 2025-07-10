import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'sti_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        (os.path.join('share', package_name, 'launch'), 
         glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='stivietnam',
    maintainer_email='stivietnam@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'stiClient = sti_control.stiClient:main',
            'stiClient_socketio = sti_control.stiClient_socketio:main',
            'stiControl = sti_control.stiControl:main',
        ],
    },
)
