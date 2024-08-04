from setuptools import setup
import os
from glob import glob

package_name = 'ak_driver_status'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
		(os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
		(os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*[yaml]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='corey',
    maintainer_email='imapsrobotics@hotmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
			'status = ak_driver_status.ak_driver_status:main',
            "control = ak_driver_status.control:main"
        ],
    },
)
