import os
from glob import glob
from setuptools import setup

package_name = 'arch_components'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))), # Adds launch files to the package
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='samuel',
    maintainer_email='samyzaitoun@campus.technion.ac.il',
    description='Project Components',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'planner = arch_components.planner:main',
            'manager = arch_components.manager:main',
        ],
    },
)
