from setuptools import setup

package_name = 'arch_test'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='samuel',
    maintainer_email='samyzaitoun@campus.technion.ac.il',
    description='Integration Testing Package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test_interfaces = arch_test.test_interfaces:main',
            'test_planner = arch_test.test_planner:main',
            'test_manager = arch_test.test_manager:main',
            'test_architecture = arch_test.test_architecture:main',
            'test_live = arch_test.test_live:main'
        ],
    },
)
