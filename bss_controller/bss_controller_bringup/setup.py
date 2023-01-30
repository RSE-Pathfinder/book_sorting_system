from setuptools import setup

package_name = 'bss_controller_bringup'

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
    maintainer='paulyong',
    maintainer_email='2102088@sit.singaporetech.edu.sg',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arm_action_server = bss_controller_bringup.arm_action_server:main',
            'arm_action_client = bss_controller_bringup.arm_action_client:main',
        ],
    },
)
