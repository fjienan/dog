from setuptools import find_packages, setup

package_name = 'omni_mpc_controller'

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
    maintainer='dell',
    maintainer_email='dell@todo.todo',
    description='Omnidirectional MPC Controller for two robots',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'omni_mpc_controller_node = omni_mpc_controller.omni_mpc_controller_node:main',
        ],
    },
)
