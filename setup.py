from setuptools import find_packages, setup

package_name = 'routecam_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['launch/routecam.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ryank',
    maintainer_email='rkaddis@ltu.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            f'routecam = {package_name}.routecam:main'
        ],
    },
)
