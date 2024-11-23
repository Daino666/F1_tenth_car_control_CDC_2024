from setuptools import find_packages, setup

package_name = 'car_control'

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
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gap = car_control.gap_following:main',
            'pure_pursuit_algo = car_control.Pure_pursuit:main',
            'CSV_maker  = car_control.CSV_maker:main',
            'wall_following  = car_control.wall_following:main'
        ],
    },
)#
