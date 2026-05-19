from setuptools import find_packages, setup

package_name = 'robotics_project'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    ('share/' + package_name + '/launch', ['launch/city.launch.py']),
    ('share/' + package_name + '/worlds', ['worlds/city.world']),
    ('share/' + package_name + '/urdf',   ['urdf/car.urdf']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='taab',
    maintainer_email='sahaabmansha333@gmail.com',
    description='Autonomous Ground Robot Navigation – RRT* + Pure Pursuit',
    license='TODO: License declaration',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            # Member 1 – global planner
            'rrt_planner=robotics_project.global_planner.rrt_planner:main',
            # Member 2 – pure pursuit controller  ← ADD THIS
            'pure_pursuit_controller=robotics_project.local_controller.pure_pursuit:main',
        ],
    },
)
