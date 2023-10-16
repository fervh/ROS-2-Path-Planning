from setuptools import find_packages, setup
import glob

package_name = 'planification'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/general.launch.py']),
        ('share/' + package_name + '/config', ['config/rviz_config.rviz']),
        ('share/' + package_name + '/data', glob.glob('data/map*')),
        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fernando',
    maintainer_email='fernando@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'readcsv = planification.readcsv:main',
            'occupancygrid = planification.occupancygrid:main',
            'startend = planification.startend:main',
            'astar = planification.astar:main',
            'bfs = planification.bfs:main',
            'greedy = planification.greedy:main',
        ],
    },
)
