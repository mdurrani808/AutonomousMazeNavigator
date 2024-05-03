import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'final'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'worlds'), glob(os.path.join('worlds', '*.world*'))),
        (os.path.join('share', package_name, 'models','tb3_4walls'), glob(os.path.join('models','tb3_4walls','*'))),
        (os.path.join('share', package_name, 'models','maze_0'), glob(os.path.join('models','maze_0','*'))),
        (os.path.join('share', package_name, 'models','maze_1'), glob(os.path.join('models','maze_1','*'))),
        (os.path.join('share', package_name, 'models','maze_2'), glob(os.path.join('models','maze_2','*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mrc-ipen',
    maintainer_email='mrc-ipen@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_node = final.my_node:main',
            'my_calc_node = final.my_calc_node:main',
            'pranav_code = final.pranav:main'
        ],
    },
)
