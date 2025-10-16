from setuptools import setup
import os
from glob import glob

package_name = 'global_planner'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # launch
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))),

        # # config
        # (os.path.join('share', package_name, 'config'),
        #     glob(os.path.join('config', '*'))),

        *[
            (os.path.join('share', package_name, os.path.dirname(f)),
             [f])
            for f in glob('config/**/*', recursive=True)
            if os.path.isfile(f)
        ],


        # scripts
        (os.path.join( 'lib', package_name),
            glob(os.path.join('scripts', '*'))),

    ],
    install_requires=['setuptools', 'trajectory-planning-helpers'],
    zip_safe=True,
    maintainer='ForzaETH',
    maintainer_email='nicolas.baumann@pbl.ee.ethz.ch',
    description='The global planner package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'global_planner = global_planner.global_planner_node:main',
            'global_planner_offline = global_planner.global_planner_offline_node:main',
            'global_trajectory_publisher = global_planner.global_trajectory_publisher:main'
        ],
    },
)
