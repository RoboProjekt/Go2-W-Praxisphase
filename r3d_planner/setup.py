from setuptools import setup
import os
from glob import glob

package_name = 'r3d_planner'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Hier installieren wir die Config Datei, damit launch files sie finden
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bauya',
    maintainer_email='baau1001@stud.hs-kl.de',
    description='R3D 3D Navigation Package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Das registriert die Executables für 'ros2 run'
            'global_planner = r3d_planner.r3d_global_planner:main',
            'local_filter = r3d_planner.r3d_local_filter:main',
        ],
    },
)
