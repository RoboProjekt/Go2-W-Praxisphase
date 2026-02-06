from setuptools import setup
import os
from glob import glob

package_name = 'r3d_preprocessor'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # HIER: Wir installieren den maps Ordner in das share-Verzeichnis des Packages
        (os.path.join('share', package_name, 'maps'), glob('maps/*.pkl')),
        # Falls du auch .pcd Dateien hast:
        (os.path.join('share', package_name, 'maps'), glob('maps/*.pcd')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bauya',
    maintainer_email='baau1001@stud.hs-kl.de',
    description='Map generation and publishing for R3D',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Name des Befehls = package.file:main_funktion
            'pcd_to_graph = r3d_preprocessor.r3d_pcd_to_graph:main',
            'voxel_map_publisher = r3d_preprocessor.r3d_voxel_map_publisher:main',
        ],
    },
)
