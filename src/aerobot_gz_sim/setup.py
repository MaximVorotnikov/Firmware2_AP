from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'aerobot_gz_sim'


def data_files_from(directory):
    data_files = []
    for path, _, files in os.walk(directory):
        if files:
            data_files.append((
                os.path.join('share', package_name, path),
                [os.path.join(path, file) for file in files],
            ))
    return data_files


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'environment'),
            [os.path.join('env-hooks', 'aerobot_gz_sim.dsv')]),
    ] + data_files_from('worlds') + data_files_from('models'),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='max',
    maintainer_email='lada.vorotnikova68@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
