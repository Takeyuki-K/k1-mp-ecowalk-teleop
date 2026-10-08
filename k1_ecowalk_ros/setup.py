from glob import glob
from setuptools import setup

pkg = 'k1_ecowalk_ros'
setup(
    name=pkg,
    version='0.1.0',
    packages=[pkg],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + pkg]),
        ('share/' + pkg, ['package.xml']),
        ('share/' + pkg + '/launch', glob('launch/*.launch.py')),
        ('share/' + pkg + '/web', glob('web/*')),
        ('share/' + pkg + '/config', glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    license='Apache-2.0',
    entry_points={'console_scripts': ['k1_sim = k1_ecowalk_ros.sim_node:main']},
)
