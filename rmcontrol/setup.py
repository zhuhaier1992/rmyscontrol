from setuptools import setup
from glob import glob
import os

package_name = 'rmcontrol'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zhe',
    maintainer_email='zhuhaier1992@163.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'agent_control = rmcontrol.agent_control:main',
            'strategy = rmcontrol.strategy:main',
            'rvo = rmcontrol.rvo:main',
            'draw_plot = rmcontrol.draw_plot:main',
        ],
    },
)
