from setuptools import setup

package_name = 'motion_capture'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'motion_capture = motion_capture.motion_capture:main',
            'inbetween = motion_capture.inbetween:main',
        ],
    },
)
