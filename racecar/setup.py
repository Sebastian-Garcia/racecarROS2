from setuptools import setup, find_packages, Extension
import glob
import os
package_name = 'racecar'


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/racecar/config', glob.glob(os.path.join('config', '*.yaml'))),
        ('share/racecar/launch', glob.glob(os.path.join('launch', '*launch.xml'))),
        ('share/racecar/launch', glob.glob(os.path.join('launch', '*launch.py')))
    ],
    install_requires=['setuptools', "Cython"],
    zip_safe=True,
    maintainer='sebastian',
    maintainer_email='sebastianag2002@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "chainRelay = racecar.chainRelay:main",
            "safetyRelay = racecar.safetyRelay:main",
            "teleopRelay = racecar.teleopRelay:main",
            "navRelay = racecar.navRelay:main",
            "defaultCmd = racecar.defaultCmd:main"
        ],
    },
    
)
