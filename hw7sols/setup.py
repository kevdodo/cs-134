from setuptools import find_packages, setup
from glob import glob

package_name = 'hw7sols'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robot',
    maintainer_email='robot@todo.todo',
    description='The 133a HW7 Solutions Code',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hw7p2sol = hw7sols.hw7p2sol:main',
            'hw7p3sol = hw7sols.hw7p3sol:main',
            'hw7p4sol = hw7sols.hw7p4sol:main',
        ],
    },
)
