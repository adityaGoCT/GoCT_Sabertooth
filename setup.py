from setuptools import setup

package_name = 'py_sabertooth'

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
    maintainer='aditya',
    maintainer_email='adityajoshi451@gmail.com',
    description='Sabertooth motor control',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'cmd_vel_to_diff=py_sabertooth.cmd_vel_to_diff_wheels:main',
	    'diff_to_motor =py_sabertooth.sabertooth_node:main',
        ],
    },
)
