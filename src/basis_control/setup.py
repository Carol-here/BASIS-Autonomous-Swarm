from setuptools import find_packages, setup

package_name = 'basis_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='caro',
    maintainer_email='carol07jb@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
		'swarm_launcher = basis_control.phase2.swarm_launcher:main',
        	'swarm_controller = basis_control.phase2.swarm_controller:main',
		'swarm_state_collector = basis_control.phase3.swarm_state_collector:main',
        ],
    },
)
