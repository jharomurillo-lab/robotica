from setuptools import find_packages, setup

package_name = 'grupo01_xarm6_kinematics'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Grupo 01 - Ivan y Rodrigo',
    maintainer_email='imt342.robotica@ucb.edu.bo',
    description='Cinemática Directa e Inversa del UFACTORY xArm6',
    license='MIT',
    entry_points={
        'console_scripts': [
            'fk_node = grupo01_xarm6_kinematics.fk_node:main',
            'ik_node = grupo01_xarm6_kinematics.ik_node:main',
        ],
    },
)
