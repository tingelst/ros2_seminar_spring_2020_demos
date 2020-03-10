from setuptools import setup

package_name = 'rpi_joint_jogger'

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
    maintainer='lars',
    maintainer_email='lars.tingelstad@ntnu.no',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rpi_joint_jogger_node = rpi_joint_jogger.rpi_joint_jogger_node:main'
        ],
    },
)
