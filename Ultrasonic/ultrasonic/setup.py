from setuptools import find_packages, setup

package_name = 'ultrasonic'

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
    maintainer='Michael',
    maintainer_email='Michael@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'ultrasonic_publisher = ultrasonic.ultrasonic_publisher:main',
        'ultrasonic_subscriber = ultrasonic.ultrasonic_subscriber:main',
        ],
    },
)
