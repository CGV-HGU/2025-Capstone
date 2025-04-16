from setuptools import find_packages, setup

package_name = 'fake_lidar_with_tf'

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
    maintainer='cgv-02',
    maintainer_email='cgv-02@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'fake_lidar_with_tf = fake_lidar_with_tf.fake_lidar_with_tf:main',
    ],
},
)
