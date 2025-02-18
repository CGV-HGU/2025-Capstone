from setuptools import find_packages, setup

package_name = 'segmentation_to_map'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'opencv-python', 'numpy'],
    zip_safe=True,
    maintainer='cgv',
    maintainer_email='iam@hsl.ee',
    description='Segmentation data to map coordinate transformation',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'segmentation_to_map = segmentation_to_map.node:main',
        ],
    },
)
