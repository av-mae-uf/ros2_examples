from setuptools import setup

package_name = 'tf2_examples'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='patrick',
    maintainer_email='neap@ufl.edu',
    description='Contains example code for using TF2 with Python',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tf2_broadcaster_example = tf2_examples.tf2_broadcaster:main',
            'tf2_listener_example = tf2_examples.tf2_listener:main'
        ],
    },
)
