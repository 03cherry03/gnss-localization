from setuptools import setup

package_name = 'gnss_twist_publisher'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='GNSS to Twist Publisher',
    license='License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gnss_twist_publisher = gnss_twist_publisher.gnss_twist_publisher:main'
        ],
    },
)
