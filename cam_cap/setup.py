from setuptools import setup

package_name = 'cam_cap'

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
    maintainer='adas',
    maintainer_email='adas@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "pub = cam_cap.camera_publisher:main",
            "sub = cam_cap.camera_subscriber:main",
            "pub_compressed = cam_cap.camera_publisher_compressed:main",
            "sub_compressed = cam_cap.camera_subscriber_compressed:main",
            "3cams = cam_cap.camera_publisher_3cams:main",
        ],
    },
)
