from setuptools import setup

package_name = 'joy_cap'

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
    py_modules=[
        "Custom_Joystick"
    ],
    entry_points={
        'console_scripts': [
            "joy_pub = joy_cap.joystick_publisher:main",
            "joy_sub = joy_cap.joystick_subscriber:main",
        ],
    },
)
