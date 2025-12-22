from setuptools import setup

package_name = 'straw'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
         ['launch/strawberry_world.launch.py']),
        ('share/' + package_name + '/worlds', ['worlds/strawberry_world.sdf']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tianqi',
    maintainer_email='tianqi@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'straw_node = straw.straw_node:main',
            'camera_viewer = straw.camera_viewer:main',
            'simple_image_capture = straw.simple_image_capture:main',
            'test_image_topics = straw.test_image_topics:main',
        ],
    },
)
