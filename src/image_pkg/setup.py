from setuptools import find_packages, setup

package_name = 'image_pkg'

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
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "image_to_topic_node = image_pkg.image_to_topic_16bit:main",
            "image_to_topic_node_8bit = image_pkg.image_to_topic_8bit:main",
        ],
    },
)
