from setuptools import find_packages, setup

package_name = 'auv_pkg'

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
    zip_safe=False,
    maintainer='techsas',
    maintainer_email='techsas@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "test_node = auv_pkg.first_node:main",
            "accumulator = auv_pkg.node_accumulator:main",
            "guidance_teensy = auv_pkg.node_guidance_teensy:main",
            'object_detection = auv_pkg.node_object_detection:main',
            "pub_teensy = auv_pkg.publisher_teensy:main",
            "tes_gui = auv_pkg.gui.gui_node:main",
            "tes_gui_v2 = auv_pkg.gui.tes_gui_v2:main",
        ],
    },
)
