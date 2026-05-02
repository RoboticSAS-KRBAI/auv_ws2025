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
            "guidance_mission = auv_pkg.node_guidance_teensy_mission1:main",
            "guidance_flare_gate = auv_pkg.node_guidance_sauvc_fg:main",
            'object_detection = auv_pkg.node_object_detection:main',
            "pub_teensy = auv_pkg.publisher_teensy:main",
            "tes_gui = auv_pkg.gui.gui_node:main",
            "tes_gui_v2 = auv_pkg.gui.tes_gui_v2:main",
            "pub_teensy_serial_simple = auv_pkg.publisher_teensy_serial:main",
            "pid_logger_pub = auv_pkg.pid_logger_pub:main",
            "guidance_sauvc_qualification = auv_pkg.node_guidance_sauvc_qualification:main",
            "guidance_new = auv_pkg.node_guidance_new:main",
            "accumulator_new = auv_pkg.node_accumulator_new:main",
            "object_detection_new = auv_pkg.node_object_detection_new:main",
            "serial_monitoring_safety = auv_pkg.node_monitoring_safety:main",
            "serial_monitoring_safety_and_bucket = auv_pkg.node_monitoring_safety_and_bucket:main",
            "accumulator_fadhil = auv_pkg.node_sauvc_accumulator_fadhil:main",
            "map_sauvc = auv_pkg.node_sauvc_map_new:main",
            "pub_yaw_only = auv_pkg.node_old_pub_yaw_only:main",
            "guidance_sauvc_fadhil = auv_pkg.node_sauvc_guidance_fadhil:main",
            "guidance_full_reynard = auv_pkg.node_guidance_full_reynard:main",
            "bucket_detection = auv_pkg.node_bucket:main"
        ],
    },
)
