from setuptools import find_packages, setup

package_name = 'vlm_ros2'

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
    maintainer='mr_robot',
    maintainer_email='kaunghtethtun2013@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'test_action_with_VLA = vlm_ros2.test_action_with_VLA:main',
            'nav2_llm_node = vlm_ros2.nav2_llm_node:main',
            'rom_nav2_no_llm_server = vlm_ros2.rom_nav2_no_llm_server:main',
            'rom_nav2_vlm_server = vlm_ros2.rom_nav2_vlm_server:main',
        ],
    },
)
