from setuptools import find_packages, setup

package_name = 'sorting_intelligence'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
        (
            'share/' + package_name + '/launch',
            ['launch/sorting_pipeline.launch.py']
        ),
    ],
    package_data={'': ['py.typed']},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sumoghh',
    maintainer_email='sumoghh@todo.todo',
    description='ROS 2 intelligence pipeline for robotic sorting workcell',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'task_manager = sorting_intelligence.task_manager:main',
            'perception_node = sorting_intelligence.perception_node:main',
        ],
    },
)
