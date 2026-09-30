from setuptools import find_packages, setup

package_name = 'vlm_nav_bridge'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Oli',
    maintainer_email='olafurbrekigudnason@gmail.com',
    description='Pixel -> map -> Nav2 goal bridge (ARIES VLM/VLA nav stack, mini-project 4.1)',
    license='MIT',
    entry_points={
        'console_scripts': [
            'pixel_to_goal = vlm_nav_bridge.pixel_to_goal:main',
        ],
    },
)
