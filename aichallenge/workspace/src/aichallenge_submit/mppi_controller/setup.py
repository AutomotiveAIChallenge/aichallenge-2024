import subprocess
import sys

# dependency
subprocess.check_call([sys.executable, '-m', 'pip', 'install', 'setuptools==58.0.4', '--quiet'])
subprocess.check_call([sys.executable, '-m', 'pip', 'install', 'torch', '--index-url', 'https://download.pytorch.org/whl/cpu', '--quiet'])
subprocess.check_call([sys.executable, '-m', 'pip', 'install', 'transforms3d', '--quiet'])

from setuptools import setup, find_packages
package_name = 'mppi_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/mppi_controller.param.yaml']),
    ],
    install_requires=['setuptools', 'scipy'],
    zip_safe=True,
    maintainer='michikuni eguchi',
    maintainer_email='egrt117@gmail.com',
    description='MPPI controller',
    license='License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mppi_controller_node = mppi_controller.mppi_controller_node:main',
        ],
    },
)