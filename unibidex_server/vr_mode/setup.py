from setuptools import setup, find_packages

def setup_package():
    setup(
        name='banana_teleop_server',
        version='0.1',
        description='Bimanual teleoperation server',
        author='Dieselmarble',
        author_email='lizhongxuanchina@gmail.com',
        packages=find_packages(),
        install_requires=[
            'numpy>=1.24.4,<2.0',
            'scipy',
            'pybind11>=2.4.3',
            'cython>=0.29.18',
            'h5py',
            'tqdm'
        ],
        entry_points={
            'console_scripts': [
                'run_vision_server=banana_teleop_server.nodes.bimanual_detector_node:main',
                'run_vision_monitor=banana_teleop_server.nodes.bimanual_monitor_node:main'
            ],
        },
    )

if __name__ == "__main__":
    setup_package()
