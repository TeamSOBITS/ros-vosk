from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'speech_recognition_vosk'

def package_data_files(src_dir, dest_dir):
    data_files = []
    for root, dirs, files in os.walk(src_dir):
        for file in files:
            src_file_path = os.path.join(root, file)
            dest_file_path = os.path.join(dest_dir, os.path.relpath(root, src_dir))
            data_files.append((dest_file_path, [src_file_path]))
    return data_files

setup(
    name=package_name,
    version='0.0.0',
    # packages=find_packages(exclude=['test']),
    packages=[package_name],
    data_files=[
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ] + package_data_files('models', os.path.join('share', package_name, 'models')),
    # install_requires=['setuptools', 'vosk-model-downloader'],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sobits',
    maintainer_email='sobits@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    # tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'model_downloader = speech_recognition_vosk.model_downloader:main',
            'vosk_node = speech_recognition_vosk.vosk_node:main',
        ],
    },
)
