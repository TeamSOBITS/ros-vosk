from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'speech_recognition_vosk'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[ (os.path.join('share', package_name), glob('launch/*.py')),
    ],
    install_requires=['setuptools', 'vosk-model-downloader'],
    zip_safe=True,
    maintainer='sobits',
    maintainer_email='sobits@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'trigger = speech_recognition_vosk.trigger:main',
            'vosk_model_downloader = speech_recognition_vosk.vosk_model_downloader:main',
            'vosk_node = speech_recognition_vosk.vosk_node:main',
        ],
    },
)
