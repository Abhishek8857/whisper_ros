from setuptools import find_packages, setup

package_name = 'whisper_listen'

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
    maintainer='abhishek',
    maintainer_email='abhishek@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['to_text = whisper_listen.to_text:main',
                            'text_to_rosa = whisper_listen.text_to_rosa:main',
                            'dummy = whisper_listen.publish_test:main'
        ],
    },
)
