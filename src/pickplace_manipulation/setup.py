from setuptools import find_packages, setup

package_name = 'pickplace_manipulation'

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
    maintainer='sweety_rathod',
    maintainer_email='sweetylappy9@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [   'move_to_cube = pickplace_manipulation.move_to_cube:main',
        'pick_descend = pickplace_manipulation.pick_descend:main',            'pick_place = pickplace_manipulation.pick_place:main',
                    'random_cube = pickplace_manipulation.random_cube:main',
                                'run_trials = pickplace_manipulation.run_trials:main',
        ],
    },
)
