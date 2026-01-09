from setuptools import find_packages, setup

package_name = 'policy'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    package_data={
        f'{package_name}': ['*.pth', '*.pt', '*.onnx', '*.bin'],
    },
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zgf',
    maintainer_email='guofeng.zhu@sihua.tech',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'policy = policy.infer:main'
        ],
    },
)
