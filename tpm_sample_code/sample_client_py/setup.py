from setuptools import setup

package_name = 'sample_client_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tpm',
    maintainer_email='tpm@tpm-pac.com',
    description='a sample client node in python',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sample_client_py = sample_client_py.entry:main'
        ],
    },
)
