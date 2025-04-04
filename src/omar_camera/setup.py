from setuptools import find_packages, setup

package_name = 'omar_camera'

def _entry_point(exec_name:str) -> str:
    return f"{exec_name} = {package_name}.{exec_name}:main"

def _data_file(to_:list[str], from_:str) -> tuple[str, list[str]]:
    return (to_, from_)

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
    maintainer='ustad',
    maintainer_email='merenbasol@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            _entry_point("compressor"),
            _entry_point("decompressor"),
            _entry_point("show")
        ],
    },
)
