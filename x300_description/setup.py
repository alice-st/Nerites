from setuptools import setup
import glob

package_name = 'x300_description'

setup(
    name=package_name,
    version='0.0.0',
    data_files=[
        # Install the package.xml file
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include URDF and mesh files in the package
        ('share/' + package_name + '/robots', ['robots/x300.urdf']),
        ('share/' + package_name + '/robots', ['robots/x300.urdf.xacro']),
        ('share/' + package_name + '/robots', ['robots/x300.gazebo.xacro']),
        ('share/' + package_name + '/robots', ['robots/x300_actuators.xacro']),
        ('share/' + package_name + '/robots', ['robots/x300_base.xacro']),
        ('share/' + package_name + '/robots', ['robots/x300_default.urdf.xacro']),
        ('share/' + package_name + '/config', ['config/TAM.yaml']),
        ('share/' + package_name + '/models/x300/meshes', glob.glob('models/x300/meshes/*')),
        # Include the model.config and model.sdf in the models directory
        ('share/' + package_name + '/models/x300', [
            'models/x300/model.config',
            'models/x300/model.sdf'
        ]),
        ('share/' + package_name + '/launch', ['launch/upload_x300_default.launch.py']),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Robot description package for the x300 robot.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
