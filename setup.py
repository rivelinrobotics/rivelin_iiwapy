from catkin_pkg.python_setup import generate_distutils_setup
from setuptools import setup

# fetch values from package.xml
config = generate_distutils_setup(packages=["iiwapy"], package_dir={"": "src"})
setup(**config)
