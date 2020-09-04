from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=['pam_models'],
    package_dir={'': 'python'},
)

setup(**setup_args)
