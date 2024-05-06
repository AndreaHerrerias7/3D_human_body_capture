from setuptools import find_packages
from setuptools import setup

setup(
    name='datos_camara',
    version='0.0.0',
    packages=find_packages(
        include=('datos_camara', 'datos_camara.*')),
)
