from setuptools import find_packages
from setuptools import setup

setup(
    name='cafe_bot',
    version='0.0.0',
    packages=find_packages(
        include=('cafe_bot', 'cafe_bot.*')),
)
