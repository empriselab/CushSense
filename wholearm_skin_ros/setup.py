from setuptools import setup, find_packages
import pathlib

here = pathlib.Path(__file__).parent.resolve()

setup(
    name='wholearm_skin_ros',
    version='0.1',
    packages=find_packages(where=''),
    python_requires='>=3.5, <4',
)
