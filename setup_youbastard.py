import os
from setuptools import setup

# Utility function to read the README file.
# Used for the long_description.  It's nice, because now 1) we have a top level
# README file and 2) it's easier to type in the README file than to put a raw
# string in below ...
def read(fname):
    return open(os.path.join(os.path.dirname(__file__), fname)).read()

setup(
    name = "youbastard",
    version = "0.0.4",
    author = "Guillaume Fontaine",
    author_email = "guillaume.fontai@gmail.com",
    description = ("A general geometry tool in 3D, named after the best mathematician of the discworld, the camel YouBastard"),
    license = "LGPL",
    keywords = "geometry",
    url = None,
    packages=['youbastard'],
    long_description=read('README.md'),
    classifiers=[
    ],
)
