#!/bin/python

import distribute_setup
distribute_setup.use_setuptools()
from setuptools import setup, Extension, find_packages

classifiers = ['Development Status :: 3 - Alpha',
               'Operating System :: POSIX :: Linux',
               'License :: OSI Approved :: MIT License',
               'Intended Audience :: Developers',
               'Programming Language :: Python :: 2.6',
               'Programming Language :: Python :: 2.7',
               'Programming Language :: Python :: 3',
               'Topic :: Software Development',
               'Topic :: Home Automation',
               'Topic :: System :: Hardware']

setup(name             = 'Ox.GPIO',
      version          = '0.4.1a',
      author           = 'Joshua Hampp',
      author_email     = 'joshua.hampp@ipa.fraunhofer.de',
      description      = 'A class to control Odroid-X GPIO channels',
      long_description = open('README.txt').read() + open('CHANGELOG.txt').read(),
      license          = 'GPL',
      keywords         = 'GPIO',
      classifiers      = classifiers,
      packages         = find_packages(),
      ext_modules      = [Extension('Ox.GPIO', ['src/py_gpio.c', 'src/c_gpio.c'], include_dirs=[])])
