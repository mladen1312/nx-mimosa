#!/usr/bin/env python3
"""NX-MIMOSA - Setup Configuration"""

from setuptools import setup, find_packages
import os

def get_version():
    return '1.0.0'

def get_long_description():
    readme_file = os.path.join(os.path.dirname(__file__), 'README.md')
    if os.path.exists(readme_file):
        with open(readme_file, 'r', encoding='utf-8') as f:
            return f.read()
    return ''

setup(
    name='nx-mimosa',
    version=get_version(),
    author='Dr. Mladen Mešter',
    author_email='mladen@nexellum.com',
    description='Production Multi-Domain Radar Tracking System',
    long_description=get_long_description(),
    long_description_content_type='text/markdown',
    url='https://github.com/mladen1312/nx-mimosa',
    packages=find_packages(where='.'),
    python_requires='>=3.8',
    install_requires=[
        'numpy>=1.21.0,<2.0.0',
        'scipy>=1.7.0',
    ],
    extras_require={
        'dev': ['pytest>=7.0.0', 'pytest-cov>=4.0.0', 'black>=23.0.0', 'flake8>=6.0.0'],
        'docs': ['sphinx>=6.0.0', 'sphinx-rtd-theme>=1.2.0'],
        'viz': ['matplotlib>=3.4.0', 'pandas>=1.3.0'],
    },
    classifiers=[
        'Development Status :: 5 - Production/Stable',
        'License :: OSI Approved :: GNU Affero General Public License v3',
        'Programming Language :: Python :: 3',
        'Programming Language :: Python :: 3.8',
        'Programming Language :: Python :: 3.9',
        'Programming Language :: Python :: 3.10',
        'Programming Language :: Python :: 3.11',
        'Programming Language :: Python :: 3.12',
    ],
    keywords=['radar', 'tracking', 'kalman-filter', 'ukf', 'imm', 'jpda', 'mht', 'aerospace'],
)
