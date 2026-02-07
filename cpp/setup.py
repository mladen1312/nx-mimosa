"""Build NX-MIMOSA C++ core extension.

Usage:
    pip install ./cpp          # install from source
    python cpp/setup.py build  # build only
"""
import os
import sys
from setuptools import setup
from pybind11.setup_helpers import Pybind11Extension, build_ext

# Find Eigen
eigen_include = "/usr/include/eigen3"
if not os.path.exists(eigen_include):
    # Try pkg-config
    import subprocess
    try:
        eigen_include = subprocess.check_output(
            ["pkg-config", "--cflags-only-I", "eigen3"]
        ).decode().strip().replace("-I", "")
    except Exception:
        eigen_include = "/usr/include/eigen3"

ext_modules = [
    Pybind11Extension(
        "_nx_core",
        ["src/bindings.cpp"],
        include_dirs=["include", eigen_include],
        extra_compile_args=["-O3", "-march=native", "-fopenmp", "-std=c++17"],
        extra_link_args=["-fopenmp"],
        define_macros=[("NDEBUG", "1")],
    ),
]

setup(
    name="nx-mimosa-core",
    version="6.0.0",
    author="Dr. Mladen Mešter",
    author_email="mladen@nexellum.com",
    description="NX-MIMOSA C++ tracking core — sub-50ms at 1000+ tracks",
    ext_modules=ext_modules,
    cmdclass={"build_ext": build_ext},
    python_requires=">=3.9",
    install_requires=["numpy>=1.21", "pybind11>=2.10"],
)
