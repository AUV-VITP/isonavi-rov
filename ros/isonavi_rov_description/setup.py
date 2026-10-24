from glob import glob
import os
from setuptools import setup

PKG = "isonavi_rov_description"

setup(
    name=PKG,
    version="0.1.0",
    packages=[PKG],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{PKG}"]),
        (f"share/{PKG}", ["package.xml"]),
        (f"share/{PKG}/urdf", glob("urdf/*")),
        (f"share/{PKG}/models/isonavi_rov", glob("models/isonavi_rov/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
)
