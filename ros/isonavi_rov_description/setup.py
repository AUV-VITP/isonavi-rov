from glob import glob
from setuptools import setup

PKG = "isonavi_rov_description"

setup(
    name=PKG,
    version="0.1.0",
    packages=[PKG],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{PKG}"]),
        (f"share/{PKG}", ["package.xml"]),
        (f"share/{PKG}/models/isonavi_rov", glob("models/isonavi_rov/*.*")),
        (f"share/{PKG}/models/isonavi_rov/meshes", glob("models/isonavi_rov/meshes/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
)
