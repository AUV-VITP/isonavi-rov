from glob import glob

from setuptools import setup

PKG = "isonavi_rov_ros"

setup(
    name=PKG,
    version="0.1.0",
    packages=[PKG],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{PKG}"]),
        (f"share/{PKG}", ["package.xml"]),
        (f"share/{PKG}/launch", glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    entry_points={"console_scripts": [
        f"pilot = {PKG}.pilot_node:main",
        f"safety = {PKG}.safety_node:main",
        f"actuator = {PKG}.actuator_node:main",
        f"vision = {PKG}.vision_node:main",
    ]},
)
