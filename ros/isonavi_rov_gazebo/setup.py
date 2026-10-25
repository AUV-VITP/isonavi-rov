from glob import glob
from setuptools import setup

PKG = "isonavi_rov_gazebo"

setup(
    name=PKG,
    version="0.1.0",
    packages=[PKG],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{PKG}"]),
        (f"share/{PKG}", ["package.xml"]),
        (f"share/{PKG}/launch", glob("launch/*.py")),
        (f"share/{PKG}/worlds", glob("worlds/*.sdf")),
        (f"share/{PKG}/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    entry_points={
        "console_scripts": [
            f"thruster_bridge = {PKG}.thruster_bridge:main",
            f"depth_adapter = {PKG}.depth_adapter:main",
        ]
    },
)
