from setuptools import setup


package_name = "ur3_pick_demo"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="chentom",
    maintainer_email="chentom@example.com",
    description="Interactive UR3 pick demo using a camera-frame target point",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "coordinate_pick = ur3_pick_demo.coordinate_pick:main",
        ],
    },
)
