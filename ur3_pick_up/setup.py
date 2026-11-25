from setuptools import setup

package_name = "ur3_pick_up"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md", "README.zh-TW.md"]),
        ("share/" + package_name + "/config", ["config/grasp_tool.example.json"]),
        ("share/" + package_name + "/scripts", ["scripts/anygrasp_worker.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="chentom",
    maintainer_email="chentom@example.com",
    description="UR3 live-camera AnyGrasp pick-up",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "pick_up = ur3_pick_up.pick_up:main",
        ],
    },
)
