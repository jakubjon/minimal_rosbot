from setuptools import setup

package_name = "minidog_scan_odom"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    maintainer="jjon",
    maintainer_email="jjon@todo.todo",
    description="Minimal 2D laser-scan odometry using ICP.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "scan_odom_node = minidog_scan_odom.scan_odom:main",
        ],
    },
)
