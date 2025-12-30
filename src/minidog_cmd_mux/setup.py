from setuptools import setup

package_name = "minidog_cmd_mux"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/mux.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="jjon",
    maintainer_email="jjon@todo.todo",
    description="Minimal cmd_vel multiplexer: manual by default, optional autonomous command when enabled.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "cmd_vel_mux = minidog_cmd_mux.cmd_vel_mux:main",
        ],
    },
)


