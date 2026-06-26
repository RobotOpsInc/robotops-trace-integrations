from setuptools import find_packages, setup

package_name = "robotops_trace_rclpy"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Kristoph Matthews",
    maintainer_email="kristophm@robotops.com",
    description=(
        "RobotOps Trace integration for rclpy (ROB-423) — STUB. Monkey-patches "
        "rclpy to propagate trace context; built on the robotops-trace SDK core."
    ),
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={},
)
