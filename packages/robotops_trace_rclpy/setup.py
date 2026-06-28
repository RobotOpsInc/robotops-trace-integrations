from setuptools import find_packages, setup

package_name = "robotops_trace_rclpy"

setup(
    name=package_name,
    version="0.2.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    # Runtime deps. The RobotOps Python SDK core (PyPI "robotops-trace", import
    # name "robotops") and the shared semconv mirror ("robotops-trace-semconv",
    # import name "robotops_trace_semconv") are NOT yet on PyPI — install them
    # from source until they publish. rclpy is provided by the ROS underlay, not
    # pip, so it is intentionally not listed here.
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Kristoph Matthews",
    maintainer_email="kristophm@robotops.com",
    description=(
        "RobotOps Trace integration for rclpy: monkey-patches rclpy at import so "
        "stock Python ROS 2 nodes get traced — per-callback spans plus the "
        "canonical action goal-UUID key for cross-language correlation with the "
        "rclcpp integration. Built on the robotops-trace Python SDK core."
    ),
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={},
)
