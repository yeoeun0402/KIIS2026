from setuptools import find_packages, setup

package_name = "car_state"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    description="[demo] 차량 상태 [x, y, θ, v, δ] · 조향 보정",
    license="Proprietary",
    entry_points={
        "console_scripts": [
            "car_state_node = car_state.car_state_node:main",
            "steer_calib = car_state.steer_calib:main",
            "sim_bag = car_state.sim_bag:main",
            "state_report = car_state.state_report:main",
        ],
    },
)
