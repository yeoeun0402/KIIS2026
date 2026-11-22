from setuptools import find_packages, setup

package_name = "car_drive"

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
    description="실차 WASD 주행 노드 (실험용)",
    license="Proprietary",
    entry_points={
        "console_scripts": [
            "wasd_drive = car_drive.wasd_drive:main",
            # 더미 — run.sh --twin / --monitor 일 때만 뜬다
            "twin_stub = car_drive.extras.twin_stub:main",
            "monitor_stub = car_drive.extras.monitor_stub:main",
        ],
    },
)
