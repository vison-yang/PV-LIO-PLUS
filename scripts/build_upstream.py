#!/usr/bin/env python3
"""Build each non-catkin upstream in isolation, retaining every exit code."""

import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys


PROJECTS = ("BIEVR-LIO", "Surfel-LIO", "i-Octree", "kiss-icp", "lio_gvm")


def ensure_link(link, target):
    if link.is_symlink() and link.resolve() == target.resolve():
        return
    if link.exists() or link.is_symlink():
        raise RuntimeError("Refusing to replace {}".format(link))
    link.parent.mkdir(parents=True, exist_ok=True)
    link.symlink_to(target.resolve(), target_is_directory=True)


def run_project(name, commands, log_dir, env):
    log = log_dir / (name + ".log")
    steps = []
    with log.open("w") as stream:
        for command, cwd in commands:
            stream.write(json.dumps({"command": command, "cwd": str(cwd)}) + "\n")
            stream.flush()
            result = subprocess.run(command, cwd=str(cwd), env=env,
                                    stdout=stream, stderr=subprocess.STDOUT)
            steps.append({"command": command, "cwd": str(cwd), "exit_code": result.returncode})
            stream.write("\nexit_code={}\n".format(result.returncode))
            stream.flush()
            if result.returncode:
                break
    return {"project": name, "exit_code": steps[-1]["exit_code"],
            "log": str(log), "steps": steps}


def main():
    workspace = Path(__file__).resolve().parents[3]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace", type=Path, default=workspace)
    parser.add_argument("--prefix", type=Path)
    parser.add_argument("--logs-dir", type=Path)
    parser.add_argument("--jobs", type=int, default=2)
    parser.add_argument("--projects", nargs="+", choices=PROJECTS, default=list(PROJECTS))
    parser.add_argument("--kiss-cmake", type=Path,
                        help="CMake >= 3.24 for KISS-ICP's pinned Sophus")
    args = parser.parse_args()
    if args.jobs < 1:
        parser.error("--jobs must be positive")
    root = args.workspace.resolve()
    prefix = (args.prefix or root / ".workspace-deps").resolve()
    logs = (args.logs_dir or root / "output/upstream-build").resolve()
    logs.mkdir(parents=True, exist_ok=True)
    up = root / "upstream"
    build = up / "build"
    env = os.environ.copy()
    env["CMAKE_PREFIX_PATH"] = str(prefix) + ":/opt/ros/noetic"
    common = ["-DCMAKE_BUILD_TYPE=Release", "-DCMAKE_FIND_USE_PACKAGE_REGISTRY=OFF",
              "-DEigen3_DIR=" + str(prefix / "share/eigen3/cmake")]
    jobs = str(args.jobs)
    ros = ["bash", "-c", "source /opt/ros/noetic/setup.bash\nexec \"$@\"", "build-upstream"]
    results = []

    for name in args.projects:
        try:
            commands = []
            if name == "BIEVR-LIO":
                ws = build / "bievr_ros1_ws"
                for package, path in (("bievr_lio", "BIEVR"),
                                      ("bievr_ros_common", "interfaces/ros_common"),
                                      ("bievr_lio_ros", "interfaces/ros1")):
                    ensure_link(ws / "src" / package, up / name / path)
                command = ros + ["catkin_make_isolated", "--install", "--cmake-args"] + common
                command += ["-DCMAKE_PREFIX_PATH=" + str(prefix) + ";/opt/ros/noetic",
                            "-DCeres_DIR=" + str(prefix / "lib/cmake/Ceres"),
                            "--make-args", "-j" + jobs]
                commands = [(command, ws)]
            elif name == "Surfel-LIO":
                pangolin = build / "pangolin"
                commands = [
                    (["cmake", "-S", str(up / name / "thirdparty/pangolin"), "-B", str(pangolin),
                      "-DCMAKE_BUILD_TYPE=Release", "-DBUILD_EXAMPLES=OFF", "-DBUILD_TESTS=OFF",
                      "-DBUILD_TOOLS=OFF", "-DBUILD_PANGOLIN_PYTHON=OFF"], root),
                    (["cmake", "--build", str(pangolin), "--parallel", jobs], root),
                    (["cmake", "-S", str(up / name), "-B", str(build / name)] + common +
                     ["-DPANGOLIN_BUILD_DIR=" + str(pangolin)], root),
                    (["cmake", "--build", str(build / name), "--parallel", jobs], root)]
            elif name == "i-Octree":
                commands = [
                    (["cmake", "-S", str(up / name), "-B", str(build / name)] + common +
                     ["-DIKD_TREE_ROOT=" + str(up / "deps/ikd-Tree")], root),
                    (["cmake", "--build", str(build / name), "--parallel", jobs], root)]
            elif name == "kiss-icp":
                local_cmake = root / ".workspace-tools/python/cmake/data/bin/cmake"
                cmake = str(args.kiss_cmake or (local_cmake if local_cmake.exists() else
                                               Path(shutil.which("cmake") or "cmake")))
                version = subprocess.check_output([cmake, "--version"], text=True).split()[2]
                if tuple(map(int, version.split(".")[:2])) < (3, 24):
                    raise RuntimeError("KISS-ICP/Sophus requires CMake >= 3.24; use --kiss-cmake")
                commands = [
                    ([cmake, "-S", str(up / name / "cpp/kiss_icp"), "-B", str(build / name)] + common +
                     ["-DFETCHCONTENT_FULLY_DISCONNECTED=ON",
                      "-DFETCHCONTENT_SOURCE_DIR_SOPHUS=" + str(up / "deps/Sophus"),
                      "-DFETCHCONTENT_SOURCE_DIR_TESSIL=" + str(up / "deps/robin-map")], root),
                    ([cmake, "--build", str(build / name), "--parallel", jobs], root)]
            else:
                ws = build / "lio_gvm_ws"
                ensure_link(ws / "src/lio_gvm", up / name)
                ensure_link(ws / "src/livox_ros_driver", root / "src/livox_ros_driver")
                command = ros + ["catkin_make", "-j" + jobs] + common
                command += ["-DCMAKE_PREFIX_PATH=" + str(prefix) + ";/opt/ros/noetic",
                            "-DCeres_DIR=" + str(prefix / "lib/cmake/Ceres")]
                commands = [(command, ws)]
            result = run_project(name, commands, logs, env)
        except (OSError, RuntimeError, subprocess.CalledProcessError) as exc:
            result = {"project": name, "exit_code": 1, "error": str(exc)}
        results.append(result)
        print("{}: exit {}".format(name, result["exit_code"]), flush=True)
        (logs / "results.json").write_text(json.dumps(results, indent=2) + "\n")
    return int(any(result["exit_code"] for result in results))


if __name__ == "__main__":
    sys.exit(main())
