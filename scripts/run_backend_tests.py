#!/usr/bin/env python3
"""Run PV-LIO-PLUS over dataset/backend combinations.

The input is a JSON manifest.  Each dataset explicitly declares the bag,
LiDAR message mode/topic, and IMU message type/topic.  A temporary launch file
overrides only those runtime parameters and ``mapping/map_type``; the package
source and checked-in YAML files are not modified.

Example:
  python3 scripts/run_backend_tests.py \
      --manifest scripts/datasets.example.json \
      --algorithms voxelmap,ivox,c3p_voxelmap

The node writes its legacy output to the workspace ``output/`` directory.  A
copy of each run's changed result, configuration, launch file, commands, and
logs is stored below ``--results-root/<dataset>/<backend>/<timestamp>/``.
The workspace is compiled first with the command used by ``.vscode/tasks.json``.
"""

from __future__ import annotations

import argparse
import json
import os
import re
import shutil
import signal
import subprocess
import sys
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple
from urllib.parse import urlparse


BACKENDS = {
    "voxelmap": "pv_lio",
    "voxelmap_plus": "pv_lio_plus",
    "ikdtree": "pv_lio_ikdtree",
    "ivox": "pv_lio_ivox",
    "c3p_voxelmap": "pv_lio_c3p_voxelmap",
}
BACKEND_ALIASES = {
    "voxel_map": "voxelmap",
    "voxel": "voxelmap",
    "voxel_map_plus": "voxelmap_plus",
    "plus": "voxelmap_plus",
    "ikd_tree": "ikdtree",
    "ikd-tree": "ikdtree",
    "ivox3d": "ivox",
    "c3p": "c3p_voxelmap",
    "c3p-voxelmap": "c3p_voxelmap",
}
LIDAR_TYPES = {
    "1": (1, "livox_ros_driver/CustomMsg"),
    "avia": (1, "livox_ros_driver/CustomMsg"),
    "livox": (1, "livox_ros_driver/CustomMsg"),
    "2": (2, "sensor_msgs/PointCloud2"),
    "velodyne": (2, "sensor_msgs/PointCloud2"),
    "3": (3, "sensor_msgs/PointCloud2"),
    "ouster": (3, "sensor_msgs/PointCloud2"),
    "4": (4, "sensor_msgs/PointCloud2"),
    "solid": (4, "sensor_msgs/PointCloud2"),
}
IMU_TYPE = "sensor_msgs/Imu"


def die(message: str) -> None:
    raise RuntimeError(message)


def canonical_backend(value: str) -> str:
    name = value.strip().lower()
    name = BACKEND_ALIASES.get(name, name)
    if name not in BACKENDS:
        die(f"unsupported backend '{value}'; choose from {', '.join(BACKENDS)}")
    return name


def canonical_lidar_type(value: Any) -> Tuple[int, str]:
    key = str(value).strip().lower()
    if key not in LIDAR_TYPES:
        die("lidar_type must be one of avia, velodyne, ouster, solid, or 1..4")
    return LIDAR_TYPES[key]


def require_string(item: Dict[str, Any], key: str, context: str) -> str:
    value = item.get(key)
    if not isinstance(value, str) or not value.strip():
        die(f"{context} requires non-empty '{key}'")
    return value.strip()


def xml_escape(value: str) -> str:
    return (value.replace("&", "&amp;").replace("<", "&lt;")
            .replace(">", "&gt;").replace('"', "&quot;").replace("'", "&apos;"))


def safe_name(value: str) -> str:
    result = re.sub(r"[^A-Za-z0-9_.-]+", "_", value.strip())
    return result.strip("._") or "unnamed"


def read_manifest(path: Path) -> Dict[str, Any]:
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        die(f"cannot read JSON manifest {path}: {exc}")
    if not isinstance(data, dict) or not isinstance(data.get("datasets"), list):
        die("manifest must be an object containing a 'datasets' array")
    return data


def ros_master_alive(env: Dict[str, str]) -> bool:
    try:
        result = subprocess.run(["rosnode", "list"], env=env, stdout=subprocess.DEVNULL,
                                stderr=subprocess.DEVNULL, timeout=3, check=False)
        return result.returncode == 0
    except (OSError, subprocess.TimeoutExpired):
        return False


def master_port(master_uri: str) -> str:
    parsed = urlparse(master_uri)
    return str(parsed.port or 11311)


class RoscoreGuard:
    def __init__(self, env: Dict[str, str], log_path: Path):
        self.env = env
        self.log_path = log_path
        self.process: Optional[subprocess.Popen[Any]] = None

    def start(self) -> None:
        if ros_master_alive(self.env):
            return
        self.log_path.parent.mkdir(parents=True, exist_ok=True)
        roscore_env = dict(self.env)
        roscore_home = self.log_path.parent / "roscore_home"
        roscore_log_dir = self.log_path.parent / "roscore_log"
        roscore_home.mkdir(parents=True, exist_ok=True)
        roscore_log_dir.mkdir(parents=True, exist_ok=True)
        roscore_env["ROS_HOME"] = str(roscore_home)
        roscore_env["ROS_LOG_DIR"] = str(roscore_log_dir)
        log_file = self.log_path.open("w", encoding="utf-8")
        self.process = subprocess.Popen(["roscore", "-p", master_port(self.env["ROS_MASTER_URI"])],
                                        env=roscore_env, stdout=log_file, stderr=subprocess.STDOUT,
                                        start_new_session=True)
        deadline = time.monotonic() + 20
        while time.monotonic() < deadline:
            if ros_master_alive(self.env):
                return
            if self.process.poll() is not None:
                die(f"roscore exited with code {self.process.returncode}; see {self.log_path}")
            time.sleep(0.25)
        die(f"timed out waiting for ROS master; see {self.log_path}")

    def stop(self) -> None:
        if self.process is None or self.process.poll() is not None:
            return
        os.killpg(self.process.pid, signal.SIGINT)
        try:
            self.process.wait(timeout=8)
        except subprocess.TimeoutExpired:
            os.killpg(self.process.pid, signal.SIGTERM)
            self.process.wait(timeout=5)


@dataclass
class Dataset:
    name: str
    bag: Path
    config: Path
    lidar_type: int
    lidar_message_type: str
    lidar_topic: str
    imu_message_type: str
    imu_topic: str
    start: Optional[float]
    duration: Optional[float]
    rate: Optional[float]
    topics: Optional[List[str]]


def parse_dataset(raw: Dict[str, Any], manifest_dir: Path, default_config: Path) -> Dataset:
    name = safe_name(require_string(raw, "name", "dataset"))
    bag_value = Path(require_string(raw, "bag", name)).expanduser()
    if not bag_value.is_absolute():
        bag_value = (manifest_dir / bag_value).resolve()
    if not bag_value.is_file():
        die(f"dataset '{name}' bag does not exist: {bag_value}")

    config_value = raw.get("config")
    config = Path(config_value).expanduser() if isinstance(config_value, str) else default_config
    if not config.is_absolute():
        config = (manifest_dir / config).resolve()
    if not config.is_file():
        die(f"dataset '{name}' config does not exist: {config}")

    lidar_type, expected_lidar = canonical_lidar_type(raw.get("lidar_type"))
    lidar_message_type = raw.get("lidar_message_type", expected_lidar)
    if lidar_message_type != expected_lidar:
        die(f"dataset '{name}' lidar_message_type '{lidar_message_type}' does not match lidar_type {lidar_type}")
    imu_message_type = raw.get("imu_type", IMU_TYPE)
    if imu_message_type != IMU_TYPE:
        die(f"dataset '{name}' imu_type must be {IMU_TYPE}; this node subscribes to sensor_msgs::Imu")

    def optional_number(key: str) -> Optional[float]:
        value = raw.get(key)
        if value is None:
            return None
        try:
            number = float(value)
        except (TypeError, ValueError):
            die(f"dataset '{name}' field '{key}' must be numeric")
        if number < 0:
            die(f"dataset '{name}' field '{key}' must be non-negative")
        return number

    topics = raw.get("play_topics")
    if topics is not None:
        if not isinstance(topics, list) or not all(isinstance(x, str) and x for x in topics):
            die(f"dataset '{name}' play_topics must be a list of non-empty strings")

    return Dataset(name, bag_value, config,
                   lidar_type, lidar_message_type,
                   require_string(raw, "lidar_topic", name),
                   imu_message_type, require_string(raw, "imu_topic", name),
                   optional_number("start"), optional_number("duration"), optional_number("rate"), topics)


def bag_topics(bag: Path, env: Dict[str, str]) -> Dict[str, str]:
    try:
        result = subprocess.run(["rosbag", "info", "--yaml", str(bag)], env=env,
                                capture_output=True, text=True, timeout=30, check=False)
    except (OSError, subprocess.TimeoutExpired) as exc:
        die(f"cannot inspect bag {bag}: {exc}")
    if result.returncode != 0:
        die(f"rosbag info failed for {bag}: {result.stderr.strip()}")
    # rosbag's YAML output is stable for topic/type pairs; avoid requiring PyYAML.
    found: Dict[str, str] = {}
    current_topic: Optional[str] = None
    for line in result.stdout.splitlines():
        topic_match = re.match(r"\s*- topic: (.+)$", line)
        type_match = re.match(r"\s*type: (.+)$", line)
        if topic_match:
            current_topic = topic_match.group(1).strip().strip("'")
        elif type_match and current_topic is not None:
            found[current_topic] = type_match.group(1).strip().strip("'")
            current_topic = None
    return found


def verify_topics(dataset: Dataset, env: Dict[str, str]) -> None:
    found = bag_topics(dataset.bag, env)
    for topic, expected in ((dataset.lidar_topic, dataset.lidar_message_type),
                            (dataset.imu_topic, dataset.imu_message_type)):
        actual = found.get(topic)
        if actual is None:
            die(f"dataset '{dataset.name}' topic not found in bag: {topic}")
        if actual != expected:
            die(f"dataset '{dataset.name}' topic {topic} has type {actual}, expected {expected}")


def launch_text(dataset: Dataset, backend: str) -> str:
    return f'''<launch>
  <rosparam command="load" file="{xml_escape(str(dataset.config))}"/>
  <param name="common/lid_topic" value="{xml_escape(dataset.lidar_topic)}"/>
  <param name="common/imu_topic" value="{xml_escape(dataset.imu_topic)}"/>
  <param name="preprocess/lidar_type" value="{dataset.lidar_type}" type="int"/>
  <param name="mapping/map_type" value="{backend}"/>
  <param name="publish/path_en" value="true" type="bool"/>
  <param name="pcd_save/pcd_save_en" value="true" type="bool"/>
  <param name="pcd_save/interval" value="-1" type="int"/>
  <node pkg="pv_lio_plus" type="pv_lio_plus_node" name="pv_lio_plus_node" output="screen"/>
</launch>
'''


def build_bag_command(dataset: Dataset) -> List[str]:
    # Keep options before the bag path; this works with the ROS Noetic
    # rosbag argument parser and leaves --topics as the final option group.
    command = ["rosbag", "play", "--clock"]
    if dataset.start is not None:
        command += ["--start", str(dataset.start)]
    if dataset.duration is not None:
        command += ["--duration", str(dataset.duration)]
    if dataset.rate is not None:
        command += ["--rate", str(dataset.rate)]
    command.append(str(dataset.bag))
    if dataset.topics:
        command += ["--topics"] + dataset.topics
    return command


def snapshot_files(root: Path) -> Dict[Path, int]:
    if not root.exists():
        return {}
    return {path: path.stat().st_mtime_ns for path in root.rglob("*") if path.is_file()}


def copy_changed_outputs(output_root: Path, run_dir: Path, before: Dict[Path, int], stem: str) -> List[str]:
    copied: List[str] = []
    if not output_root.exists():
        return copied
    result_dir = run_dir / "results"
    for source in output_root.rglob("*"):
        if not source.is_file():
            continue
        old_time = before.get(source)
        changed = old_time is None or source.stat().st_mtime_ns > old_time
        if not changed:
            continue
        if source.parent == output_root and not source.name.startswith(stem):
            continue
        if source.parent != output_root and source.parent.name != "PCD":
            continue
        relative = source.relative_to(output_root)
        destination = result_dir / relative
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, destination)
        copied.append(str(relative))
    return sorted(copied)


def wait_for_backend_outputs(output_root: Path, stem: str, before: Dict[Path, int], timeout: float = 20.0) -> None:
    """Allow the node's SIGINT handler to finish writing final files."""
    expected = (output_root / f"{stem}_pos.txt", output_root / f"{stem}.pcd")
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        for path in expected:
            if path.is_file() and (path not in before or path.stat().st_mtime_ns > before[path]):
                return
        time.sleep(0.25)


def build_workspace(workspace_root: Path, results_root: Path) -> None:
    """Build the exact source tree before any replay is started."""
    build_log_path = results_root / "build.log"
    build_record = {
        "workspace": str(workspace_root),
        "command": ["/bin/bash", "-c",
                    "source /opt/ros/noetic/setup.bash && catkin_make -j6 -DCMAKE_EXPORT_COMPILE_COMMANDS=ON"],
        "started_at": datetime.now(timezone.utc).isoformat(),
    }
    build_env = dict(os.environ)
    build_env["ROS_HOME"] = str(results_root / "build_ros_home")
    build_env["ROS_LOG_DIR"] = str(results_root / "build_ros_log")
    Path(build_env["ROS_HOME"]).mkdir(parents=True, exist_ok=True)
    Path(build_env["ROS_LOG_DIR"]).mkdir(parents=True, exist_ok=True)
    with build_log_path.open("w", encoding="utf-8") as log_file:
        result = subprocess.run(build_record["command"], cwd=workspace_root, env=build_env,
                                stdout=log_file, stderr=subprocess.STDOUT, check=False)
    build_record["exit_code"] = result.returncode
    build_record["finished_at"] = datetime.now(timezone.utc).isoformat()
    (results_root / "build.json").write_text(json.dumps(build_record, indent=2) + "\n", encoding="utf-8")
    if result.returncode != 0:
        die(f"catkin_make failed with code {result.returncode}; see {build_log_path}")


def terminate_process(process: subprocess.Popen[Any], label: str) -> int:
    if process.poll() is not None:
        return int(process.returncode)
    try:
        os.killpg(process.pid, signal.SIGINT)
        process.wait(timeout=15)
    except subprocess.TimeoutExpired:
        print(f"warning: {label} did not stop after SIGINT; sending SIGTERM", file=sys.stderr)
        os.killpg(process.pid, signal.SIGTERM)
        process.wait(timeout=10)
    return int(process.returncode)


def run_one(dataset: Dataset, backend: str, args: argparse.Namespace,
            env: Dict[str, str], output_root: Path) -> Dict[str, Any]:
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    run_dir = Path(args.results_root).expanduser().resolve() / dataset.name / backend / stamp
    run_dir.mkdir(parents=True, exist_ok=False)
    dataset_record = dict(dataset.__dict__)
    dataset_record["bag"] = str(dataset.bag)
    dataset_record["config"] = str(dataset.config)
    (run_dir / "dataset.json").write_text(json.dumps(dataset_record, indent=2, ensure_ascii=False) + "\n",
                                           encoding="utf-8")
    shutil.copy2(dataset.config, run_dir / "base_config.yaml")
    launch_file = run_dir / "generated.launch"
    launch_file.write_text(launch_text(dataset, backend), encoding="utf-8")

    run_env = dict(env)
    run_env["ROS_HOME"] = str(run_dir / "ros_home")
    run_env["ROS_LOG_DIR"] = str(run_dir / "ros_log")
    (run_dir / "ros_home").mkdir()
    (run_dir / "ros_log").mkdir()
    launch_log = (run_dir / "roslaunch.log").open("w", encoding="utf-8")
    bag_log = (run_dir / "rosbag.log").open("w", encoding="utf-8")
    before = snapshot_files(output_root)
    launch_command = ["roslaunch", str(launch_file)]
    bag_command = build_bag_command(dataset)
    (run_dir / "commands.json").write_text(json.dumps({"roslaunch": launch_command, "rosbag": bag_command},
                                                        indent=2) + "\n", encoding="utf-8")

    launch_process: Optional[subprocess.Popen[Any]] = None
    bag_process: Optional[subprocess.Popen[Any]] = None
    launch_code: Optional[int] = None
    bag_code: Optional[int] = None
    error: Optional[str] = None
    started = time.time()
    try:
        launch_process = subprocess.Popen(launch_command, env=run_env, stdout=launch_log,
                                          stderr=subprocess.STDOUT, start_new_session=True)
        deadline = time.monotonic() + args.startup_timeout
        while time.monotonic() < deadline:
            if launch_process.poll() is not None:
                die(f"roslaunch exited with code {launch_process.returncode}; see {run_dir / 'roslaunch.log'}")
            nodes = subprocess.run(["rosnode", "list"], env=run_env, capture_output=True,
                                   text=True, timeout=3, check=False)
            if nodes.returncode == 0 and "/pv_lio_plus_node" in nodes.stdout.splitlines():
                break
            time.sleep(0.25)
        else:
            die(f"timed out waiting for pv_lio_plus_node; see {run_dir / 'roslaunch.log'}")

        bag_process = subprocess.Popen(bag_command, env=run_env, stdout=bag_log,
                                       stderr=subprocess.STDOUT, start_new_session=True)
        bag_code = bag_process.wait()
        if bag_code != 0:
            die(f"rosbag exited with code {bag_code}; see {run_dir / 'rosbag.log'}")
    except (OSError, RuntimeError) as exc:
        error = str(exc)
    finally:
        if bag_process is not None and bag_process.poll() is None:
            terminate_process(bag_process, "rosbag")
        if launch_process is not None:
            launch_code = terminate_process(launch_process, "roslaunch")
        launch_log.close()
        bag_log.close()

    wait_for_backend_outputs(output_root, BACKENDS[backend], before)
    copied = copy_changed_outputs(output_root, run_dir, before, BACKENDS[backend])
    status = {
        "dataset": dataset.name, "backend": backend, "success": error is None and bag_code == 0,
        "error": error, "roslaunch_exit_code": launch_code, "rosbag_exit_code": bag_code,
        "duration_sec": round(time.time() - started, 3), "copied_results": copied,
    }
    (run_dir / "status.json").write_text(json.dumps(status, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    return status


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--manifest", required=True, type=Path, help="JSON dataset manifest")
    parser.add_argument("--algorithms", help="comma-separated backends; overrides manifest algorithms")
    parser.add_argument("--default-config", type=Path,
                        default=Path(__file__).resolve().parents[1] / "config" / "avia.yaml")
    parser.add_argument("--results-root", type=Path,
                        default=Path(__file__).resolve().parents[3] / "backend_test_results")
    parser.add_argument("--output-root", type=Path,
                        default=Path(__file__).resolve().parents[3] / "output")
    parser.add_argument("--master-uri", default=os.environ.get("ROS_MASTER_URI", "http://127.0.0.1:11311"))
    parser.add_argument("--startup-timeout", type=float, default=30.0)
    parser.add_argument("--skip-topic-check", action="store_true",
                        help="skip rosbag info type/topic validation")
    parser.add_argument("--continue-on-error", action="store_true")
    args = parser.parse_args()

    manifest_path = args.manifest.expanduser().resolve()
    data = read_manifest(manifest_path)
    default_config = args.default_config.expanduser().resolve()
    datasets = [parse_dataset(item, manifest_path.parent, default_config) for item in data["datasets"]]
    if len({item.name for item in datasets}) != len(datasets):
        die("dataset names must be unique")
    raw_algorithms: Iterable[str]
    if args.algorithms:
        raw_algorithms = args.algorithms.split(",")
    else:
        raw_algorithms = data.get("algorithms", list(BACKENDS))
    algorithms = list(dict.fromkeys(canonical_backend(value) for value in raw_algorithms))
    if not algorithms:
        die("at least one backend is required")

    env = dict(os.environ)
    env["ROS_MASTER_URI"] = args.master_uri
    results_root = args.results_root.expanduser().resolve()
    results_root.mkdir(parents=True, exist_ok=True)
    output_root = args.output_root.expanduser().resolve()
    # The node's ROOT_DIR is compile-time fixed and the node does not create
    # its root output directory before writing the final trajectory/PCD.
    output_root.mkdir(parents=True, exist_ok=True)
    build_workspace(Path(__file__).resolve().parents[3], results_root)
    results: List[Dict[str, Any]] = []
    guard = RoscoreGuard(env, results_root / "roscore.log")
    try:
        guard.start()
        for dataset in datasets:
            if not args.skip_topic_check:
                verify_topics(dataset, env)
            for backend in algorithms:
                print(f"[RUN] dataset={dataset.name} backend={backend}", flush=True)
                status = run_one(dataset, backend, args, env, output_root)
                results.append(status)
                print(f"[{'PASS' if status['success'] else 'FAIL'}] dataset={dataset.name} backend={backend}", flush=True)
                if not status["success"] and not args.continue_on_error:
                    return 1
    except (OSError, RuntimeError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2
    finally:
        guard.stop()

    summary = {"results": results, "count": len(results), "passed": sum(x["success"] for x in results)}
    summary_path = results_root / "summary.json"
    summary_path.write_text(json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(f"Summary: {summary['passed']}/{summary['count']} passed; see {summary_path}")
    return 0 if summary["passed"] == summary["count"] else 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except RuntimeError as exc:
        print(f"error: {exc}", file=sys.stderr)
        sys.exit(2)
