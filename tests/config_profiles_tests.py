#!/usr/bin/env python3
"""验证传感器配置与地图后端预设的 roslaunch 合并契约。"""

import importlib.util
import os
import re
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

import yaml


REPO_ROOT = Path(__file__).resolve().parents[1]
SENSOR_PROFILES = {
    "avia": REPO_ROOT / "config/avia.yaml",
    "mid360_indoor": REPO_ROOT / "config/mid360_indoor.yaml",
    "solid": REPO_ROOT / "config/solid.yaml",
}
BACKEND_PROFILES = {
    "voxelmap": "voxelmap",
    "voxelmap_plus": "voxelmap_plus",
    "ikdtree": "ikdtree",
    "ivox": "ivox",
    "octvox": "octvox",
    "c3p_voxelmap": "c3p_voxelmap",
    "r_voxelmap": "r_voxelmap",
    "hvox": "hvox",
    "i_octree": "i_octree",
    "ig_lio_plane": "ig_lio_plane",
}
LEGACY_WRAPPERS = {
    "mapping_avia.launch": "voxelmap_plus",
    "mapping_mid360.launch": "voxelmap_plus",
    "mapping_solid.launch": "ivox",
}


def flatten_mapping(value, prefix=""):
    """将 YAML 字典展平为 roslaunch 参数路径。"""
    flattened = {}
    for key, child in value.items():
        path = f"{prefix}/{key}"
        if isinstance(child, dict):
            flattened.update(flatten_mapping(child, path))
        else:
            flattened[path] = child
    return flattened


class MapConfigProfileTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls._environment = {
            key: os.environ.get(key)
            for key in ("ROS_PACKAGE_PATH", "ROS_HOME", "ROS_LOG_DIR")
        }
        cls._temporary = tempfile.TemporaryDirectory(prefix="pv_lio_config_profiles_")
        system_packages = Path("/opt/ros/noetic/share")
        package_paths = [REPO_ROOT.parent]
        if system_packages.is_dir():
            package_paths.append(system_packages)
        os.environ["ROS_PACKAGE_PATH"] = os.pathsep.join(str(path) for path in package_paths)
        os.environ["ROS_HOME"] = cls._temporary.name
        os.environ["ROS_LOG_DIR"] = cls._temporary.name

        import roslaunch.config
        import rospkg

        resolved_package = Path(rospkg.RosPack().get_path("pv_lio_plus")).resolve()
        if resolved_package != REPO_ROOT:
            raise AssertionError(f"resolved pv_lio_plus from another checkout: {resolved_package}")

        cls._load_config_default = staticmethod(roslaunch.config.load_config_default)
        runner_path = REPO_ROOT / "scripts/run_backend_tests.py"
        runner_name = "pv_lio_plus_run_backend_tests_for_config_contract"
        runner_spec = importlib.util.spec_from_file_location(runner_name, runner_path)
        if runner_spec is None or runner_spec.loader is None:
            raise AssertionError(f"cannot import backend test runner: {runner_path}")
        cls._runner = importlib.util.module_from_spec(runner_spec)
        sys.modules[runner_name] = cls._runner
        write_bytecode = sys.dont_write_bytecode
        sys.dont_write_bytecode = True
        try:
            runner_spec.loader.exec_module(cls._runner)
        finally:
            sys.dont_write_bytecode = write_bytecode

    @classmethod
    def tearDownClass(cls):
        for key, value in cls._environment.items():
            if value is None:
                os.environ.pop(key, None)
            else:
                os.environ[key] = value
        sys.modules.pop("pv_lio_plus_run_backend_tests_for_config_contract", None)
        cls._temporary.cleanup()

    @classmethod
    def load_launch(cls, launch_path, args=()):
        launch_args = [f"{key}:={value}" for key, value in args]
        return cls._load_config_default(
            [(str(launch_path), launch_args)],
            port=0,
            verbose=False,
            assign_machines=False,
        )

    @classmethod
    def load_launch_text(cls, filename, launch_text):
        launch_path = Path(cls._temporary.name) / filename
        launch_path.write_text(launch_text, encoding="utf-8")
        return cls.load_launch(launch_path)

    @staticmethod
    def param_values(launch_config):
        return {key: param.value for key, param in launch_config.params.items()}

    def test_backend_yaml_keys_are_read_by_config_loader(self):
        source = (REPO_ROOT / "src/config.cpp").read_text(encoding="utf-8")
        consumed_keys = set(re.findall(r'nh\.param[^\(]*\(\s*"([^"]+)"', source, re.S))

        for backend in BACKEND_PROFILES:
            with self.subTest(backend=backend):
                preset = yaml.safe_load(
                    (REPO_ROOT / "config/backends" / f"{backend}.yaml").read_text(encoding="utf-8")
                )
                keys = set(flatten_mapping(preset))
                self.assertEqual(set(preset), {"mapping"})
                self.assertIn("/mapping/map_type", keys)
                self.assertIn("/mapping/local_window_en", keys)
                self.assertFalse(any(key.startswith(("/common/", "/preprocess/", "/noise_model/"))
                                     for key in keys))
                self.assertFalse(any(key in ("/time_offset", "/mapping/extrinsic_T", "/mapping/extrinsic_R")
                                     for key in keys))
                self.assertFalse(any(key.startswith("/publish/") for key in keys))
                self.assertTrue(keys.issubset({f"/{key}" for key in consumed_keys}))
                self.assertFalse(preset["mapping"]["local_window_en"])

    def test_three_sensor_profiles_with_all_backend_overlays(self):
        for sensor, sensor_path in SENSOR_PROFILES.items():
            sensor_values = flatten_mapping(yaml.safe_load(sensor_path.read_text(encoding="utf-8")))
            for backend, selector in BACKEND_PROFILES.items():
                with self.subTest(sensor=sensor, backend=backend):
                    overlay = flatten_mapping(
                        yaml.safe_load(
                            (REPO_ROOT / "config/backends" / f"{backend}.yaml").read_text(encoding="utf-8")
                        )
                    )
                    launch_config = self.load_launch(
                        REPO_ROOT / "launch/mapping.launch",
                        (("sensor_config", sensor_path), ("backend", backend), ("rviz", "false")),
                    )
                    actual = self.param_values(launch_config)

                    self.assertEqual(actual["/mapping/map_type"], selector)
                    for path, value in sensor_values.items():
                        expected = overlay.get(path, value)
                        self.assertEqual(actual[path], expected, path)
                    for path, value in overlay.items():
                        self.assertEqual(actual[path], value, path)

                    expected_mapping_paths = {
                        path for path in set(sensor_values).union(overlay) if path.startswith("/mapping/")
                    }
                    actual_mapping_paths = {path for path in actual if path.startswith("/mapping/")}
                    self.assertEqual(actual_mapping_paths, expected_mapping_paths)

                    for path in (
                        "/common/lid_topic",
                        "/common/imu_topic",
                        "/preprocess/lidar_type",
                        "/mapping/down_sample_size",
                        "/mapping/extrinsic_est_en",
                        "/mapping/extrinsic_T",
                        "/mapping/extrinsic_R",
                        "/noise_model/ranging_cov",
                        "/noise_model/acc_cov",
                    ):
                        self.assertEqual(actual[path], sensor_values[path], path)
                    self.assertEqual(actual["/mapping/point_map_downsample_size"],
                                     sensor_values["/mapping/point_map_downsample_size"])
                    self.assertFalse(any(node.package == "rviz" for node in launch_config.nodes))

    def test_generic_defaults_unknown_backend_and_rviz_toggle(self):
        generic = REPO_ROOT / "launch/mapping.launch"
        default_config = self.load_launch(generic)
        self.assertEqual(self.param_values(default_config)["/mapping/map_type"], "voxelmap_plus")
        self.assertTrue(any(node.package == "rviz" for node in default_config.nodes))

        no_rviz = self.load_launch(generic, (("rviz", "false"),))
        self.assertFalse(any(node.package == "rviz" for node in no_rviz.nodes))
        with_rviz = self.load_launch(generic, (("rviz", "true"),))
        self.assertTrue(any(node.package == "rviz" for node in with_rviz.nodes))

        with self.assertRaises(Exception):
            self.load_launch(generic, (("backend", "unknown_backend"),))

    def test_legacy_wrappers_keep_sensor_selectors_and_rviz_defaults(self):
        for filename, selector in LEGACY_WRAPPERS.items():
            with self.subTest(launch=filename):
                wrapper = REPO_ROOT / "launch" / filename
                launch_config = self.load_launch(wrapper)
                values = self.param_values(launch_config)
                self.assertEqual(values["/mapping/map_type"], selector)
                self.assertTrue(any(node.package == "rviz" for node in launch_config.nodes))

                no_rviz = self.load_launch(wrapper, (("rviz", "false"),))
                self.assertFalse(any(node.package == "rviz" for node in no_rviz.nodes))

    def test_runner_preserves_dataset_yaml_by_default_and_opt_in_matches_launch(self):
        sensor_path = SENSOR_PROFILES["solid"]
        sensor_values = flatten_mapping(yaml.safe_load(sensor_path.read_text(encoding="utf-8")))
        dataset = SimpleNamespace(
            config=sensor_path,
            lidar_topic=sensor_values["/common/lid_topic"],
            imu_topic=sensor_values["/common/imu_topic"],
            lidar_type=sensor_values["/preprocess/lidar_type"],
        )
        backend = "ig_lio_plane"
        legacy_text = self._runner.launch_text(dataset, backend)
        legacy_config = self.load_launch_text("runner_dataset_yaml.launch", legacy_text)
        legacy_values = self.param_values(legacy_config)

        expected_legacy_mapping = {
            path: value for path, value in sensor_values.items() if path.startswith("/mapping/")
        }
        expected_legacy_mapping["/mapping/map_type"] = backend
        actual_legacy_mapping = {
            path: value for path, value in legacy_values.items() if path.startswith("/mapping/")
        }
        self.assertEqual(actual_legacy_mapping, expected_legacy_mapping)
        self.assertEqual(legacy_values["/mapping/nearest_max_range"], 1.75)
        self.assertNotIn("backend_config.yaml", legacy_text)

        backend_path = self._runner.backend_preset_path(backend)
        preset_text = self._runner.launch_text(dataset, backend, backend_path)
        preset_config = self.load_launch_text("runner_backend_preset.launch", preset_text)
        preset_values = self.param_values(preset_config)
        self.assertEqual(preset_values["/mapping/nearest_max_range"], 5.0)
        self.assertIn(str(backend_path), preset_text)

        wrapper_config = self.load_launch(
            REPO_ROOT / "launch/mapping_solid.launch",
            (("backend", backend), ("rviz", "false")),
        )
        wrapper_values = self.param_values(wrapper_config)
        preset_mapping = {
            path: value for path, value in preset_values.items() if path.startswith("/mapping/")
        }
        wrapper_mapping = {
            path: value for path, value in wrapper_values.items() if path.startswith("/mapping/")
        }
        self.assertEqual(preset_mapping, wrapper_mapping)

        for path in (
            "/common/lid_topic",
            "/common/imu_topic",
            "/preprocess/lidar_type",
            "/mapping/down_sample_size",
            "/mapping/point_map_downsample_size",
            "/mapping/extrinsic_est_en",
            "/mapping/extrinsic_T",
            "/mapping/extrinsic_R",
            "/noise_model/ranging_cov",
            "/noise_model/angle_cov",
            "/noise_model/acc_cov",
            "/noise_model/gyr_cov",
            "/noise_model/b_acc_cov",
            "/noise_model/b_gyr_cov",
        ):
            self.assertEqual(preset_values[path], sensor_values[path], path)

    def test_runner_preset_matrix_matches_generic_launch_mapping(self):
        generic_launch = REPO_ROOT / "launch/mapping.launch"
        for sensor, sensor_path in SENSOR_PROFILES.items():
            sensor_values = flatten_mapping(yaml.safe_load(sensor_path.read_text(encoding="utf-8")))
            dataset = SimpleNamespace(
                config=sensor_path,
                lidar_topic=sensor_values["/common/lid_topic"],
                imu_topic=sensor_values["/common/imu_topic"],
                lidar_type=sensor_values["/preprocess/lidar_type"],
            )
            for backend in BACKEND_PROFILES:
                with self.subTest(sensor=sensor, backend=backend):
                    backend_path = self._runner.backend_preset_path(backend)
                    runner_config = self.load_launch_text(
                        f"runner_{sensor}_{backend}.launch",
                        self._runner.launch_text(dataset, backend, backend_path),
                    )
                    generic_config = self.load_launch(
                        generic_launch,
                        (("sensor_config", sensor_path), ("backend", backend), ("rviz", "false")),
                    )
                    runner_values = self.param_values(runner_config)
                    generic_values = self.param_values(generic_config)
                    runner_mapping = {
                        path: value for path, value in runner_values.items() if path.startswith("/mapping/")
                    }
                    generic_mapping = {
                        path: value for path, value in generic_values.items() if path.startswith("/mapping/")
                    }
                    self.assertEqual(runner_mapping, generic_mapping)


if __name__ == "__main__":
    unittest.main(verbosity=2)
