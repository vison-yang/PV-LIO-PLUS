#!/usr/bin/env python3
"""验证所有地图参数的唯一配置来源及真实 Noetic 离线加载。"""

import hashlib
import importlib.util
import json
import os
from pathlib import Path
import re
import sys
import tempfile
from types import SimpleNamespace
import unittest

import yaml

REPO_ROOT = Path(__file__).resolve().parents[1]
SENSORS = ("avia", "mid360_indoor", "solid")
BACKENDS = ("voxelmap", "voxelmap_plus", "ikdtree", "ivox", "octvox", "c3p_voxelmap",
            "r_voxelmap", "hvox", "i_octree", "ig_lio_plane", "bievr")
GM_BACKENDS = ("ikdtree", "ivox", "i_octree", "ig_lio_plane")
WRAPPERS = {"mapping_avia.launch": ("avia", "voxelmap_plus"),
            "mapping_mid360.launch": ("mid360_indoor", "voxelmap_plus"),
            "mapping_solid.launch": ("solid", "ivox")}
SENSOR_KEYS = {"down_sample_size", "max_iteration", "extrinsic_est_en", "extrinsic_T", "extrinsic_R"}
WINDOW_BACKENDS = {"voxelmap", "voxelmap_plus", "ikdtree", "ivox", "c3p_voxelmap", "i_octree"}
POINT_PLANE_KEYS = {"nearest_point_count", "nearest_max_range", "plane_fit_threshold"}
GEOMETRY_KEYS = {"voxel_size", "max_layer", "layer_point_size", "plannar_threshold"}
REQUIRED_KEYS = {
    "voxelmap": GEOMETRY_KEYS | {"max_points_size", "max_cov_points_size", "sigma_num"},
    "voxelmap_plus": {"voxel_size", "plannar_threshold", "max_points_size", "sigma_num", "update_size_threshold"},
    "c3p_voxelmap": GEOMETRY_KEYS | {"sigma_num", "c3p_enable_voxel_merging", "c3p_merge_theta_thresh",
        "c3p_merge_dist_thresh", "c3p_merge_cov_min_eigen_val_thresh", "c3p_merge_x_coord_diff_thresh",
        "c3p_merge_y_coord_diff_thresh"},
    "ikdtree": POINT_PLANE_KEYS | {"ikd_delete_param", "ikd_balance_param", "ikd_box_length"},
    "ivox": POINT_PLANE_KEYS | {"ivox_resolution", "ivox_nearby_type", "ivox_capacity", "ivox_optimized"},
    "octvox": {"octvox_resolution", "octvox_capacity"},
    "hvox": {"hvox_voxel_size", "hvox_hierarchy_factor", "hvox_planarity_threshold", "hvox_max_distance",
              "hvox_box_multiplier", "hvox_lidar_noise_std"},
    "i_octree": POINT_PLANE_KEYS | {"i_octree_bucket_size", "i_octree_min_extent", "i_octree_on_tree_downsample"},
    "ig_lio_plane": POINT_PLANE_KEYS | {"ig_lio_resolution", "ig_lio_search_method", "ig_lio_capacity",
                                      "ig_lio_grid_max_points"},
    "bievr": {"bievr_voxel_size", "bievr_pixel_size", "bievr_capacity", "bievr_max_image_pixels",
              "bievr_weighted", "bievr_smooth", "bievr_normal_tolerance_deg", "bievr_height_sigma_m",
              "bievr_huber_delta_m"},
}


def read_yaml(path):
    return yaml.safe_load(Path(path).read_text(encoding="utf-8"))


def flatten(value, prefix=""):
    result = {}
    for key, child in value.items():
        path = f"{prefix}/{key}"
        if isinstance(child, dict):
            result.update(flatten(child, path))
        else:
            result[path] = child
    return result


class MapConfigProfileTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls._environment = {key: os.environ.get(key) for key in ("ROS_PACKAGE_PATH", "ROS_HOME", "ROS_LOG_DIR")}
        cls._temporary = tempfile.TemporaryDirectory(prefix="pv_lio_config_profiles_")
        os.environ["ROS_PACKAGE_PATH"] = os.pathsep.join((str(REPO_ROOT.parent), "/opt/ros/noetic/share"))
        os.environ["ROS_HOME"] = cls._temporary.name
        os.environ["ROS_LOG_DIR"] = cls._temporary.name
        import roslaunch.config
        import rospkg
        if Path(rospkg.RosPack().get_path("pv_lio_plus")).resolve() != REPO_ROOT:
            raise AssertionError("pv_lio_plus resolved from another checkout")
        cls._load_config = staticmethod(roslaunch.config.load_config_default)
        spec = importlib.util.spec_from_file_location("pv_lio_config_runner", REPO_ROOT / "scripts/run_backend_tests.py")
        cls._runner = importlib.util.module_from_spec(spec)
        sys.modules[spec.name] = cls._runner
        old_bytecode = sys.dont_write_bytecode
        sys.dont_write_bytecode = True
        try:
            spec.loader.exec_module(cls._runner)
        finally:
            sys.dont_write_bytecode = old_bytecode
        cls._fixture = json.loads((REPO_ROOT / "tests/data/config_pre_split.json").read_text())
        cls._comparisons = 0

    @classmethod
    def tearDownClass(cls):
        for key, value in cls._environment.items():
            if value is None:
                os.environ.pop(key, None)
            else:
                os.environ[key] = value
        sys.modules.pop("pv_lio_config_runner", None)
        cls._temporary.cleanup()
        print(f"Verified backend/sensor mapping comparisons: {cls._comparisons}")

    @classmethod
    def load(cls, path, args=()):
        return cls._load_config([(str(path), [f"{key}:={value}" for key, value in args])],
                                port=0, verbose=False, assign_machines=False)

    @classmethod
    def load_text(cls, name, text):
        path = Path(cls._temporary.name) / name
        path.write_text(text, encoding="utf-8")
        return cls.load(path)

    @staticmethod
    def values(config):
        return {key: param.value for key, param in config.params.items()}

    @staticmethod
    def sensor_path(sensor):
        return REPO_ROOT / "config" / f"{sensor}.yaml"

    @classmethod
    def dataset(cls, sensor, config=None):
        path = config or cls.sensor_path(sensor)
        values = read_yaml(path)
        return SimpleNamespace(config=path, lidar_topic=values["common"]["lid_topic"],
                               imu_topic=values["common"]["imu_topic"], lidar_type=values["preprocess"]["lidar_type"])

    @classmethod
    def expected_mapping(cls, sensor, backend):
        values = dict(read_yaml(cls.sensor_path(sensor))["mapping"])
        values.update(read_yaml(REPO_ROOT / "config/backends" / f"{backend}.yaml")["mapping"])
        values["map_type"] = backend
        return {f"/mapping/{key}": value for key, value in values.items()}

    def assert_mapping(self, actual, sensor, backend, allow_inactive=False):
        expected = self.expected_mapping(sensor, backend)
        if not allow_inactive:
            self.assertEqual({key: value for key, value in actual.items() if key.startswith("/mapping/")}, expected)
        else:
            for key, value in expected.items():
                self.assertEqual(actual[key], value, f"{sensor}/{backend}: {key}")
        type(self)._comparisons += 1

    def test_backend_parameters_have_one_runtime_owner_and_all_consumed_keys(self):
        directory = REPO_ROOT / "config/backends"
        self.assertEqual({path.stem for path in directory.glob("*.yaml")}, set(BACKENDS))
        self.assertFalse((directory / "sensors").exists())
        source = (REPO_ROOT / "src/config.cpp").read_text()
        consumed = set(re.findall(r'nh\.param[^\(]*\(\s*"([^"]+)"', source, re.S))
        required = dict(REQUIRED_KEYS)
        required["r_voxelmap"] = GEOMETRY_KEYS | {"max_points_size"} | {
            key for key in self._fixture["backends"]["r_voxelmap"]["mapping"] if key.startswith("r_voxelmap_")}
        for backend in BACKENDS:
            with self.subTest(backend=backend):
                profile = read_yaml(directory / f"{backend}.yaml")
                self.assertEqual(set(profile), {"mapping"})
                mapping = profile["mapping"]
                keys = required[backend] | {"map_type", "local_window_en"}
                if backend in WINDOW_BACKENDS:
                    keys |= {"det_range"}
                if backend in {"ikdtree", "ivox"}:
                    keys |= {"point_map_downsample_size"}
                self.assertEqual(set(mapping), keys)
                self.assertEqual(mapping["map_type"], backend)
                self.assertFalse(mapping["local_window_en"])
                self.assertFalse(SENSOR_KEYS.intersection(mapping))
                self.assertTrue(set(flatten(profile)).issubset({f"/{key}" for key in consumed}))
                old = dict(self._fixture["sensors"]["avia"]["mapping"])
                old.update(self._fixture["backends"][backend]["mapping"])
                for key, value in mapping.items():
                    self.assertEqual(value, old[key], f"{backend}/{key}")
        for sensor in SENSORS:
            self.assertFalse(any(key.startswith("c3p_") for key in read_yaml(self.sensor_path(sensor))["mapping"]))

    def test_main_yaml_retains_only_sensor_and_shared_filter_parameters(self):
        for sensor in SENSORS:
            actual = read_yaml(self.sensor_path(sensor))
            old = self._fixture["sensors"][sensor]
            keys = SENSOR_KEYS
            self.assertEqual(actual["mapping"], {key: old["mapping"][key] for key in keys})
            self.assertEqual({key: value for key, value in actual.items() if key != "mapping"},
                             {key: value for key, value in old.items() if key != "mapping"})

    def test_all_sensor_backend_combinations_load_sensor_then_one_backend(self):
        for sensor in SENSORS:
            for backend in BACKENDS:
                with self.subTest(sensor=sensor, backend=backend):
                    config = self.load(REPO_ROOT / "launch/mapping.launch",
                        (("sensor_config", self.sensor_path(sensor)), ("backend", backend), ("rviz", "false")))
                    self.assert_mapping(self.values(config), sensor, backend)
                    self.assertFalse(any(node.package == "rviz" for node in config.nodes))

    def test_wrapper_defaults_and_explicit_backend_selection(self):
        for wrapper, (sensor, default) in WRAPPERS.items():
            config = self.load(REPO_ROOT / "launch" / wrapper)
            self.assert_mapping(self.values(config), sensor, default)
            self.assertTrue(any(node.package == "rviz" for node in config.nodes))
            for backend in BACKENDS:
                config = self.load(REPO_ROOT / "launch" / wrapper, (("backend", backend), ("rviz", "false")))
                self.assert_mapping(self.values(config), sensor, backend)

    def test_runner_default_is_the_same_as_explicit_backend_archive_and_launch(self):
        for sensor in SENSORS:
            dataset = self.dataset(sensor)
            for backend in BACKENDS:
                preset = self._runner.backend_preset_path(backend)
                for explicit in (None, preset):
                    config = self.load_text(f"runner_{sensor}_{backend}_{explicit is not None}.launch",
                        self._runner.launch_text(dataset, backend, explicit))
                    self.assert_mapping(self.values(config), sensor, backend)

    def test_mapping_parameters_reset_across_sequential_sensor_and_backend_choices(self):
        master = {}
        for index, (sensor, backend) in enumerate((sensor, backend) for sensor in ("solid", "avia", "mid360_indoor")
                                                  for backend in reversed(BACKENDS)):
            config = self.load_text(f"sequential_{index}.launch", self._runner.launch_text(self.dataset(sensor), backend))
            master.update(self.values(config))  # 模拟同一 master 的叶参数覆盖，不启动 master。
            self.assert_mapping(master, sensor, backend, allow_inactive=True)
        for sensor in ("solid", "avia", "solid", "mid360_indoor"):
            master.update(self.values(self.load_text(f"sequential_ivox_{sensor}.launch",
                self._runner.launch_text(self.dataset(sensor), "ivox"))))
            self.assert_mapping(master, sensor, "ivox", allow_inactive=True)

    def test_paths_need_no_sensor_detection_and_selected_backend_wins_old_selector(self):
        alias = REPO_ROOT / "config/../config/solid.yaml"
        config = self.load(REPO_ROOT / "launch/mapping.launch",
            (("sensor_config", alias), ("backend", "c3p_voxelmap"), ("rviz", "false")))
        self.assert_mapping(self.values(config), "solid", "c3p_voxelmap")
        external = Path(self._temporary.name) / "external_sensor.yaml"
        values = read_yaml(self.sensor_path("avia"))
        values["mapping"]["map_type"] = "ivox"
        values["mapping"]["voxel_size"] = 0.75
        external.write_text(yaml.safe_dump(values))
        dataset = self.dataset("avia", external)
        for config in [self.load(REPO_ROOT / "launch/mapping.launch",
                           (("sensor_config", external), ("backend", "voxelmap"), ("rviz", "false"))),
                       self.load_text("external_runner.launch", self._runner.launch_text(dataset, "voxelmap"))]:
            actual = self.values(config)
            self.assertEqual(actual["/mapping/map_type"], "voxelmap")
            self.assertEqual(actual["/mapping/voxel_size"], 0.5)

    def test_backend_and_sensor_archives_are_loaded_in_documented_order(self):
        run = Path(self._temporary.name) / "archives"
        run.mkdir()
        metadata = self._runner.backend_profile_metadata("ivox", run)
        self.assertEqual(metadata["backend_config_mode"], "sensor_then_backend")
        record = metadata["backend_preset"]
        preset = Path(record["archive"])
        self.assertEqual(preset, run / "backend_config.yaml")
        self.assertEqual(record["sha256"], hashlib.sha256(preset.read_bytes()).hexdigest())
        self.assertEqual(record["source"], str(self._runner.backend_preset_path("ivox")))
        sensor = run / "base_config.yaml"
        sensor.write_bytes(self.sensor_path("solid").read_bytes())
        dataset = self.dataset("solid", sensor)
        text = self._runner.launch_text(dataset, "ivox", preset)
        self.assertLess(text.index(str(sensor)), text.index(str(preset)))
        self.assertNotIn("map_configs/", text)
        self.assert_mapping(self.values(self.load_text("archived.launch", text)), "solid", "ivox")

    def test_observation_profiles_stay_independent_and_after_sensor(self):
        expected = {"backend_default": {"weight_model": "backend_default"},
                    "geman_mcclure": {"weight_model": "geman_mcclure", "gm_scale_m": 0.1}}
        for sensor in SENSORS:
            for backend in GM_BACKENDS:
                for profile in expected:
                    config = self.load(REPO_ROOT / "launch/mapping.launch",
                        (("sensor_config", self.sensor_path(sensor)), ("backend", backend),
                         ("observation", profile), ("rviz", "false")))
                    actual = self.values(config)
                    for key, value in expected[profile].items():
                        self.assertEqual(actual[f"/observation/{key}"], value)
        for profile in expected:
            path = self._runner.observation_profile_path(profile)
            text = self._runner.launch_text(self.dataset("solid"), "ivox", observation_config=path)
            self.assertGreater(text.index(str(path)), text.index(str(self.sensor_path("solid"))))
            values = self.values(self.load_text(f"obs_{profile}.launch", text))
            for key, value in expected[profile].items():
                self.assertEqual(values[f"/observation/{key}"], value)

    def test_no_mapping_parameter_is_defined_in_both_sensor_and_backend_yaml(self):
        for sensor in SENSORS:
            sensor_keys = set(read_yaml(self.sensor_path(sensor))["mapping"])
            for backend in BACKENDS:
                backend_keys = set(read_yaml(REPO_ROOT / "config/backends" / f"{backend}.yaml")["mapping"])
                with self.subTest(sensor=sensor, backend=backend):
                    self.assertEqual(sensor_keys.intersection(backend_keys), set())
        solid = self.values(self.load(REPO_ROOT / "launch/mapping_solid.launch", (("rviz", "false"),)))
        self.assertEqual(solid["/mapping/nearest_max_range"], 5.0)
        self.assertEqual(solid["/mapping/ivox_resolution"], 0.2)
        self.assertEqual(solid["/mapping/ivox_capacity"], 1000000)
        self.assertEqual(solid["/mapping/point_map_downsample_size"], 0.1)
        octvox = self.values(self.load(REPO_ROOT / "launch/mapping_solid.launch",
                                      (("backend", "octvox"), ("rviz", "false"))))
        self.assertEqual(octvox["/mapping/octvox_capacity"], 1000000)

    def test_point_time_sensor_profile_and_explicit_runtime_output_directory(self):
        sensor = "solid_point_time"
        values = read_yaml(self.sensor_path(sensor))
        self.assertEqual(set(values["mapping"]), SENSOR_KEYS)
        self.assertEqual(values["preprocess"]["solid_timestamp_mode"], "point_absolute")
        for backend in BACKENDS:
            mapping = read_yaml(REPO_ROOT / "config/backends" / f"{backend}.yaml")["mapping"]
            self.assertFalse(SENSOR_KEYS.intersection(mapping))
            config = self.load(REPO_ROOT / "launch/mapping.launch",
                (("sensor_config", self.sensor_path(sensor)), ("backend", backend), ("rviz", "false")))
            self.assert_mapping(self.values(config), sensor, backend)
        output = Path(self._temporary.name) / "isolated output"
        text = self._runner.launch_text(self.dataset(sensor), "bievr", output_root=output)
        actual = self.values(self.load_text("point_time_output.launch", text))
        self.assertEqual(actual["/output/root_dir"], str(output))
        self.assertEqual(actual["/preprocess/solid_timestamp_mode"], "point_absolute")

    def test_removed_map_layers_fail_explicitly_and_unknown_backends_are_rejected(self):
        root = Path(self._temporary.name)
        bag = root / "placeholder.bag"
        bag.write_bytes(b"")  # 只测试 manifest 解析，不读取该占位文件的消息。
        raw = {"name": "test", "bag": str(bag), "config": str(self.sensor_path("solid")),
               "lidar_type": "solid", "lidar_topic": "/lidar/data", "imu_topic": "/s316_imu", "map_configs": []}
        with self.assertRaises(RuntimeError):
            self._runner.parse_dataset(raw, root, self.sensor_path("avia"))
        for backend in ("unknown_backend", "none"):
            with self.assertRaises(Exception):
                self.load(REPO_ROOT / "launch/mapping.launch", (("backend", backend),))
        with self.assertRaises(Exception):
            self.load(REPO_ROOT / "launch/mapping.launch", (("observation", "unknown_profile"),))


if __name__ == "__main__":
    unittest.main(verbosity=2)
