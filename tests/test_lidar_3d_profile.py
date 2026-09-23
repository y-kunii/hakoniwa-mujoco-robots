"""Tests for the lidar_3d sensor, its profile and the scenes it scans.

The example code beside it, the palette and the Open3D scene rebuild, is
covered by test_livox_mid360s_example.py.

These live at tests/ root rather than under tests/sensors, because unittest
discovery skips directories without __init__.py and those directories hold C++
tests only.
"""

import copy
import json
import unittest
from pathlib import Path

try:
    from jsonschema import Draft202012Validator, RefResolver
except ImportError:  # pragma: no cover - exercised only without jsonschema
    Draft202012Validator = None
    RefResolver = None

try:
    import mujoco
    import numpy as np
except ImportError:  # pragma: no cover - exercised only without mujoco
    mujoco = None
    np = None


ROOT = Path(__file__).resolve().parents[1]
SENSOR_DIR = ROOT / "config/sensors"
SCHEMA_PATH = SENSOR_DIR / "schema/lidar-3d.schema.json"
UNIFORM_PROFILE = SENSOR_DIR / "lidar/livox-mid360s.json"
TABLE_PROFILE = SENSOR_DIR / "lidar/livox-mid360s-table.json"
SAMPLE_SCENE = ROOT / "models/sensors/lidar_3d/livox-mid360s-sample.xml"
MOVING_SCENE = ROOT / "models/sensors/lidar_3d/livox-mid360s-moving-sample.xml"
PDU_DEF = ROOT / "config/livox-mid360s-pdudef-compact.json"
PDU_TYPES = ROOT / "config/livox-mid360s-pdutypes.json"
ENVELOPE_BYTES = 760  # PointCloud2 serialises to this whatever the point count


def load_json(path: Path):
    return json.loads(path.read_text(encoding="utf-8"))


@unittest.skipUnless(Draft202012Validator, "install jsonschema to validate schemas")
class Lidar3dSchemaTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.schema = load_json(SCHEMA_PATH)
        Draft202012Validator.check_schema(cls.schema)
        store = {}
        for path in (SENSOR_DIR / "schema").glob("*.json"):
            doc = load_json(path)
            if "$id" in doc:
                store[doc["$id"]] = doc
        cls.validator = Draft202012Validator(
            cls.schema,
            resolver=RefResolver(base_uri=cls.schema["$id"], referrer=cls.schema, store=store),
        )
        cls.uniform = load_json(UNIFORM_PROFILE)

    def assert_valid(self, doc):
        errors = sorted(self.validator.iter_errors(doc), key=lambda e: list(e.path))
        self.assertEqual(
            errors, [], "\n".join(f"{'/'.join(map(str, e.path))}: {e.message}" for e in errors)
        )

    def test_shipped_profiles_validate(self):
        for path in (UNIFORM_PROFILE, TABLE_PROFILE):
            with self.subTest(profile=path.name):
                self.assert_valid(load_json(path))

    def test_lidar_2d_profiles_still_validate(self):
        """The 3D schema must not disturb the existing 2D profiles."""
        schema_2d = load_json(SENSOR_DIR / "schema/lidar-2d.schema.json")
        store = {}
        for path in (SENSOR_DIR / "schema").glob("*.json"):
            doc = load_json(path)
            if "$id" in doc:
                store[doc["$id"]] = doc
        validator = Draft202012Validator(
            schema_2d,
            resolver=RefResolver(base_uri=schema_2d["$id"], referrer=schema_2d, store=store),
        )
        profiles = sorted((SENSOR_DIR / "lidar").glob("*.json"))
        two_d = [p for p in profiles if load_json(p)["spec"]["type"] == "lidar_2d"]
        self.assertTrue(two_d, "expected at least one lidar_2d profile to guard against")
        for path in two_d:
            with self.subTest(profile=path.name):
                self.assertEqual(list(validator.iter_errors(load_json(path))), [])

    def test_table_pattern_requires_a_table_file(self):
        doc = copy.deepcopy(self.uniform)
        doc["spec"]["ScanPattern"]["Type"] = "table"
        messages = [e.message for e in self.validator.iter_errors(doc)]
        self.assertTrue(
            any("TableFile" in m for m in messages),
            f"a table pattern without TableFile must be rejected, got {messages}",
        )

    def test_point_cloud_message_type_is_pinned(self):
        doc = copy.deepcopy(self.uniform)
        doc["pdu_config"]["message_type"] = "sensor_msgs/LaserScan"
        self.assertTrue(list(self.validator.iter_errors(doc)))

    def test_datasheet_values_are_what_the_profile_declares(self):
        spec = self.uniform["spec"]
        fov = spec["FieldOfView"]
        self.assertEqual(fov["Vertical"]["Min"], -7.0)
        self.assertEqual(fov["Vertical"]["Max"], 52.0)
        self.assertEqual(fov["Horizontal"]["Max"] - fov["Horizontal"]["Min"], 360.0)
        self.assertEqual(spec["ScanPattern"]["PointRate"], 200000)
        self.assertEqual(spec["ScanPattern"]["FrameRate"], 10.0)
        # Millimetres, matching lidar-2d: 0.1 m blind zone, 100 m cutoff.
        self.assertEqual(spec["DetectionDistance"]["Min"], 100)
        self.assertEqual(spec["DetectionDistance"]["Max"], 100000)


class ShippedChannelTest(unittest.TestCase):
    """The example must run from a plain checkout, as color_camera does.

    That needs a pdudef and pdutypes in config/, and they must agree with the
    profile: the asset budgets points against the channel the runtime opens, so
    a channel smaller than the profile's budget truncates every frame.
    """

    def setUp(self):
        self.profile = load_json(UNIFORM_PROFILE)
        self.definition = load_json(PDU_DEF)
        self.entries = load_json(PDU_TYPES)

    def test_the_pdudef_resolves_to_the_shipped_pdutypes(self):
        robot = self.definition["robots"][0]
        self.assertEqual(robot["name"], "Mid360S", "the assets default to this robot name")
        path = next(p["path"] for p in self.definition["paths"]
                    if p["id"] == robot["pdutypes_id"])
        self.assertEqual((PDU_DEF.parent / path).resolve(), PDU_TYPES.resolve())

    def test_the_channel_matches_what_the_profile_declares(self):
        pdu = self.profile["pdu_config"]
        entry = next(e for e in self.entries if e["name"] == pdu["pdu_name"])
        self.assertEqual(entry["type"], pdu["message_type"])
        budget = ENVELOPE_BYTES + int(pdu["max_points"]) * int(pdu["point_step"])
        self.assertGreaterEqual(
            entry["pdu_size"], budget,
            "the shipped channel is smaller than the profile's point budget, "
            "so frames would be truncated",
        )

    def test_the_channel_is_page_aligned(self):
        self.assertEqual(self.entries[0]["pdu_size"] % 4096, 0)


@unittest.skipUnless(mujoco, "install mujoco to exercise the sensor")
class Lidar3dSensorTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        import sys

        sys.path.insert(0, str(ROOT / "python"))
        from livox_mid360s_sensor import LivoxMid360SSensor

        cls.SensorClass = LivoxMid360SSensor
        cls.model = mujoco.MjModel.from_xml_path(str(SAMPLE_SCENE))
        cls.data = mujoco.MjData(cls.model)
        mujoco.mj_forward(cls.model, cls.data)

    def make_sensor(self, **kwargs):
        return self.SensorClass(self.model, UNIFORM_PROFILE, **kwargs)

    def test_rays_per_frame_follow_point_rate_over_frame_rate(self):
        self.assertEqual(self.make_sensor().samples_per_frame, 20000)

    def test_returns_stay_inside_the_declared_range_gate(self):
        sensor = self.make_sensor(apply_noise=False)
        scan = sensor.scan(self.data)
        self.assertGreater(len(scan.distances), 0)
        self.assertGreaterEqual(float(scan.distances.min()), sensor.min_range)
        self.assertLess(float(scan.distances.max()), sensor.max_range)

    def test_points_are_consistent_with_distances(self):
        scan = self.make_sensor(apply_noise=False).scan(self.data)
        radius = np.linalg.norm(scan.points, axis=1)
        np.testing.assert_allclose(radius, scan.distances, rtol=1e-5, atol=1e-5)

    def test_sensor_does_not_detect_its_own_mount(self):
        """exclude_body must keep the post and housing out of the returns."""
        scan = self.make_sensor(apply_noise=False).scan(self.data)
        mount = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "lidar_mount")
        mount_geoms = {
            g for g in range(self.model.ngeom) if self.model.geom_bodyid[g] == mount
        }
        self.assertFalse(mount_geoms & set(scan.geom_ids.tolist()))

    def test_low_object_inside_the_blind_cone_returns_nothing(self):
        """The -7 deg lower limit hides a short object 1.2 m away at 0.5 m height."""
        scan = self.make_sensor(apply_noise=False).scan(self.data)
        hidden = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_GEOM, "box_low_near_geom"
        )
        visible = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_GEOM, "box_tall_near_geom"
        )
        hits = set(scan.geom_ids.tolist())
        self.assertNotIn(hidden, hits, "a low near object must fall under the blind cone")
        self.assertIn(visible, hits, "a tall object at the same distance must be seen")

    def test_noise_perturbs_ranges_without_moving_the_geometry(self):
        clean = self.make_sensor(apply_noise=False, seed=7).scan(self.data)
        noisy = self.make_sensor(apply_noise=True, seed=7).scan(self.data)
        self.assertEqual(len(clean.distances), len(noisy.distances))
        np.testing.assert_array_equal(clean.geom_ids, noisy.geom_ids)
        delta = np.abs(noisy.distances - clean.distances)
        self.assertGreater(float(delta.max()), 0.0, "the accuracy model must do something")
        # Both bands declare a 4 cm or 2 cm sigma; 6 sigma is a generous ceiling.
        self.assertLess(float(delta.max()), 0.24)

    def test_uniform_pattern_needs_no_external_table(self):
        sensor = self.make_sensor()
        self.assertEqual(sensor.pattern.kind, "uniform")
        self.assertEqual(sensor.pattern.table_rays, 0)

    def test_missing_table_file_explains_itself(self):
        profile = load_json(TABLE_PROFILE)
        table_path = (TABLE_PROFILE.parent / profile["spec"]["ScanPattern"]["TableFile"]).resolve()
        if table_path.exists():
            self.skipTest(f"{table_path} is present; this guards the absent case")
        with self.assertRaises(FileNotFoundError) as ctx:
            self.SensorClass(self.model, TABLE_PROFILE)
        self.assertIn("uniform", str(ctx.exception))


if __name__ == "__main__":
    unittest.main()


@unittest.skipUnless(mujoco, "install mujoco to exercise the moving scene")
class MovingSceneTest(unittest.TestCase):
    """The scene that shows the asset advances physics.

    Its whole purpose is that stepping changes what the sensor sees, so the
    tests check exactly that, and that the static scene stays static.
    """

    @classmethod
    def setUpClass(cls):
        import sys

        sys.path.insert(0, str(ROOT / "python"))
        from livox_mid360s_sensor import LivoxMid360SSensor

        cls.SensorClass = LivoxMid360SSensor

    def load(self, scene):
        model = mujoco.MjModel.from_xml_path(str(scene))
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        return model, data

    def geom_position(self, model, data, name):
        geom = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
        self.assertGreaterEqual(geom, 0, f"{name} should exist in the scene")
        return np.asarray(data.geom_xpos[geom]).copy()

    def step(self, model, data, seconds):
        for _ in range(int(round(seconds / model.opt.timestep))):
            mujoco.mj_step(model, data)

    def test_the_static_scene_does_not_move(self):
        """Its objects carry no joints, so stepping must change nothing. The
        blind cone and occlusion tests depend on the positions staying put."""
        model, data = self.load(SAMPLE_SCENE)
        self.assertEqual(model.njnt, 0, "the static scene should have no joints")
        before = self.geom_position(model, data, "ball_geom")
        self.step(model, data, 1.0)
        np.testing.assert_allclose(self.geom_position(model, data, "ball_geom"), before)

    def test_the_moving_scene_moves_under_gravity_alone(self):
        """No actuator and no initial velocity: the pendulum starts horizontal
        and the tower starts unbalanced, so gravity is enough."""
        model, data = self.load(MOVING_SCENE)
        self.assertLess(model.opt.gravity[2], -9.0, "the moving scene needs gravity")
        moving = ("pendulum_bob", "falling_ball_geom", "stack_top_geom")
        before = {name: self.geom_position(model, data, name) for name in moving}
        self.step(model, data, 1.0)
        for name in moving:
            travelled = np.linalg.norm(self.geom_position(model, data, name) - before[name])
            self.assertGreater(travelled, 0.5, f"{name} should have moved after a second")

    def test_the_moving_scene_keeps_a_static_reference(self):
        """Something has to stay still, or a moving return could be the sensor
        rather than the scene."""
        model, data = self.load(MOVING_SCENE)
        still = ("box_static_geom", "wall_back_geom", "lidar_housing")
        before = {name: self.geom_position(model, data, name) for name in still}
        self.step(model, data, 1.0)
        for name in still:
            np.testing.assert_allclose(
                self.geom_position(model, data, name), before[name], atol=1e-9,
                err_msg=f"{name} should not move")

    def test_the_sensor_sees_the_motion(self):
        """Returns on a moving object must change while the static reference
        keeps returning. Without stepping, both would be constant."""
        model, data = self.load(MOVING_SCENE)
        sensor = self.SensorClass(model, UNIFORM_PROFILE, apply_noise=False)
        ball = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "falling_ball_geom")
        static = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "box_static_geom")

        heights, static_hits = [], []
        for _ in range(6):
            self.step(model, data, 0.1)
            scan = sensor.scan(data)
            hit = scan.points[scan.geom_ids == ball]
            if len(hit):
                heights.append(float(np.mean(hit[:, 2])))
            static_hits.append(int((scan.geom_ids == static).sum()))

        self.assertGreaterEqual(len(heights), 3, "the falling ball should be in view")
        self.assertGreater(heights[0] - heights[-1], 0.5,
                           "the ball's returns should descend as it falls")
        self.assertTrue(all(count > 0 for count in static_hits),
                        "the static reference should return throughout")
