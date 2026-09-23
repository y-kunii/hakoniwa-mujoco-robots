"""Tests for the example code beside the Livox Mid-360S sensor.

The sensor and its profile are covered by test_lidar_3d_profile.py. This file
covers what only the example ships: the range palette both windows share, and
the rebuild of a MuJoCo scene as Open3D meshes.

These live at tests/ root rather than under tests/sensors, because unittest
discovery skips directories without __init__.py and those directories hold C++
tests only.
"""

import unittest
from pathlib import Path

try:
    import mujoco
    import numpy as np
except ImportError:  # pragma: no cover - exercised only without mujoco
    mujoco = None
    np = None

try:
    import open3d  # noqa: F401
except ImportError:  # pragma: no cover - exercised only without open3d
    open3d = None


ROOT = Path(__file__).resolve().parents[1]
SAMPLE_SCENE = ROOT / "models/sensors/lidar_3d/livox-mid360s-sample.xml"


def _example_module(name):
    import sys

    sys.path.insert(0, str(ROOT / "examples/sensors/livox_mid360s"))
    return __import__(name)


@unittest.skipUnless(np, "install numpy to exercise the palette")
class PointColorsTest(unittest.TestCase):
    """Both windows colour by range, so the mapping has to be shared and stable."""

    @classmethod
    def setUpClass(cls):
        cls.colors = _example_module("point_colors")

    def test_near_and_far_are_told_apart(self):
        near, far = self.colors.by_range(np.array([0.5])), self.colors.by_range(np.array([24.0]))
        self.assertGreater(float(np.abs(near - far).max()), 0.3)

    def test_beyond_full_scale_clamps_rather_than_wrapping(self):
        """A point past full scale must stay the far colour, not wrap to near."""
        at_scale = self.colors.by_range(np.array([self.colors.FULL_SCALE_M]))
        beyond = self.colors.by_range(np.array([10 * self.colors.FULL_SCALE_M]))
        np.testing.assert_allclose(at_scale, beyond)

    def test_every_channel_stays_in_gamut(self):
        values = self.colors.by_range(np.linspace(0, 40, 200))
        self.assertGreaterEqual(float(values.min()), 0.0)
        self.assertLessEqual(float(values.max()), 1.0)


@unittest.skipUnless(mujoco and open3d, "install mujoco and open3d to exercise the scene view")
class SceneViewTest(unittest.TestCase):
    """The scene is rebuilt from MjModel, so every shipped geom must convert."""

    @classmethod
    def setUpClass(cls):
        cls.scene_view = _example_module("scene_view")
        cls.model = mujoco.MjModel.from_xml_path(str(SAMPLE_SCENE))

    def test_every_geom_in_the_sample_scene_becomes_a_mesh(self):
        for geom in range(self.model.ngeom):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, geom)
            with self.subTest(geom=name):
                mesh = self.scene_view._mesh_for(self.model, geom)
                self.assertIsNotNone(mesh, "a shipped geom type is not drawn")
                self.assertGreater(len(mesh.vertices), 0)

    def test_an_unsupported_geom_type_is_skipped_rather_than_crashing(self):
        model = mujoco.MjModel.from_xml_string(
            """<mujoco><worldbody><body><geom type="ellipsoid" size="1 2 3"/>
               </body></worldbody></mujoco>""")
        self.assertIsNone(self.scene_view._mesh_for(model, 0))

    def test_colour_comes_from_the_material_when_there_is_one(self):
        """MJCF colours through materials and leaves geom_rgba at its grey
        default, so reading geom_rgba alone renders the whole scene grey."""
        coloured = [g for g in range(self.model.ngeom) if int(self.model.geom_matid[g]) >= 0]
        self.assertTrue(coloured, "the sample scene should colour through materials")
        for geom in coloured:
            material = int(self.model.geom_matid[geom])
            np.testing.assert_allclose(
                self.scene_view._color_of(self.model, geom),
                np.asarray(self.model.mat_rgba[material])[:3])

    def test_an_infinite_plane_gets_a_finite_mesh(self):
        model = mujoco.MjModel.from_xml_string(
            """<mujoco><worldbody><geom type="plane" size="0 0 1"/></worldbody></mujoco>""")
        mesh = self.scene_view._mesh_for(model, 0)
        extent = np.asarray(mesh.get_max_bound()) - np.asarray(mesh.get_min_bound())
        self.assertAlmostEqual(extent[0], 2 * self.scene_view.PLANE_FALLBACK_HALF, places=5)
