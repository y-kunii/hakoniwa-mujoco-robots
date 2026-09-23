"""Draw the MuJoCo scene and the LiDAR returns in one Open3D window.

MuJoCo has its own passive viewer, and drawing into it is the obvious thing to
try. It was tried. On the machine this example was developed on the window
rendered but would not reliably take mouse input, through several attempts, so
the display moved to Open3D, which does. Nothing is lost by that: MuJoCo is used
here as a ray caster and a scene description, not as a renderer and not as a
physics engine. Only mj_forward and mj_multiRay are called; nothing steps.

Meshes are built once and then re-posed each frame from MjModel, so a scene
with joints follows its physics. Building them once and leaving them put was a
bug worth naming: the cloud moved while the geometry it came from stayed at its
starting pose, which reads as the sensor drifting rather than the scene moving.
"""

from __future__ import annotations

import numpy as np
import mujoco
import open3d as o3d

from point_colors import by_range

# MuJoCo geom sizes are half-extents; Open3D primitives take full sizes.
PLANE_FALLBACK_HALF = 30.0  # a MuJoCo plane with size 0 is infinite


def _color_of(model: mujoco.MjModel, geom: int) -> np.ndarray:
    """The geom's colour, from its material when it has one.

    MJCF usually assigns colour through a material, leaving geom_rgba at its
    grey default. Reading only geom_rgba renders the whole scene grey.
    """
    material = int(model.geom_matid[geom])
    if material >= 0:
        return np.asarray(model.mat_rgba[material])[:3]
    return np.asarray(model.geom_rgba[geom])[:3]


def _mesh_for(model: mujoco.MjModel, geom: int):
    """One Open3D mesh for one MuJoCo geom, or None if the type is not drawn."""
    kind = int(model.geom_type[geom])
    size = model.geom_size[geom]
    G = mujoco.mjtGeom

    if kind == G.mjGEOM_PLANE:
        x = size[0] if size[0] > 0 else PLANE_FALLBACK_HALF
        y = size[1] if size[1] > 0 else PLANE_FALLBACK_HALF
        mesh = o3d.geometry.TriangleMesh.create_box(2 * x, 2 * y, 0.01)
        mesh.translate((-x, -y, -0.01))
    elif kind == G.mjGEOM_BOX:
        mesh = o3d.geometry.TriangleMesh.create_box(*(2 * size[:3]))
        mesh.translate(-size[:3])
    elif kind == G.mjGEOM_SPHERE:
        mesh = o3d.geometry.TriangleMesh.create_sphere(float(size[0]), resolution=12)
    elif kind == G.mjGEOM_CYLINDER:
        mesh = o3d.geometry.TriangleMesh.create_cylinder(float(size[0]), 2 * float(size[1]),
                                                         resolution=12)
    elif kind == G.mjGEOM_CAPSULE:
        # Close enough for a backdrop; the returns come from MuJoCo either way.
        mesh = o3d.geometry.TriangleMesh.create_cylinder(float(size[0]), 2 * float(size[1]),
                                                         resolution=12)
    else:
        return None
    return mesh


class SceneView:
    """A window showing the scene once and the cloud every frame."""

    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData,
                 point_size: float = 2.5, decimate: int = 1):
        self.decimate = max(1, decimate)
        self.movable: list = []
        self.vis = o3d.visualization.Visualizer()
        self.opened = self.vis.create_window(
            window_name="Livox Mid-360S on the MuJoCo scene", width=1400, height=900)
        if not self.opened:
            return

        option = self.vis.get_render_option()
        option.point_size = point_size
        option.background_color = np.array([0.06, 0.07, 0.09])
        option.mesh_show_back_face = True

        for geom in range(model.ngeom):
            mesh = _mesh_for(model, geom)
            if mesh is None:
                continue
            # Keep the untransformed vertices so the mesh can be re-posed each
            # frame without accumulating floating point drift.
            local = np.asarray(mesh.vertices).copy()
            mesh.rotate(np.asarray(data.geom_xmat[geom]).reshape(3, 3),
                        center=(0, 0, 0))
            mesh.translate(np.asarray(data.geom_xpos[geom]))
            mesh.compute_vertex_normals()
            mesh.paint_uniform_color(_color_of(model, geom))
            self.vis.add_geometry(mesh)
            if int(model.body_dofnum[int(model.geom_bodyid[geom])]) > 0:
                # Only geoms on a body with degrees of freedom can move, so
                # only those are worth re-posing.
                self.movable.append((geom, mesh, local))

        self.cloud = o3d.geometry.PointCloud()
        self.added = False

    def follow_scene(self, data: mujoco.MjData) -> None:
        """Move the meshes to where physics has put them."""
        for geom, mesh, local in self.movable:
            rotation = np.asarray(data.geom_xmat[geom]).reshape(3, 3)
            mesh.vertices = o3d.utility.Vector3dVector(
                local @ rotation.T + np.asarray(data.geom_xpos[geom]))
            mesh.compute_vertex_normals()
            self.vis.update_geometry(mesh)

    def update(self, points: np.ndarray, distances: np.ndarray,
               origin: np.ndarray, rotation: np.ndarray) -> bool:
        """Draw one frame. False once the window has been closed."""
        if not self.opened:
            return False
        points = points[::self.decimate]
        distances = distances[::self.decimate]
        self.cloud.points = o3d.utility.Vector3dVector(points @ rotation.T + origin)
        self.cloud.colors = o3d.utility.Vector3dVector(by_range(distances))
        if not self.added:
            self.vis.add_geometry(self.cloud, reset_bounding_box=True)
            self.added = True
        else:
            self.vis.update_geometry(self.cloud)
        if not self.vis.poll_events():
            return False
        self.vis.update_renderer()
        return True

    def capture(self, path: str) -> None:
        self.vis.poll_events()
        self.vis.update_renderer()
        self.vis.capture_screen_image(path, do_render=True)

    def close(self) -> None:
        if self.opened:
            self.vis.destroy_window()
