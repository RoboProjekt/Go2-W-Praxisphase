# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

# needed to import for allowing type-hinting: np.ndarray | torch.Tensor | None
from __future__ import annotations

import numpy as np
import torch
import trimesh

import warp as wp

from isaaclab.utils.warp import raycast_mesh

import matplotlib.pyplot as plt  # NEW


def color_meshes_by_height(meshes: list[trimesh.Trimesh], **kwargs) -> trimesh.Trimesh:
    """
    Color the vertices of a trimesh object based on the z-coordinate (height) of each vertex,
    using the Turbo colormap. If the z-coordinates are all the same, the vertices will be colored
    with a single color.

    Args:
        meshes: A list of trimesh objects.

    Keyword Args:
        color: A list of 3 integers in the range [0,255] representing the RGB
            color of the mesh. Used when the z-coordinates of all vertices are the same.
            Defaults to [172, 216, 230].
        color_map: The name of the color map to be used. Defaults to "turbo".

    Returns:
        A trimesh object with the vertices colored based on the z-coordinate (height) of each vertex.
    """
    # Combine all meshes into a single mesh
    mesh = trimesh.util.concatenate(meshes)
    # Get the z-coordinates of each vertex
    heights = mesh.vertices[:, 2]
    # Check if the z-coordinates are all the same
    if np.max(heights) == np.min(heights):
        # Obtain a single color: light blue
        color = kwargs.pop("color", (172, 216, 230))
        color = np.asarray(color, dtype=np.uint8)
        # Set the color for all vertices
        mesh.visual.vertex_colors = color
    else:
        # Normalize the heights to [0,1]
        heights_normalized = (heights - np.min(heights)) / (np.max(heights) - np.min(heights))
        # Clip lower and upper bounds to have better color mapping
        heights_normalized = np.clip(heights_normalized, 0.1, 0.9)
        # Get the color for each vertex based on the height
        color_map = kwargs.pop("color_map", "turbo")
        try:
            # Versuch mit eingebauten Trimesh-Colormaps
            colors = trimesh.visual.color.interpolate(heights_normalized, color_map=color_map)
        except ValueError:
            # Fallback auf Matplotlib-Colormap
            try:
                cmap = plt.get_cmap(color_map)
                colors = trimesh.visual.color.interpolate(heights_normalized, color_map=cmap)
            except Exception as e:
                print(f"[WARN] Unknown color_map '{color_map}', using default 'viridis'. Error: {e}")
                cmap = plt.get_cmap("viridis")
                colors = trimesh.visual.color.interpolate(heights_normalized, color_map=cmap)
        # Set the vertex colors
        mesh.visual.vertex_colors = colors
    # Return the mesh
    return mesh


def create_prim_from_mesh(prim_path: str, mesh: trimesh.Trimesh, **kwargs):
    """Create a USD prim with mesh defined from vertices and triangles."""
    # need to import these here to prevent isaacsim launching when importing this module
    import isaacsim.core.utils.prims as prim_utils
    from pxr import UsdGeom

    import isaaclab.sim as sim_utils

    # create parent prim
    prim_utils.create_prim(prim_path, "Xform")
    # create mesh prim
    prim = prim_utils.create_prim(
        f"{prim_path}/mesh",
        "Mesh",
        translation=kwargs.get("translation"),
        orientation=kwargs.get("orientation"),
        attributes={
            "points": mesh.vertices,
            "faceVertexIndices": mesh.faces.flatten(),
            "faceVertexCounts": np.asarray([3] * len(mesh.faces)),
            "subdivisionScheme": "bilinear",
        },
    )
    # apply collider properties
    collider_cfg = sim_utils.CollisionPropertiesCfg(collision_enabled=True)
    sim_utils.define_collision_properties(prim.GetPrimPath(), collider_cfg)
    # add rgba color to the mesh primvars
    if mesh.visual.vertex_colors is not None:
        rgba_colors = np.asarray(mesh.visual.vertex_colors).astype(np.float32) / 255.0
        color_prim_attr = prim.GetAttribute("primvars:displayColor")
        color_prim_var = UsdGeom.Primvar(color_prim_attr)
        color_prim_var.SetInterpolation(UsdGeom.Tokens.vertex)
        color_prim_attr.Set(rgba_colors[:, :3])
        display_prim_attr = prim.GetAttribute("primvars:displayOpacity")
        display_prim_var = UsdGeom.Primvar(display_prim_attr)
        display_prim_var.SetInterpolation(UsdGeom.Tokens.vertex)
        display_prim_var.Set(rgba_colors[:, 3])

    # create visual material
    if kwargs.get("visual_material") is not None:
        visual_material_cfg: sim_utils.VisualMaterialCfg = kwargs.get("visual_material")
        visual_material_cfg.func(f"{prim_path}/visualMaterial", visual_material_cfg)
        sim_utils.bind_visual_material(prim.GetPrimPath(), f"{prim_path}/visualMaterial")
    # create physics material
    if kwargs.get("physics_material") is not None:
        physics_material_cfg: sim_utils.RigidBodyMaterialCfg = kwargs.get("physics_material")
        physics_material_cfg.func(f"{prim_path}/physicsMaterial", physics_material_cfg)
        sim_utils.bind_physics_material(prim.GetPrimPath(), f"{prim_path}/physicsMaterial")


def find_flat_patches(
    wp_mesh: wp.Mesh,
    num_patches: int,
    patch_radius: float | list[float],
    origin: np.ndarray | torch.Tensor | tuple[float, float, float],
    x_range: tuple[float, float],
    y_range: tuple[float, float],
    z_range: tuple[float, float],
    max_height_diff: float,
) -> torch.Tensor:
    """Finds flat patches of given radius in the input mesh."""
    device = wp.device_to_torch(wp_mesh.device)

    # Resolve inputs
    if isinstance(patch_radius, float):
        patch_radius = [patch_radius]
    if isinstance(origin, np.ndarray):
        origin = torch.from_numpy(origin).to(torch.float).to(device)
    elif isinstance(origin, torch.Tensor):
        origin = origin.to(device)
    else:
        origin = torch.tensor(origin, dtype=torch.float, device=device)

    # Create bounded ranges
    x_range = (
        max(x_range[0] + origin[0].item(), wp_mesh.points.numpy()[:, 0].min()),
        min(x_range[1] + origin[0].item(), wp_mesh.points.numpy()[:, 0].max()),
    )
    y_range = (
        max(y_range[0] + origin[1].item(), wp_mesh.points.numpy()[:, 1].min()),
        min(y_range[1] + origin[1].item(), wp_mesh.points.numpy()[:, 1].max()),
    )
    z_range = (z_range[0] + origin[2].item(), z_range[1] + origin[2].item())

    # Create circle query points
    angle = torch.linspace(0, 2 * np.pi, 10, device=device)
    query_x, query_y = [], []
    for radius in patch_radius:
        query_x.append(radius * torch.cos(angle))
        query_y.append(radius * torch.sin(angle))
    query_x = torch.cat(query_x).unsqueeze(1)
    query_y = torch.cat(query_y).unsqueeze(1)
    query_points = torch.cat([query_x, query_y, torch.zeros_like(query_x)], dim=-1)

    # Buffers
    points_ids = torch.arange(num_patches, device=device)
    flat_patches = torch.zeros(num_patches, 3, device=device)

    # Sampling loop
    iter_count = 0
    while len(points_ids) > 0 and iter_count < 10000:
        pos_x = torch.empty(len(points_ids), device=device).uniform_(*x_range)
        pos_y = torch.empty(len(points_ids), device=device).uniform_(*y_range)
        flat_patches[points_ids, :2] = torch.stack([pos_x, pos_y], dim=-1)

        points = flat_patches[points_ids].unsqueeze(1) + query_points
        points[..., 2] = 100.0
        dirs = torch.zeros_like(points)
        dirs[..., 2] = -1.0

        ray_hits = raycast_mesh(points.view(-1, 3), dirs.view(-1, 3), wp_mesh)[0]
        heights = ray_hits.view(points.shape)[..., 2]
        flat_patches[points_ids, 2] = heights[..., -1]

        not_valid = torch.any(torch.logical_or(heights < z_range[0], heights > z_range[1]), dim=1)
        not_valid = torch.logical_or(not_valid, (heights.max(dim=1)[0] - heights.min(dim=1)[0]) > max_height_diff)

        points_ids = points_ids[not_valid]
        iter_count += 1

    if len(points_ids) > 0:
        raise RuntimeError(
            "Failed to find valid patches! Please check the input parameters."
            f"\n\tMaximum number of iterations reached: {iter_count}"
            f"\n\tNumber of invalid patches: {len(points_ids)}"
            f"\n\tMaximum height difference: {max_height_diff}"
        )

    return flat_patches - origin

