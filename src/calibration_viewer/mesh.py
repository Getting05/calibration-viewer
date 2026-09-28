"""Cached geometric skinning for meshes without rig/skin-weight metadata."""
from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation

from .calibration import UPPER


class MeshDeformer:
    """Blend rest-bone affine transforms; always deform the original vertices.

    Weights are geometric approximations, not SMPL/SOMA model skin weights.
    Bone mode preserves transverse thickness while stretching bone length.
    """

    def __init__(self, vertices, points, edges):
        self.vertices = np.asarray(vertices, dtype=float).copy()
        self.edges = tuple((a, b) for a, b in edges if a in points and b in points
                           and np.linalg.norm(points[b] - points[a]) > 1e-9)
        if not self.edges:
            raise ValueError("Mesh deformation requires at least one nonzero bone")
        self.starts = np.array([points[a] for a, _ in self.edges])
        self.bones = np.array([points[b] - points[a] for a, b in self.edges])
        self.lengths = np.linalg.norm(self.bones, axis=1)
        self.directions = self.bones / self.lengths[:, None]
        relative = self.vertices[:, None] - self.starts
        t = np.clip(np.einsum('vbi,bi->vb', relative, self.directions), 0, self.lengths)
        distances = np.linalg.norm(relative - t[..., None] * self.directions, axis=-1)
        count = min(4, len(self.edges))
        self.indices = np.argsort(distances, axis=1)[:, :count]
        nearest = np.take_along_axis(distances, self.indices, axis=1)
        # Strong locality avoids attaching a wrist to the torso or opposite leg.
        weights = 1 / np.maximum(nearest, 1e-5)**4
        self.weights = weights / weights.sum(axis=1, keepdims=True)
        self.relative = self.vertices[:, None] - self.starts[self.indices]

    def deform(self, target, params):
        matrices, starts = [], []
        for i, (a, b) in enumerate(self.edges):
            start = np.asarray(target[a])
            end = np.asarray(target[b])
            delta = end - start
            direction = self.directions[i]
            if params.mode == "legacy":
                scale = params.upper_scale if b in UPPER else params.lower_scale
                matrix = np.diag(scale)
                matrix += np.outer(delta - matrix @ self.bones[i], direction) / self.lengths[i]
            else:
                length = np.linalg.norm(delta)
                rotation = np.eye(3)
                if length > 1e-9:
                    unit = delta / length
                    cross = np.cross(direction, unit)
                    sine = np.linalg.norm(cross)
                    cosine = np.clip(direction @ unit, -1., 1.)
                    if sine > 1e-9:
                        rotation = Rotation.from_rotvec(cross/sine * np.arctan2(sine, cosine)).as_matrix()
                    elif cosine < 0:
                        axis = np.cross(direction, np.eye(3)[np.argmin(np.abs(direction))])
                        rotation = Rotation.from_rotvec(np.pi * axis / np.linalg.norm(axis)).as_matrix()
                matrix = rotation @ (np.eye(3) + (length/self.lengths[i]-1)*np.outer(direction, direction))
            matrices.append(matrix)
            starts.append(start)
        transformed = np.einsum('vkij,vkj->vki', np.asarray(matrices)[self.indices], self.relative)
        transformed += np.asarray(starts)[self.indices]
        return np.sum(transformed * self.weights[..., None], axis=1)
