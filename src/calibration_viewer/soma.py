"""SOMA77 / ProtoMotions SOMA23 adapters with native body topology.

The semantic names are shared with robot correspondence maps, not SMPL indices.
No synthetic SMPL joints are inserted. Fingers/face/leaf endpoints are excluded.
"""
from __future__ import annotations

from pathlib import Path
from typing import Mapping, Sequence
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation

from .human import HumanSkeleton, _array, _axis_vector, _read_pickle


SOMA23_NAMES = (
    "Hips", "Spine1", "Spine2", "Chest", "Neck1", "Neck2", "Head",
    "RightShoulder", "RightArm", "RightForeArm", "RightHand",
    "LeftShoulder", "LeftArm", "LeftForeArm", "LeftHand",
    "RightLeg", "RightShin", "RightFoot", "RightToeBase",
    "LeftLeg", "LeftShin", "LeftFoot", "LeftToeBase",
)
# Official SOMASkeleton77 array order used by the BONES/ProtoMotions adapters.
SOMA77_NAMES = (
    "Hips", "Spine1", "Spine2", "Chest", "Neck1", "Neck2", "Head", "HeadEnd",
    "Jaw", "LeftEye", "RightEye",
) + tuple(
    name for side in ("Left", "Right") for name in (
        f"{side}Shoulder", f"{side}Arm", f"{side}ForeArm", f"{side}Hand",
        *(f"{side}HandThumb{suffix}" for suffix in ("1", "2", "3", "End")),
        *(f"{side}Hand{finger}{suffix}" for finger in ("Index", "Middle", "Ring", "Pinky")
          for suffix in ("1", "2", "3", "4", "End")),
    )
) + tuple(f"{side}{joint}" for side in ("Left", "Right")
          for joint in ("Leg", "Shin", "Foot", "ToeBase", "ToeEnd"))

SOMA_TO_SEMANTIC = {
    "Hips": "pelvis", "Spine1": "spine1", "Spine2": "spine2", "Chest": "spine3",
    "Neck1": "neck", "Neck2": "neck2", "Head": "head",
    **{f"{side}{native}": f"{side.lower()}_{semantic}"
       for side in ("Left", "Right") for native, semantic in (
           ("Shoulder", "collar"), ("Arm", "shoulder"), ("ForeArm", "elbow"),
           ("Hand", "wrist"), ("Leg", "hip"), ("Shin", "knee"),
           ("Foot", "ankle"), ("ToeBase", "foot"))},
}
SOMA_NAMES = tuple(SOMA_TO_SEMANTIC[n] for n in SOMA23_NAMES)
SOMA_EDGES = (
    ("pelvis", "spine1"), ("spine1", "spine2"), ("spine2", "spine3"),
    ("spine3", "neck"), ("neck", "neck2"), ("neck2", "head"),
) + tuple(edge for side in ("right", "left") for edge in (
    ("spine3", f"{side}_collar"), (f"{side}_collar", f"{side}_shoulder"),
    (f"{side}_shoulder", f"{side}_elbow"), (f"{side}_elbow", f"{side}_wrist"),
    ("pelvis", f"{side}_hip"), (f"{side}_hip", f"{side}_knee"),
    (f"{side}_knee", f"{side}_ankle"), (f"{side}_ankle", f"{side}_foot"),
))
UNIT_SCALES = {"m": 1., "cm": .01, "mm": .001}


def _unit_scale(units: str) -> float:
    if units not in UNIT_SCALES:
        raise ValueError("SOMA units must be m, cm or mm")
    return UNIT_SCALES[units]


def _read_soma(path: Path) -> dict:
    suffix = path.suffix.lower()
    if suffix == ".npz":
        with np.load(path, allow_pickle=False) as data:
            # Do not load unrelated object metadata or huge body surfaces implicitly.
            keys = ("posed_joints", "joints", "rigid_body_pos", "positions", "joint_names",
                    "body_names", "fps", "mocap_framerate", "root_quat_xyzw", "vertices", "faces")
            return {k: data[k] for k in keys if k in data}
    if suffix == ".npy":
        return {"posed_joints": np.load(path, allow_pickle=False)}
    if suffix in (".motion", ".pt"):
        try:
            import torch
        except ImportError as exc:
            raise RuntimeError("SOMA .motion requires the soma-motion extra, or export_soma_motion.py in your ProtoMotions environment") from exc
        try:
            data = torch.load(path, map_location="cpu", weights_only=False)
        except ModuleNotFoundError as exc:
            raise RuntimeError("This .motion references ProtoMotions metadata. Run examples/export_soma_motion.py in its original environment, then load the exported NPZ.") from exc
    elif suffix in (".pkl", ".pickle", ".p"):
        data = _read_pickle(path)
    else:
        raise ValueError("SOMA input must be .npz, .npy, .pkl, .motion, or a SOMA23 .xml rest skeleton")
    if not isinstance(data, Mapping):
        raise ValueError("SOMA input must contain world joint positions; rotation-only files are not skeletons")
    def as_numpy(v):
        return v.detach().cpu().numpy() if hasattr(v, "detach") else v
    return {str(k): as_numpy(v) for k, v in data.items()}


def _select_body_points(data: Mapping, layout: str) -> np.ndarray:
    if layout not in ("auto", "soma23", "soma77"):
        raise ValueError("SOMA layout must be auto, soma23, or soma77")
    value = next((data[k] for k in ("posed_joints", "rigid_body_pos", "joints", "positions") if k in data), None)
    if value is None:
        raise ValueError("SOMA requires posed_joints/joints/rigid_body_pos world positions; rotations alone are insufficient")
    points = _array(value).astype(float)
    if points.ndim == 2:
        points = points[None]
    if points.ndim != 3 or points.shape[-1] != 3 or not len(points) or not np.isfinite(points).all():
        raise ValueError("SOMA joints must be nonempty finite [J,3] or [T,J,3]")
    supplied_names = data.get("joint_names", data.get("body_names"))
    if supplied_names is not None:
        names = tuple(n.decode() if isinstance(n, bytes) else str(n) for n in supplied_names)
        if len(names) != points.shape[1] or len(set(names)) != len(names):
            raise ValueError("SOMA joint_names must be unique and match the joint dimension")
        # Named native data is mapped by name, even when in a different order.
        missing = set(SOMA23_NAMES) - set(names)
        if missing:
            raise ValueError(f"Missing SOMA body names: {sorted(missing)}")
        if layout != "auto" and len(names) != (23 if layout == "soma23" else 77):
            raise ValueError("SOMA layout conflicts with joint_names dimension")
    else:
        expected = {23: SOMA23_NAMES, 77: SOMA77_NAMES}
        if points.shape[1] not in expected:
            raise ValueError("Unnamed SOMA arrays require 23 (MJCF order) or 77 joints")
        names = expected[points.shape[1]]
        if layout != "auto" and len(names) != (23 if layout == "soma23" else 77):
            raise ValueError("SOMA layout conflicts with array joint count")
    return points[:, [names.index(n) for n in SOMA23_NAMES]]


def _transform(element: ET.Element) -> np.ndarray:
    if any(key in element.attrib for key in ("euler", "axisangle", "xyaxes", "zaxis")):
        raise ValueError("SOMA rest XML supports pos/quat transforms; export other MJCF orientations to NPZ first")
    pos = np.fromstring(element.get("pos", "0 0 0"), sep=" ")
    quat = np.fromstring(element.get("quat", "1 0 0 0"), sep=" ")
    if pos.shape != (3,) or quat.shape != (4,) or not np.isfinite(pos).all() or not np.isfinite(quat).all() or np.linalg.norm(quat) < 1e-8:
        raise ValueError("Invalid SOMA XML pos/quat")
    transform = np.eye(4)
    transform[:3, :3] = Rotation.from_quat(quat[[1, 2, 3, 0]]).as_matrix()
    transform[:3, 3] = pos
    return transform


def load_soma_rest_xml(path: Path, *, units="m") -> HumanSkeleton:
    """Read a self-contained SOMA23 MJCF at rest; no simulator import required."""
    import trimesh
    root = ET.parse(path).getroot()
    compiler = root.find("compiler")
    if root.tag != "mujoco" or root.find(".//include") is not None or (compiler is not None and compiler.get("coordinate", "local") != "local"):
        raise ValueError("Expected self-contained local-coordinate SOMA23 MJCF")
    body = root.find("worldbody/body")
    if body is None or body.get("name") != "Hips":
        raise ValueError("SOMA23 MJCF must have Hips as its first root body")
    points, parents, meshes = {}, {}, []

    def walk(node, parent_name, parent_transform):
        name = node.get("name")
        if name in points:
            raise ValueError(f"Duplicate SOMA body {name}")
        transform = parent_transform @ _transform(node)
        points[name] = transform[:3, 3]
        parents[name] = parent_name
        for geom in node.findall("geom"):
            kind = geom.get("type", "sphere")
            size = np.fromstring(geom.get("size", ""), sep=" ")
            if not size.size or np.any(size <= 0) or not np.isfinite(size).all():
                raise ValueError("SOMA primitive geometry requires finite positive explicit size")
            local = _transform(geom)
            if kind in ("capsule", "cylinder"):
                if "fromto" in geom.attrib:
                    segment = np.fromstring(geom.attrib["fromto"], sep=" ")
                    if segment.shape != (6,) or not np.isfinite(segment).all():
                        raise ValueError("Invalid SOMA geom fromto")
                    start, end = segment[:3], segment[3:]
                    length = np.linalg.norm(end-start)
                    if length < 1e-9:
                        raise ValueError("Degenerate SOMA capsule/cylinder")
                    local = trimesh.geometry.align_vectors([0., 0., 1.], (end-start)/length)
                    local[:3, 3] = (start+end)/2
                elif size.size == 2:
                    length = 2*size[1]
                else:
                    raise ValueError("SOMA capsule/cylinder requires fromto or radius/half-length")
                mesh = (trimesh.creation.capsule(radius=size[0], height=length, count=[8, 8])
                        if kind == "capsule" else trimesh.creation.cylinder(radius=size[0], height=length, sections=12))
                mesh.apply_translation(-mesh.bounds.mean(axis=0))
            elif kind == "sphere" and size.size == 1:
                mesh = trimesh.creation.icosphere(subdivisions=1, radius=size[0])
            elif kind == "box" and size.size == 3:
                mesh = trimesh.creation.box(extents=2*size)
            else:
                raise ValueError(f"Unsupported SOMA geometry {kind}; export a joints NPZ instead")
            mesh.apply_transform(transform @ local)
            meshes.append(mesh)
        for child in node.findall("body"):
            walk(child, name, transform)

    walk(body, None, np.eye(4))
    if set(points) != set(SOMA23_NAMES):
        raise ValueError("SOMA rest XML must contain exactly the 23 supported bodies")
    semantic_parents = {SOMA_TO_SEMANTIC[n]: SOMA_TO_SEMANTIC[p] for n, p in parents.items() if p is not None}
    if semantic_parents != {child: parent for parent, child in SOMA_EDGES}:
        raise ValueError("MJCF parent tree does not match SOMA23")
    mesh = trimesh.util.concatenate(meshes) if meshes else None
    scale = _unit_scale(units)
    return HumanSkeleton(SOMA_NAMES, np.array([points[n] for n in SOMA23_NAMES])*scale,
                         edges=SOMA_EDGES, vertices=None if mesh is None else np.asarray(mesh.vertices)*scale,
                         faces=None if mesh is None else mesh.faces, label="SOMA23")


def load_soma(path: str | Path, *, frame=0, layout="auto", units="m") -> HumanSkeleton:
    path = Path(path)
    if path.suffix.lower() == ".xml":
        if frame != 0 or layout == "soma77":
            raise ValueError("SOMA23 rest XML has only frame 0 and cannot use soma77 layout")
        return load_soma_rest_xml(path, units=units)
    data = _read_soma(path)
    points = _select_body_points(data, layout)
    if not 0 <= frame < len(points):
        raise ValueError(f"SOMA frame must be within [0, {len(points)-1}]")
    scale = _unit_scale(units)
    vertices = faces = None
    if "vertices" in data and "faces" in data:
        vertices = np.asarray(data["vertices"], dtype=float)
        if vertices.ndim == 3:
            if len(vertices) != len(points):
                raise ValueError("SOMA vertex and joint frame counts differ")
            vertices = vertices[frame]
        vertices = vertices*scale
        faces = np.asarray(data["faces"])
    return HumanSkeleton(SOMA_NAMES, points[frame]*scale, edges=SOMA_EDGES,
                         vertices=vertices, faces=faces, label="SOMA23")


def load_soma_motion(path: str | Path, *, axes: Sequence[str], fps=None, layout="auto",
                     units="m", start=0, stop=None, stride=1):
    from .motion import HumanMotion
    if start < 0 or stride < 1 or (stop is not None and stop <= start):
        raise ValueError("Require 0 <= start < stop and stride >= 1")
    data = _read_soma(Path(path))
    all_points = _select_body_points(data, layout)
    indices = np.arange(len(all_points))[start:stop:stride]
    if not len(indices):
        raise ValueError("SOMA frame selection is empty")
    rate = fps if fps is not None else data.get("fps", data.get("mocap_framerate"))
    if rate is None:
        raise ValueError("SOMA motion fps missing; supply --fps or an fps field")
    points = all_points[indices]*_unit_scale(units)
    HumanSkeleton(SOMA_NAMES, points[0], edges=SOMA_EDGES).transformed(axes)
    basis = np.stack([_axis_vector(a) for a in axes])
    rotations = None
    # SOMA joint-frame global_rot_mats / rigid_body_rot are NOT a canonical
    # forward-left-up body frame. Never silently use them as root heading.
    if "root_quat_xyzw" in data:
        quat = np.asarray(data["root_quat_xyzw"], dtype=float)
        if quat.shape != (len(all_points), 4) or not np.isfinite(quat).all() or np.any(np.linalg.norm(quat, axis=1) < 1e-8):
            raise ValueError("Canonical root_quat_xyzw must be finite nonzero [T,4]")
        rotations = basis @ Rotation.from_quat(quat[indices]).as_matrix() @ basis.T
    return HumanMotion(SOMA_NAMES, points @ basis.T, float(rate)/stride, rotations, indices,
                       edges=SOMA_EDGES, label="SOMA23")
