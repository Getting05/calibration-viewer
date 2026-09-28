import numpy as np
import pytest

from calibration_viewer.mesh import MeshDeformer
from calibration_viewer.calibration import CalibrationParams


@pytest.mark.parametrize('mode', ['bone', 'legacy'])
def test_identity_translation_and_no_drift(mode):
    points = {'pelvis': np.zeros(3), 'spine1': np.array([0., 0., 1.]),
              'neck': np.array([0., 0., 2.])}
    vertices = np.array([[.2, .1, .5], [-.1, .2, 1.5]])
    deform = MeshDeformer(vertices, points, [('pelvis', 'spine1'), ('spine1', 'neck')])
    params = CalibrationParams.defaults(mode=mode)
    np.testing.assert_allclose(deform.deform(points, params), vertices)
    translated = {k: v + [1, 2, 3] for k, v in points.items()}
    np.testing.assert_allclose(deform.deform(translated, params), vertices + [1, 2, 3])
    np.testing.assert_allclose(deform.deform(points, params), vertices)


def test_length_stretch_rotation_and_thickness():
    points = {'pelvis': np.zeros(3), 'spine1': np.array([0., 0., 1.])}
    vertices = np.array([[0, 0, 0], [0, 0, 1], [.2, 0, .5]])
    deform = MeshDeformer(vertices, points, [('pelvis', 'spine1')])
    params = CalibrationParams.defaults(mode='bone')
    for end in ([0, 0, 2.], [0, 2., 0], [0, 0, -2.], [0, 0, 0]):
        result = deform.deform({'pelvis': np.zeros(3), 'spine1': np.array(end)}, params)
        np.testing.assert_allclose(result[:2], [[0, 0, 0], end], atol=1e-10)
        assert np.linalg.norm(result[2] - np.array(end)*.5) == pytest.approx(.2)


def test_legacy_scales_surface_in_all_axes():
    points = {'pelvis': np.zeros(3), 'spine1': np.array([0., 0., 1.])}
    vertices = np.array([[.2, .1, .5]])
    params = CalibrationParams.from_mapping({'mode': 'legacy', 'upper_scale': [2., 3., 4.]})
    deform = MeshDeformer(vertices, points, [('pelvis', 'spine1')])
    result = deform.deform({'pelvis': np.zeros(3), 'spine1': np.array([0, 0, 4.])}, params)
    np.testing.assert_allclose(result, vertices * [2, 3, 4])
