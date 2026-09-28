from pathlib import Path

import numpy as np
import pytest

from calibration_viewer.calibration import CalibrationParams, calibrate_human_points, fit_calibration, fit_calibration_frames
from calibration_viewer.cli import build_parser
from calibration_viewer.config import load_config
from calibration_viewer.motion import RootTrajectoryParams
from calibration_viewer.soma import (SOMA23_NAMES, SOMA77_NAMES, SOMA_TO_SEMANTIC,
                                    SOMA_NAMES, SOMA_EDGES, load_soma, load_soma_motion)


@pytest.fixture
def body_points():
    # Forward=X, left=Y, up=Z; native SOMA23 array order.
    rng = np.random.default_rng(103)
    points = {'pelvis':np.array([.2,.1,1.])}
    for parent, child in SOMA_EDGES:
        points[child] = points[parent] + rng.normal(size=3)*.2
    points['left_hip'] = points['pelvis'] + [0,.1,-.05]
    points['right_hip'] = points['pelvis'] + [0,-.1,-.05]
    return np.array([points[n] for n in SOMA_NAMES])


def test_native_indices_and_no_fake_smpl_joints(tmp_path):
    assert len(SOMA77_NAMES) == len(set(SOMA77_NAMES)) == 77
    assert SOMA77_NAMES[12] == 'LeftArm'
    assert SOMA77_NAMES[40] == 'RightArm'
    assert SOMA77_NAMES[67] == 'LeftLeg'
    raw = np.arange(77*3,dtype=float).reshape(77,3)
    path=tmp_path/'source.npz'
    np.savez(path,posed_joints=raw)
    human=load_soma(path,layout='soma77')
    assert human.label == 'SOMA23' and len(human.names)==23
    assert 'neck2' in human.names and 'left_hand' not in human.names
    np.testing.assert_equal(human.by_name['left_collar'],raw[11])
    np.testing.assert_equal(human.by_name['left_shoulder'],raw[12])
    np.testing.assert_equal(human.by_name['right_wrist'],raw[42])
    assert ('neck','neck2') in human.edges and ('neck2','head') in human.edges


def test_named_reordering_units_and_axes(tmp_path,body_points):
    path=tmp_path/'source.npz'
    np.savez(path,joints=body_points[::-1]*1000,joint_names=np.array(SOMA23_NAMES[::-1]))
    human=load_soma(path,units='mm').transformed(['-y','x','z'])
    np.testing.assert_allclose(human.points,body_points[:,[1,0,2]]*[-1,1,1])
    assert human.label=='SOMA23'


def test_soma_bone_propagation_and_fit(tmp_path,body_points):
    path=tmp_path/'source.npy';np.save(path,body_points)
    human=load_soma(path)
    params=CalibrationParams.defaults('bone')
    params.bone_scales={'torso':.8,'upper_arm':.7,'forearm':.9,'thigh':.8,'shank':1.1}
    params.joint_offsets={'hip':[.02,.03,-.01], 'shoulder':[.01,.02,.01]}
    target=calibrate_human_points(human.by_name,params,edges=human.edges)
    np.testing.assert_allclose(target['neck2']-target['neck'],.8*(human.by_name['neck2']-human.by_name['neck']))
    np.testing.assert_allclose(target['left_wrist']-target['left_elbow'],.9*(human.by_name['left_wrist']-human.by_name['left_elbow']))
    fit,info=fit_calibration(human.by_name,target,list(human.names),initial=CalibrationParams.defaults('bone'),
                             edges=human.edges,scale_regularization=1e-9,offset_regularization=1e-9)
    assert info['rmse_m']<1e-6
    np.testing.assert_allclose(fit.vector(),params.vector(),atol=1e-5)


def test_motion_selection_heading_and_multiframe_fit(tmp_path,body_points):
    path=tmp_path/'motion.npz'
    points=np.repeat(body_points[None],6,axis=0)
    points[:,:,0]+=np.arange(6)[:,None]*.1
    np.savez(path,posed_joints=points,fps=60,global_rot_mats=np.zeros((6,23,3,3)))
    motion=load_soma_motion(path,axes=['x','y','z'],start=1,stop=6,stride=2)
    assert motion.source_frames.tolist()==[1,3,5] and motion.fps==30
    assert motion.edges==SOMA_EDGES and motion.label=='SOMA23'
    np.testing.assert_allclose(motion.root_rotations,np.repeat(np.eye(3)[None],3,axis=0))
    frames=motion.frames()
    fit,info=fit_calibration_frames(frames,frames,list(motion.names),initial=CalibrationParams.defaults('bone'),edges=motion.edges)
    assert info['frame_count']==3 and info['rmse_m']<1e-9
    local,_=motion.calibrated(fit,RootTrajectoryParams())
    np.testing.assert_allclose(local,motion.local_points,atol=1e-9)


@pytest.mark.parametrize('count,layout',[(24,'auto'),(23,'soma77'),(77,'soma23')])
def test_reject_wrong_skeleton_counts(tmp_path,count,layout):
    path=tmp_path/'bad.npy';np.save(path,np.zeros((count,3)))
    with pytest.raises(ValueError):load_soma(path,layout=layout)


def test_missing_named_body_is_not_silently_invented(tmp_path,body_points):
    path=tmp_path/'bad.npz';np.savez(path,joints=body_points[:-1],joint_names=np.array(SOMA23_NAMES[:-1]))
    with pytest.raises(ValueError,match='Missing SOMA'):load_soma(path)


def test_fps_and_frame_validation(tmp_path,body_points):
    path=tmp_path/'motion.npy';np.save(path,body_points[None])
    with pytest.raises(ValueError,match='fps'):load_soma_motion(path,axes=['x','y','z'])
    with pytest.raises(ValueError,match='frame'):load_soma(path,frame=1)


def write_rest_xml(path,body_points):
    import xml.etree.ElementTree as E
    root=E.Element('mujoco');world=E.SubElement(root,'worldbody')
    E.SubElement(root,'compiler',coordinate='local')
    body=E.SubElement(world,'body',name='Hips',pos=' '.join(map(str,body_points[0])))
    nodes={'pelvis':body};points=dict(zip(SOMA_NAMES,body_points,strict=True))
    reverse={v:k for k,v in SOMA_TO_SEMANTIC.items()}
    for parent,child in SOMA_EDGES:
        nodes[child]=E.SubElement(nodes[parent],'body',name=reverse[child],pos=' '.join(map(str,points[child]-points[parent])))
        E.SubElement(nodes[child],'geom',type='sphere',size='.025')
    E.SubElement(body,'geom',type='capsule',size='.03',fromto='0 0 0 0 0 .1')
    E.ElementTree(root).write(path)


def test_soma_xml_native_rest_and_primitive_mesh(tmp_path,body_points):
    path=tmp_path/'soma23.xml';write_rest_xml(path,body_points)
    human=load_soma(path,layout='soma23')
    np.testing.assert_allclose(human.points,body_points)
    assert human.vertices.shape[1]==3 and len(human.faces)>0
    assert np.isfinite(human.vertices).all()


def test_xml_rejects_wrong_parent_tree(tmp_path,body_points):
    import xml.etree.ElementTree as E
    path=tmp_path/'soma23.xml';write_rest_xml(path,body_points)
    tree=E.parse(path)
    neck=tree.find('.//body[@name="Neck1"]');neck2=neck.find('body')
    neck.remove(neck2);tree.find('.//body[@name="Chest"]').append(neck2)
    tree.write(path)
    with pytest.raises(ValueError,match='parent tree'):load_soma(path)


def test_source_arguments_and_presets():
    args=build_parser().parse_args(['--soma','soma.npz','--config','c.yaml','--urdf','r.urdf'])
    assert args.soma==Path('soma.npz')
    with pytest.raises(SystemExit):
        build_parser().parse_args(['--smpl','a.pkl','--soma','b.npy','--config','c.yaml','--urdf','r.urdf'])
    for layout,axes in [('soma23',['-y','x','z']),('soma77',['z','x','y'])]:
        for robot in ('astro_p2','unitree_g1','unitree_h1_2'):
            cfg=load_config(Path(__file__).parents[1]/'configs'/f'{layout}_to_{robot}.yaml')
            assert cfg['human']['format']=='soma' and cfg['human']['axes']==axes
