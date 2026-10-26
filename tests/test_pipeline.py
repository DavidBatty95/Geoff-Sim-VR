import asyncio
from types import SimpleNamespace
import numpy as np
import pytest
from dougie_vr.control import Clutch, XR_TO_ROBOT, valid_pose
from dougie_vr.simulation import Simulation

def test_axes():
    np.testing.assert_allclose(XR_TO_ROBOT @ [0, 0, -1], [1, 0, 0])
    np.testing.assert_allclose(XR_TO_ROBOT @ [1, 0, 0], [0, -1, 0])
    np.testing.assert_allclose(XR_TO_ROBOT @ [0, 1, 0], [0, 0, 1])

@pytest.mark.parametrize('kind', ['zero', 'nan', 'reflection', 'nonrigid'])
def test_invalid_pose(kind):
    p = np.eye(4)
    if kind == 'zero': p[:] = 0
    if kind == 'nan': p[0, 3] = np.nan
    if kind == 'reflection': p[0, 0] = -1
    if kind == 'nonrigid': p[0, 0] = 2
    assert not valid_pose(p)
    assert Clutch().update(p, 1, True, 1, np.eye(4)) is None

def test_clutch_timeout_recalibration_and_workspace():
    c, p, robot = Clutch(), np.eye(4), np.eye(4)
    robot[2, 3] = 1
    np.testing.assert_allclose(c.update(p, 1, True, 1, robot), robot)
    p[2, 3] = -10
    target = c.update(p, 1.1, True, 1.1, robot)
    assert np.linalg.norm(target[:3, 3] - robot[:3, 3]) == pytest.approx(0.35)
    assert c.update(p, 1.1, True, 2, robot) is None
    np.testing.assert_allclose(c.update(p, 2, True, 2, robot), robot)
    assert c.update(p, 2, False, 2, robot) is None
    assert c.update(p, 3, True, 2, robot) is None

def test_wrist_rotation():
    c, p = Clutch(), np.eye(4)
    c.update(p, 0, True, 0, p)
    q = p.copy()
    q[:3, :3] = [[0, 0, 1], [0, 1, 0], [-1, 0, 0]]
    out = c.update(q, 0.1, True, 0.1, p)
    np.testing.assert_allclose(out[:3, :3], XR_TO_ROBOT @ q[:3, :3] @ XR_TO_ROBOT.T)

def test_real_physics_reaches_and_holds_both_arms():
    sim = Simulation()
    try:
        targets = {s: sim.pose(s).copy() for s in sim.arms}
        for target in targets.values():
            target[:3, 3] += [0.06, 0, 0.06]
        start = sim.data.qpos.copy()
        for _ in range(100): sim.step(targets)
        assert np.linalg.norm(sim.data.qpos - start) > 0.1
        for side in sim.arms:
            assert np.linalg.norm(sim.pose(side)[:3, 3] - targets[side][:3, 3]) < 0.025
        for _ in range(50): sim.step({s: None for s in sim.arms})
        held = sim.data.qpos.copy()
        for _ in range(100): sim.step({s: None for s in sim.arms})
        assert np.max(np.abs(sim.data.qpos - held)) < 0.015
        assert np.all(sim.data.qpos >= sim.model.jnt_range[:, 0] - 0.01)
        assert np.all(sim.data.qpos <= sim.model.jnt_range[:, 1] + 0.01)
        image = sim.render()
        assert image.shape == (360, 960, 3) and image.std() > 10
    finally:
        sim.close()

def test_televuer_controller_events_reach_physics():
    # Exercise the actual vendored event handler without spawning a browser server.
    from multiprocessing import Array, Value
    from televuer import TeleVuer
    tv = TeleVuer.__new__(TeleVuer)
    tv.controller_move_count_shared = Value('i', 0)
    for side in ('left', 'right'):
        setattr(tv, f'{side}_arm_pose_shared', Array('d', 16))
        setattr(tv, f'{side}_arm_update_time_shared', Value('d', 0.))
        for key in ('trigger', 'squeeze', 'thumbstick', 'aButton', 'bButton'):
            setattr(tv, f'{side}_ctrl_{key}_shared', Value('b', False))
        for key in ('triggerValue', 'squeezeValue'):
            setattr(tv, f'{side}_ctrl_{key}_shared', Value('d', 0.))
        setattr(tv, f'{side}_ctrl_thumbstickValue_shared', Array('d', 2))
    p = np.eye(4)
    event = SimpleNamespace(value={'left': p.flatten(order='F').tolist(),
                                  'right': p.flatten(order='F').tolist(),
                                  'leftState': {'squeeze': True}, 'rightState': {'squeeze': True}})
    asyncio.run(tv.on_controller_move(event, None))
    assert tv.left_ctrl_squeeze and tv.right_ctrl_squeeze
    sim, c = Simulation(), Clutch()
    try:
        initial = sim.pose('left')
        c.update(tv.left_arm_pose, tv.left_arm_update_time, tv.left_ctrl_squeeze,
                 tv.left_arm_update_time, initial)
        p[2, 3] = -0.08
        event.value['left'] = p.flatten(order='F').tolist()
        asyncio.run(tv.on_controller_move(event, None))
        target = c.update(tv.left_arm_pose, tv.left_arm_update_time, tv.left_ctrl_squeeze,
                          tv.left_arm_update_time, sim.pose('left'))
        for _ in range(100): sim.step({'left': target, 'right': None})
        assert sim.pose('left')[0, 3] > initial[0, 3] + 0.03
        event.value.pop('left')
        asyncio.run(tv.on_controller_move(event, None))
        assert c.update(tv.left_arm_pose, tv.left_arm_update_time, tv.left_ctrl_squeeze,
                        1, sim.pose('left')) is None
    finally:
        sim.close()
