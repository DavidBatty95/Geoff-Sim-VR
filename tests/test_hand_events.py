import asyncio
from multiprocessing import Array, Value
from types import SimpleNamespace
import numpy as np
from televuer import TeleVuer
from dougie_vr.control import Clutch

def test_hand_wrist_pinch_and_tracking_loss():
    tv = TeleVuer.__new__(TeleVuer)
    tv.hand_move_count_shared = Value('i', 0)
    tv._last_hand_diag_time = 0
    tv.process_hand_landmarks = True
    tv.process_hand_controls = True
    for side in ('left', 'right'):
        for key, size in (('arm_pose', 16), ('hand_position', 75), ('hand_orientation', 225)):
            setattr(tv, f'{side}_{key}_shared', Array('d', size))
        setattr(tv, f'{side}_arm_update_time_shared', Value('d', 0.))
        for name in ('pinch', 'squeeze'):
            setattr(tv, f'{side}_hand_{name}_shared', Value('b', False))
            setattr(tv, f'{side}_hand_{name}Value_shared', Value('d', 0.))
    p = np.eye(4)
    value = {'left': p.flatten(order='F').tolist() * 25,
             'right': p.flatten(order='F').tolist() * 25,
             'leftState': {'pinch': True}, 'rightState': {'pinch': False}}
    asyncio.run(tv.on_hand_move(SimpleNamespace(value=value), None))
    assert tv.left_hand_pinch and not tv.right_hand_pinch
    np.testing.assert_allclose(tv.left_arm_pose, p)
    c = Clutch()
    assert c.update(tv.left_arm_pose, tv.left_arm_update_time, tv.left_hand_pinch,
                    tv.left_arm_update_time, p) is not None
    value['left'] = []
    asyncio.run(tv.on_hand_move(SimpleNamespace(value=value), None))
    assert c.update(tv.left_arm_pose, tv.left_arm_update_time, tv.left_hand_pinch, 1, p) is None
