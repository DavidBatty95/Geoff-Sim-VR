"""Live synthetic WebSocket probe; run only with no headset session connected."""
import asyncio
import json
import time
import aiohttp
import msgpack
import numpy as np

async def probe():
    async with aiohttp.ClientSession() as client:
        async def health():
            async with client.get('http://127.0.0.1:8080/health') as response:
                response.raise_for_status()
                return await response.json()
        start = await health()
        hand = start['input_mode'] == 'hand'
        frame_messages = 0
        tracking_component = False
        async with client.ws_connect('https://127.0.0.1:8012/', ssl=False) as ws:
            async def receiver():
                nonlocal frame_messages, tracking_component
                async for message in ws:
                    if message.type == aiohttp.WSMsgType.BINARY:
                        payload = msgpack.unpackb(message.data, raw=False)
                        if 'ImageBackground' in repr(payload):
                            frame_messages += 1
                        if ('Hands' if hand else 'MotionControllers') in repr(payload):
                            tracking_component = True
            reader = asyncio.create_task(receiver())
            try:
                for i in range(150):
                    pose = np.eye(4)
                    pose[2, 3] = -0.08 * min(1, max(0, i-25)/50)
                    matrix = pose.flatten(order='F').tolist()
                    value = {'left': matrix * 25 if hand else matrix,
                             'right': matrix * 25 if hand else matrix,
                             'leftState': {'pinch': True} if hand else {'squeeze': True},
                             'rightState': {'pinch': True} if hand else {'squeeze': True}}
                    await ws.send_bytes(msgpack.packb({'etype': 'HAND_MOVE' if hand else 'CONTROLLER_MOVE',
                                                     'value': value}, use_bin_type=True))
                    await asyncio.sleep(0.02)
                following = await health()
                assert following['left'] == following['right'] == 'FOLLOW', following
                for side in ('left', 'right'):
                    assert following['wrist_positions'][side][0] - start['wrist_positions'][side][0] > 0.03
            finally:
                reader.cancel()
                await asyncio.gather(reader, return_exceptions=True)
        await asyncio.sleep(0.6)
        stopped = await health()
        assert stopped['left'] == stopped['right'] == 'HOLD', stopped
        assert frame_messages > 0, 'No stereo image messages received over the live WebSocket'
        assert tracking_component, 'Browser did not receive tracking component configuration'
        assert stopped['tracking_events'] - start['tracking_events'] >= 140
        assert stopped['video_frames'] > start['video_frames']
        result = {'source': 'synthetic Vuer messages over TLS; no physical headset',
                  'input_mode': start['input_mode'], 'image_messages': frame_messages,
                  'tracking_component_received': tracking_component,
                  'events_received': stopped['tracking_events'] - start['tracking_events'],
                  'left_displacement_m': following['wrist_positions']['left'][0] - start['wrist_positions']['left'][0],
                  'right_displacement_m': following['wrist_positions']['right'][0] - start['wrist_positions']['right'][0],
                  'disconnect_hold': True,
                  'sim_to_wall_ratio': (stopped['simulation_time']-start['simulation_time']) / (stopped['wall_seconds']-start['wall_seconds'])}
        print(json.dumps(result, indent=2))

if __name__ == '__main__':
    asyncio.run(probe())
