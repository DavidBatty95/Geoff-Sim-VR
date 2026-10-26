from vuer import Vuer
from vuer.schemas import Box, Capsule, ImageBackground, Hands, Line, MotionControllers, Sphere, WebRTCVideoPlane, WebRTCStereoVideoPlane
from multiprocessing import Value, Array, Process, shared_memory
import numpy as np
import asyncio
import threading
import cv2
import os
import signal
import time
from pathlib import Path
from typing import Literal


class TeleVuer:
    def __init__(self, use_hand_tracking: bool, binocular: bool=True, img_shape: tuple=None, display_fps: float=30.0,
                       display_mode: Literal["immersive", "pass-through", "ego"]="immersive", zmq: bool=False, webrtc: bool=False, webrtc_url: str=None, 
                       cert_file: str=None, key_file: str=None, arm_overlay: bool=False,
                       image_quality: int=55, image_interpolate: bool=False, arm_overlay_fps: float=12.0,
                       camera_plane_height: float=1.35, camera_plane_distance: float=0.85,
                       camera_plane_fixed: bool=True, camera_plane_position: tuple=(0.0, 1.2, -0.85),
                       queue_len: int=1, process_hand_landmarks: bool=True, process_hand_controls: bool=True):
        """
        TeleVuer class for OpenXR-based XR teleoperate applications.
        This class handles the communication with the Vuer server and manages image and pose data.

        :param use_hand_tracking: bool, whether to use hand tracking or controller tracking.
        :param binocular: bool, whether the application is binocular (stereoscopic) or monocular.
        :param img_shape: tuple, shape of the head image (height, width).
        :param display_fps: float, target frames per second for display updates (default: 30.0).
        
        :param display_mode: str, controls the VR viewing mode. Options are "immersive", "pass-through", and "ego".
        :param zmq: bool, whether to use zmq for image transmission.
        :param webrtc: bool, whether to use webrtc for real-time communication.
        :param webrtc_url: str, URL for the webrtc offer. must be provided if webrtc is True.
        :param cert_file: str, path to the SSL certificate file.
        :param key_file: str, path to the SSL key file.

        Note:

        - display_mode controls what the VR headset displays:
            * "immersive": fully immersive mode; VR shows the robot's first-person view (zmq or webrtc must be enabled).
            * "pass-through": VR shows the real world through the VR headset cameras; no image from zmq or webrtc is displayed (even if enabled).
            * "ego": a small window in the center shows the robot's first-person view, while the surrounding area shows the real world.
        
        - Only one image mode is active at a time.
        - Image transmission to VR occurs only if display_mode is "immersive" or "ego" and the corresponding zmq or webrtc option is enabled.
        - If zmq and webrtc simultaneously enabled, webrtc will be prioritized.

        --------------              -------------------           --------------       -----------------                     -------
         display_mode       |        display behavior         |    image to VR     |      image source        |               Notes
        --------------              -------------------           --------------       -----------------                     ------- 
           immersive        |   fully immersive view (robot)  |     Yes (full)     |     zmq or webrtc        |   if both enabled, webrtc prioritized
        --------------              -------------------           --------------       -----------------                     -------
         pass-through       |       Real world view (VR)      |         No         |          N/A             |  even if image source enabled, don't display
        --------------              -------------------           --------------       -----------------                     -------
              ego           |      ego view (robot + VR)      |    Yes (small)     |     zmq or webrtc        |   if both enabled, webrtc prioritized
        --------------              -------------------           --------------       -----------------                     -------

        """
        self.use_hand_tracking = use_hand_tracking
        self.binocular = binocular
        if img_shape is None:
            raise ValueError("[TeleVuer] img_shape must be provided.")
        self.img_shape = (img_shape[0], img_shape[1], 3)
        self.display_fps = display_fps
        self.img_height = self.img_shape[0]
        if self.binocular:
            self.img_width  = self.img_shape[1] // 2
        else:
            self.img_width  = self.img_shape[1]
        self.aspect_ratio = self.img_width / self.img_height

        # SSL certificate path resolution
        env_cert = os.getenv("XR_TELEOP_CERT")
        env_key = os.getenv("XR_TELEOP_KEY")
        if cert_file is None or key_file is None:
            # 1.Try environment variables
            if env_cert and env_key:
                cert_file = cert_file or env_cert
                key_file = key_file or env_key
            else:
                # 2.Try ~/.config/xr_teleoperate/
                user_conf_dir = Path.home() / ".config" / "xr_teleoperate"
                cert_path_user = user_conf_dir / "cert.pem"
                key_path_user = user_conf_dir / "key.pem"

                if cert_path_user.exists() and key_path_user.exists():
                    cert_file = cert_file or str(cert_path_user)
                    key_file = key_file or str(key_path_user)
                else:
                    # 3.Fallback to package root (current logic)
                    current_module_dir = Path(__file__).resolve().parent.parent.parent
                    cert_file = cert_file or str(current_module_dir / "cert.pem")
                    key_file = key_file or str(current_module_dir / "key.pem")

        self.vuer = Vuer(host='0.0.0.0', cert=cert_file, key=key_file, queries=dict(grid=False), queue_len=max(int(queue_len), 1))
        self.vuer.add_handler("CAMERA_MOVE")(self.on_cam_move)
        if self.use_hand_tracking:
            self.vuer.add_handler("HAND_MOVE")(self.on_hand_move)
        else:
            self.vuer.add_handler("CONTROLLER_MOVE")(self.on_controller_move)

        self.display_mode = display_mode
        self.zmq = zmq
        self.webrtc = webrtc
        self.webrtc_url = webrtc_url
        self.arm_overlay = arm_overlay
        self.process_hand_landmarks = process_hand_landmarks
        self.process_hand_controls = process_hand_controls
        self.image_quality = int(min(max(image_quality, 20), 95))
        self.image_interpolate = image_interpolate
        self.arm_overlay_period = 1.0 / max(float(arm_overlay_fps), 1.0)
        self._last_arm_overlay_time = 0.0
        self._last_camera_diag_time = 0.0
        self._last_hand_diag_time = 0.0
        self.camera_plane_height = float(camera_plane_height)
        self.camera_plane_distance = float(camera_plane_distance)
        self.camera_plane_fixed = camera_plane_fixed
        self.camera_plane_position = tuple(float(v) for v in camera_plane_position)

        if self.display_mode == "immersive":
            if self.webrtc:
                fn = self.main_image_binocular_webrtc if self.binocular else self.main_image_monocular_webrtc
            elif self.zmq:
                self.img2display_shm = shared_memory.SharedMemory(create=True, size=np.prod(self.img_shape) * np.uint8().itemsize)
                self.img2display = np.ndarray(self.img_shape, dtype=np.uint8, buffer=self.img2display_shm.buf)
                self.latest_frame = None
                self.new_frame_event = threading.Event()
                self.stop_writer_event = threading.Event()
                self.writer_thread = threading.Thread(target=self._xr_render_loop, daemon=True)
                self.writer_thread.start()
                fn = self.main_image_binocular_zmq if self.binocular else self.main_image_monocular_zmq
            else:
                raise ValueError("[TeleVuer] immersive mode requires zmq=True or webrtc=True.")
        elif self.display_mode == "ego":
            if self.webrtc:
                fn = self.main_image_binocular_webrtc_ego if self.binocular else self.main_image_monocular_webrtc_ego
            elif self.zmq:
                self.img2display_shm = shared_memory.SharedMemory(create=True, size=np.prod(self.img_shape) * np.uint8().itemsize)
                self.img2display = np.ndarray(self.img_shape, dtype=np.uint8, buffer=self.img2display_shm.buf)
                self.latest_frame = None
                self.new_frame_event = threading.Event()
                self.stop_writer_event = threading.Event()
                self.writer_thread = threading.Thread(target=self._xr_render_loop, daemon=True)
                self.writer_thread.start()
                fn = self.main_image_binocular_zmq_ego if self.binocular else self.main_image_monocular_zmq_ego
            else:
                raise ValueError("[TeleVuer] ego mode requires zmq=True or webrtc=True.")
        elif self.display_mode == "pass-through":
            fn = self.main_pass_through
        else:
            raise ValueError(f"[TeleVuer] Unknown display_mode: {self.display_mode}")
        
        self.vuer.spawn(start=False)(fn)

        self.head_pose_shared = Array('d', 16, lock=True)
        self.left_arm_pose_shared = Array('d', 16, lock=True)
        self.right_arm_pose_shared = Array('d', 16, lock=True)
        self.left_overlay_points_shared = Array('d', 9, lock=True)
        self.right_overlay_points_shared = Array('d', 9, lock=True)
        self.left_overlay_hand_points_shared = Array('d', 75, lock=True)
        self.right_overlay_hand_points_shared = Array('d', 75, lock=True)
        self.left_overlay_valid_shared = Value('b', False, lock=True)
        self.right_overlay_valid_shared = Value('b', False, lock=True)
        self.left_overlay_hand_valid_shared = Value('b', False, lock=True)
        self.right_overlay_hand_valid_shared = Value('b', False, lock=True)
        self.left_arm_update_time_shared = Value('d', 0.0, lock=True)
        self.right_arm_update_time_shared = Value('d', 0.0, lock=True)
        self.head_update_time_shared = Value('d', 0.0, lock=True)
        self.camera_move_count_shared = Value('i', 0, lock=True)
        self.hand_move_count_shared = Value('i', 0, lock=True)
        self.controller_move_count_shared = Value('i', 0, lock=True)
        if self.use_hand_tracking:
            self.left_hand_position_shared = Array('d', 75, lock=True)
            self.right_hand_position_shared = Array('d', 75, lock=True)
            self.left_hand_orientation_shared = Array('d', 25 * 9, lock=True)
            self.right_hand_orientation_shared = Array('d', 25 * 9, lock=True)

            self.left_hand_pinch_shared = Value('b', False, lock=True)
            self.left_hand_pinchValue_shared = Value('d', 0.0, lock=True)
            self.left_hand_squeeze_shared = Value('b', False, lock=True)
            self.left_hand_squeezeValue_shared = Value('d', 0.0, lock=True)

            self.right_hand_pinch_shared = Value('b', False, lock=True)
            self.right_hand_pinchValue_shared = Value('d', 0.0, lock=True)
            self.right_hand_squeeze_shared = Value('b', False, lock=True)
            self.right_hand_squeezeValue_shared = Value('d', 0.0, lock=True)
        else:
            self.left_ctrl_trigger_shared = Value('b', False, lock=True)
            self.left_ctrl_triggerValue_shared = Value('d', 0.0, lock=True)
            self.left_ctrl_squeeze_shared = Value('b', False, lock=True)
            self.left_ctrl_squeezeValue_shared = Value('d', 0.0, lock=True)
            self.left_ctrl_thumbstick_shared = Value('b', False, lock=True)
            self.left_ctrl_thumbstickValue_shared = Array('d', 2, lock=True)
            self.left_ctrl_aButton_shared = Value('b', False, lock=True)
            self.left_ctrl_bButton_shared = Value('b', False, lock=True)

            self.right_ctrl_trigger_shared = Value('b', False, lock=True)
            self.right_ctrl_triggerValue_shared = Value('d', 0.0, lock=True)
            self.right_ctrl_squeeze_shared = Value('b', False, lock=True)
            self.right_ctrl_squeezeValue_shared = Value('d', 0.0, lock=True)
            self.right_ctrl_thumbstick_shared = Value('b', False, lock=True)
            self.right_ctrl_thumbstickValue_shared = Array('d', 2, lock=True)
            self.right_ctrl_aButton_shared = Value('b', False, lock=True)
            self.right_ctrl_bButton_shared = Value('b', False, lock=True)

        self.process = Process(target=self._vuer_run)
        self.process.daemon = True
        self.process.start()
        tracking_event = "HAND_MOVE" if self.use_hand_tracking else "CONTROLLER_MOVE"
        print(
            "[TeleVuer] Started: "
            f"tracking={'hands' if self.use_hand_tracking else 'controllers'} "
            f"event={tracking_event} mode={self.display_mode} binocular={self.binocular} "
            f"zmq={self.zmq} webrtc={self.webrtc} fps={self.display_fps} "
            f"queue_len={queue_len} landmarks={self.process_hand_landmarks} controls={self.process_hand_controls}",
            flush=True,
        )
    
    def _vuer_run(self):
        try:
            self._exit_with_parent()
            self.vuer.run()
        except KeyboardInterrupt:
            pass
        except Exception as e:
            print(f"Vuer encountered an error: {e}")
        finally:
            if hasattr(self, "stop_writer_event"):
                self.stop_writer_event.set()

    def _exit_with_parent(self):
        if os.name != "posix":
            return
        try:
            import ctypes
            libc = ctypes.CDLL("libc.so.6")
            libc.prctl(1, signal.SIGTERM)
        except Exception as e:
            print(f"[TeleVuer] Warning: failed to set parent-death signal: {e}")

    def _xr_render_loop(self):
        while not self.stop_writer_event.is_set():
            if not self.new_frame_event.wait(timeout=0.1):
                continue
            self.new_frame_event.clear()
            if self.latest_frame is None:
                continue
            latest_frame = self.latest_frame
            latest_frame = cv2.cvtColor(latest_frame, cv2.COLOR_BGR2RGB)
            self.img2display[:] = latest_frame
    
    def render_to_xr(self, image):
        if self.webrtc or self.display_mode == "pass-through":
            print("[TeleVuer] Warning: render_to_xr is ignored when webrtc is enabled or pass_through is True.")
            return
        self.latest_frame = image
        self.new_frame_event.set()

    def render_arm_overlay(self, left_points=None, right_points=None, left_hand_points=None, right_hand_points=None):
        if not self.arm_overlay:
            return

        self._set_arm_overlay('left', left_points)
        self._set_arm_overlay('right', right_points)
        self._set_hand_overlay('left', left_hand_points)
        self._set_hand_overlay('right', right_hand_points)

    def _set_arm_overlay(self, side, points):
        valid_shared = self.left_overlay_valid_shared if side == 'left' else self.right_overlay_valid_shared
        points_shared = self.left_overlay_points_shared if side == 'left' else self.right_overlay_points_shared
        if points is None:
            with valid_shared.get_lock():
                valid_shared.value = False
            return

        flat_points = np.asarray(points, dtype=float).reshape(3, 3).reshape(-1)
        if not np.all(np.isfinite(flat_points)):
            with valid_shared.get_lock():
                valid_shared.value = False
            return

        with points_shared.get_lock():
            points_shared[:] = flat_points.tolist()
        with valid_shared.get_lock():
            valid_shared.value = True

    def _set_hand_overlay(self, side, points):
        valid_shared = self.left_overlay_hand_valid_shared if side == 'left' else self.right_overlay_hand_valid_shared
        points_shared = self.left_overlay_hand_points_shared if side == 'left' else self.right_overlay_hand_points_shared
        if points is None:
            with valid_shared.get_lock():
                valid_shared.value = False
            return

        flat_points = np.asarray(points, dtype=float).reshape(25, 3).reshape(-1)
        if not np.all(np.isfinite(flat_points)) or np.linalg.norm(flat_points.reshape(25, 3), axis=1).max() < 1e-6:
            with valid_shared.get_lock():
                valid_shared.value = False
            return

        with points_shared.get_lock():
            points_shared[:] = flat_points.tolist()
        with valid_shared.get_lock():
            valid_shared.value = True

    def _arm_overlay_elements(self, side, points, hand_points=None):
        color = '#00d5ff' if side == 'left' else '#ff7a00'
        joint_color = '#d7dde2'
        if hand_points is None:
            _, elbow, wrist = points.tolist()
            return self._hand_elements(side, elbow, wrist, color, joint_color)
        return self._tracked_hand_elements(side, hand_points, color, joint_color)

    def _tracked_hand_elements(self, side, hand_points, color, joint_color):
        finger_chains = (
            (0, 1, 2, 3, 4),
            (0, 5, 6, 7, 8, 9),
            (0, 10, 11, 12, 13, 14),
            (0, 15, 16, 17, 18, 19),
            (0, 20, 21, 22, 23, 24),
        )
        palm_connections = ((0, 5), (5, 10), (10, 15), (15, 20), (20, 0), (5, 15), (10, 20))
        mechanical_color = '#202a33'
        pad_color = '#d7dde2'
        elements = []

        palm_center = np.mean(hand_points[[0, 5, 10, 15, 20]], axis=0)
        palm_forward = np.mean(hand_points[[5, 10, 15, 20]], axis=0) - hand_points[0]
        palm_forward_norm = float(np.linalg.norm(palm_forward))
        if palm_forward_norm < 1e-6:
            palm_forward = np.array([0.0, 1.0, 0.0])
        else:
            palm_forward = palm_forward / palm_forward_norm

        palm_side = hand_points[20] - hand_points[5]
        palm_side_norm = float(np.linalg.norm(palm_side))
        if palm_side_norm < 1e-6:
            palm_side = np.array([1.0, 0.0, 0.0])
        else:
            palm_side = palm_side / palm_side_norm
        palm_normal = np.cross(palm_forward, palm_side)
        palm_normal_norm = float(np.linalg.norm(palm_normal))
        if palm_normal_norm < 1e-6:
            palm_normal = np.array([0.0, 0.0, 1.0])
        else:
            palm_normal = palm_normal / palm_normal_norm
        palm_side = np.cross(palm_normal, palm_forward)
        palm_side_norm = float(np.linalg.norm(palm_side))
        if palm_side_norm < 1e-6:
            palm_side = np.array([1.0, 0.0, 0.0])
        else:
            palm_side = palm_side / palm_side_norm

        palm_width = max(float(np.linalg.norm(hand_points[20] - hand_points[5])) * 0.9, 0.055)
        palm_length = max(float(np.linalg.norm(np.mean(hand_points[[5, 10, 15, 20]], axis=0) - hand_points[0])) * 1.15, 0.075)
        elements.append(Box(
            args=[palm_width, 0.018, palm_length],
            position=palm_center.tolist(),
            quaternion=self._quat_from_axes(palm_side, palm_normal, palm_forward),
            color=mechanical_color,
            materialType='basic',
            material={'color': mechanical_color},
            key=f'{side}-tracked-hand-palm-plate',
        ))

        for idx, (start, end) in enumerate(palm_connections):
            elements.append(self._capsule_between(
                hand_points[start],
                hand_points[end],
                0.006,
                color=mechanical_color,
                key=f'{side}-tracked-hand-palm-{idx}',
            ))
        bone_idx = 0
        for chain in finger_chains:
            for start, end in zip(chain[:-1], chain[1:]):
                elements.append(self._capsule_between(
                    hand_points[start],
                    hand_points[end],
                    0.007 if start == 0 else 0.005,
                    color=color,
                    key=f'{side}-tracked-hand-bone-{bone_idx}',
                ))
                bone_idx += 1
        fingertip_segments = ((3, 4), (8, 9), (13, 14), (18, 19), (23, 24))
        for start, idx in fingertip_segments:
            tip_axis = hand_points[idx] - hand_points[start]
            tip_axis_norm = float(np.linalg.norm(tip_axis))
            if tip_axis_norm < 1e-6:
                tip_axis = palm_forward
            else:
                tip_axis = tip_axis / tip_axis_norm
            tip_normal = palm_normal - tip_axis * float(np.dot(palm_normal, tip_axis))
            tip_normal_norm = float(np.linalg.norm(tip_normal))
            if tip_normal_norm < 1e-6:
                tip_normal = palm_normal
            else:
                tip_normal = tip_normal / tip_normal_norm
            tip_side = np.cross(tip_axis, tip_normal)
            tip_side_norm = float(np.linalg.norm(tip_side))
            if tip_side_norm < 1e-6:
                tip_side = palm_side
            else:
                tip_side = tip_side / tip_side_norm
            elements.append(Box(
                args=[0.022, 0.014, 0.014],
                position=hand_points[idx].tolist(),
                quaternion=self._quat_from_axes(tip_axis, tip_normal, tip_side),
                color=pad_color,
                materialType='basic',
                material={'color': pad_color},
                key=f'{side}-tracked-hand-pad-{idx}',
            ))
        return elements

    def _capsule_between(self, start, end, radius, color, key):
        start = np.asarray(start, dtype=float)
        end = np.asarray(end, dtype=float)
        delta = end - start
        length = float(np.linalg.norm(delta))
        if length < 1e-6:
            length = 1e-6
            direction = np.array([0.0, 1.0, 0.0])
        else:
            direction = delta / length
        return Capsule(
            args=[radius, max(length - 2.0 * radius, 0.001), 4, 10],
            position=((start + end) * 0.5).tolist(),
            quaternion=self._quat_from_y_axis(direction),
            color=color,
            materialType='basic',
            material={'color': color},
            key=key,
        )

    def _quat_from_y_axis(self, direction):
        source = np.array([0.0, 1.0, 0.0])
        target = np.asarray(direction, dtype=float)
        target_norm = float(np.linalg.norm(target))
        if target_norm < 1e-6:
            return [0.0, 0.0, 0.0, 1.0]
        target = target / target_norm
        dot = float(np.dot(source, target))
        if dot < -0.999999:
            return [1.0, 0.0, 0.0, 0.0]
        cross = np.cross(source, target)
        quat = np.array([cross[0], cross[1], cross[2], 1.0 + dot], dtype=float)
        quat_norm = float(np.linalg.norm(quat))
        if quat_norm < 1e-6:
            return [0.0, 0.0, 0.0, 1.0]
        return (quat / quat_norm).tolist()

    def _quat_from_axes(self, x_axis, y_axis, z_axis):
        rot = np.column_stack((x_axis, y_axis, z_axis))
        trace = float(np.trace(rot))
        if trace > 0.0:
            s = np.sqrt(trace + 1.0) * 2.0
            qw = 0.25 * s
            qx = (rot[2, 1] - rot[1, 2]) / s
            qy = (rot[0, 2] - rot[2, 0]) / s
            qz = (rot[1, 0] - rot[0, 1]) / s
        elif rot[0, 0] > rot[1, 1] and rot[0, 0] > rot[2, 2]:
            s = np.sqrt(1.0 + rot[0, 0] - rot[1, 1] - rot[2, 2]) * 2.0
            qw = (rot[2, 1] - rot[1, 2]) / s
            qx = 0.25 * s
            qy = (rot[0, 1] + rot[1, 0]) / s
            qz = (rot[0, 2] + rot[2, 0]) / s
        elif rot[1, 1] > rot[2, 2]:
            s = np.sqrt(1.0 + rot[1, 1] - rot[0, 0] - rot[2, 2]) * 2.0
            qw = (rot[0, 2] - rot[2, 0]) / s
            qx = (rot[0, 1] + rot[1, 0]) / s
            qy = 0.25 * s
            qz = (rot[1, 2] + rot[2, 1]) / s
        else:
            s = np.sqrt(1.0 + rot[2, 2] - rot[0, 0] - rot[1, 1]) * 2.0
            qw = (rot[1, 0] - rot[0, 1]) / s
            qx = (rot[0, 2] + rot[2, 0]) / s
            qy = (rot[1, 2] + rot[2, 1]) / s
            qz = 0.25 * s
        quat = np.array([qx, qy, qz, qw], dtype=float)
        quat_norm = float(np.linalg.norm(quat))
        if quat_norm < 1e-6 or not np.isfinite(quat_norm):
            return [0.0, 0.0, 0.0, 1.0]
        return (quat / quat_norm).tolist()

    def _hand_elements(self, side, elbow, wrist, color, joint_color):
        elbow = np.array(elbow, dtype=float)
        wrist = np.array(wrist, dtype=float)
        forward = wrist - elbow
        forward_norm = float(np.linalg.norm(forward))
        if forward_norm < 1e-6:
            forward = np.array([0.0, 0.0, -1.0])
        else:
            forward = forward / forward_norm

        up = np.array([0.0, 1.0, 0.0])
        side_axis = np.cross(forward, up)
        side_norm = float(np.linalg.norm(side_axis))
        if side_norm < 1e-6:
            side_axis = np.array([1.0, 0.0, 0.0])
        else:
            side_axis = side_axis / side_norm

        palm = wrist + forward * 0.065
        elements = [
            Sphere(
                args=[0.095, 24, 24],
                position=palm.tolist(),
                color=joint_color,
                materialType='basic',
                material={'color': joint_color},
                key=f'{side}-robot-hand-palm',
            )
        ]

        finger_offsets = (-0.06, -0.03, 0.0, 0.03)
        for idx, offset in enumerate(finger_offsets):
            base = palm + side_axis * offset + up * 0.018
            tip = base + forward * 0.105
            elements.append(Line(
                points=[base.tolist(), tip.tolist()],
                color=color,
                lineWidth=16,
                materialType='basic',
                material={'color': color},
                key=f'{side}-robot-hand-finger-{idx}',
            ))
            elements.append(Sphere(
                args=[0.025, 12, 12],
                position=tip.tolist(),
                color=color,
                materialType='basic',
                material={'color': color},
                key=f'{side}-robot-hand-fingertip-{idx}',
            ))

        thumb_side = -1.0 if side == 'left' else 1.0
        thumb_base = palm + side_axis * thumb_side * 0.065 - up * 0.01
        thumb_tip = thumb_base + forward * 0.055 + side_axis * thumb_side * 0.075
        elements.append(Line(
            points=[thumb_base.tolist(), thumb_tip.tolist()],
            color=color,
            lineWidth=18,
            materialType='basic',
            material={'color': color},
            key=f'{side}-robot-hand-thumb',
        ))
        elements.append(Sphere(
            args=[0.03, 12, 12],
            position=thumb_tip.tolist(),
            color=color,
            materialType='basic',
            material={'color': color},
            key=f'{side}-robot-hand-thumbtip',
        ))
        return elements

    def _arm_segment_spheres(self, side, segment, start, end, color, radius):
        return [
            Sphere(
                args=[radius, 16, 16],
                position=(np.array(start) * (1.0 - t) + np.array(end) * t).tolist(),
                color=color,
                materialType='basic',
                material={'color': color},
                key=f'{side}-robot-{segment}-volume-{idx}',
            )
            for idx, t in enumerate((0.25, 0.5, 0.75))
        ]

    def _update_arm_overlay(self, session):
        if not self.arm_overlay:
            return
        now = time.monotonic()
        if now - self._last_arm_overlay_time < self.arm_overlay_period:
            return
        self._last_arm_overlay_time = now

        elements = []
        remove_keys = []
        for side, points_shared, valid_shared, hand_points_shared, hand_valid_shared in (
            ('left', self.left_overlay_points_shared, self.left_overlay_valid_shared, self.left_overlay_hand_points_shared, self.left_overlay_hand_valid_shared),
            ('right', self.right_overlay_points_shared, self.right_overlay_valid_shared, self.right_overlay_hand_points_shared, self.right_overlay_hand_valid_shared),
        ):
            valid = valid_shared.value
            if valid:
                with points_shared.get_lock():
                    points = np.array(points_shared[:], dtype=float).reshape(3, 3)
                hand_points = None
                if hand_valid_shared.value:
                    with hand_points_shared.get_lock():
                        hand_points = np.array(hand_points_shared[:], dtype=float).reshape(25, 3)
                elements.extend(self._arm_overlay_elements(side, points, hand_points))
                if hand_points is None:
                    remove_keys.append(f'{side}-tracked-hand-palm-plate')
                    for idx in range(24):
                        remove_keys.append(f'{side}-tracked-hand-bone-{idx}')
                    for idx in range(7):
                        remove_keys.append(f'{side}-tracked-hand-palm-{idx}')
                    for idx in (4, 9, 14, 19, 24):
                        remove_keys.append(f'{side}-tracked-hand-pad-{idx}')
                    for idx in range(25):
                        remove_keys.append(f'{side}-tracked-hand-joint-{idx}')
                else:
                    remove_keys.extend([
                        f'{side}-robot-hand-palm',
                        f'{side}-robot-hand-thumb',
                        f'{side}-robot-hand-thumbtip',
                    ])
                    for idx in range(4):
                        remove_keys.extend([
                            f'{side}-robot-hand-finger-{idx}',
                            f'{side}-robot-hand-fingertip-{idx}',
                        ])
                    for idx in range(25):
                        remove_keys.append(f'{side}-tracked-hand-joint-{idx}')
            else:
                remove_keys.extend([
                    f'{side}-robot-upper-arm-line',
                    f'{side}-robot-forearm-line',
                    f'{side}-robot-arm-shoulder',
                    f'{side}-robot-arm-elbow',
                    f'{side}-robot-arm-wrist',
                    f'{side}-robot-upper-volume-0',
                    f'{side}-robot-upper-volume-1',
                    f'{side}-robot-upper-volume-2',
                    f'{side}-robot-forearm-volume-0',
                    f'{side}-robot-forearm-volume-1',
                    f'{side}-robot-forearm-volume-2',
                    f'{side}-robot-hand-palm',
                    f'{side}-robot-hand-thumb',
                    f'{side}-robot-hand-thumbtip',
                ])
                for idx in range(4):
                    remove_keys.extend([
                        f'{side}-robot-hand-finger-{idx}',
                        f'{side}-robot-hand-fingertip-{idx}',
                    ])
                remove_keys.append(f'{side}-tracked-hand-palm-plate')
                for idx in range(24):
                    remove_keys.append(f'{side}-tracked-hand-bone-{idx}')
                for idx in range(7):
                    remove_keys.append(f'{side}-tracked-hand-palm-{idx}')
                for idx in (4, 9, 14, 19, 24):
                    remove_keys.append(f'{side}-tracked-hand-pad-{idx}')
                for idx in range(25):
                    remove_keys.append(f'{side}-tracked-hand-joint-{idx}')

        if elements:
            session.upsert(elements, to='bgChildren')
        if remove_keys:
            session.remove(remove_keys)

    def close(self):
        if self.process.is_alive():
            self.process.terminate()
            try:
                self.process.join(timeout=0.5)
            except KeyboardInterrupt:
                pass
            if self.process.is_alive():
                self.process.kill()
                self.process.join(timeout=0.5)
        if self.display_mode in ("immersive", "ego") and not self.webrtc:
            self.stop_writer_event.set()
            self.new_frame_event.set()
            self.writer_thread.join(timeout=0.5)
            try:
                self.img2display_shm.close()
                self.img2display_shm.unlink()
            except:
                pass

    def _safe_upsert(self, session, element, to="bgChildren"):
        try:
            session.upsert(element, to=to)
            return True
        except AssertionError as exc:
            if "Websocket session is missing" in str(exc):
                return False
            raise

    def _tracking_component(self):
        if self.use_hand_tracking:
            return Hands(
                stream=True,
                key="hands",
                hideLeft=False,
                hideRight=False,
            )
        return MotionControllers(
            stream=True,
            key="motionControllers",
            left=True,
            right=True,
            hideLeft=True,
            hideRight=True,
        )

    async def on_cam_move(self, event, session, fps=60):
        try:
            with self.head_pose_shared.get_lock():
                self.head_pose_shared[:] = event.value["camera"]["matrix"]
            with self.head_update_time_shared.get_lock():
                self.head_update_time_shared.value = time.time()
            with self.camera_move_count_shared.get_lock():
                self.camera_move_count_shared.value += 1
                camera_count = self.camera_move_count_shared.value
            now = time.time()
            if now - self._last_camera_diag_time > 5.0:
                self._last_camera_diag_time = now
                matrix = event.value.get("camera", {}).get("matrix", []) if isinstance(event.value, dict) else []
                xyz = matrix[12:15] if isinstance(matrix, (list, tuple)) and len(matrix) >= 15 else []
                print(f"[TeleVuer] CAMERA_MOVE count={camera_count} xyz={xyz}", flush=True)
        except Exception as e:
            print(f"[TeleVuer] CAMERA_MOVE handler error: {e}; value={getattr(event, 'value', None)}")

    async def on_controller_move(self, event, session, fps=60):
        """https://docs.vuer.ai/en/latest/examples/20_motion_controllers.html"""
        try:
            with self.controller_move_count_shared.get_lock():
                self.controller_move_count_shared.value += 1
            # ControllerData
            left_pose = event.value.get("left")
            right_pose = event.value.get("right")
            left_valid = isinstance(left_pose, (list, tuple)) and len(left_pose) == 16
            right_valid = isinstance(right_pose, (list, tuple)) and len(right_pose) == 16

            if left_valid:
                with self.left_arm_pose_shared.get_lock():
                    self.left_arm_pose_shared[:] = left_pose
                with self.left_arm_update_time_shared.get_lock():
                    self.left_arm_update_time_shared.value = time.time()
            else:
                with self.left_arm_pose_shared.get_lock():
                    self.left_arm_pose_shared[:] = [0.0] * 16
                with self.left_arm_update_time_shared.get_lock():
                    self.left_arm_update_time_shared.value = 0.0
            if right_valid:
                with self.right_arm_pose_shared.get_lock():
                    self.right_arm_pose_shared[:] = right_pose
                with self.right_arm_update_time_shared.get_lock():
                    self.right_arm_update_time_shared.value = time.time()
            else:
                with self.right_arm_pose_shared.get_lock():
                    self.right_arm_pose_shared[:] = [0.0] * 16
                with self.right_arm_update_time_shared.get_lock():
                    self.right_arm_update_time_shared.value = 0.0

            # ControllerState
            left_controller = event.value.get("leftState", {})
            right_controller = event.value.get("rightState", {})

            def extract_controllers(controllerState, prefix):
                # trigger
                with getattr(self, f"{prefix}_ctrl_trigger_shared").get_lock():
                    getattr(self, f"{prefix}_ctrl_trigger_shared").value = bool(controllerState.get("trigger", False))
                with getattr(self, f"{prefix}_ctrl_triggerValue_shared").get_lock():
                    getattr(self, f"{prefix}_ctrl_triggerValue_shared").value = float(controllerState.get("triggerValue", 0.0))
                # squeeze
                with getattr(self, f"{prefix}_ctrl_squeeze_shared").get_lock():
                    getattr(self, f"{prefix}_ctrl_squeeze_shared").value = bool(controllerState.get("squeeze", False))
                with getattr(self, f"{prefix}_ctrl_squeezeValue_shared").get_lock():
                    getattr(self, f"{prefix}_ctrl_squeezeValue_shared").value = float(controllerState.get("squeezeValue", 0.0))
                # thumbstick
                with getattr(self, f"{prefix}_ctrl_thumbstick_shared").get_lock():
                    getattr(self, f"{prefix}_ctrl_thumbstick_shared").value = bool(controllerState.get("thumbstick", False))
                with getattr(self, f"{prefix}_ctrl_thumbstickValue_shared").get_lock():
                    getattr(self, f"{prefix}_ctrl_thumbstickValue_shared")[:] = controllerState.get("thumbstickValue", [0.0, 0.0])
                # buttons
                with getattr(self, f"{prefix}_ctrl_aButton_shared").get_lock():
                    getattr(self, f"{prefix}_ctrl_aButton_shared").value = bool(controllerState.get("aButton", False))
                with getattr(self, f"{prefix}_ctrl_bButton_shared").get_lock():
                    getattr(self, f"{prefix}_ctrl_bButton_shared").value = bool(controllerState.get("bButton", False))

            if left_valid:
                extract_controllers(left_controller, "left")
            if right_valid:
                extract_controllers(right_controller, "right")
        except Exception as e:
            print(f"[TeleVuer] CONTROLLER_MOVE handler error: {e}; value={getattr(event, 'value', None)}")

    async def on_hand_move(self, event, session, fps=60):
        """https://docs.vuer.ai/en/latest/examples/19_hand_tracking.html"""
        try:
            with self.hand_move_count_shared.get_lock():
                self.hand_move_count_shared.value += 1
                hand_count = self.hand_move_count_shared.value
            # HandsData
            left_hand_data = event.value.get("left")
            right_hand_data = event.value.get("right")
            left_hand = event.value.get("leftState", {})
            right_hand = event.value.get("rightState", {})
            now = time.time()
            if now - self._last_hand_diag_time > 2.0:
                self._last_hand_diag_time = now
                left_len = len(left_hand_data) if isinstance(left_hand_data, (list, tuple)) else 0
                right_len = len(right_hand_data) if isinstance(right_hand_data, (list, tuple)) else 0
                print(
                    "[TeleVuer] HAND_MOVE payload: "
                    f"count={hand_count} "
                    f"keys={list(event.value.keys()) if isinstance(event.value, dict) else type(event.value).__name__} "
                    f"left_len={left_len} right_len={right_len} "
                    f"left_state={left_hand} right_state={right_hand}",
                    flush=True,
                )
            # HandState
            def extract_hand_poses(hand_data, arm_pose_shared, hand_position_shared, hand_orientation_shared):
                if not isinstance(hand_data, (list, tuple)) or len(hand_data) < 25 * 16:
                    with arm_pose_shared.get_lock():
                        arm_pose_shared[:] = [0.0] * 16
                    if self.process_hand_landmarks:
                        with hand_position_shared.get_lock():
                            hand_position_shared[:] = [0.0] * 75
                        with hand_orientation_shared.get_lock():
                            hand_orientation_shared[:] = [0.0] * (25 * 9)
                    return False

                with arm_pose_shared.get_lock():
                    arm_pose_shared[:] = hand_data[0:16]

                if not self.process_hand_landmarks:
                    return True

                with hand_position_shared.get_lock():
                    for i in range(25):
                        base = i * 16
                        hand_position_shared[i * 3: i * 3 + 3] = [hand_data[base + 12], hand_data[base + 13], hand_data[base + 14]]

                with hand_orientation_shared.get_lock():
                    for i in range(25):
                        base = i * 16
                        hand_orientation_shared[i * 9: i * 9 + 9] = [
                            hand_data[base + 0], hand_data[base + 1], hand_data[base + 2],
                            hand_data[base + 4], hand_data[base + 5], hand_data[base + 6],
                            hand_data[base + 8], hand_data[base + 9], hand_data[base + 10],
                        ]
                return True

            def extract_hands(handState, prefix):
                # pinch
                with getattr(self, f"{prefix}_hand_pinch_shared").get_lock():
                    getattr(self, f"{prefix}_hand_pinch_shared").value = bool(handState.get("pinch", False))
                with getattr(self, f"{prefix}_hand_pinchValue_shared").get_lock():
                    getattr(self, f"{prefix}_hand_pinchValue_shared").value = float(handState.get("pinchValue", 0.0))
                # squeeze
                with getattr(self, f"{prefix}_hand_squeeze_shared").get_lock():
                    getattr(self, f"{prefix}_hand_squeeze_shared").value = bool(handState.get("squeeze", False))
                with getattr(self, f"{prefix}_hand_squeezeValue_shared").get_lock():
                    getattr(self, f"{prefix}_hand_squeezeValue_shared").value = float(handState.get("squeezeValue", 0.0))

            left_valid = extract_hand_poses(left_hand_data, self.left_arm_pose_shared, self.left_hand_position_shared, self.left_hand_orientation_shared)
            right_valid = extract_hand_poses(right_hand_data, self.right_arm_pose_shared, self.right_hand_position_shared, self.right_hand_orientation_shared)
            with self.left_arm_update_time_shared.get_lock():
                self.left_arm_update_time_shared.value = time.time() if left_valid else 0.0
            with self.right_arm_update_time_shared.get_lock():
                self.right_arm_update_time_shared.value = time.time() if right_valid else 0.0
            if self.process_hand_controls and left_valid:
                extract_hands(left_hand, "left")
            if self.process_hand_controls and right_valid:
                extract_hands(right_hand, "right")

        except Exception as e:
            print(f"[TeleVuer] HAND_MOVE handler error: {e}; value={getattr(event, 'value', None)}")
    
    ## immersive MODE
    async def main_image_binocular_zmq(self, session):
        tracking_ready = False
        while True:
            if not tracking_ready:
                tracking_ready = self._safe_upsert(session, self._tracking_component())
                # Let Vuer drain its one-element queue before the first image.
                # Otherwise the image replaces the Hands/MotionControllers setup.
                if tracking_ready:
                    await asyncio.sleep(0.1)
            self._safe_upsert(
                session,
                [
                    ImageBackground(
                        self.img2display[:, :self.img_width],
                        aspect=self.aspect_ratio,
                        height=self.camera_plane_height,
                        distanceToCamera=self.camera_plane_distance,
                        fixed=self.camera_plane_fixed,
                        position=self.camera_plane_position,
                        # The underlying rendering engine supported a layer binary bitmask for both objects and the camera. 
                        # Below we set the two image planes, left and right, to layers=1 and layers=2. 
                        # Note that these two masks are associated with left eye’s camera and the right eye’s camera.
                        layers=1,
                        format="jpeg",
                        quality=self.image_quality,
                        key="background-left",
                        interpolate=self.image_interpolate,
                    ),
                    ImageBackground(
                        self.img2display[:, self.img_width:],
                        aspect=self.aspect_ratio,
                        height=self.camera_plane_height,
                        distanceToCamera=self.camera_plane_distance,
                        fixed=self.camera_plane_fixed,
                        position=self.camera_plane_position,
                        layers=2,
                        format="jpeg",
                        quality=self.image_quality,
                        key="background-right",
                        interpolate=self.image_interpolate,
                    ),
                ],
            )
            self._update_arm_overlay(session)
            # 'jpeg' encoding should give you about 30fps with a 16ms wait in-between.
            await asyncio.sleep(1.0 / self.display_fps)

    async def main_image_monocular_zmq(self, session):
        if self.use_hand_tracking:
            session.upsert(
                Hands(
                    stream=True,
                    key="hands",
                    hideLeft=False,
                    hideRight=False
                ),
                to="bgChildren",
            )
        else:
            session.upsert(
                MotionControllers(
                    stream=True, 
                    key="motionControllers",
                    left=True,
                    right=True,
                ),
                to="bgChildren",
            )

        while True:
            self._update_arm_overlay(session)
            session.upsert(
                [
                    ImageBackground(
                        self.img2display,
                        aspect=self.aspect_ratio,
                        height=self.camera_plane_height,
                        distanceToCamera=self.camera_plane_distance,
                        fixed=self.camera_plane_fixed,
                        position=self.camera_plane_position,
                        format="jpeg",
                        quality=self.image_quality,
                        key="background-mono",
                        interpolate=self.image_interpolate,
                    ),
                ],
                to="bgChildren",
            )
            await asyncio.sleep(1.0 / self.display_fps)

    async def main_image_binocular_webrtc(self, session):
        if self.use_hand_tracking:
            session.upsert(
                Hands(
                    stream=True,
                    key="hands",
                    hideLeft=False,
                    hideRight=False
                ),
                to="bgChildren",
            )
        else:
            session.upsert(
                MotionControllers(
                    stream=True, 
                    key="motionControllers",
                    left=True,
                    right=True,
                ),
                to="bgChildren",
            )

        while True:
            self._update_arm_overlay(session)
            session.upsert(
                WebRTCStereoVideoPlane(
                    src=self.webrtc_url,
                    iceServer=None,
                    iceServers=[], 
                    key="video-quad",
                    aspect=self.aspect_ratio,
                    height = 7,
                    layout="stereo-left-right"
                ),
                to="bgChildren",
            )
            await asyncio.sleep(1.0 / self.display_fps)

    async def main_image_monocular_webrtc(self, session):
        if self.use_hand_tracking:
            session.upsert(
                Hands(
                    stream=True,
                    key="hands",
                    hideLeft=False,
                    hideRight=False
                ),
                to="bgChildren",
            )
        else:
            session.upsert(
                MotionControllers(
                    stream=True, 
                    key="motionControllers",
                    left=True,
                    right=True,
                ),
                to="bgChildren",
            )

        while True:
            self._update_arm_overlay(session)
            session.upsert(
                WebRTCVideoPlane(
                    src=self.webrtc_url,
                    iceServer=None,
                    iceServers=[],
                    key="video-quad",
                    aspect=self.aspect_ratio,
                    height = 7,
                ),
                to="bgChildren",
            )
            await asyncio.sleep(1.0 / self.display_fps)

    ## ego MODE
    async def main_image_binocular_zmq_ego(self, session):
        if self.use_hand_tracking:
            session.upsert(
                Hands(
                    stream=True,
                    key="hands",
                    hideLeft=False,
                    hideRight=False
                ),
                to="bgChildren",
            )
        else:
            session.upsert(
                MotionControllers(
                    stream=True,
                    key="motionControllers",
                    left=True,
                    right=True,
                ),
                to="bgChildren",
            )
        # Preserve initial tracking configuration before streaming image updates.
        await asyncio.sleep(0.1)
        while True:
            self._update_arm_overlay(session)
            session.upsert(
                [
                    ImageBackground(
                        self.img2display[:, :self.img_width],
                        aspect=self.aspect_ratio,
                        height=0.75,
                        distanceToCamera=2,
                        # The underlying rendering engine supported a layer binary bitmask for both objects and the camera. 
                        # Below we set the two image planes, left and right, to layers=1 and layers=2. 
                        # Note that these two masks are associated with left eye’s camera and the right eye’s camera.
                        layers=1,
                        format="jpeg",
                        quality=self.image_quality,
                        key="background-left",
                        interpolate=self.image_interpolate,
                    ),
                    ImageBackground(
                        self.img2display[:, self.img_width:],
                        aspect=self.aspect_ratio,
                        height=0.75,
                        distanceToCamera=2,
                        layers=2,
                        format="jpeg",
                        quality=self.image_quality,
                        key="background-right",
                        interpolate=self.image_interpolate,
                    ),
                ],
                to="bgChildren",
            )
            # 'jpeg' encoding should give you about 30fps with a 16ms wait in-between.
            await asyncio.sleep(1.0 / self.display_fps)

    async def main_image_monocular_zmq_ego(self, session):
        if self.use_hand_tracking:
            session.upsert(
                Hands(
                    stream=True,
                    key="hands",
                    hideLeft=False,
                    hideRight=False
                ),
                to="bgChildren",
            )
        else:
            session.upsert(
                MotionControllers(
                    stream=True, 
                    key="motionControllers",
                    left=True,
                    right=True,
                ),
                to="bgChildren",
            )

        while True:
            self._update_arm_overlay(session)
            session.upsert(
                [
                    ImageBackground(
                        self.img2display,
                        aspect=self.aspect_ratio,
                        height=0.75,
                        distanceToCamera=2,
                        format="jpeg",
                        quality=self.image_quality,
                        key="background-mono",
                        interpolate=self.image_interpolate,
                    ),
                ],
                to="bgChildren",
            )
            await asyncio.sleep(1.0 / self.display_fps)

    async def main_image_binocular_webrtc_ego(self, session):
        if self.use_hand_tracking:
            session.upsert(
                Hands(
                    stream=True,
                    key="hands",
                    hideLeft=False,
                    hideRight=False
                ),
                to="bgChildren",
            )
        else:
            session.upsert(
                MotionControllers(
                    stream=True, 
                    key="motionControllers",
                    left=True,
                    right=True,
                ),
                to="bgChildren",
            )

        while True:
            self._update_arm_overlay(session)
            session.upsert(
                WebRTCStereoVideoPlane(
                    src=self.webrtc_url,
                    iceServer=None,
                    iceServers=[], 
                    key="video-quad",
                    aspect=self.aspect_ratio,
                    height=3,
                    layout="stereo-left-right"
                ),
                to="bgChildren",
            )
            await asyncio.sleep(1.0 / self.display_fps)

    async def main_image_monocular_webrtc_ego(self, session):
        if self.use_hand_tracking:
            session.upsert(
                Hands(
                    stream=True,
                    key="hands",
                    hideLeft=False,
                    hideRight=False
                ),
                to="bgChildren",
            )
        else:
            session.upsert(
                MotionControllers(
                    stream=True, 
                    key="motionControllers",
                    left=True,
                    right=True,
                ),
                to="bgChildren",
            )

        while True:
            self._update_arm_overlay(session)
            session.upsert(
                WebRTCVideoPlane(
                    src=self.webrtc_url,
                    iceServer=None,
                    iceServers=[],
                    key="video-quad",
                    aspect=self.aspect_ratio,
                    height=3,
                ),
                to="bgChildren",
            )
            await asyncio.sleep(1.0 / self.display_fps)

    ## pass-through MODE
    async def main_pass_through(self, session):
        if self.use_hand_tracking:
            session.upsert(
                Hands(
                    stream=True,
                    key="hands",
                    hideLeft=False,
                    hideRight=False
                ),
                to="bgChildren",
            )
        else:
            session.upsert(
                MotionControllers(
                    stream=True, 
                    key="motionControllers",
                    left=True,
                    right=True,
                ),
                to="bgChildren",
            )

        while True:
            await asyncio.sleep(1.0 / self.display_fps)

    # ==================== common data ====================
    @property
    def head_pose(self):
        """np.ndarray, shape (4, 4), head SE(3) pose matrix from Vuer (basis OpenXR Convention)."""
        with self.head_pose_shared.get_lock():
            return np.array(self.head_pose_shared[:]).reshape(4, 4, order="F")

    @property
    def left_arm_pose(self):
        """np.ndarray, shape (4, 4), left arm SE(3) pose matrix from Vuer (basis OpenXR Convention)."""
        with self.left_arm_pose_shared.get_lock():
            return np.array(self.left_arm_pose_shared[:]).reshape(4, 4, order="F")

    @property
    def right_arm_pose(self):
        """np.ndarray, shape (4, 4), right arm SE(3) pose matrix from Vuer (basis OpenXR Convention)."""
        with self.right_arm_pose_shared.get_lock():
            return np.array(self.right_arm_pose_shared[:]).reshape(4, 4, order="F")

    @property
    def left_arm_update_time(self):
        with self.left_arm_update_time_shared.get_lock():
            return self.left_arm_update_time_shared.value

    @property
    def right_arm_update_time(self):
        with self.right_arm_update_time_shared.get_lock():
            return self.right_arm_update_time_shared.value

    @property
    def head_update_time(self):
        with self.head_update_time_shared.get_lock():
            return self.head_update_time_shared.value

    @property
    def camera_move_count(self):
        with self.camera_move_count_shared.get_lock():
            return self.camera_move_count_shared.value

    @property
    def hand_move_count(self):
        with self.hand_move_count_shared.get_lock():
            return self.hand_move_count_shared.value

    @property
    def controller_move_count(self):
        with self.controller_move_count_shared.get_lock():
            return self.controller_move_count_shared.value

    # ==================== Hand Tracking Data ====================
    @property
    def left_hand_positions(self):
        """np.ndarray, shape (25, 3), left hand 25 landmarks' 3D positions."""
        with self.left_hand_position_shared.get_lock():
            return np.array(self.left_hand_position_shared[:]).reshape(25, 3)

    @property
    def right_hand_positions(self):
        """np.ndarray, shape (25, 3), right hand 25 landmarks' 3D positions."""
        with self.right_hand_position_shared.get_lock():
            return np.array(self.right_hand_position_shared[:]).reshape(25, 3)

    @property
    def left_hand_orientations(self):
        """np.ndarray, shape (25, 3, 3), left hand 25 landmarks' orientations (flattened 3x3 matrices, column-major)."""
        with self.left_hand_orientation_shared.get_lock():
            return np.array(self.left_hand_orientation_shared[:]).reshape(25, 9).reshape(25, 3, 3, order="F")

    @property
    def right_hand_orientations(self):
        """np.ndarray, shape (25, 3, 3), right hand 25 landmarks' orientations (flattened 3x3 matrices, column-major)."""
        with self.right_hand_orientation_shared.get_lock():
            return np.array(self.right_hand_orientation_shared[:]).reshape(25, 9).reshape(25, 3, 3, order="F")

    @property
    def left_hand_pinch(self):
        """bool, whether left hand is pinching."""
        with self.left_hand_pinch_shared.get_lock():
            return self.left_hand_pinch_shared.value

    @property
    def left_hand_pinchValue(self):
        """float, pinch strength of left hand."""
        with self.left_hand_pinchValue_shared.get_lock():
            return self.left_hand_pinchValue_shared.value

    @property
    def left_hand_squeeze(self):
        """bool, whether left hand is squeezing."""
        with self.left_hand_squeeze_shared.get_lock():
            return self.left_hand_squeeze_shared.value

    @property
    def left_hand_squeezeValue(self):
        """float, squeeze strength of left hand."""
        with self.left_hand_squeezeValue_shared.get_lock():
            return self.left_hand_squeezeValue_shared.value

    @property
    def right_hand_pinch(self):
        """bool, whether right hand is pinching."""
        with self.right_hand_pinch_shared.get_lock():
            return self.right_hand_pinch_shared.value

    @property
    def right_hand_pinchValue(self):
        """float, pinch strength of right hand."""
        with self.right_hand_pinchValue_shared.get_lock():
            return self.right_hand_pinchValue_shared.value

    @property
    def right_hand_squeeze(self):
        """bool, whether right hand is squeezing."""
        with self.right_hand_squeeze_shared.get_lock():
            return self.right_hand_squeeze_shared.value

    @property
    def right_hand_squeezeValue(self):
        """float, squeeze strength of right hand."""
        with self.right_hand_squeezeValue_shared.get_lock():
            return self.right_hand_squeezeValue_shared.value

    # ==================== Controller Data ====================
    @property
    def left_ctrl_trigger(self):
        """bool, left controller trigger pressed or not."""
        with self.left_ctrl_trigger_shared.get_lock():
            return self.left_ctrl_trigger_shared.value

    @property
    def left_ctrl_triggerValue(self):
        """float, left controller trigger analog value (0.0 ~ 1.0)."""
        with self.left_ctrl_triggerValue_shared.get_lock():
            return self.left_ctrl_triggerValue_shared.value

    @property
    def left_ctrl_squeeze(self):
        """bool, left controller squeeze pressed or not."""
        with self.left_ctrl_squeeze_shared.get_lock():
            return self.left_ctrl_squeeze_shared.value

    @property
    def left_ctrl_squeezeValue(self):
        """float, left controller squeeze analog value (0.0 ~ 1.0)."""
        with self.left_ctrl_squeezeValue_shared.get_lock():
            return self.left_ctrl_squeezeValue_shared.value

    @property
    def left_ctrl_thumbstick(self):
        """bool, whether left thumbstick is touched or clicked."""
        with self.left_ctrl_thumbstick_shared.get_lock():
            return self.left_ctrl_thumbstick_shared.value

    @property
    def left_ctrl_thumbstickValue(self):
        """np.ndarray, shape (2,), left thumbstick 2D axis values (x, y)."""
        with self.left_ctrl_thumbstickValue_shared.get_lock():
            return np.array(self.left_ctrl_thumbstickValue_shared[:])

    @property
    def left_ctrl_aButton(self):
        """bool, left controller 'A' button pressed."""
        with self.left_ctrl_aButton_shared.get_lock():
            return self.left_ctrl_aButton_shared.value

    @property
    def left_ctrl_bButton(self):
        """bool, left controller 'B' button pressed."""
        with self.left_ctrl_bButton_shared.get_lock():
            return self.left_ctrl_bButton_shared.value

    @property
    def right_ctrl_trigger(self):
        """bool, right controller trigger pressed or not."""
        with self.right_ctrl_trigger_shared.get_lock():
            return self.right_ctrl_trigger_shared.value

    @property
    def right_ctrl_triggerValue(self):
        """float, right controller trigger analog value (0.0 ~ 1.0)."""
        with self.right_ctrl_triggerValue_shared.get_lock():
            return self.right_ctrl_triggerValue_shared.value

    @property
    def right_ctrl_squeeze(self):
        """bool, right controller squeeze pressed or not."""
        with self.right_ctrl_squeeze_shared.get_lock():
            return self.right_ctrl_squeeze_shared.value

    @property
    def right_ctrl_squeezeValue(self):
        """float, right controller squeeze analog value (0.0 ~ 1.0)."""
        with self.right_ctrl_squeezeValue_shared.get_lock():
            return self.right_ctrl_squeezeValue_shared.value

    @property
    def right_ctrl_thumbstick(self):
        """bool, whether right thumbstick is touched or clicked."""
        with self.right_ctrl_thumbstick_shared.get_lock():
            return self.right_ctrl_thumbstick_shared.value

    @property
    def right_ctrl_thumbstickValue(self):
        """np.ndarray, shape (2,), right thumbstick 2D axis values (x, y)."""
        with self.right_ctrl_thumbstickValue_shared.get_lock():
            return np.array(self.right_ctrl_thumbstickValue_shared[:])

    @property
    def right_ctrl_aButton(self):
        """bool, right controller 'A' button pressed."""
        with self.right_ctrl_aButton_shared.get_lock():
            return self.right_ctrl_aButton_shared.value

    @property
    def right_ctrl_bButton(self):
        """bool, right controller 'B' button pressed."""
        with self.right_ctrl_bButton_shared.get_lock():
            return self.right_ctrl_bButton_shared.value
