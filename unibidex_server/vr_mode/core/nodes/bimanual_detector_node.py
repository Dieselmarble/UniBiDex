#!/usr/bin/env python3

import os
import threading
import asyncio
import struct
import traceback
import yaml
import numpy as np
import rclpy
import rclpy.duration
from visualization_msgs.msg import Marker, MarkerArray 
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Quaternion
from visualization_msgs.msg import MarkerArray
from hand_controller_msgs.msg import BimanualHandDetection, BimanualControllerState
from utils.viz_utils import (
    draw_mark_array_points,
    draw_mark_array_lines,
    HAND_CONNECTIONS,
)

# ----------------------------
# Global state for the latest parsed pose
# ----------------------------
_last_full_pose = None
_last_full_pose_lock = threading.Lock()


def parse_full_pose_binary(data: bytes) -> dict:
    """
    Parse a single FullPoseMessage payload (binary) into a Python dict.
    """
    offset = 0
    timestamp, = struct.unpack_from('<q', data, offset)
    offset += 8

    lx, ly, lz, lqx, lqy, lqz, lqw = struct.unpack_from('<7f', data, offset)
    offset += 7 * 4

    l_index_trig, l_hand_trig = struct.unpack_from('<2f', data, offset)
    offset += 2 * 4

    l_stick_x, l_stick_y = struct.unpack_from('<2f', data, offset)
    offset += 2 * 4

    l_stick_pressed, l_stick_touched, l_buttonA, l_buttonB, l_buttonX, l_buttonY, l_buttonMenu, l_buttonSystem = \
        struct.unpack_from('<8B', data, offset)
    offset += 8

    rx, ry, rz, rqx, rqy, rqz, rqw = struct.unpack_from('<7f', data, offset)
    offset += 7 * 4

    r_index_trig, r_hand_trig = struct.unpack_from('<2f', data, offset)
    offset += 2 * 4

    r_stick_x, r_stick_y = struct.unpack_from('<2f', data, offset)
    offset += 2 * 4

    r_stick_pressed, r_stick_touched, r_buttonA, r_buttonB, r_buttonX, r_buttonY, r_buttonMenu, r_buttonSystem = \
        struct.unpack_from('<8B', data, offset)
    offset += 8

    left_bone_count, = struct.unpack_from('<i', data, offset)
    offset += 4
    left_bones = []
    for _ in range(left_bone_count):
        bx, by, bz, bqx, bqy, bqz, bqw = struct.unpack_from('<7f', data, offset)
        offset += 7 * 4
        left_bones.append((bx, by, bz, bqx, bqy, bqz, bqw))

    right_bone_count, = struct.unpack_from('<i', data, offset)
    offset += 4
    right_bones = []
    for _ in range(right_bone_count):
        bx, by, bz, bqx, bqy, bqz, bqw = struct.unpack_from('<7f', data, offset)
        offset += 7 * 4
        right_bones.append((bx, by, bz, bqx, bqy, bqz, bqw))

    return {
        'timestamp': timestamp,
        'left_pos':           (lx, ly, lz),
        'left_rot':           (lqx, lqy, lqz, lqw),
        'left_index_trig':    l_index_trig,
        'left_hand_trig':     l_hand_trig,
        'left_stick':         (l_stick_x, l_stick_y),
        'left_stick_pressed': bool(l_stick_pressed),
        'left_stick_touched': bool(l_stick_touched),
        'left_buttonA':       bool(l_buttonA),
        'left_buttonB':       bool(l_buttonB),
        'left_buttonX':       bool(l_buttonX),
        'left_buttonY':       bool(l_buttonY),
        'left_buttonMenu':    bool(l_buttonMenu),
        'left_buttonSystem':  bool(l_buttonSystem),
        'right_pos':          (rx, ry, rz),
        'right_rot':          (rqx, rqy, rqz, rqw),
        'right_index_trig':   r_index_trig,
        'right_hand_trig':    r_hand_trig,
        'right_stick':        (r_stick_x, r_stick_y),
        'right_stick_pressed':bool(r_stick_pressed),
        'right_stick_touched':bool(r_stick_touched),
        'right_buttonA':      bool(r_buttonA),
        'right_buttonB':      bool(r_buttonB),
        'right_buttonX':      bool(r_buttonX),
        'right_buttonY':      bool(r_buttonY),
        'right_buttonMenu':   bool(r_buttonMenu),
        'right_buttonSystem': bool(r_buttonSystem),
        'left_bone_count':    left_bone_count,
        'left_bones':         left_bones,
        'right_bone_count':   right_bone_count,
        'right_bones':        right_bones,
    }


async def _async_tcp_client(host: str, port: int):
    """
    Asyncio-based TCP client that connects to the Quest-3 sender (server),
    reads length-prefixed pose messages, parses them, and updates global _last_full_pose.
    If the connection drops, it will retry every 2 seconds.
    """
    while True:
        try:
            print(f"[TCP Client] Attempting to connect to {host}:{port}...")
            reader, writer = await asyncio.open_connection(host, port)
            print(f"[TCP Client] Connected to {host}:{port}")
            while True:
                length_prefix = await reader.readexactly(4)
                msg_length = struct.unpack('<I', length_prefix)[0]
                if msg_length <= 0:
                    print("[TCP Client] Received invalid length, reconnecting...")
                    break
                payload = await reader.readexactly(msg_length)
                try:
                    parsed = parse_full_pose_binary(payload)
                except Exception:
                    traceback.print_exc()
                    continue

                with _last_full_pose_lock:
                    global _last_full_pose
                    _last_full_pose = parsed

        except (asyncio.IncompleteReadError, ConnectionResetError):
            print("[TCP Client] Connection closed by server, retrying in 2 seconds...")
        except Exception as e:
            print(f"[TCP Client] Unexpected error: {e}")
            traceback.print_exc()

        # Wait before retrying
        await asyncio.sleep(2)


def start_tcp_client_in_thread(host: str, port: int):
    """
    Launch the asyncio TCP client in a separate thread so it doesn't block ROS.
    """
    loop = asyncio.new_event_loop()
    threading.Thread(
        target=loop.run_until_complete,
        args=(_async_tcp_client(host, port),),
        daemon=True
    ).start()


def pose_from_ctrl_data(pos_tuple, rot_tuple) -> Pose:
    px, py, pz = pos_tuple
    qx, qy, qz, qw = rot_tuple
    p = Pose()
    p.position = Point(x=px, y=py, z=pz)
    p.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
    return p

class Quest3HandDetector(Node):
    """
    ROS2 node publishing hand detections and bimanual controller states.
    """
    def __init__(self, quest3_host: str, quest3_port: int, visualize_3d_detection=False):
        super().__init__('quest3_hand_detector')
        self.visualize_3d_detection = visualize_3d_detection

        self.detection_pub = self.create_publisher(
            BimanualHandDetection,
            '/quest3/bimanual_hand_detection/results',
            10
        )
        self.controller_pub = self.create_publisher(
            BimanualControllerState,
            '/quest3/bimanual_controller_state',
            10
        )
        if self.visualize_3d_detection:
            self.skeleton_pub = self.create_publisher(
                MarkerArray,
                '/quest3/bimanual_hand_detection/skeleton',
                5
            )
            self.controller_marker_pub = self.create_publisher(
                MarkerArray,
                '/quest3/bimanual_controller_markers',
                5
            )

        # Start the TCP client in background using quest3_host and quest3_port
        start_tcp_client_in_thread(quest3_host, quest3_port)

        # Publish at 20 Hz
        self.create_timer(0.025, self.publish_from_last_full_pose)
        self.get_logger().info("Quest3HandDetector initialized and TCP client started.")


    def normalize_joints(self, bones: list[tuple]) -> list[float]:
        coords = [coord for b in bones for coord in b[:3]]
        required = 21 * 3
        if len(coords) != required:
            # pad with 0.0 or truncate
            coords = (coords + [0.0] * required)[:required]
            self.get_logger().warning(
                f"normalize_joints: got {len(bones)} bones → {len(coords)} coords, forced to {required}"
            )
        return coords


    def publish_from_last_full_pose(self):
        with _last_full_pose_lock:
            data = _last_full_pose.copy() if _last_full_pose is not None else None
        if data is None:
            return

        # Publish hand detection
        hand_msg = BimanualHandDetection()
        hand_msg.detected = True
        hand_msg.left_wrist_pose = pose_from_ctrl_data(data['left_pos'], data['left_rot'])
        hand_msg.right_wrist_pose = pose_from_ctrl_data(data['right_pos'], data['right_rot'])
        # normalize to exactly 21 bones × 3 coords = 63 floats

        hand_msg.left_joints = self.normalize_joints(data['left_bones'])
        hand_msg.right_joints = self.normalize_joints(data['right_bones'])

        self.detection_pub.publish(hand_msg)

        # Publish controller state
        ctrl_msg = BimanualControllerState()
        ctrl_msg.header.stamp = self.get_clock().now().to_msg()
        # left
        ctrl_msg.left_pose = pose_from_ctrl_data(data['left_pos'], data['left_rot'])
        ctrl_msg.left_index_trigger = data['left_index_trig']
        ctrl_msg.left_hand_trigger = data['left_hand_trig']
        ctrl_msg.left_stick = list(data['left_stick'])
        ctrl_msg.left_stick_pressed = data['left_stick_pressed']
        ctrl_msg.left_stick_touched = data['left_stick_touched']
        ctrl_msg.left_buttons = [
            data['left_buttonA'],
            data['left_buttonB'],
            data['left_buttonX'],
            data['left_buttonY'],
            data['left_buttonMenu'],
            data['left_buttonSystem'],
        ]
        # right
        ctrl_msg.right_pose = pose_from_ctrl_data(data['right_pos'], data['right_rot'])
        ctrl_msg.right_index_trigger = data['right_index_trig']
        ctrl_msg.right_hand_trigger = data['right_hand_trig']
        ctrl_msg.right_stick = list(data['right_stick'])
        ctrl_msg.right_stick_pressed = data['right_stick_pressed']
        ctrl_msg.right_stick_touched = data['right_stick_touched']
        ctrl_msg.right_buttons = [
            data['right_buttonA'],
            data['right_buttonB'],
            data['right_buttonX'],
            data['right_buttonY'],
            data['right_buttonMenu'],
            data['right_buttonSystem'],
        ]
        print(ctrl_msg.right_pose)
        self.controller_pub.publish(ctrl_msg)

        # Optionally publish skeleton markers
        if self.visualize_3d_detection and self.skeleton_pub.get_subscription_count() > 0:
            self.publish_skeleton_markers(data['left_bones'], data['right_bones'])

        # Optionally publish controller markers  ← new
        if self.visualize_3d_detection and self.controller_marker_pub.get_subscription_count() > 0:
            self.publish_controller_markers(
            ctrl_msg.left_pose,
            ctrl_msg.right_pose
            )

    def publish_skeleton_markers(self, left_bones, right_bones):
        stamp = self.get_clock().now().to_msg()
        color = np.array([0.0, 1.0, 0.0, 1.0])
        ns_base = "quest3"
        ns_id = 1
        left_kp = np.array([[b[0], b[1], b[2]] for b in left_bones])
        right_kp = np.array([[b[0], b[1], b[2]] for b in right_bones])
        msg = MarkerArray()
        for keypoints, hand_label in [(right_kp, "right"), (left_kp, "left")]:
            
            # skip if keypoints array is empty or malformed
            if keypoints.ndim != 2 or keypoints.shape[1] != 3:
                continue
            
            msg = draw_mark_array_points(
                msg, keypoints, f"{ns_base}_{hand_label}_points",
                "world", stamp, color, 0.01, ns_id=ns_id, id_offset=0, lifetime=0.1
            )
            msg = draw_mark_array_lines(
                msg, keypoints, HAND_CONNECTIONS,
                f"{ns_base}_{hand_label}_lines",
                "world", stamp,
                np.array([0.2, 0.2, 0.2, 1.0]),
                0.005, ns_id=ns_id, id_offset=21, lifetime=0.1
            )
            ns_id += 1
        self.skeleton_pub.publish(msg)


    def publish_controller_markers(self, left_pose, right_pose):
        """Publish sphere markers at each controller pose."""
        stamp = self.get_clock().now().to_msg()
        lifetime = rclpy.duration.Duration(seconds=0.1).to_msg()
        msg = MarkerArray()

        for idx, (pose, hand) in enumerate([(left_pose, 'left'), (right_pose, 'right')]):
            m = Marker()
            m.header.frame_id = 'world'
            m.header.stamp = stamp
            m.ns = f'quest3_controller_{hand}'
            m.id = idx
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose = pose
            # sphere scale: x/y/z = diameter
            m.scale.x = 0.05
            m.scale.y = 0.05
            m.scale.z = 0.05
            # color: blue=left, red=right
            if hand == 'left':
                m.color.r = 0.0; m.color.g = 0.0; m.color.b = 1.0
            else:
                m.color.r = 1.0; m.color.g = 0.0; m.color.b = 0.0
            m.color.a = 1.0
            m.lifetime = lifetime

            msg.markers.append(m)

        self.controller_marker_pub.publish(msg)

def main(args=None):
    # parser = argparse.ArgumentParser(description="Quest3 Hand Detector Node")
    # parser.add_argument(
    #     "-c", "--config", type=str, required=True,
    #     help="Path to Quest-3 config YAML file"
    # )
    # parsed_args = parser.parse_args()
    config_path = '/server/banana_teleop_server/configs/vr_config/quest_config.yml'
    print(f"Using config: {config_path}")
    if not os.path.isfile(config_path):
        raise FileNotFoundError(f"Config file not found: {config_path}")
    with open(config_path, 'r') as f:
        cfg = yaml.safe_load(f)
    quest3_ip = cfg.get('quest3_ip', '127.0.0.1')
    quest3_port = cfg.get('quest3_port', 8005)

    rclpy.init(args=args)
    node = Quest3HandDetector(quest3_ip, quest3_port, visualize_3d_detection=True)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutdown Quest3HandDetector.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()