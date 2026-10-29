"""Gazebo side of the arisim 3D view. Runs under the Python that has the gz bindings
(python3.12 here), receives UDP packets from arisim and moves the quad in arisim_world.sdf.

Packets (little endian):
  b'P' + 7 doubles (x, y, z, qw, qx, qy, qz) + 4 floats (rotor speeds, rad/s, signed by spin)
  b'B' + 5 doubles (center x, center y, size x, size y, height)  -> static box
  b'F'                                                          -> camera follows the quad
"""

import argparse
import socket
import struct
import sys
import time

sys.path.insert(0, "/usr/lib/python3/dist-packages")
from gz.msgs10.boolean_pb2 import Boolean          # noqa: E402
from gz.msgs10.double_pb2 import Double            # noqa: E402
from gz.msgs10.entity_factory_pb2 import EntityFactory  # noqa: E402
from gz.msgs10.pose_pb2 import Pose                # noqa: E402
from gz.msgs10.stringmsg_pb2 import StringMsg      # noqa: E402
from gz.msgs10.vector3d_pb2 import Vector3d        # noqa: E402
from gz.transport13 import Node                    # noqa: E402

POSE = struct.Struct("<7d4f")
BOX = struct.Struct("<5d")
PROP_VISUAL_SCALE = 0.03          # real rotor speeds alias on screen; spin the props slower


def box_sdf(name, cx, cy, sx, sy, h):
    return f"""<sdf version="1.9"><model name="{name}"><static>true</static><pose>{cx} {cy} {h / 2} 0 0 0</pose>
<link name="link"><visual name="v"><geometry><box><size>{sx} {sy} {h}</size></box></geometry>
<material><ambient>0.55 0.45 0.35 1</ambient><diffuse>0.55 0.45 0.35 1</diffuse></material></visual></link>
</model></sdf>"""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=47800)
    ap.add_argument("--world", default="arisim")
    ap.add_argument("--model", default="x3")
    args = ap.parse_args()

    node = Node()
    set_pose = f"/world/{args.world}/set_pose"
    create = f"/world/{args.world}/create"
    joint_pubs = [node.advertise(f"/model/{args.model}/joint/j{i}/cmd_vel", Double) for i in range(4)]

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", args.port))
    sock.settimeout(0.5)
    print(f"gz_relay: listening on udp {args.port}, world '{args.world}'", flush=True)

    boxes = 0
    while True:
        try:
            data, _ = sock.recvfrom(256)
        except socket.timeout:
            continue
        kind, body = data[:1], data[1:]
        if kind == b"P" and len(body) == POSE.size:
            v = POSE.unpack(body)
            req = Pose()
            req.name = args.model
            req.position.x, req.position.y, req.position.z = v[0:3]
            req.orientation.w, req.orientation.x, req.orientation.y, req.orientation.z = v[3:7]
            node.request(set_pose, req, Pose, Boolean, 50)
            for pub, w in zip(joint_pubs, v[7:11]):
                pub.publish(Double(data=w * PROP_VISUAL_SCALE))
        elif kind == b"B" and len(body) == BOX.size:
            req = EntityFactory()
            req.sdf = box_sdf(f"box_{boxes}", *BOX.unpack(body))
            boxes += 1
            node.request(create, req, EntityFactory, Boolean, 2000)
        elif kind == b"F":
            node.request("/gui/follow", StringMsg(data=args.model), StringMsg, Boolean, 2000)
            off = Vector3d(x=-2.0, y=-1.5, z=1.2)
            node.request("/gui/follow/offset", off, Vector3d, Boolean, 2000)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
