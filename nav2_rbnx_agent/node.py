#!/usr/bin/env python3
"""Nav2 MCP bridge for Robonix agent integration."""

from __future__ import annotations

import atexit
import asyncio
import json
import math
import os
import socket
import subprocess
import sys
import threading
import time
import uuid
from pathlib import Path

from mcp.server.fastmcp import FastMCP


def _ensure_proto_gen() -> None:
    extra = os.environ.get("ROBONIX_PROTO_GEN", "").strip()
    if extra:
        pg = Path(extra)
        if pg.is_dir() and (pg / "robonix_runtime_pb2.py").exists():
            sys.path.insert(0, str(pg))
            return

    d = Path(__file__).resolve().parent
    while d.parent != d:
        pg = d / "proto_gen"
        if pg.is_dir() and (pg / "robonix_runtime_pb2.py").exists():
            sys.path.insert(0, str(pg))
            return
        d = d.parent

    sibling = Path(__file__).resolve().parents[2] / "robonix" / "rust" / "examples" / "proto_gen"
    if sibling.is_dir() and (sibling / "robonix_runtime_pb2.py").exists():
        sys.path.insert(0, str(sibling))
        return


_ensure_proto_gen()

import grpc
import robonix_runtime_pb2 as pb
import robonix_runtime_pb2_grpc as pb_grpc

_rclpy = None
_PoseStamped = None
_NavigateToPose = None
_GoalStatus = None

mcp = FastMCP("nav2-rbnx")
_ros_node = None
_goal_pub = None
_nav_client = None
_nav_ready = False
_goal_states: dict[str, dict] = {}
_goal_handles: dict[str, object] = {}
_state_lock = threading.Lock()
_nav_process: subprocess.Popen[str] | None = None


def _import_ros2():
    global _rclpy, _PoseStamped, _NavigateToPose, _GoalStatus
    import rclpy  # type: ignore
    from geometry_msgs.msg import PoseStamped  # type: ignore

    _rclpy = rclpy
    _PoseStamped = PoseStamped

    try:
        from nav2_msgs.action import NavigateToPose  # type: ignore
        _NavigateToPose = NavigateToPose
    except ImportError:
        _NavigateToPose = None

    try:
        from action_msgs.msg import GoalStatus  # type: ignore
        _GoalStatus = GoalStatus
    except ImportError:
        _GoalStatus = None


def _goal_status_name(status: int) -> str:
    if _GoalStatus is None:
        return str(int(status))
    g = _GoalStatus
    mapping = {
        int(g.STATUS_UNKNOWN): "UNKNOWN",
        int(g.STATUS_ACCEPTED): "ACCEPTED",
        int(g.STATUS_EXECUTING): "EXECUTING",
        int(g.STATUS_CANCELING): "CANCELING",
        int(g.STATUS_SUCCEEDED): "SUCCEEDED",
        int(g.STATUS_CANCELED): "CANCELED",
        int(g.STATUS_ABORTED): "ABORTED",
    }
    return mapping.get(int(status), str(int(status)))


def _make_pose(frame_id: str, x: float, y: float, yaw: float):
    goal = _PoseStamped()
    goal.header.frame_id = frame_id
    goal.header.stamp = _ros_node.get_clock().now().to_msg()
    goal.pose.position.x = float(x)
    goal.pose.position.y = float(y)
    goal.pose.position.z = 0.0
    goal.pose.orientation.z = math.sin(yaw / 2.0)
    goal.pose.orientation.w = math.cos(yaw / 2.0)
    return goal


def _feedback_cb(goal_id: str, _feedback) -> None:
    with _state_lock:
        if goal_id in _goal_states:
            _goal_states[goal_id]["status"] = "EXECUTING"


def _goal_response_cb(fut, goal_id: str) -> None:
    try:
        gh = fut.result()
    except Exception as exc:
        with _state_lock:
            _goal_states[goal_id] = {"status": "FAILED", "error": str(exc)}
        return

    if not gh.accepted:
        with _state_lock:
            _goal_states[goal_id] = {"status": "REJECTED"}
        return

    with _state_lock:
        _goal_handles[goal_id] = gh
        _goal_states[goal_id] = {"status": "ACCEPTED"}

    res_fut = gh.get_result_async()
    res_fut.add_done_callback(lambda f: _result_cb(f, goal_id))


def _result_cb(fut, goal_id: str) -> None:
    try:
        result = fut.result()
        status = getattr(result, "status", None)
        with _state_lock:
            _goal_states[goal_id] = {
                "status": _goal_status_name(status) if status is not None else "UNKNOWN",
                "terminal": True,
            }
            _goal_handles.pop(goal_id, None)
    except Exception as exc:
        with _state_lock:
            _goal_states[goal_id] = {"status": "FAILED", "terminal": True, "error": str(exc)}
            _goal_handles.pop(goal_id, None)


@mcp.tool()
def navigate_to(x: float, y: float, yaw: float = 0.0, frame_id: str = "map") -> str:
    """Send a navigation goal to Nav2."""
    if _ros_node is None:
        return json.dumps({"error": "ROS2 node not initialized"})

    goal_id = str(uuid.uuid4())
    pose = _make_pose(frame_id, x, y, yaw)

    if _nav_client is not None and _nav_ready:
        goal_msg = _NavigateToPose.Goal()
        goal_msg.pose = pose
        fut = _nav_client.send_goal_async(
            goal_msg,
            feedback_callback=lambda fb: _feedback_cb(goal_id, fb),
        )
        fut.add_done_callback(lambda f: _goal_response_cb(f, goal_id))
        with _state_lock:
            _goal_states[goal_id] = {"status": "SENT"}
    else:
        _goal_pub.publish(pose)
        with _state_lock:
            _goal_states[goal_id] = {"status": "PUBLISHED_TOPIC", "topic": "/goal_pose"}

    return json.dumps({"goal_id": goal_id, **_goal_states[goal_id]})


@mcp.tool()
def get_nav_status(goal_id: str) -> str:
    """Query navigation status for a goal_id."""
    with _state_lock:
        status = _goal_states.get(goal_id)
    if status is None:
        return json.dumps({"error": "unknown goal_id", "goal_id": goal_id})
    return json.dumps({"goal_id": goal_id, **status})


@mcp.tool()
def cancel_nav(goal_id: str) -> str:
    """Cancel an active navigation goal."""
    with _state_lock:
        gh = _goal_handles.get(goal_id)
    if gh is None:
        return json.dumps({"error": "no active goal handle", "goal_id": goal_id})
    gh.cancel_goal_async()  # type: ignore[union-attr]
    return json.dumps({"goal_id": goal_id, "status": "cancel_requested"})


def _mcp_tools_list() -> list[dict]:
    async def _list():
        return await mcp.list_tools()

    tools = asyncio.run(_list())
    out = []
    for t in tools:
        schema = t.inputSchema if isinstance(t.inputSchema, dict) else dict(t.inputSchema)
        out.append({"name": t.name, "description": t.description or "", "input_schema": schema})
    return out


def _pick_port(env_name: str) -> int:
    raw = os.environ.get(env_name, "").strip()
    if raw.isdigit():
        p = int(raw)
        if 1 <= p <= 65535:
            return p
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(("127.0.0.1", 0))
    p = int(s.getsockname()[1])
    s.close()
    return p


def _start_mcp_http(port: int) -> None:
    import uvicorn

    uvicorn.run(mcp.streamable_http_app(), host="0.0.0.0", port=port, log_level="warning")


def _start_nav2_launch() -> None:
    global _nav_process
    if os.environ.get("NAV2_RBNX_AUTOSTART", "1").strip() in ("0", "false", "False"):
        return

    pkg_root = Path(__file__).resolve().parents[1]
    params_file = os.environ.get("NAV2_PARAMS_FILE", str(pkg_root / "config/nav2_params_slam.yml"))
    cmd = (
        "source /opt/ros/humble/setup.bash && "
        f"cd {pkg_root} && "
        f"exec ros2 launch nav2_bringup navigation_launch.py use_sim_time:=false params_file:={params_file}"
    )
    _nav_process = subprocess.Popen(["bash", "-lc", cmd])
    print("[nav2-rbnx] started nav2_bringup", flush=True)


def _start_ros2() -> None:
    global _ros_node, _goal_pub, _nav_client, _nav_ready
    _import_ros2()
    _rclpy.init()

    from rclpy.executors import MultiThreadedExecutor  # type: ignore

    node = _rclpy.create_node("nav2_rbnx_agent")
    _ros_node = node
    _goal_pub = node.create_publisher(_PoseStamped, "/goal_pose", 10)

    if _NavigateToPose is not None:
        from rclpy.action import ActionClient  # type: ignore

        _nav_client = ActionClient(node, _NavigateToPose, "navigate_to_pose")
        _nav_ready = _nav_client.wait_for_server(timeout_sec=float(os.environ.get("NAV2_WAIT_SEC", "30")))
        if not _nav_ready:
            print("[nav2-rbnx] nav2 action server not ready; using /goal_pose fallback", flush=True)

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    while _rclpy.ok():
        executor.spin_once(timeout_sec=0.1)


def main() -> None:
    channel = grpc.insecure_channel(os.environ.get("ROBONIX_SERVER", "localhost:50051"))
    stub = pb_grpc.RobonixRuntimeStub(channel)
    node_id = os.environ.get("ROBONIX_NODE_ID", "com.vendor.nav2")

    registered = False

    def _cleanup() -> None:
        nonlocal registered
        if registered:
            try:
                stub.UnregisterNode(pb.UnregisterNodeRequest(node_id=node_id))
            except grpc.RpcError:
                pass
        if _nav_process is not None and _nav_process.poll() is None:
            _nav_process.terminate()

    atexit.register(_cleanup)

    stub.RegisterNode(
        pb.RegisterNodeRequest(
            node_id=node_id,
            namespace=os.environ.get("ROBONIX_NAMESPACE", "vendor/navigation"),
            kind="service",
            skill_md="",
            distro=os.environ.get("ROBONIX_DISTRO", "humble"),
            container_id=os.environ.get("ROBONIX_CONTAINER_ID", ""),
        )
    )
    registered = True

    mcp_port = _pick_port("ROBONIX_MCP_LISTEN_PORT")
    resp = stub.DeclareInterface(
        pb.DeclareInterfaceRequest(
            node_id=node_id,
            name="mcp_tools",
            supported_transports=["mcp"],
            metadata_json=json.dumps({"tools": _mcp_tools_list()}),
            listen_port=mcp_port,
        )
    )
    print(f"[nav2-rbnx] MCP endpoint {resp.allocated_endpoint}", flush=True)

    _start_nav2_launch()
    # threading.Thread(target=_start_ros2, daemon=True).start()s
    threading.Thread(target=_start_mcp_http, args=(mcp_port,), daemon=True).start()
    print("[nav2-rbnx] ready", flush=True)

    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
