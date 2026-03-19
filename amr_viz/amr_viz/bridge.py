import asyncio
import json
import math
import queue
import threading
import traceback
from dataclasses import dataclass
from typing import Any, Dict, Optional, Set
from uuid import uuid4

import rclpy
import websockets
from amr_msgs.action import NavigateToPose
from amr_msgs.msg import MotionStatus, ObstacleReport
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from .serialization import (
    non_null_snapshot,
    serialize_motion_status,
    serialize_obstacle_report,
    serialize_occupancy_grid,
    serialize_path,
    serialize_pose_stamped,
    serialize_pose_with_covariance,
)


@dataclass
class PendingCommand:
    request_id: str
    command: str
    payload: Dict[str, Any]


class VizBridge(Node):
    def __init__(self) -> None:
        super().__init__("viz_bridge")
        self._validate_websockets_runtime()

        self._declare_parameters()

        self.websocket_host_ = self.get_parameter("websocket.host").value
        self.websocket_port_ = int(self.get_parameter("websocket.port").value)

        self.robot_pose_topic_ = self.get_parameter("topics.robot_pose").value
        self.global_path_topic_ = self.get_parameter("topics.global_path").value
        self.local_path_topic_ = self.get_parameter("topics.local_path").value
        self.map_topic_ = self.get_parameter("topics.map").value
        self.global_costmap_topic_ = self.get_parameter("topics.global_costmap").value
        self.local_costmap_topic_ = self.get_parameter("topics.local_costmap").value
        self.motion_status_topic_ = self.get_parameter("topics.motion_status").value
        self.obstacle_report_topic_ = self.get_parameter("topics.obstacle_report").value
        self.initial_pose_topic_ = self.get_parameter("topics.initial_pose").value
        self.navigate_to_pose_action_ = self.get_parameter("actions.navigate_to_pose").value

        self.initial_pose_publisher_ = self.create_publisher(
            PoseWithCovarianceStamped,
            self.initial_pose_topic_,
            10,
        )
        self.navigate_to_pose_client_ = ActionClient(
            self,
            NavigateToPose,
            self.navigate_to_pose_action_,
        )

        self.state_: Dict[str, Optional[Dict[str, Any]]] = {
            "robot_pose": None,
            "global_path": None,
            "local_path": None,
            "map": None,
            "global_costmap": None,
            "local_costmap": None,
            "motion_status": None,
            "obstacle_report": None,
        }

        self.command_queue_: "queue.Queue[PendingCommand]" = queue.Queue()
        self.websocket_loop_: Optional[asyncio.AbstractEventLoop] = None
        self.websocket_thread_: Optional[threading.Thread] = None
        self.websocket_server_: Any = None
        self.websocket_ready_ = threading.Event()
        self.websocket_error_: Optional[str] = None
        self.clients_: Set[Any] = set()

        self.create_subscription(
            PoseStamped,
            self.robot_pose_topic_,
            self._make_topic_callback("robot_pose", serialize_pose_stamped),
            10,
        )
        self.create_subscription(
            Path,
            self.global_path_topic_,
            self._make_topic_callback("global_path", serialize_path),
            10,
        )
        self.create_subscription(
            Path,
            self.local_path_topic_,
            self._make_topic_callback("local_path", serialize_path),
            10,
        )
        self.create_subscription(
            OccupancyGrid,
            self.map_topic_,
            self._make_topic_callback("map", serialize_occupancy_grid),
            10,
        )
        self.create_subscription(
            OccupancyGrid,
            self.global_costmap_topic_,
            self._make_topic_callback("global_costmap", serialize_occupancy_grid),
            10,
        )
        self.create_subscription(
            OccupancyGrid,
            self.local_costmap_topic_,
            self._make_topic_callback("local_costmap", serialize_occupancy_grid),
            10,
        )
        self.create_subscription(
            MotionStatus,
            self.motion_status_topic_,
            self._make_topic_callback("motion_status", serialize_motion_status),
            10,
        )
        self.create_subscription(
            ObstacleReport,
            self.obstacle_report_topic_,
            self._make_topic_callback("obstacle_report", serialize_obstacle_report),
            10,
        )

        self.command_timer_ = self.create_timer(0.05, self._process_pending_commands)
        self._start_websocket_server()

    def _validate_websockets_runtime(self) -> None:
        version_text = getattr(websockets, "__version__", "0.0")
        try:
            major_version = int(str(version_text).split(".", maxsplit=1)[0])
        except ValueError:
            major_version = 0

        if major_version >= 10:
            return

        raise RuntimeError(
            "amr_viz requires python websockets>=10 for Python 3.10 compatibility. "
            f"Detected websockets {version_text} at {getattr(websockets, '__file__', 'unknown')}."
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("websocket.host", "0.0.0.0")
        self.declare_parameter("websocket.port", 8765)
        self.declare_parameter("topics.robot_pose", "/amr/localization/pose")
        self.declare_parameter("topics.global_path", "/amr/planner/global")
        self.declare_parameter("topics.local_path", "/amr/planner/local")
        self.declare_parameter("topics.map", "/amr/map/data")
        self.declare_parameter("topics.global_costmap", "/amr/costmap/global")
        self.declare_parameter("topics.local_costmap", "/amr/costmap/local")
        self.declare_parameter("topics.motion_status", "/amr/motion/status")
        self.declare_parameter("topics.obstacle_report", "/amr/obstacle/report")
        self.declare_parameter("topics.initial_pose", "/amr/localization/initial_pose")
        self.declare_parameter("actions.navigate_to_pose", "/amr/navigator/navigate_to_pose")

    def _make_topic_callback(self, channel: str, serializer):
        def callback(message: Any) -> None:
            serialized = serializer(message)
            self.state_[channel] = serialized
            self._enqueue_bridge_message(
                {
                    "type": "topic_update",
                    "channel": channel,
                    "payload": serialized,
                }
            )

        return callback

    def _start_websocket_server(self) -> None:
        self.websocket_thread_ = threading.Thread(
            target=self._run_websocket_server,
            name="amr_viz_websocket",
            daemon=True,
        )
        self.websocket_thread_.start()
        self.websocket_ready_.wait(timeout=2.0)

        if self.websocket_error_ is not None:
            raise RuntimeError(self.websocket_error_)

        if not self.websocket_ready_.is_set():
            raise RuntimeError("Timed out while starting the AMR Viz WebSocket server")

        self.get_logger().info(
            f"Started amr_viz bridge on ws://{self.websocket_host_}:{self.websocket_port_}"
        )

    def _run_websocket_server(self) -> None:
        try:
            self.websocket_loop_ = asyncio.new_event_loop()
            asyncio.set_event_loop(self.websocket_loop_)
            self.websocket_server_ = self.websocket_loop_.run_until_complete(
                websockets.serve(self._handle_client, self.websocket_host_, self.websocket_port_)
            )
            self.websocket_ready_.set()
            self.websocket_loop_.run_forever()
        except Exception as exc:
            self.websocket_error_ = f"Failed to start the AMR Viz WebSocket server: {exc}"
            self.get_logger().error(f"{self.websocket_error_}\n{traceback.format_exc()}")
            self.websocket_ready_.set()
        finally:
            if self.websocket_server_ is not None and self.websocket_loop_ is not None:
                self.websocket_server_.close()
                self.websocket_loop_.run_until_complete(self.websocket_server_.wait_closed())
            if self.websocket_loop_ is not None:
                self.websocket_loop_.close()

    async def _handle_client(self, websocket: Any, path: Optional[str] = None) -> None:
        self.clients_.add(websocket)
        remote_address = getattr(websocket, "remote_address", None)
        self.get_logger().info(f"WebSocket client connected: {remote_address} path={path or '/'}")
        try:
            await websocket.send(
                json.dumps(
                    {
                        "type": "hello",
                        "payload": {
                            "server": "amr_viz_bridge",
                            "protocol_version": 1,
                            "capabilities": [
                                "navigate_to_pose",
                                "set_initial_pose",
                                "topic_updates",
                                "snapshots",
                            ],
                        },
                    }
                )
            )
            await websocket.send(
                json.dumps(
                    {
                        "type": "snapshot",
                        "payload": non_null_snapshot(self.state_),
                    }
                )
            )

            async for raw_message in websocket:
                await self._handle_client_message(raw_message, websocket)
        except Exception as exc:
            self.get_logger().error(
                f"WebSocket client handling failed for {remote_address}: {exc}\n{traceback.format_exc()}"
            )
        finally:
            self.clients_.discard(websocket)
            self.get_logger().info(f"WebSocket client disconnected: {remote_address}")

    async def _handle_client_message(self, raw_message: str, websocket: Any) -> None:
        try:
            message = json.loads(raw_message)
        except json.JSONDecodeError:
            await websocket.send(
                json.dumps(
                    {
                        "type": "error",
                        "payload": {
                            "message": "Invalid JSON payload",
                        },
                    }
                )
            )
            return

        message_type = message.get("type")
        if message_type == "ping":
            await websocket.send(
                json.dumps(
                    {
                        "type": "pong",
                        "payload": {"id": message.get("id", str(uuid4()))},
                    }
                )
            )
            return

        if message_type != "command":
            await websocket.send(
                json.dumps(
                    {
                        "type": "error",
                        "payload": {
                            "message": f"Unsupported message type: {message_type}",
                        },
                    }
                )
            )
            return

        request_id = message.get("id", str(uuid4()))
        command = message.get("command", "")
        payload = message.get("payload", {})
        self.command_queue_.put(PendingCommand(request_id=request_id, command=command, payload=payload))

    def _process_pending_commands(self) -> None:
        while True:
            try:
                command = self.command_queue_.get_nowait()
            except queue.Empty:
                return

            if command.command == "navigate_to_pose":
                self._handle_navigate_to_pose(command)
            elif command.command == "set_initial_pose":
                self._handle_set_initial_pose(command)
            else:
                self._emit_command_result(
                    request_id=command.request_id,
                    command=command.command,
                    success=False,
                    message=f"Unknown command '{command.command}'",
                )

    def _handle_navigate_to_pose(self, command: PendingCommand) -> None:
        if not self.navigate_to_pose_client_.wait_for_server(timeout_sec=0.25):
            self._emit_command_result(
                request_id=command.request_id,
                command=command.command,
                success=False,
                message="NavigateToPose action server is not available",
            )
            return

        payload = command.payload
        frame_id = payload.get("frame_id", "map")
        yaw = float(payload.get("yaw", 0.0))

        goal = NavigateToPose.Goal()
        goal.goal_pose.header.frame_id = frame_id
        goal.goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal.goal_pose.pose.position.x = float(payload.get("x", 0.0))
        goal.goal_pose.pose.position.y = float(payload.get("y", 0.0))
        goal.goal_pose.pose.position.z = 0.0
        goal.goal_pose.pose.orientation.x = 0.0
        goal.goal_pose.pose.orientation.y = 0.0
        goal.goal_pose.pose.orientation.z = float(math.sin(yaw * 0.5))
        goal.goal_pose.pose.orientation.w = float(math.cos(yaw * 0.5))

        future = self.navigate_to_pose_client_.send_goal_async(goal)
        future.add_done_callback(
            lambda result_future: self._handle_navigation_goal_response(command, result_future)
        )

    def _handle_navigation_goal_response(self, command: PendingCommand, result_future: Any) -> None:
        try:
            goal_handle = result_future.result()
        except Exception as exc:  # pragma: no cover - defensive branch
            self._emit_command_result(
                request_id=command.request_id,
                command=command.command,
                success=False,
                message=f"Failed to send goal: {exc}",
            )
            return

        if not goal_handle.accepted:
            self._emit_command_result(
                request_id=command.request_id,
                command=command.command,
                success=False,
                message="NavigateToPose goal was rejected",
            )
            return

        self._emit_command_result(
            request_id=command.request_id,
            command=command.command,
            success=True,
            message="NavigateToPose goal accepted",
        )

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda wrapped_future: self._handle_navigation_result(command, wrapped_future)
        )

    def _handle_navigation_result(self, command: PendingCommand, result_future: Any) -> None:
        try:
            wrapped_result = result_future.result()
            result = wrapped_result.result
            success = bool(result.success)
            message = result.message
        except Exception as exc:  # pragma: no cover - defensive branch
            success = False
            message = f"Failed to receive navigate result: {exc}"

        self._enqueue_bridge_message(
            {
                "type": "command_feedback",
                "channel": command.command,
                "payload": {
                    "id": command.request_id,
                    "success": success,
                    "message": message,
                },
            }
        )

    def _handle_set_initial_pose(self, command: PendingCommand) -> None:
        payload = command.payload
        yaw = float(payload.get("yaw", 0.0))
        message = PoseWithCovarianceStamped()
        message.header.frame_id = payload.get("frame_id", "map")
        message.header.stamp = self.get_clock().now().to_msg()
        message.pose.pose.position.x = float(payload.get("x", 0.0))
        message.pose.pose.position.y = float(payload.get("y", 0.0))
        message.pose.pose.position.z = 0.0
        message.pose.pose.orientation.x = 0.0
        message.pose.pose.orientation.y = 0.0
        message.pose.pose.orientation.z = float(math.sin(yaw * 0.5))
        message.pose.pose.orientation.w = float(math.cos(yaw * 0.5))
        message.pose.covariance = [0.0] * 36
        message.pose.covariance[0] = float(payload.get("covariance_x", 0.25))
        message.pose.covariance[7] = float(payload.get("covariance_y", 0.25))
        message.pose.covariance[35] = float(payload.get("covariance_yaw", 0.06853891945200942))

        self.initial_pose_publisher_.publish(message)
        self._emit_command_result(
            request_id=command.request_id,
            command=command.command,
            success=True,
            message="Initial pose published",
            payload=serialize_pose_with_covariance(message),
        )

    def _emit_command_result(
        self,
        request_id: str,
        command: str,
        success: bool,
        message: str,
        payload: Optional[Dict[str, Any]] = None,
    ) -> None:
        self._enqueue_bridge_message(
            {
                "type": "command_result",
                "channel": command,
                "payload": {
                    "id": request_id,
                    "success": success,
                    "message": message,
                    "data": payload,
                },
            }
        )

    def _enqueue_bridge_message(self, message: Dict[str, Any]) -> None:
        if self.websocket_loop_ is None:
            return
        asyncio.run_coroutine_threadsafe(self._broadcast(message), self.websocket_loop_)

    async def _broadcast(self, message: Dict[str, Any]) -> None:
        if not self.clients_:
            return

        encoded = json.dumps(message)
        stale_clients = []
        for client in list(self.clients_):
            try:
                await client.send(encoded)
            except Exception:
                stale_clients.append(client)

        for client in stale_clients:
            self.clients_.discard(client)

    def destroy_node(self) -> bool:
        if self.websocket_loop_ is not None:
            self.websocket_loop_.call_soon_threadsafe(self.websocket_loop_.stop)
        if self.websocket_thread_ is not None and self.websocket_thread_.is_alive():
            self.websocket_thread_.join(timeout=1.0)
        return super().destroy_node()


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = VizBridge()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
