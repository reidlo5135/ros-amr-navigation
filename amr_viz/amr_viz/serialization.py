import math
from typing import Any, Dict, Optional

from amr_msgs.msg import MotionStatus, ObstacleReport
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Path


def _quaternion_to_yaw(orientation: Any) -> float:
    siny_cosp = 2.0 * ((orientation.w * orientation.z) + (orientation.x * orientation.y))
    cosy_cosp = 1.0 - 2.0 * ((orientation.y * orientation.y) + (orientation.z * orientation.z))
    return math.atan2(siny_cosp, cosy_cosp)


def serialize_header(header: Any) -> Dict[str, Any]:
    return {
        "stamp": {
            "sec": int(header.stamp.sec),
            "nanosec": int(header.stamp.nanosec),
        },
        "frame_id": header.frame_id,
    }


def serialize_point(point: Any) -> Dict[str, float]:
    return {
        "x": float(point.x),
        "y": float(point.y),
        "z": float(point.z),
    }


def serialize_pose_stamped(message: PoseStamped) -> Dict[str, Any]:
    return {
        "header": serialize_header(message.header),
        "position": serialize_point(message.pose.position),
        "orientation": {
            "x": float(message.pose.orientation.x),
            "y": float(message.pose.orientation.y),
            "z": float(message.pose.orientation.z),
            "w": float(message.pose.orientation.w),
            "yaw": _quaternion_to_yaw(message.pose.orientation),
        },
    }


def serialize_pose_with_covariance(message: PoseWithCovarianceStamped) -> Dict[str, Any]:
    return {
        "header": serialize_header(message.header),
        "pose": {
            "position": serialize_point(message.pose.pose.position),
            "orientation": {
                "x": float(message.pose.pose.orientation.x),
                "y": float(message.pose.pose.orientation.y),
                "z": float(message.pose.pose.orientation.z),
                "w": float(message.pose.pose.orientation.w),
                "yaw": _quaternion_to_yaw(message.pose.pose.orientation),
            },
            "covariance": [float(value) for value in message.pose.covariance],
        },
    }


def serialize_path(message: Path) -> Dict[str, Any]:
    return {
        "header": serialize_header(message.header),
        "poses": [serialize_pose_stamped(pose) for pose in message.poses],
    }


def serialize_occupancy_grid(message: OccupancyGrid) -> Dict[str, Any]:
    return {
        "header": serialize_header(message.header),
        "info": {
            "width": int(message.info.width),
            "height": int(message.info.height),
            "resolution": float(message.info.resolution),
            "origin": {
                "position": serialize_point(message.info.origin.position),
                "orientation": {
                    "x": float(message.info.origin.orientation.x),
                    "y": float(message.info.origin.orientation.y),
                    "z": float(message.info.origin.orientation.z),
                    "w": float(message.info.origin.orientation.w),
                    "yaw": _quaternion_to_yaw(message.info.origin.orientation),
                },
            },
        },
        "data": [int(value) for value in message.data],
    }


def serialize_motion_status(message: MotionStatus) -> Dict[str, Any]:
    return {
        "header": serialize_header(message.header),
        "command_id": int(message.command_id),
        "active": bool(message.active),
        "goal_reached": bool(message.goal_reached),
        "obstacle_detected": bool(message.obstacle_detected),
        "current_pose": serialize_pose_stamped(message.current_pose),
        "remaining_distance": float(message.remaining_distance),
        "heading_error": float(message.heading_error),
    }


def serialize_obstacle_report(message: ObstacleReport) -> Dict[str, Any]:
    return {
        "header": serialize_header(message.header),
        "active": bool(message.active),
        "is_dynamic": bool(message.is_dynamic),
        "blocks_path": bool(message.blocks_path),
        "severity": int(message.severity),
        "distance": float(message.distance),
        "bearing": float(message.bearing),
        "obstacle_point": serialize_point(message.obstacle_point),
        "source": message.source,
    }


def non_null_snapshot(snapshot: Dict[str, Optional[Dict[str, Any]]]) -> Dict[str, Any]:
    return {key: value for key, value in snapshot.items() if value is not None}
