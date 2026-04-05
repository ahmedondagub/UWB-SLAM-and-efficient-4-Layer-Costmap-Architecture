#!/usr/bin/env python3
"""Mission controller that submits Nav2 goals directly from RViz interactions."""

from __future__ import annotations

import time

import rclpy
from geometry_msgs.msg import Point, PointStamped, PoseStamped
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
from rcl_interfaces.msg import SetParametersResult
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray

from uwb_slam.math_utils import attach_debugger_if_requested


PATH_PLANNING = 'path_planning'
AREA_COVERAGE = 'area_coverage'


class AutonomousMotionNode(Node):
    """Translate RViz interactions into Nav2 action goals."""

    def __init__(self):
        super().__init__('autonomous_motion')

        self.declare_parameter('mission_mode', PATH_PLANNING)
        self.declare_parameter('max_speed', 0.30)
        self.declare_parameter('coverage_lane_spacing', 0.50)
        self.declare_parameter('runtime_sec', 0.0)
        self.declare_parameter('goal_tolerance_m', 0.12)
        self.declare_parameter('control_rate_hz', 10.0)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('navigate_to_pose_action', '/navigate_to_pose')
        self.declare_parameter('navigate_through_poses_action', '/navigate_through_poses')
        # Preferred controller plugin name for logging/diagnostics only.
        # NOTE: NavigateToPose/NavigateThroughPoses.Goal in Nav2 Jazzy has no
        # controller_id field, so this value is NOT sent to Nav2. The active
        # controller is selected at the controller_server level via
        # controller_plugins in nav2_params.yaml.
        self.declare_parameter('controller_id', 'FollowPath')

        self.mission_mode = self._normalize_mode(str(self.get_parameter('mission_mode').value))
        self.max_speed = float(self.get_parameter('max_speed').value)
        self.coverage_lane_spacing = max(float(self.get_parameter('coverage_lane_spacing').value), 0.05)
        self.runtime_sec = max(0.0, float(self.get_parameter('runtime_sec').value))
        self.goal_tolerance_m = float(self.get_parameter('goal_tolerance_m').value)
        self.control_rate_hz = max(float(self.get_parameter('control_rate_hz').value), 1.0)
        self.frame_id = str(self.get_parameter('frame_id').value).strip() or 'map'

        self.navigate_to_pose_action = str(self.get_parameter('navigate_to_pose_action').value)
        self.navigate_through_poses_action = str(self.get_parameter('navigate_through_poses_action').value)
        self.controller_id = str(self.get_parameter('controller_id').value).strip() or 'FollowPath'

        self.coverage_corners: list[tuple[float, float]] = []
        self.coverage_bounds: list[float] | None = None
        self.current_goal_pose: PoseStamped | None = None
        self.active_goal_handle = None
        self.active_goal_kind: str | None = None
        self.start_time = time.time()

        rviz_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.rviz_goal_callback, rviz_qos)
        self.point_sub = self.create_subscription(PointStamped, '/clicked_point', self.rviz_point_callback, rviz_qos)
        self.marker_pub = self.create_publisher(MarkerArray, '/mission/visuals', rviz_qos)

        self.navigate_to_pose_client = ActionClient(self, NavigateToPose, self.navigate_to_pose_action)
        self.navigate_through_poses_client = ActionClient(self, NavigateThroughPoses, self.navigate_through_poses_action)

        self.create_timer(1.0 / self.control_rate_hz, self.control_tick)
        self.add_on_set_parameters_callback(self.on_set_parameters)

        self.get_logger().info(
            f'Autonomous motion ready | mode={self.mission_mode}, max_speed={self.max_speed:.2f} m/s, '
            f'lane_spacing={self.coverage_lane_spacing:.2f} m'
        )

    def _normalize_mode(self, mode: str) -> str:
        normalized = str(mode).strip().lower()
        if normalized == PATH_PLANNING:
            return PATH_PLANNING
        if normalized == AREA_COVERAGE:
            return AREA_COVERAGE
        return PATH_PLANNING

    def on_set_parameters(self, params):
        new_mode = self.mission_mode
        new_speed = self.max_speed
        new_spacing = self.coverage_lane_spacing
        new_runtime = self.runtime_sec
        new_frame_id = self.frame_id
        new_controller_id = self.controller_id

        for param in params:
            if param.name == 'mission_mode':
                candidate = self._normalize_mode(str(param.value))
                if candidate not in (PATH_PLANNING, AREA_COVERAGE):
                    return SetParametersResult(successful=False, reason='mission_mode must be path_planning or area_coverage')
                new_mode = candidate
            elif param.name == 'max_speed':
                try:
                    new_speed = max(0.01, float(param.value))
                except (TypeError, ValueError):
                    return SetParametersResult(successful=False, reason='max_speed must be numeric')
            elif param.name == 'coverage_lane_spacing':
                try:
                    new_spacing = max(0.05, float(param.value))
                except (TypeError, ValueError):
                    return SetParametersResult(successful=False, reason='coverage_lane_spacing must be numeric')
            elif param.name == 'runtime_sec':
                try:
                    new_runtime = max(0.0, float(param.value))
                except (TypeError, ValueError):
                    return SetParametersResult(successful=False, reason='runtime_sec must be numeric')
            elif param.name == 'frame_id':
                value = str(param.value).strip()
                if not value:
                    return SetParametersResult(successful=False, reason='frame_id must be a non-empty string')
                new_frame_id = value
            elif param.name == 'controller_id':
                value = str(param.value).strip()
                if not value:
                    return SetParametersResult(successful=False, reason='controller_id must be a non-empty string')
                new_controller_id = value

        mode_changed = new_mode != self.mission_mode
        self.mission_mode = new_mode
        self.max_speed = new_speed
        self.coverage_lane_spacing = new_spacing
        self.runtime_sec = new_runtime
        self.frame_id = new_frame_id
        if new_controller_id != self.controller_id:
            self.controller_id = new_controller_id
            self.get_logger().info(
                f'controller_id updated to: {self.controller_id} '
                '(note: has no effect on Nav2 Jazzy — controller selection is via nav2_params.yaml)'
            )

        if mode_changed:
            self._cancel_active_goal()
            self.coverage_corners.clear()
            self.coverage_bounds = None
            self.current_goal_pose = None
            self.start_time = time.time()
            self.publish_markers()

        return SetParametersResult(successful=True)

    def _clear_markers(self) -> list[Marker]:
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.action = Marker.DELETEALL
        return [marker]

    def _make_pose(self, x: float, y: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = self.frame_id
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.orientation.w = 1.0
        return pose

    def _build_coverage_waypoints(self, bounds: list[float]) -> list[PoseStamped]:
        if len(bounds) != 4:
            return []

        xmin, ymin, xmax, ymax = bounds
        if xmax <= xmin or ymax <= ymin:
            return []

        waypoints: list[PoseStamped] = []
        lane_spacing = max(self.coverage_lane_spacing, 0.05)
        y = ymin
        left_to_right = True

        while y <= ymax + 1e-9:
            x_start, x_end = (xmin, xmax) if left_to_right else (xmax, xmin)
            waypoints.append(self._make_pose(x_start, y))
            waypoints.append(self._make_pose(x_end, y))
            left_to_right = not left_to_right
            y += lane_spacing

        return waypoints

    def publish_markers(self) -> None:
        marker_array = MarkerArray()
        marker_array.markers.extend(self._clear_markers())

        if self.current_goal_pose is not None and self.mission_mode == PATH_PLANNING:
            goal_marker = Marker()
            goal_marker.header.frame_id = self.current_goal_pose.header.frame_id or self.frame_id
            goal_marker.header.stamp = self.get_clock().now().to_msg()
            goal_marker.ns = 'mission_target'
            goal_marker.id = 0
            goal_marker.type = Marker.SPHERE
            goal_marker.action = Marker.ADD
            goal_marker.pose.position.x = self.current_goal_pose.pose.position.x
            goal_marker.pose.position.y = self.current_goal_pose.pose.position.y
            goal_marker.pose.position.z = 0.08
            goal_marker.pose.orientation.w = 1.0
            goal_marker.scale.x = 0.20
            goal_marker.scale.y = 0.20
            goal_marker.scale.z = 0.20
            goal_marker.color.r = 0.0
            goal_marker.color.g = 1.0
            goal_marker.color.b = 0.0
            goal_marker.color.a = 1.0
            marker_array.markers.append(goal_marker)

        if self.coverage_bounds is not None and self.mission_mode == AREA_COVERAGE:
            xmin, ymin, xmax, ymax = self.coverage_bounds

            boundary = Marker()
            boundary.header.frame_id = self.frame_id
            boundary.header.stamp = self.get_clock().now().to_msg()
            boundary.ns = 'mission_coverage'
            boundary.id = 1
            boundary.type = Marker.LINE_STRIP
            boundary.action = Marker.ADD
            boundary.scale.x = 0.06
            boundary.color.r = 1.0
            boundary.color.g = 0.6
            boundary.color.b = 0.0
            boundary.color.a = 1.0
            boundary_points = [
                (xmin, ymin),
                (xmax, ymin),
                (xmax, ymax),
                (xmin, ymax),
                (xmin, ymin),
            ]
            for x, y in boundary_points:
                point = Point()
                point.x = x
                point.y = y
                point.z = 0.05
                boundary.points.append(point)
            marker_array.markers.append(boundary)

            corners = Marker()
            corners.header.frame_id = self.frame_id
            corners.header.stamp = self.get_clock().now().to_msg()
            corners.ns = 'mission_coverage'
            corners.id = 2
            corners.type = Marker.SPHERE_LIST
            corners.action = Marker.ADD
            corners.scale.x = 0.18
            corners.scale.y = 0.18
            corners.scale.z = 0.18
            corners.color.r = 0.1
            corners.color.g = 0.9
            corners.color.b = 1.0
            corners.color.a = 1.0
            for x, y in boundary_points[:-1]:
                point = Point()
                point.x = x
                point.y = y
                point.z = 0.08
                corners.points.append(point)
            marker_array.markers.append(corners)

        self.marker_pub.publish(marker_array)

    def _nav2_ready(self) -> bool:
        return self.navigate_to_pose_client.wait_for_server(timeout_sec=0.0) and self.navigate_through_poses_client.wait_for_server(timeout_sec=0.0)

    def _active_goal_in_flight(self) -> bool:
        return self.active_goal_handle is not None

    def _reset_active_goal(self) -> None:
        self.active_goal_kind = None
        self.active_goal_handle = None

    def _cancel_active_goal(self) -> None:
        if self.active_goal_handle is None:
            return
        try:
            self.active_goal_handle.cancel_goal_async()
        finally:
            self._reset_active_goal()

    def _send_navigate_to_pose(self, pose: PoseStamped) -> None:
        if not self._nav2_ready():
            self.get_logger().warn('Nav2 action server is not ready for NavigateToPose')
            return

        goal = NavigateToPose.Goal()
        goal.pose = pose

        future = self.navigate_to_pose_client.send_goal_async(goal, feedback_callback=self._on_navigate_to_pose_feedback)
        future.add_done_callback(self._on_navigate_to_pose_response)
        self.active_goal_kind = 'navigate_to_pose'

    def _send_navigate_through_poses(self, poses: list[PoseStamped]) -> None:
        if not self._nav2_ready():
            self.get_logger().warn('Nav2 action server is not ready for NavigateThroughPoses')
            return

        goal = NavigateThroughPoses.Goal()
        goal.poses = poses

        future = self.navigate_through_poses_client.send_goal_async(
            goal,
            feedback_callback=self._on_navigate_through_poses_feedback,
        )
        future.add_done_callback(self._on_navigate_through_poses_response)
        self.active_goal_kind = 'navigate_through_poses'

    def _on_navigate_to_pose_response(self, future):
        try:
            goal_handle = future.result()
        except Exception as e:
            self.get_logger().error(f'NavigateToPose goal response failed: {e}')
            self._reset_active_goal()
            return
        if not goal_handle.accepted:
            self.get_logger().warn('NavigateToPose goal rejected by Nav2')
            self._reset_active_goal()
            return

        self.active_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_navigate_to_pose_result)
        self.get_logger().info('NavigateToPose goal accepted by Nav2')

    def _on_navigate_through_poses_response(self, future):
        try:
            goal_handle = future.result()
        except Exception as e:
            self.get_logger().error(f'NavigateThroughPoses goal response failed: {e}')
            self._reset_active_goal()
            return
        if not goal_handle.accepted:
            self.get_logger().warn('NavigateThroughPoses goal rejected by Nav2')
            self._reset_active_goal()
            return

        self.active_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_navigate_through_poses_result)
        self.get_logger().info('NavigateThroughPoses goal accepted by Nav2')

    def _on_navigate_to_pose_feedback(self, feedback_msg):
        del feedback_msg

    def _on_navigate_through_poses_feedback(self, feedback_msg):
        del feedback_msg

    def _on_navigate_to_pose_result(self, future):
        try:
            result = future.result()
        except Exception as e:
            self.get_logger().error(f'NavigateToPose result retrieval failed: {e}')
            self._reset_active_goal()
            return
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('NavigateToPose succeeded')
        else:
            self.get_logger().warn(f'NavigateToPose finished with non-success status={result.status}')
        self._reset_active_goal()

    def _on_navigate_through_poses_result(self, future):
        try:
            result = future.result()
        except Exception as e:
            self.get_logger().error(f'NavigateThroughPoses result retrieval failed: {e}')
            self._reset_active_goal()
            return
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('NavigateThroughPoses succeeded')
        else:
            self.get_logger().warn(f'NavigateThroughPoses finished with non-success status={result.status}')
        self._reset_active_goal()

    def rviz_goal_callback(self, msg: PoseStamped) -> None:
        if self.mission_mode != PATH_PLANNING:
            self.get_logger().info('Ignored /goal_pose because mission_mode is not path_planning')
            return

        self.current_goal_pose = self._make_pose(float(msg.pose.position.x), float(msg.pose.position.y))
        self.current_goal_pose.pose.orientation = msg.pose.orientation

        if self._active_goal_in_flight():
            self._cancel_active_goal()

        self.start_time = time.time()
        self._send_navigate_to_pose(self.current_goal_pose)
        self.publish_markers()
        self.get_logger().info(
            f'Received RViz goal | mode={self.mission_mode}, '
            f'goal=({self.current_goal_pose.pose.position.x:.2f}, {self.current_goal_pose.pose.position.y:.2f}), '
            f'max_speed={self.max_speed:.2f} m/s'
        )

    def rviz_point_callback(self, msg: PointStamped) -> None:
        if self.mission_mode != AREA_COVERAGE:
            self.get_logger().info('Ignored /clicked_point because mission_mode is not area_coverage')
            return

        self.coverage_corners.append((float(msg.point.x), float(msg.point.y)))
        self.publish_markers()
        self.get_logger().info(f'Coverage corner {len(self.coverage_corners)}/4 logged')

        if len(self.coverage_corners) < 4:
            return

        x_coords = [point[0] for point in self.coverage_corners]
        y_coords = [point[1] for point in self.coverage_corners]
        self.coverage_bounds = [min(x_coords), min(y_coords), max(x_coords), max(y_coords)]

        coverage_waypoints = self._build_coverage_waypoints(self.coverage_bounds)
        if not coverage_waypoints:
            self.get_logger().warn('Unable to build coverage waypoints from clicked corners')
            self.coverage_corners.clear()
            self.coverage_bounds = None
            self.publish_markers()
            return

        if self._active_goal_in_flight():
            self._cancel_active_goal()

        self.start_time = time.time()
        self._send_navigate_through_poses(coverage_waypoints)
        self.get_logger().info(
            'Received 4 coverage corners; dispatched NavigateThroughPoses with '
            f'{len(coverage_waypoints)} waypoints and lane_spacing={self.coverage_lane_spacing:.2f} m'
        )

        self.coverage_corners.clear()
        self.coverage_bounds = None
        self.publish_markers()

    def control_tick(self) -> None:
        if self.runtime_sec > 0.0 and (time.time() - self.start_time) > self.runtime_sec:
            if self._active_goal_in_flight():
                self.get_logger().warn('Mission runtime expired; cancelling active Nav2 goal')
                self._cancel_active_goal()
            return

        self.publish_markers()


def main(args=None):
    attach_debugger_if_requested()
    rclpy.init(args=args)
    node = AutonomousMotionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down autonomous motion orchestrator')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()