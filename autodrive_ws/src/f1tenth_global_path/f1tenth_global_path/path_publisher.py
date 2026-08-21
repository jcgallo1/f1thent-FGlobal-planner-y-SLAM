#!/usr/bin/env python3

import csv
import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path as PathMessage
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray


class GlobalPathPublisher(Node):
    """Publica la ruta D* suavizada y sus marcadores para RViz."""

    def __init__(self):
        super().__init__('f1tenth_global_path_publisher')

        package_share = Path(get_package_share_directory('f1tenth_global_path'))
        default_csv = package_share / 'paths' / 'dstar_0_5m.csv'

        self.declare_parameter('csv_path', str(default_csv))
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('path_topic', '/global_path')
        self.declare_parameter('marker_topic', '/global_path_markers')
        self.declare_parameter('vehicle_frame', 'f1tenth_1')
        self.declare_parameter('publish_period', 1.0)

        self.frame_id = self.get_parameter('frame_id').value
        self.vehicle_frame = self.get_parameter('vehicle_frame').value
        csv_path = Path(self.get_parameter('csv_path').value).expanduser()
        self.waypoints = self._load_waypoints(csv_path)

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.path_publisher = self.create_publisher(
            PathMessage, self.get_parameter('path_topic').value, qos
        )
        self.marker_publisher = self.create_publisher(
            MarkerArray, self.get_parameter('marker_topic').value, qos
        )

        self.path_message = self._create_path()
        self.marker_message = self._create_markers()
        period = float(self.get_parameter('publish_period').value)
        if period <= 0.0:
            raise ValueError('publish_period debe ser mayor que cero.')
        self.timer = self.create_timer(period, self.publish_route)
        self.publish_route()

        self.get_logger().info(
            f'Ruta D* lista: {len(self.waypoints)} waypoints de {csv_path}'
        )

    @staticmethod
    def _load_waypoints(csv_path):
        if not csv_path.is_file():
            raise FileNotFoundError(f'No existe el CSV de waypoints: {csv_path}')

        waypoints = []
        with csv_path.open(newline='', encoding='utf-8') as csv_file:
            reader = csv.DictReader(csv_file)
            if not reader.fieldnames or not {'x', 'y'}.issubset(reader.fieldnames):
                raise ValueError('El CSV debe contener las columnas x,y.')
            for row_number, row in enumerate(reader, start=2):
                try:
                    waypoints.append((float(row['x']), float(row['y'])))
                except (TypeError, ValueError) as error:
                    raise ValueError(
                        f'Waypoint invalido en la fila {row_number}: {row}'
                    ) from error

        if len(waypoints) < 2:
            raise ValueError('La ruta necesita al menos dos waypoints.')
        return waypoints

    def _yaw_at(self, index):
        if index < len(self.waypoints) - 1:
            x0, y0 = self.waypoints[index]
            x1, y1 = self.waypoints[index + 1]
        else:
            x0, y0 = self.waypoints[index - 1]
            x1, y1 = self.waypoints[index]
        return math.atan2(y1 - y0, x1 - x0)

    def _create_path(self):
        message = PathMessage()
        message.header.frame_id = self.frame_id
        for index, (x, y) in enumerate(self.waypoints):
            pose = PoseStamped()
            pose.header.frame_id = self.frame_id
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.05
            yaw = self._yaw_at(index)
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
            message.poses.append(pose)
        return message

    @staticmethod
    def _set_color(marker, red, green, blue, alpha=1.0):
        marker.color.r = red
        marker.color.g = green
        marker.color.b = blue
        marker.color.a = alpha

    def _base_marker(self, marker_id, namespace, marker_type, frame_id=None):
        marker = Marker()
        marker.header.frame_id = frame_id or self.frame_id
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        return marker

    def _create_markers(self):
        markers = MarkerArray()

        waypoint_marker = self._base_marker(0, 'dstar_waypoints', Marker.SPHERE_LIST)
        waypoint_marker.scale.x = 0.11
        waypoint_marker.scale.y = 0.11
        waypoint_marker.scale.z = 0.11
        self._set_color(waypoint_marker, 0.05, 0.35, 1.0)
        waypoint_marker.points = [Point(x=x, y=y, z=0.08) for x, y in self.waypoints]
        markers.markers.append(waypoint_marker)

        for marker_id, namespace, waypoint, color in (
            (1, 'inicio', self.waypoints[0], (1.0, 0.05, 0.05)),
            (2, 'meta', self.waypoints[-1], (0.05, 0.25, 1.0)),
        ):
            marker = self._base_marker(marker_id, namespace, Marker.CUBE)
            marker.pose.position.x = waypoint[0]
            marker.pose.position.y = waypoint[1]
            marker.pose.position.z = 0.10
            marker.scale.x = 0.25
            marker.scale.y = 0.25
            marker.scale.z = 0.20
            self._set_color(marker, *color)
            markers.markers.append(marker)

        vehicle = self._base_marker(
            3, 'vehiculo_f1tenth', Marker.CUBE, frame_id=self.vehicle_frame
        )
        vehicle.frame_locked = True
        vehicle.pose.position.z = 0.10
        vehicle.scale.x = 0.45
        vehicle.scale.y = 0.25
        vehicle.scale.z = 0.15
        self._set_color(vehicle, 1.0, 0.55, 0.05, 0.9)
        markers.markers.append(vehicle)

        return markers

    def publish_route(self):
        stamp = self.get_clock().now().to_msg()
        self.path_message.header.stamp = stamp
        for pose in self.path_message.poses:
            pose.header.stamp = stamp
        for marker in self.marker_message.markers:
            marker.header.stamp = stamp
        self.path_publisher.publish(self.path_message)
        self.marker_publisher.publish(self.marker_message)


def main(args=None):
    rclpy.init(args=args)
    node = GlobalPathPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
