import math

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
)

from geometry_msgs.msg import PoseArray
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.msg import CostmapFilterInfo


class CrowdKeepoutMaskPublisher(Node):
    def __init__(self):
        super().__init__('crowd_keepout_mask_publisher')

        # hot_place 주변 keepout 반경 [m]
        self.keepout_radius = 0.4

        self.map_msg = None
        self.hot_places = []

        # 같은 map을 반복 수신했을 때 불필요하게 다시 발행하지 않기 위한 상태
        self.initial_mask_published = False

        self.latched_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        # =========================
        # 구독
        # =========================
        self.create_subscription(
            OccupancyGrid,
            '/robot1/map',
            self.map_callback,
            self.latched_qos,
        )

        self.create_subscription(
            PoseArray,
            '/hot_place',
            self.hot_place_callback,
            10,
        )

        # =========================
        # 발행
        # =========================
        self.mask_pub = self.create_publisher(
            OccupancyGrid,
            '/robot1/keepout_filter_mask',
            self.latched_qos,
        )

        self.info_pub = self.create_publisher(
            CostmapFilterInfo,
            '/robot1/keepout_costmap_filter_info',
            self.latched_qos,
        )

        # FilterInfo는 크기가 작으므로 노드 시작 시 한 번 발행
        self.publish_filter_info()

        self.get_logger().info(
            'Crowd keepout mask publisher started'
        )

        self.get_logger().info(
            'Keepout mask is published only when map or hot_place changes.'
        )

    def map_callback(self, msg):
        """
        map을 처음 받으면 빈 keepout mask를 한 번 발행한다.

        이 빈 mask가 TRANSIENT_LOCAL로 유지되기 때문에
        hot_place가 없어도 Nav2 Keepout Filter에서 WARN이 발생하지 않는다.
        """
        self.map_msg = msg

        if not self.initial_mask_published:
            self.publish_filter_info()
            self.publish_mask()

            self.initial_mask_published = True

            self.get_logger().info(
                'Initial empty keepout mask published.'
            )

    def hot_place_callback(self, msg):
        """
        hot_place가 들어왔을 때만 새로운 mask를 발행한다.

        PoseArray가 비어 있으면 빈 mask가 발행되어
        기존 keepout 영역이 제거된다.
        """
        self.hot_places = list(msg.poses)

        if self.map_msg is None:
            self.get_logger().warn(
                'Received /hot_place, but /robot1/map is not available yet.'
            )
            return

        if len(self.hot_places) == 0:
            self.get_logger().info(
                'Received empty hot_place PoseArray. '
                'Clearing keepout zones.'
            )
        else:
            positions_text = ', '.join(
                f'({pose.position.x:.2f}, '
                f'{pose.position.y:.2f})'
                for pose in self.hot_places
            )

            self.get_logger().info(
                f'Received {len(self.hot_places)} hot place(s): '
                f'{positions_text}'
            )

        # hot_place가 변경된 시점에만 발행
        self.publish_filter_info()
        self.publish_mask()

    def publish_filter_info(self):
        """
        Nav2 Keepout Filter에 mask 토픽 정보를 제공한다.

        TRANSIENT_LOCAL이므로 한 번 발행한 마지막 메시지가 유지된다.
        """
        info = CostmapFilterInfo()

        info.header.stamp = self.get_clock().now().to_msg()

        if self.map_msg is not None:
            info.header.frame_id = self.map_msg.header.frame_id
        else:
            info.header.frame_id = 'map'

        info.type = 0
        info.filter_mask_topic = '/robot1/keepout_filter_mask'
        info.base = 0.0
        info.multiplier = 1.0

        self.info_pub.publish(info)

    def world_to_grid(self, x, y):
        origin_x = self.map_msg.info.origin.position.x
        origin_y = self.map_msg.info.origin.position.y
        resolution = self.map_msg.info.resolution

        gx = int(math.floor((x - origin_x) / resolution))
        gy = int(math.floor((y - origin_y) / resolution))

        return gx, gy

    def draw_keepout_circle(
        self,
        data,
        width,
        height,
        resolution,
        center_x,
        center_y,
    ):
        center_gx, center_gy = self.world_to_grid(
            center_x,
            center_y,
        )

        radius_cell = max(
            1,
            int(math.ceil(self.keepout_radius / resolution)),
        )

        processed_cells = 0

        for gy in range(
            center_gy - radius_cell,
            center_gy + radius_cell + 1,
        ):
            for gx in range(
                center_gx - radius_cell,
                center_gx + radius_cell + 1,
            ):
                if gx < 0 or gy < 0:
                    continue

                if gx >= width or gy >= height:
                    continue

                # 셀 중심 위치를 실제 map 좌표로 변환해 거리 계산
                cell_x = (
                    self.map_msg.info.origin.position.x
                    + (gx + 0.5) * resolution
                )

                cell_y = (
                    self.map_msg.info.origin.position.y
                    + (gy + 0.5) * resolution
                )

                distance = math.hypot(
                    cell_x - center_x,
                    cell_y - center_y,
                )

                if distance <= self.keepout_radius:
                    grid_index = gy * width + gx
                    data[grid_index] = 100
                    processed_cells += 1

        return center_gx, center_gy, processed_cells

    def publish_mask(self):
        """
        현재 hot_places 상태에 해당하는 mask를 한 번 발행한다.

        hot_places가 비어 있으면 모든 값이 0인 빈 mask를 발행한다.
        """
        if self.map_msg is None:
            return

        width = self.map_msg.info.width
        height = self.map_msg.info.height
        resolution = self.map_msg.info.resolution

        mask = OccupancyGrid()

        mask.header.stamp = self.get_clock().now().to_msg()
        mask.header.frame_id = self.map_msg.header.frame_id
        mask.info = self.map_msg.info

        # 기본 빈 mask
        data = [0] * (width * height)

        result_texts = []

        for index, pose in enumerate(self.hot_places):
            hot_x = pose.position.x
            hot_y = pose.position.y

            center_gx, center_gy, cell_count = (
                self.draw_keepout_circle(
                    data=data,
                    width=width,
                    height=height,
                    resolution=resolution,
                    center_x=hot_x,
                    center_y=hot_y,
                )
            )

            result_texts.append(
                f'#{index + 1}: '
                f'map=({hot_x:.2f}, {hot_y:.2f}), '
                f'grid=({center_gx}, {center_gy}), '
                f'cells={cell_count}'
            )

        mask.data = data
        self.mask_pub.publish(mask)

        actual_keepout_cells = sum(
            1 for value in data if value == 100
        )

        if len(self.hot_places) == 0:
            self.get_logger().info(
                'Empty keepout mask published.'
            )
        else:
            self.get_logger().info(
                f'Keepout mask published: '
                f'hot_place_count={len(self.hot_places)}, '
                f'radius={self.keepout_radius:.2f}m, '
                f'keepout_cells={actual_keepout_cells}, '
                f'positions=[{"; ".join(result_texts)}]'
            )


def main(args=None):
    rclpy.init(args=args)

    node = CrowdKeepoutMaskPublisher()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
