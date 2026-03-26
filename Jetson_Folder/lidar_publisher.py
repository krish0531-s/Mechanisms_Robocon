#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32, Int32

# from YDLIDAR_SDM15_python.SDM15 import SDM15, BaudRate
from YDLIDAR_SDM15_python.SDM15 import SDM15, BaudRate


from arena import initialize_arena
from step_estimator import estimate_lidar_only, estimate_full


# ==========================================================
# NODE
# ==========================================================

class LidarStepEstimator(Node):

    def __init__(self):
        super().__init__('lidar_step_estimator')

        # --------------------------------------------------
        # LiDAR Publishers (for debugging / monitoring)
        # --------------------------------------------------

        self.pub_left  = self.create_publisher(Float32, '/sdm15/distance_left', 10)
        self.pub_back  = self.create_publisher(Float32, '/sdm15/distance_back', 10)
        self.pub_front = self.create_publisher(Float32, '/sdm15/distance_front', 10)
        self.pub_right = self.create_publisher(Float32, '/sdm15/distance_right', 10)

        # --------------------------------------------------
        # Step Estimation Publisher
        # --------------------------------------------------

        self.pub_step = self.create_publisher(Int32, '/estimated_step', 10)

        # --------------------------------------------------
        # LiDAR Devices
        # --------------------------------------------------

        self.lidar_left  = SDM15("/dev/ttyUSB3", BaudRate.BAUD_460800)
        self.lidar_back  = SDM15("/dev/ttyUSB2", BaudRate.BAUD_460800)
        self.lidar_front = SDM15("/dev/ttyUSB0", BaudRate.BAUD_460800)
        self.lidar_right = SDM15("/dev/ttyUSB1", BaudRate.BAUD_460800)

        self.init_lidar(self.lidar_left,  "LEFT")
        self.init_lidar(self.lidar_back,  "BACK")
        self.init_lidar(self.lidar_front, "FRONT")
        self.init_lidar(self.lidar_right, "RIGHT")

        # --------------------------------------------------
        # Arena + Step Map
        # --------------------------------------------------

        self.arena, self.step_map, _ = initialize_arena(
            r2_count=0,
            r1_count=0,
            fake_count=0,
            require_r2_on_entry=False
        )

        # --------------------------------------------------
        # State (for full estimation)
        # --------------------------------------------------

        self.previous_step = None
        self.height_diff = 0

        # Facing (placeholder → will come from BNO later)
        self.facing = 2   # 0=N,1=E,2=S,3=W

        # --------------------------------------------------
        # Timer
        # --------------------------------------------------

        self.timer = self.create_timer(0.1, self.loop)

        self.get_logger().info("LiDAR Step Estimator Started")

    # ======================================================
    # INITIALIZE LIDAR
    # ======================================================

    def init_lidar(self, lidar, name):
        try:
            lidar.obtain_version_info()
            lidar.lidar_self_test()
            lidar.start_scan()
            self.get_logger().info(f"{name} LiDAR started")
        except Exception as e:
            self.get_logger().error(f"{name} LiDAR init failed: {e}")

    # ======================================================
    # DISTANCE → BINARY
    # ======================================================

    # def distance_to_binary(self, dist, threshold=300):
    #     """
    #     Convert distance to binary:
    #     1 → obstacle (higher step)
    #     0 → free / same or lower
    #     """
    #     if dist is None:
    #         return 0
    #     return 1 if dist < threshold else 0
    def distance_to_binary(self, dist):
        """
        1 → step detected (< 500)
        0 → no step detected (>= 500)
    """
        if dist is None:
            return 0

        return 1 if dist < 500 else 0

    # ======================================================
    # MAIN LOOP
    # ======================================================

    def loop(self):

        try:
            dist_left,  _, _ = self.lidar_left.get_distance()
            dist_back,  _, _ = self.lidar_back.get_distance()
            dist_front, _, _ = self.lidar_front.get_distance()
            dist_right, _, _ = self.lidar_right.get_distance()

        except Exception as e:
            self.get_logger().warn(f"LiDAR read failed: {e}")
            return

        # --------------------------------------------------
        # Publish raw distances (optional debug)
        # --------------------------------------------------

        self.publish_distance(self.pub_left,  dist_left)
        self.publish_distance(self.pub_back,  dist_back)
        self.publish_distance(self.pub_front, dist_front)
        self.publish_distance(self.pub_right, dist_right)

        # --------------------------------------------------
        # Build LIDAR SIGNATURE
        # --------------------------------------------------

        lidar_readings = {
            "FRONT": self.distance_to_binary(dist_front),
            "LEFT":  self.distance_to_binary(dist_left),
            "BACK":  self.distance_to_binary(dist_back),
            "RIGHT": self.distance_to_binary(dist_right),
        }

        # --------------------------------------------------
        # STEP ESTIMATION
        # --------------------------------------------------

        lidar_candidates = estimate_lidar_only(
            self.step_map,
            lidar_readings,
            self.facing
        )

        full_candidates = estimate_full(
            self.step_map,
            lidar_readings,
            self.facing,
            self.previous_step,
            self.height_diff
        )

        # Prefer full estimation if available
        candidates = full_candidates if full_candidates else lidar_candidates

        # --------------------------------------------------
        # FINAL DECISION
        # --------------------------------------------------

        if len(candidates) == 1:

            estimated_step = candidates[0]

            # Publish result
            msg = Int32()
            msg.data = estimated_step
            self.pub_step.publish(msg)

            self.get_logger().info(
                f"Step: {estimated_step} | Lidar: {lidar_readings}"
            )

            # Update state
            if self.previous_step is not None:
                self.height_diff = (
                    self.arena[estimated_step]["height"] -
                    self.arena[self.previous_step]["height"]
                )

            self.previous_step = estimated_step

        else:
            self.get_logger().warn(f"Ambiguous: {candidates}")

    # ======================================================
    # HELPER
    # ======================================================

    def publish_distance(self, publisher, value):
        msg = Float32()
        msg.data = float(value)
        publisher.publish(msg)


# ==========================================================
# MAIN
# ==========================================================

def main(args=None):
    rclpy.init(args=args)

    node = LidarStepEstimator()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()