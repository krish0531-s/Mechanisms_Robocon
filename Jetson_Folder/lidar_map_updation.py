import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Int32

from arena import initialize_arena


# ----------------------------------------------------------
# INITIALIZE ARENA
# ----------------------------------------------------------

arena, step_map, boxes = initialize_arena(
    r2_count=0,
    r1_count=0,
    fake_count=0,
    require_r2_on_entry=False
)


# ----------------------------------------------------------
# MAP (step → int encoding)
# ----------------------------------------------------------

map_data = {i: 0 for i in range(1, 13)}  # 1–12 steps


# ----------------------------------------------------------
# HELPER: get step from (row, col)
# ----------------------------------------------------------

def get_step_from_position(row, col):
    for step, data in arena.items():
        if data["row"] == row and data["col"] == col:
            return step
    return None


# ----------------------------------------------------------
# RESOLVE STEP FROM CAMERA (UNCHANGED)
# ----------------------------------------------------------

def resolve_box_steps(current_step,
                     camera_front, front_type,
                     camera_left, left_type,
                     camera_right, right_type):

    detected_steps = []

    row = arena[current_step]["row"]
    col = arena[current_step]["col"]

    if camera_front:
        step = get_step_from_position(row + 1, col)
        if step is not None:
            detected_steps.append((step, front_type))

    if camera_left:
        step = get_step_from_position(row, col - 1)
        if step is not None:
            detected_steps.append((step, left_type))

    if camera_right:
        step = get_step_from_position(row, col + 1)
        if step is not None:
            detected_steps.append((step, right_type))

    return detected_steps


# ----------------------------------------------------------
# ROS NODE
# ----------------------------------------------------------

class CameraSubscriber(Node):

    def __init__(self):
        super().__init__('camera_box_subscriber')

        # Camera topic
        self.subscription = self.create_subscription(
            Int32MultiArray,
            '/camera_box_detection',
            self.listener_callback,
            10
        )

        # LiDAR step topic
        self.step_subscription = self.create_subscription(
            Int32,
            '/estimated_step',
            self.step_callback,
            10
        )

        self.current_step = None

    # ------------------------------------------------------

    def step_callback(self, msg):
        self.current_step = msg.data

    # ------------------------------------------------------

    def decode_type(self, t):
        if t == 1:
            return 1   # R1
        elif t == 2:
            return 2   # R2
        elif t == 3:
            return 3   # FAKE
        else:
            return 0   # NONE

    # ------------------------------------------------------

    def listener_callback(self, msg):

        if self.current_step is None:
            return

        data = msg.data

        # Camera parsing (UNCHANGED structure)
        camera_front = bool(data[1])
        front_type = self.decode_type(data[2])

        camera_left = bool(data[3])
        left_type = self.decode_type(data[4])

        camera_right = bool(data[5])
        right_type = self.decode_type(data[6])

        predicted_steps = resolve_box_steps(
            self.current_step,
            camera_front, front_type,
            camera_left, left_type,
            camera_right, right_type
        )

        if len(predicted_steps) == 0:
            return

        # --------------------------------------------------
        # UPDATE DICTIONARY MAP
        # --------------------------------------------------

        for step, box_type in predicted_steps:
            map_data[step] = box_type

            self.get_logger().info(
                f"Detected box at step {step} → type {box_type}"
            )

        # --------------------------------------------------
        # PRINT FULL MAP
        # --------------------------------------------------

        self.get_logger().info(f"Map: {map_data}")


# ----------------------------------------------------------
# MAIN
# ----------------------------------------------------------

def main(args=None):

    rclpy.init(args=args)

    node = CameraSubscriber()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()