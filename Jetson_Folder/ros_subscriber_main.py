#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

from arena import initialize_arena
from arena_render import ArenaRenderer
from map_update import MapUpdater

import matplotlib.pyplot as plt


# ----------------------------------------------------------
# INITIALIZE ARENA
# ----------------------------------------------------------

arena, step_map, boxes = initialize_arena(
    r2_count=0,
    r1_count=0,
    fake_count=0,
    require_r2_on_entry=False
)

map_updater = MapUpdater()
renderer = ArenaRenderer(rows=4, cols=3)


# ----------------------------------------------------------
# HELPER: get step from (row, col)
# ---------------------------------f------------------------

def get_step_from_position(row, col):
    for step, data in arena.items():
        if data["row"] == row and data["col"] == col:
            return step
    return None


# ----------------------------------------------------------
# RESOLVE STEP FROM CAMERA
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

        self.subscription = self.create_subscription(
            Int32MultiArray,
            '/camera_box_detection',
            self.listener_callback,
            10
        )

    def decode_type(self, t):

        if t == 1:
            return "R1"
        elif t == 2:
            return "R2"
        elif t == 3:
            return "FAKE"
        else:
            return None

    def listener_callback(self, msg):

        data = msg.data

        current_step = data[0]

        camera_front = bool(data[1])
        front_type = self.decode_type(data[2])

        camera_left = bool(data[3])
        left_type = self.decode_type(data[4])

        camera_right = bool(data[5])
        right_type = self.decode_type(data[6])

        predicted_steps = resolve_box_steps(
            current_step,
            camera_front, front_type,
            camera_left, left_type,
            camera_right, right_type
        )

        if len(predicted_steps) == 0:
            return

        for step, box_type in predicted_steps:
            map_updater.update_box(step, box_type)

        global boxes
        boxes = map_updater.get_map()

        bot_state = {
            "current_step": current_step,
            "facing": 2
        }

        renderer.draw(arena, boxes, bot_state)
        plt.draw()


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