#!/usr/bin/env python3

from arena import initialize_arena
from map_update import MapUpdater


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

current_step = None
facing = 2  # 0=N,1=E,2=S,3=W (keep same as your system)


# ----------------------------------------------------------
# HELPER: get step from (row, col)
# ----------------------------------------------------------

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
# INPUT HELPERS
# ----------------------------------------------------------

def decode_type(t):
    if t == 1:
        return "R1"
    elif t == 2:
        return "R2"
    elif t == 3:
        return "FAKE"
    else:
        return None


def get_camera_input():
    print("\nEnter camera data:")

    front_detect = int(input("Front detect (0/1): "))
    front_type = decode_type(int(input("Front type (1=R1,2=R2,3=FAKE,0=None): ")))

    left_detect = int(input("Left detect (0/1): "))
    left_type = decode_type(int(input("Left type (1=R1,2=R2,3=FAKE,0=None): ")))

    right_detect = int(input("Right detect (0/1): "))
    right_type = decode_type(int(input("Right type (1=R1,2=R2,3=FAKE,0=None): ")))

    return (
        bool(front_detect), front_type,
        bool(left_detect), left_type,
        bool(right_detect), right_type
    )


# ----------------------------------------------------------
# MAIN LOOP
# ----------------------------------------------------------

def main():
    global current_step

    print("=== Manual Map Testing ===")

    while True:

        # Step input (simulate LiDAR)
        step_input = input("\nEnter current step (or 'q' to quit): ")
        if step_input == 'q':
            break

        current_step = int(step_input)

        # Camera input
        (
            camera_front, front_type,
            camera_left, left_type,
            camera_right, right_type
        ) = get_camera_input()

        # Resolve steps
        predicted_steps = resolve_box_steps(
            current_step,
            camera_front, front_type,
            camera_left, left_type,
            camera_right, right_type
        )

        # Update map
        if len(predicted_steps) == 0:
            print("No detections")
            continue

        for step, box_type in predicted_steps:
            map_updater.update_box(step, box_type)

        # Print map
        current_map = map_updater.get_map()

        print("\n--- MAP STATE ---")
        print(f"Current Step: {current_step}")
        print(current_map)


# ----------------------------------------------------------

if __name__ == "__main__":
    main()