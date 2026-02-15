import random
import matplotlib.pyplot as plt
import matplotlib.patches as patches

# =====================================
# ARENA CONFIG
# =====================================

ROWS = 4
COLS = 3

step_heights = [
    400, 200, 400,
    200, 400, 600,
    400, 600, 400,
    200, 400, 200
]

arena = {}
step_number = 1
for r in range(ROWS):
    for c in range(COLS):
        arena[step_number] = {
            "row": r,
            "col": c,
            "height": step_heights[step_number - 1]
        }
        step_number += 1

# =====================================
# BOX PLACEMENT RULES
# =====================================

entry_row = [1,2,3]
middle_rows = [4,5,6,7,8,9]
exit_row = [10,11,12]

boxes = {}

real_boxes = [random.choice(entry_row)]
real_boxes += random.sample(middle_rows, 3)

fake_box = random.choice([s for s in middle_rows if s not in real_boxes])

for r in real_boxes:
    boxes[r] = "real"

boxes[fake_box] = "fake"

# =====================================
# DIRECTION SYSTEM
# =====================================

# 0 = North, 1 = East, 2 = South, 3 = West
directions = ["N", "E", "S", "W"]

def rotate_left(d): return (d - 1) % 4
def rotate_right(d): return (d + 1) % 4

# =====================================
# MOVEMENT HELPERS
# =====================================

def get_adjacent(step, dir_symbol):
    r = arena[step]["row"]
    c = arena[step]["col"]

    if dir_symbol == "N": r -= 1
    if dir_symbol == "S": r += 1
    if dir_symbol == "E": c += 1
    if dir_symbol == "W": c -= 1

    for s, data in arena.items():
        if data["row"] == r and data["col"] == c:
            return s
    return None

# =====================================
# LIDAR SIMULATION (ORIENTATION DEPENDENT)
# =====================================

def simulate_lidar(step, facing):
    current_height = arena[step]["height"]

    # relative directions based on facing
    mapping = {
        0: ["N","E","S","W"],   # facing north
        1: ["E","S","W","N"],   # east
        2: ["S","W","N","E"],   # south
        3: ["W","N","E","S"]    # west
    }

    dirs = mapping[facing]

    labels = ["FRONT", "LEFT", "BACK", "RIGHT"]
    readings = {}

    for label, dir_symbol in zip(labels, dirs):
        adj = get_adjacent(step, dir_symbol)

        if adj is None:
            readings[label] = False
        else:
            readings[label] = arena[adj]["height"] > current_height

    return readings


def identify_step_from_lidar(lidar_readings, facing):
    candidates = []
    for s in arena.keys():
        if simulate_lidar(s, facing) == lidar_readings:
            candidates.append(s)
    return candidates

# def identify_step_lidar_height(
#     lidar_readings,
#     facing,
#     last_height_diff,
#     previous_estimated_step
# ):
#     candidates = []

#     for s in arena.keys():

#         # 1️⃣ Lidar match
#         if simulate_lidar(s, facing) != lidar_readings:
#             continue

#         # 2️⃣ First iteration
#         if previous_estimated_step is None:
#             candidates.append(s)
#             continue

#         # 3️⃣ Must be reachable from previous step
#         if not is_neighbor(previous_estimated_step, s):
#             continue


#         if previous_estimated_step is not None: 
#             valid_transition = False
#             if get_adjacent(previous_estimated_step, dir_symbol) == s:
#                 for dir_symbol in directions:
#                 # enforce entry direction consistency
    
#                     if entry_direction is None or dir_symbol == entry_direction:
                
#                         valid_transition = True
    
#         if not valid_transition:
#             continue


#         # 4️⃣ Height difference must match transition
#         expected_diff = (
#             arena[s]["height"]
#             - arena[previous_estimated_step]["height"]
#         )

#         if expected_diff == last_height_diff:
#             candidates.append(s)

#     return candidates

def identify_step_lidar_height(
    lidar_readings,
    facing,
    last_height_diff,
    previous_estimated_step
):
    candidates = []

    for s in arena.keys():

        # 1️⃣ Lidar match
        if simulate_lidar(s, facing) != lidar_readings:
            continue

        # 2️⃣ First iteration
        if previous_estimated_step is None:
            candidates.append(s)
            continue

        valid_transition = False

        # 3️⃣ Check all directions from previous step
        for dir_symbol in directions:

            if get_adjacent(previous_estimated_step, dir_symbol) == s:

                # Enforce entry direction consistency
                if entry_direction is None or dir_symbol == entry_direction:

                    expected_diff = (
                        arena[s]["height"]
                        - arena[previous_estimated_step]["height"]
                    )

                    if expected_diff == last_height_diff:
                        valid_transition = True

        if valid_transition:
            candidates.append(s)

    return candidates


def is_neighbor(step_a, step_b):
    for dir_symbol in directions:
        if get_adjacent(step_a, dir_symbol) == step_b:
            return True
    return False



# =====================================
# GRAPHICS
# =====================================

fig, ax = plt.subplots()
plt.ion()

def height_color(h):
    if h == 200: return "#A8D5BA"
    if h == 400: return "#5DA9E9"
    if h == 600: return "#F25F5C"

def draw(step, facing, lidar_readings, estimated_steps, collected):
    ax.clear()

    # Draw arena
    for s, data in arena.items():
        x = data["col"]
        y = ROWS - 1 - data["row"]

        rect = patches.Rectangle((x, y), 1, 1,
                                 facecolor=height_color(data["height"]),
                                 edgecolor="black")
        ax.add_patch(rect)

        ax.text(x+0.5, y+0.85, str(s),
                ha='center', color='white')

        if s in boxes:
            color = "yellow" if boxes[s] == "real" else "black"
            box = patches.Rectangle((x+0.25, y+0.25),
                                    0.5, 0.5,
                                    facecolor=color)
            ax.add_patch(box)

    # Draw bot
    data = arena[step]
    bx = data["col"] + 0.5
    by = ROWS - 1 - data["row"] + 0.5

    bot = patches.Rectangle((bx-0.1, by-0.1),
                            0.2, 0.2,
                            color="red")
    ax.add_patch(bot)

    # Direction arrow
    dir_vectors = [(0,0.3),(0.3,0),(0,-0.3),(-0.3,0)]
    dx, dy = dir_vectors[facing]
    ax.arrow(bx, by, dx, dy, head_width=0.05, color="black")

    info = (
        f"Facing: {directions[facing]}\n"
        f"True Step: {step}\n"
        f"Lidar: {lidar_readings}\n"
        f"Lidar Only: {estimated_steps}\n"
        f"Lidar + Height: {estimated_steps_lidar_height}\n"
        f"Collected: {collected}\n"
        f"Last Height Change: {last_height_diff}\n"
        f"Visited: {visited_steps}\n"


    )

    ax.text(3.2, 2.5, info, fontsize=10, verticalalignment='top')

    ax.set_xlim(0, 5)
    ax.set_ylim(0, ROWS)
    ax.set_aspect("equal")
    ax.set_xticks([])
    ax.set_yticks([])

    plt.draw()
    plt.pause(7)

# =====================================
# SIMULATION LOOP
# =====================================
last_height_diff = 0

previous_estimated_step = None
visited_steps = []
entry_direction = None


true_step = random.choice(entry_row)
facing = 2  # start facing south
collected = 0

for _ in range(15):
    previous_step = true_step

    lidar_readings = simulate_lidar(true_step, facing)
    estimated_steps = identify_step_from_lidar(lidar_readings, facing)

    estimated_steps_lidar_height = identify_step_lidar_height(
        lidar_readings,
        facing,
        last_height_diff,
        previous_estimated_step
    )

    if estimated_steps_lidar_height:
        current_estimate = estimated_steps_lidar_height[0]
    else:
        current_estimate = None
    
    if current_estimate is not None:
        visited_steps.append(current_estimate)
        previous_estimated_step = current_estimate



    if estimated_steps_lidar_height:
        previous_estimated_step = estimated_steps_lidar_height[0]
        if previous_estimated_step is not None and current_estimate is not None:
            for dir_symbol in directions:
                if get_adjacent(previous_estimated_step, dir_symbol) == current_estimate:
                    entry_direction = dir_symbol
                    break



    draw(true_step, facing, lidar_readings, estimated_steps, collected)

    # Collect real box
    if true_step in boxes and boxes[true_step] == "real":
        collected += 1
        del boxes[true_step]

    # Decide movement
    moved = False

    # Check all directions for real box
    for i, dir_symbol in enumerate(directions):
        adj = get_adjacent(true_step, dir_symbol)
        if adj and boxes.get(adj) == "real":
            facing = i
            true_step = adj
            moved = True
            break

    # Avoid fake
    if not moved:
        forward_step = get_adjacent(true_step, directions[facing])
        if forward_step and boxes.get(forward_step) != "fake":
            true_step = forward_step
            moved = True
    new_height = arena[true_step]["height"]
    last_height_diff = new_height - arena[previous_step]["height"]


    if not moved:
        break

