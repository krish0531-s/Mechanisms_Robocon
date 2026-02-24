# ==========================================================
# arena.py
# Responsible for:
# - Creating grid
# - Assigning heights
# - Computing neighbors
# - Computing entry requirements
# - Precomputing lidar signatures (all 4 orientations)
# - Customizable box placement
# ==========================================================

import random

# ----------------------------------------------------------
# GLOBAL CONSTANTS
# ----------------------------------------------------------

ROWS = 4
COLS = 3

STEP_HEIGHTS = [
    400, 200, 400,
    200, 400, 600,
    400, 600, 400,
    200, 400, 200
]

DIRECTIONS = ["N", "E", "S", "W"]
DIR_INDEX = {"N": 0, "E": 1, "S": 2, "W": 3}
INDEX_DIR = {0: "N", 1: "E", 2: "S", 3: "W"}


# ==========================================================
# 1. CREATE BASIC GRID
# ==========================================================

def create_arena_grid():
    arena = {}
    step_id = 1

    for r in range(ROWS):
        for c in range(COLS):
            arena[step_id] = {
                "row": r,
                "col": c,
                "height": STEP_HEIGHTS[step_id - 1],
                "neighbors": {}
            }
            step_id += 1

    return arena


# ==========================================================
# 2. COMPUTE NEIGHBORS
# ==========================================================

def compute_neighbors(arena):
    for step, data in arena.items():

        r = data["row"]
        c = data["col"]

        neighbors = {}

        for d in DIRECTIONS:

            nr = r
            nc = c

            if d == "N":
                nr -= 1
            elif d == "S":
                nr += 1
            elif d == "E":
                nc += 1
            elif d == "W":
                nc -= 1

            neighbor_step = None

            for s, info in arena.items():
                if info["row"] == nr and info["col"] == nc:
                    neighbor_step = s
                    break

            neighbors[d] = neighbor_step

        arena[step]["neighbors"] = neighbors


# ==========================================================
# 3. COMPUTE ENTRY REQUIREMENTS
# ==========================================================

def compute_entry_requirements(arena):
    entry_map = {}

    for step, data in arena.items():
        entry_map[step] = {}

        for direction, neighbor in data["neighbors"].items():
            if neighbor is not None:
                # To enter current step from neighbor,
                # bot must face from neighbor toward this step.
                required_facing = DIR_INDEX[direction]
                entry_map[step][neighbor] = required_facing

    return entry_map


# ==========================================================
# 4. PRECOMPUTE LIDAR SIGNATURES
# ==========================================================

def compute_lidar_signatures(arena):
    lidar_map = {}

    orientation_map = {
        0: ["N", "E", "S", "W"],
        1: ["E", "S", "W", "N"],
        2: ["S", "W", "N", "E"],
        3: ["W", "N", "E", "S"]
    }

    for step, data in arena.items():

        lidar_map[step] = {}
        current_height = data["height"]

        for facing in range(4):

            dirs = orientation_map[facing]
            labels = ["FRONT", "LEFT", "BACK", "RIGHT"]

            readings = {}

            for label, d in zip(labels, dirs):
                neighbor = data["neighbors"][d]

                if neighbor is None:
                    readings[label] = 0
                else:
                    neighbor_height = arena[neighbor]["height"]
                    readings[label] = 1 if neighbor_height > current_height else 0

            lidar_map[step][facing] = readings

    return lidar_map


# ==========================================================
# 5. BUILD STEP_MAP
# ==========================================================

def build_step_map(arena):
    entry_requirements = compute_entry_requirements(arena)
    lidar_signatures = compute_lidar_signatures(arena)

    step_map = {}

    for step in arena.keys():
        step_map[step] = {
            "height": arena[step]["height"],
            "lidar_signature": lidar_signatures[step],
            "entry_requirements": entry_requirements[step]
        }

    return step_map


# ==========================================================
# 6. CUSTOMIZABLE BOX PLACEMENT
# ==========================================================

def place_boxes(
    arena,
    r2_count=4,
    r1_count=3,
    fake_count=1,
    require_r2_on_entry=True
):
    boxes = {}

    all_steps = list(arena.keys())
    entry_row = [1, 2, 3]
    boundary_steps = [1,2,3,4,6,7,9,10,11,12]

    # ----------------------------
    # R2 Placement
    # ----------------------------

    r2_steps = []

    if require_r2_on_entry:
        entry_choice = random.choice(entry_row)
        r2_steps.append(entry_choice)

    remaining_candidates = [
        s for s in all_steps if s not in r2_steps
    ]

    while len(r2_steps) < r2_count:
        s = random.choice(remaining_candidates)
        r2_steps.append(s)
        remaining_candidates.remove(s)

    for s in r2_steps:
        boxes[s] = "R2"

    # ----------------------------
    # FAKE Placement
    # ----------------------------

    fake_candidates = [
        s for s in all_steps
        if s not in entry_row and s not in boxes
    ]

    for _ in range(fake_count):
        s = random.choice(fake_candidates)
        boxes[s] = "FAKE"
        fake_candidates.remove(s)

    # ----------------------------
    # R1 Placement (boundary only)
    # ----------------------------

    r1_candidates = [
        s for s in boundary_steps
        if s not in boxes
    ]

    for _ in range(r1_count):
        s = random.choice(r1_candidates)
        boxes[s] = "R1"
        r1_candidates.remove(s)

    return boxes


# ==========================================================
# 7. PUBLIC INITIALIZER
# ==========================================================

def initialize_arena(
    r2_count=4,
    r1_count=3,
    fake_count=1,
    require_r2_on_entry=True
):
    arena = create_arena_grid()
    compute_neighbors(arena)
    step_map = build_step_map(arena)

    boxes = place_boxes(
        arena,
        r2_count=r2_count,
        r1_count=r1_count,
        fake_count=fake_count,
        require_r2_on_entry=require_r2_on_entry
    )

    return arena, step_map, boxes