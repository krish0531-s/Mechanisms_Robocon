# ==========================================================
# main.py
# Glue layer only
# ==========================================================

from arena import initialize_arena
from arena_render import ArenaRenderer
from dstar_lite import DStarLite, CostModel
from step_estimator import estimate_lidar_only, estimate_full

import matplotlib.pyplot as plt

plt.ion()
# ==========================================================
# INITIALIZE ARENA
# ==========================================================

arena, step_map, boxes = initialize_arena(
    r2_count=4,
    r1_count=3,
    fake_count=1,
    require_r2_on_entry=True
)

# ==========================================================
# BOT STATE
# ==========================================================

bot_state = {
    "current_step": 1,
    "facing": 2   # 0=N,1=E,2=S,3=W
}

previous_step = None
height_diff = 0
time_taken = 0


# ==========================================================
# COST MODEL (R1 + FAKE = obstacles)
# ==========================================================

cost_model = CostModel()

def apply_obstacles():
    for step, box_type in boxes.items():
        if box_type in ["R1", "FAKE"]:
            cost_model.set_node_cost(step, float("inf"))
        else:
            cost_model.set_node_cost(step, 0)

apply_obstacles()


# ==========================================================
# D* LITE PLANNER
# ==========================================================

goals = [10, 11, 12]

planner = DStarLite(
    graph=arena,
    start=bot_state["current_step"],
    goals=goals,
    cost_model=cost_model
)

planner.compute_shortest_path()


# ==========================================================
# RENDERER
# ==========================================================

renderer = ArenaRenderer(rows=4, cols=3)



# ==========================================================
# CLICK HANDLER (REMOVE R1)
# ==========================================================

# def handle_click(row, col):

#     for step, data in arena.items():
#         if data["row"] == row and data["col"] == col:

#             if step in boxes and boxes[step] == "R1":

#                 del boxes[step]

#                 apply_obstacles()
#                 planner.notify_cost_change(step)
#                 planner.compute_shortest_path()

#                 print(f"Removed R1 at step {step}")

#             break

#     # Force redraw after click
#     renderer.draw(arena, boxes, bot_state)

def handle_click(row, col):

    for step, data in arena.items():

        if data["row"] == row and data["col"] == col:

            if step in boxes and boxes[step] == "R1":

                # Remove box visually
                del boxes[step]

                # Restore traversal cost
                cost_model.set_node_cost(step, 0)

                # Notify planner
                planner.notify_cost_change(step)
                planner.compute_shortest_path()

                print(f"Removed R1 at step {step}")

            break

    # Always redraw
    renderer.draw(arena, boxes, bot_state)
    
renderer.register_click_handler(handle_click)

# ==========================================================
# MAIN LOOP
# ==========================================================

while True:

    # ------------------------------
    # ESTIMATION (Independent System)
    # ------------------------------

    lidar = step_map[bot_state["current_step"]]["lidar_signature"][bot_state["facing"]]

    lidar_est = estimate_lidar_only(
        step_map,
        lidar,
        bot_state["facing"]
    )

    full_est = estimate_full(
        step_map,
        lidar,
        bot_state["facing"],
        previous_step,
        height_diff
    )

    # ------------------------------
    # DISPLAY
    # ------------------------------

    info = {
        "True Step": bot_state["current_step"],
        "Facing": bot_state["facing"],
        "Lidar": lidar,
        "Lidar Only": lidar_est,
        "Full Estimate": full_est,
        "Time Taken": time_taken
    }

    renderer.draw(arena, boxes, bot_state, info)
    plt.pause(1.0)

    # ------------------------------
    # PLANNER MOVE
    # ------------------------------

    next_step = planner.get_next_step()

    if next_step is None:
        print("No path available.")
        break

    if next_step in goals:
        bot_state["current_step"] = next_step
        renderer.draw(arena, boxes, bot_state, info)
        print("Reached goal.")
        break

    # Height difference calculation
    previous_step = bot_state["current_step"]
    new_height = arena[next_step]["height"]
    old_height = arena[previous_step]["height"]

    height_diff = new_height - old_height

    # Time update for height
    if height_diff > 0:
        time_taken += 5
    elif height_diff < 0:
        time_taken += 8

    # Move bot
    bot_state["current_step"] = next_step

    planner.move_start(next_step)
    planner.compute_shortest_path()

    plt.pause(1.0)


# renderer.show()
plt.ioff()
plt.show(block=True)