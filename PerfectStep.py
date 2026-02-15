import random
import matplotlib.pyplot as plt
import matplotlib.patches as patches

# ==========================================================
# 1️⃣ ARENA CREATION
# ==========================================================

last_picked_step = None

time_taken = 0


def create_arena():

    step_heights = [
        400, 200, 400,
        200, 400, 600,
        400, 600, 400,
        200, 400, 200
    ]

    arena = {}
    step_number = 1

    for r in range(4):
        for c in range(3):
            arena[step_number] = {
                "row": r,
                "col": c,
                "height": step_heights[step_number - 1]
            }
            step_number += 1

    return arena


# ==========================================================
# 2️⃣ BOX PLACEMENT RULES
# ==========================================================

def place_boxes(arena):

    entry_row = [1,2,3]
    exit_row = [10,11,12]
    boundary = [1,2,3,4,6,7,9,10,11,12]
    all_steps = list(arena.keys())

    boxes = {}

    # ---- 4 R2 ----
    r2 = [random.choice(entry_row)]
    remaining = [s for s in all_steps if s not in r2]
    r2 += random.sample(remaining, 3)
    for s in r2:
        boxes[s] = "R2"

    # ---- FAKE ----
    fake_candidates = [s for s in all_steps if s not in entry_row and s not in boxes]
    fake = random.choice(fake_candidates)
    boxes[fake] = "FAKE"

    # ---- R1 ----
    r1_candidates = [s for s in boundary if s not in boxes]
    r1 = random.sample(r1_candidates, 3)
    for s in r1:
        boxes[s] = "R1"

    return boxes


# ==========================================================
# 3️⃣ MOVEMENT UTILITIES
# ==========================================================

directions = ["N","E","S","W"]

def turn_to(current_facing, new_direction):
    global time_taken

    if new_direction != current_facing:

        diff = abs(new_direction - current_facing)

        if diff == 3:
            diff = 1

        time_taken += 2 * diff

    return new_direction



def get_adjacent(arena, step, direction):

    r = arena[step]["row"]
    c = arena[step]["col"]

    if direction == "N": r -= 1
    if direction == "S": r += 1
    if direction == "E": c += 1
    if direction == "W": c -= 1

    for s,data in arena.items():
        if data["row"]==r and data["col"]==c:
            return s
    return None


# ==========================================================
# 4️⃣ LIDAR SIMULATION
# ==========================================================

def simulate_lidar(arena, step, facing):

    current_height = arena[step]["height"]

    mapping = {
        0:["N","E","S","W"],
        1:["E","S","W","N"],
        2:["S","W","N","E"],
        3:["W","N","E","S"]
    }

    dirs = mapping[facing]
    labels = ["FRONT","LEFT","BACK","RIGHT"]

    readings = {}

    for label,d in zip(labels,dirs):
        adj = get_adjacent(arena,step,d)
        if adj is None:
            readings[label]=0
        else:
            readings[label] = 1 if arena[adj]["height"]>current_height else 0

    return readings


# ==========================================================
# 5️⃣ STEP ESTIMATION (LIDAR ONLY)
# ==========================================================

def estimate_lidar_only(arena, lidar, facing):

    candidates=[]

    for s in arena:
        if simulate_lidar(arena,s,facing)==lidar:
            candidates.append(s)

    return candidates


# ==========================================================
# 6️⃣ STEP ESTIMATION (FULL FILTER)
# ==========================================================

def estimate_full(arena, lidar, facing, prev_step, height_diff):

    candidates=[]

    for s in arena:

        # Lidar filter
        if simulate_lidar(arena,s,facing)!=lidar:
            continue

        if prev_step is None:
            candidates.append(s)
            continue

        # Feasibility + entry direction
        for i,d in enumerate(directions):
            if get_adjacent(arena,prev_step,d)==s and i==facing:

                expected = arena[s]["height"] - arena[prev_step]["height"]
                if expected==height_diff:
                    candidates.append(s)

    return candidates


# ==========================================================
# 7️⃣ PICK ADJACENT R2
# ==========================================================

def pick_adjacent_r2(arena, boxes, step, facing):

    global last_picked_step, time_taken

    for i, d in enumerate(directions):

        adj = get_adjacent(arena, step, d)

        if adj and boxes.get(adj) == "R2":

            # Turn first
            # facing = i

            facing = turn_to(facing, i)


            # Redraw turning state
            lidar = simulate_lidar(arena, step, facing)
            est_lidar = estimate_lidar_only(arena, lidar, facing)
            est_full = estimate_full(arena, lidar, facing, prev_step, height_diff)

            draw(arena, boxes, step, facing,
                 lidar, est_lidar, est_full,
                 visited, height_diff, collected,
                 last_picked_step)

            plt.pause(0.5)

            # Pick box
            del boxes[adj]
            last_picked_step = adj
            time_taken+=5

            return facing, 1

    return facing, 0




# ==========================================================
# 8️⃣ MOVEMENT LOGIC
# ==========================================================

def move_bot(arena, boxes, step, facing):
    global time_taken

    # Try moving FORWARD first
    forward_dir = facing
    forward_step = get_adjacent(arena, step, directions[forward_dir])

    if forward_step and boxes.get(forward_step) not in ["R1", "FAKE"]:
        if forward_step and boxes.get(forward_step) not in ["R1", "FAKE"]:
            prev = step
            step = forward_step
            height_diff = arena[step]["height"] - arena[prev]["height"]

            if height_diff > 0:
                time_taken += 5
            elif height_diff < 0:
                time_taken += 8

            return step, facing, height_diff, True

    # Otherwise check LEFT, RIGHT, BACK
    for turn in [-1, 1, 2]:  # left, right, back

        new_dir = (facing + turn) % 4
        adj = get_adjacent(arena, step, directions[new_dir])

        if not adj:
            continue

        if boxes.get(adj) in ["R1", "FAKE"]:
            continue

        prev = step
        step = adj
        facing = new_dir
        height_diff = arena[step]["height"] - arena[prev]["height"]

        if height_diff > 0:
            time_taken += 5   # step up
        elif height_diff < 0:
            time_taken += 8   # step down

        return step, facing, height_diff, True

    return step, facing, 0, False



# ==========================================================
# 9️⃣ GRAPHICS
# ==========================================================

def draw(arena, boxes, step, facing,
         lidar, est_lidar, est_full,
         visited, height_diff,
         collected, last_picked_step):


    ax.clear()

    # Draw arena
    for s,data in arena.items():

        x=data["col"]
        y=3-data["row"]

        color={200:"#A8D5BA",400:"#5DA9E9",600:"#F25F5C"}[data["height"]]
        ax.add_patch(patches.Rectangle((x,y),1,1,facecolor=color,edgecolor="black"))
        ax.text(x+0.5,y+0.85,str(s),ha='center',color='white')

        if s in boxes:
            if boxes[s]=="R1": bc="blue"
            elif boxes[s]=="R2": bc="yellow"
            else: bc="black"
            ax.add_patch(patches.Rectangle((x+0.25,y+0.25),0.5,0.5,facecolor=bc))

    # Draw bot
    bx=arena[step]["col"]+0.5
    by=3-arena[step]["row"]+0.5
    ax.add_patch(patches.Rectangle((bx-0.1,by-0.1),0.2,0.2,color="red"))

    dx,dy=[(0,0.3),(0.3,0),(0,-0.3),(-0.3,0)][facing]
    ax.arrow(bx,by,dx,dy,head_width=0.05,color="black")

    # Print information on screen
    info = (
    f"TRUE STEP: {step}\n"
    f"Facing: {directions[facing]}\n\n"
    f"Lidar Values:\n{lidar}\n\n"
    f"Height Difference: {height_diff}\n\n"
    f"Estimation (Lidar Only):\n{est_lidar}\n\n"
    f"Estimation (Full Filtered):\n{est_full}\n\n"
    f"Visited Steps:\n{visited}\n\n"
    f"R2 Collected: {collected}\n"
    f"Last Picked From: {last_picked_step}"
)


    ax.text(3.2,2.8,info,fontsize=9,verticalalignment='top')

    ax.set_xlim(0,5)
    ax.set_ylim(0,4)
    ax.set_aspect("equal")
    ax.set_xticks([])
    ax.set_yticks([])

    plt.draw()
    plt.pause(2)


# ==========================================================
# 🔟 MOUSE CLICK (REMOVE R1)
# ==========================================================

def on_click(event):
    global boxes
    if event.inaxes!=ax:
        return

    col=int(event.xdata)
    row=3-int(event.ydata)

    for s,data in arena.items():
        if data["row"]==row and data["col"]==col:
            if boxes.get(s)=="R1":
                del boxes[s]
            break


# ==========================================================
# MAIN LOOP
# ==========================================================

arena = create_arena()
boxes = place_boxes(arena)

fig,ax = plt.subplots()
plt.ion()
fig.canvas.mpl_connect('button_press_event',on_click)

true_step = random.choice([1,2,3])
facing = 2
collected = 0
prev_step = None
height_diff = 0
visited = [true_step]

for _ in range(40):

    if true_step in [10,11,12]:
        draw(arena,boxes,true_step,facing,
             lidar,est_lidar,est_full,
             visited,height_diff,
             collected,last_picked_step)
        plt.pause(1)
        print("FINAL TIME:", time_taken)
        break


    lidar = simulate_lidar(arena,true_step,facing)
    est_lidar = estimate_lidar_only(arena,lidar,facing)
    est_full = estimate_full(arena,lidar,facing,prev_step,height_diff)

    draw(arena,boxes,true_step,facing,
     lidar,est_lidar,est_full,
     visited,height_diff,
     collected,last_picked_step)


    # Pick adjacent R2
    facing, picked = pick_adjacent_r2(arena,boxes,true_step,facing)
    if picked:
        collected += picked

    # Move
    prev_step = true_step
    true_step, facing, height_diff, moved = move_bot(arena,boxes,true_step,facing)

    if moved:
        visited.append(true_step)
    else:
        break
