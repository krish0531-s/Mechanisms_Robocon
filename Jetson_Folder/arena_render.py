# ==========================================================
# renderer.py
# Responsible ONLY for:
# - Drawing arena
# - Drawing boxes
# - Drawing bot
# - Displaying info
# - Mouse click callback support
# ==========================================================

import matplotlib.pyplot as plt
import matplotlib.patches as patches


# ----------------------------------------------------------
# CONSTANTS
# ----------------------------------------------------------

DIRECTIONS = ["N", "E", "S", "W"]
INDEX_DIR = {0: "N", 1: "E", 2: "S", 3: "W"}

HEIGHT_COLORS = {
    200: "#A8D5BA",
    400: "#5DA9E9",
    600: "#F25F5C"
}


# ==========================================================
# RENDERER CLASS
# ==========================================================

class ArenaRenderer:

    def __init__(self, rows=4, cols=3):

        self.rows = rows
        self.cols = cols

        self.fig, self.ax = plt.subplots()
        self.click_callback = None

        self.fig.canvas.mpl_connect(
            "button_press_event",
            self._handle_click
        )

        plt.ion()

    # ------------------------------------------------------

    def register_click_handler(self, callback):
        """
        Callback signature:
        callback(row, col)
        """
        self.click_callback = callback

    # ------------------------------------------------------

    def _handle_click(self, event):

        if event.inaxes != self.ax:
            return

        if event.xdata is None or event.ydata is None:
            return

        col = int(event.xdata)
        row = self.rows - 1 - int(event.ydata)

        if self.click_callback:
            self.click_callback(row, col)

    # ------------------------------------------------------

    def draw(self, arena, boxes, bot_state, info_dict=None):

        self.ax.clear()

        # ------------------------------
        # Draw grid cells
        # ------------------------------

        for step, data in arena.items():

            x = data["col"]
            y = self.rows - 1 - data["row"]

            height = data["height"]
            color = HEIGHT_COLORS.get(height, "#CCCCCC")

            rect = patches.Rectangle(
                (x, y),
                1,
                1,
                facecolor=color,
                edgecolor="black"
            )

            self.ax.add_patch(rect)

            # Step ID text
            self.ax.text(
                x + 0.5,
                y + 0.85,
                str(step),
                ha="center",
                color="white"
            )

            # --------------------------
            # Draw box if present
            # --------------------------

            if step in boxes:

                box_type = boxes[step]

                if box_type == "R1":
                    box_color = "blue"
                elif box_type == "R2":
                    box_color = "yellow"
                elif box_type == "FAKE":
                    box_color = "black"
                else:
                    box_color = "gray"

                box_rect = patches.Rectangle(
                    (x + 0.25, y + 0.25),
                    0.5,
                    0.5,
                    facecolor=box_color
                )

                self.ax.add_patch(box_rect)

        # ------------------------------
        # Draw bot
        # ------------------------------

        if bot_state is not None:

            step = bot_state["current_step"]
            facing = bot_state["facing"]

            bx = arena[step]["col"] + 0.5
            by = self.rows - 1 - arena[step]["row"] + 0.5

            bot_rect = patches.Rectangle(
                (bx - 0.1, by - 0.1),
                0.2,
                0.2,
                color="red"
            )

            self.ax.add_patch(bot_rect)

            # Direction arrow
            dx, dy = [
                (0, 0.3),
                (0.3, 0),
                (0, -0.3),
                (-0.3, 0)
            ][facing]

            self.ax.arrow(
                bx,
                by,
                dx,
                dy,
                head_width=0.05,
                color="black"
            )

        # ------------------------------
        # Display info text
        # ------------------------------

        if info_dict:

            text_lines = []

            for key, value in info_dict.items():
                text_lines.append(f"{key}: {value}")

            info_text = "\n".join(text_lines)

            self.ax.text(
                self.cols + 0.2,
                self.rows - 0.2,
                info_text,
                fontsize=9,
                verticalalignment="top"
            )

        # ------------------------------
        # Finalize canvas
        # ------------------------------

        self.ax.set_xlim(0, self.cols + 3)
        self.ax.set_ylim(0, self.rows)
        self.ax.set_aspect("equal")
        self.ax.set_xticks([])
        self.ax.set_yticks([])

        plt.draw()
        plt.pause(0.001)

    # ------------------------------------------------------

    def show(self):
        plt.ioff()
        plt.show()