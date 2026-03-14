# ==========================================================
# map_update.py
# Maintains the dynamic map of detected boxes
# ==========================================================


class MapUpdater:

    def __init__(self):
        """
        Map starts empty.
        step_id -> box_type
        """
        self.box_map = {}

    # ------------------------------------------------------
    # Add / update a box detection
    # ------------------------------------------------------

    def update_box(self, step_id, box_type="R2"):
        """
        Called when camera detects a box on a step.
        """
        self.box_map[step_id] = box_type

    # ------------------------------------------------------
    # Remove box (after pickup or verification)
    # ------------------------------------------------------

    def remove_box(self, step_id):
        if step_id in self.box_map:
            del self.box_map[step_id]

    # ------------------------------------------------------
    # Get full map
    # ------------------------------------------------------

    def get_map(self):
        return self.box_map

    # ------------------------------------------------------
    # Debug print
    # ------------------------------------------------------

    def print_map(self):

        for step in range(1, 13):

            if step in self.box_map:
                value = self.box_map[step]
            else:
                value = "."

            print(f"{step}:{value}", end=" ")

            if step % 3 == 0:
                print()