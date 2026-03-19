# ==========================================================
# estimator.py
# Responsible for:
# - Lidar-only step estimation
# - Full filtered step estimation
#   (lidar + height difference + feasibility + entry direction)
# ==========================================================


# ----------------------------------------------------------
# 1. LIDAR-ONLY ESTIMATION
# ----------------------------------------------------------

def estimate_lidar_only(step_map, lidar_readings, facing):
    """
    Returns list of possible step IDs whose lidar signature
    matches the given readings for the given facing.
    """

    candidates = []

    for step, data in step_map.items():
        if data["lidar_signature"][facing] == lidar_readings:
            candidates.append(step)

    return candidates


# ----------------------------------------------------------
# 2. FULL ESTIMATION (ALL FILTERS)
# ----------------------------------------------------------

def estimate_full(
    step_map,
    lidar_readings,
    facing,
    previous_step,
    height_diff
):
    """
    Returns list of possible step IDs after applying:
    - Lidar signature filter
    - Height difference filter
    - Feasibility (neighbor connectivity)
    - Entry direction requirement
    """

    candidates = []

    for step, data in step_map.items():

        # --------------------------------------------------
        # 1. LIDAR FILTER
        # --------------------------------------------------
        if data["lidar_signature"][facing] != lidar_readings:
            continue

        # If no previous step, cannot apply movement filters
        if previous_step is None:
            candidates.append(step)
            continue

        # --------------------------------------------------
        # 2. FEASIBILITY CHECK
        # Step must be reachable from previous_step
        # --------------------------------------------------
        if previous_step not in data["entry_requirements"]:
            continue

        # --------------------------------------------------
        # 3. ENTRY DIRECTION CHECK
        # Bot must be facing correct direction when entering
        # --------------------------------------------------
        required_facing = data["entry_requirements"][previous_step]

        if required_facing != facing:
            continue

        # --------------------------------------------------
        # 4. HEIGHT DIFFERENCE CHECK
        # --------------------------------------------------
        expected_height_diff = (
            data["height"] -
            step_map[previous_step]["height"]
        )

        if expected_height_diff != height_diff:
            continue

        candidates.append(step)

    return candidates