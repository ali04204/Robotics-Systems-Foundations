import math
import time


def process_lidar_scan(
    ranges,
    scan_timestamp,
    current_time,
    angle_min,
    angle_increment,
    range_min,
    range_max,
    front_arc_deg=60.0,
    stop_threshold_m=0.5,
    stale_timeout_s=0.3,
):
    """
    Takes raw LiDAR ranges and outputs:
    - blocked_aisle: True or False
    - obstacle_distance_m: closest valid obstacle in front arc
    """

    # 1. Check if scan is stale
    scan_age = current_time - scan_timestamp

    if scan_age > stale_timeout_s:
        return {
            "scan_status": "STALE_SCAN",
            "blocked_aisle": True,
            "obstacle_distance_m": None,
            "scan_age_s": scan_age,
        }

    # 2. Define front arc
    half_front_arc_rad = math.radians(front_arc_deg / 2.0)

    front_distances = []

    # 3. Loop through every LiDAR reading
    for i, r in enumerate(ranges):
        beam_angle = angle_min + i * angle_increment

        # Keep only beams inside front arc
        if -half_front_arc_rad <= beam_angle <= half_front_arc_rad:

            # Finite check: remove NaN, inf, -inf
            if not math.isfinite(r):
                continue

            # Clip to valid min and max
            r_clipped = max(range_min, min(r, range_max))

            front_distances.append(r_clipped)

    # 4. If no valid front readings exist, treat scan as unsafe
    if len(front_distances) == 0:
        return {
            "scan_status": "NO_VALID_FRONT_READINGS",
            "blocked_aisle": True,
            "obstacle_distance_m": None,
            "scan_age_s": scan_age,
        }

    # 5. Find closest obstacle in front arc
    obstacle_distance_m = min(front_distances)

    # 6. Decide blocked aisle
    blocked_aisle = obstacle_distance_m < stop_threshold_m

    return {
        "scan_status": "OK",
        "blocked_aisle": blocked_aisle,
        "obstacle_distance_m": obstacle_distance_m,
        "scan_age_s": scan_age,
    }


if __name__ == "__main__":
    # Example simulated scan
    # Assume 181 beams from -90 degrees to +90 degrees
    angle_min = math.radians(-90)
    angle_increment = math.radians(1)

    ranges = [2.5] * 181

    # Add bad values
    ranges[80] = float("inf")
    ranges[85] = float("nan")

    # Add obstacle in front
    ranges[90] = 0.42

    scan_timestamp = time.time()
    current_time = time.time()

    result = process_lidar_scan(
        ranges=ranges,
        scan_timestamp=scan_timestamp,
        current_time=current_time,
        angle_min=angle_min,
        angle_increment=angle_increment,
        range_min=0.12,
        range_max=3.5,
        front_arc_deg=60.0,
        stop_threshold_m=0.5,
        stale_timeout_s=0.3,
    )

    print("=== LiDAR Scan Filter Output ===")
    print(f"Scan status: {result['scan_status']}")
    print(f"Blocked aisle: {result['blocked_aisle']}")
    print(f"Obstacle distance: {result['obstacle_distance_m']} m")
    print(f"Scan age: {result['scan_age_s']:.4f} s")

    # Simple confirmation check
    assert result["blocked_aisle"] is True
    assert result["obstacle_distance_m"] == 0.42

    print("Test passed: scan_filter logic works.")