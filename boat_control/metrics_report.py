import csv
import json
import math
import os
import sys
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib")

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


def load_csv_rows(path: Path):
    with path.open("r", encoding="utf-8", newline="") as file_obj:
        return list(csv.DictReader(file_obj))


def to_float(rows, key):
    return [float(row[key]) for row in rows]


def path_length(points):
    total = 0.0
    for idx in range(1, len(points)):
        dx = points[idx][0] - points[idx - 1][0]
        dy = points[idx][1] - points[idx - 1][1]
        total += math.hypot(dx, dy)
    return total


def cumulative_lengths(points):
    lengths = [0.0]
    total = 0.0
    for idx in range(1, len(points)):
        dx = points[idx][0] - points[idx - 1][0]
        dy = points[idx][1] - points[idx - 1][1]
        total += math.hypot(dx, dy)
        lengths.append(total)
    return lengths


def interpolate_polyline(points, cum_lengths, target_length):
    if not points:
        return (0.0, 0.0)

    if len(points) == 1 or target_length <= 0.0:
        return points[0]

    total_length = cum_lengths[-1]
    if total_length <= 0.0 or target_length >= total_length:
        return points[-1]

    for idx in range(1, len(points)):
        seg_start = cum_lengths[idx - 1]
        seg_end = cum_lengths[idx]
        if target_length <= seg_end:
            seg_len = seg_end - seg_start
            if seg_len <= 0.0:
                return points[idx]
            ratio = (target_length - seg_start) / seg_len
            x = points[idx - 1][0] + ratio * (points[idx][0] - points[idx - 1][0])
            y = points[idx - 1][1] + ratio * (points[idx][1] - points[idx - 1][1])
            return (x, y)

    return points[-1]


def distance_point_to_segment(point, seg_a, seg_b):
    px, py = point
    ax, ay = seg_a
    bx, by = seg_b
    dx = bx - ax
    dy = by - ay
    seg_len_sq = dx * dx + dy * dy

    if seg_len_sq <= 0.0:
        return math.hypot(px - ax, py - ay)

    t = ((px - ax) * dx + (py - ay) * dy) / seg_len_sq
    t = max(0.0, min(1.0, t))
    proj_x = ax + t * dx
    proj_y = ay + t * dy
    return math.hypot(px - proj_x, py - proj_y)


def distance_point_to_polyline(point, polyline):
    if not polyline:
        return 0.0
    if len(polyline) == 1:
        return math.hypot(point[0] - polyline[0][0], point[1] - polyline[0][1])

    best = float("inf")
    for idx in range(1, len(polyline)):
        best = min(best, distance_point_to_segment(point, polyline[idx - 1], polyline[idx]))
    return best


def mean(values):
    if not values:
        return 0.0
    return sum(values) / len(values)


def build_planned_path(odom_rows, waypoint_rows):
    if not odom_rows:
        raise ValueError("odom.csv is empty")
    if not waypoint_rows:
        raise ValueError("waypoints.csv is empty")

    first_mission_id = min(int(row["mission_id"]) for row in waypoint_rows)
    mission_rows = [
        row for row in waypoint_rows if int(row["mission_id"]) == first_mission_id
    ]
    mission_rows.sort(key=lambda row: int(row["point_idx"]))

    start_point = (float(odom_rows[0]["x"]), float(odom_rows[0]["y"]))
    waypoint_points = [(float(row["x"]), float(row["y"])) for row in mission_rows]

    if waypoint_points and math.hypot(
        waypoint_points[0][0] - start_point[0],
        waypoint_points[0][1] - start_point[1],
    ) < 1e-9:
        return waypoint_points

    return [start_point] + waypoint_points


def ensure_input_files(run_dir: Path):
    required = [
        run_dir / "odom.csv",
        run_dir / "cmd_vel.csv",
        run_dir / "disturbance.csv",
        run_dir / "waypoints.csv",
    ]
    missing = [str(path) for path in required if not path.exists()]
    if missing:
        raise FileNotFoundError("Missing input files: " + ", ".join(missing))


def save_trajectory_plot(analysis_dir, planned_path, expected_points, actual_points):
    fig, ax = plt.subplots(figsize=(10, 7))

    planned_x = [point[0] for point in planned_path]
    planned_y = [point[1] for point in planned_path]
    expected_x = [point[0] for point in expected_points]
    expected_y = [point[1] for point in expected_points]
    actual_x = [point[0] for point in actual_points]
    actual_y = [point[1] for point in actual_points]

    ax.plot(planned_x, planned_y, "o--", linewidth=2.0, label="planned trajectory")
    ax.plot(expected_x, expected_y, linewidth=2.0, label="expected trajectory")
    ax.plot(actual_x, actual_y, linewidth=2.0, label="actual trajectory")
    ax.set_title("Planned / Expected Trajectory vs Actual Trajectory")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.axis("equal")
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(analysis_dir / "trajectory_expected_vs_actual.png", dpi=160)
    plt.close(fig)


def save_cmd_plots(analysis_dir, cmd_t, cmd_linear_x, cmd_angular_z):
    fig, ax = plt.subplots(figsize=(10, 4.5))
    ax.plot(cmd_t, cmd_linear_x, linewidth=1.8)
    ax.set_title("cmd_vel.linear.x Over Time")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("linear.x [m/s]")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(analysis_dir / "cmd_vel_linear_x_over_time.png", dpi=160)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(10, 4.5))
    ax.plot(cmd_t, cmd_angular_z, linewidth=1.8)
    ax.set_title("cmd_vel.angular.z Over Time")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("angular.z [rad/s]")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(analysis_dir / "cmd_vel_angular_z_over_time.png", dpi=160)
    plt.close(fig)


def save_disturbance_plot(analysis_dir, disturbance_t, disturbance_vx, disturbance_vy):
    magnitude = [
        math.hypot(vx, vy) for vx, vy in zip(disturbance_vx, disturbance_vy)
    ]

    fig, ax = plt.subplots(figsize=(10, 4.8))
    ax.plot(disturbance_t, disturbance_vx, label="flow vx", linewidth=1.8)
    ax.plot(disturbance_t, disturbance_vy, label="flow vy", linewidth=1.8)
    ax.plot(disturbance_t, magnitude, label="flow magnitude", linewidth=1.8)
    ax.set_title("Disturbance / Flow Over Time")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("velocity [m/s]")
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(analysis_dir / "disturbance_over_time.png", dpi=160)
    plt.close(fig)


def save_error_plot(analysis_dir, odom_t, expected_error, actual_error):
    fig, ax = plt.subplots(figsize=(10, 4.8))
    ax.plot(odom_t, expected_error, label="expected error", linewidth=1.8)
    ax.plot(odom_t, actual_error, label="actual error", linewidth=1.8)
    ax.set_title("Expected Error vs Actual Error")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("error [m]")
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(analysis_dir / "expected_vs_actual_error.png", dpi=160)
    plt.close(fig)


def write_error_series_csv(analysis_dir, odom_t, expected_points, actual_points, expected_error, actual_error):
    csv_path = analysis_dir / "error_timeseries.csv"
    with csv_path.open("w", encoding="utf-8", newline="") as file_obj:
        writer = csv.writer(file_obj)
        writer.writerow(
            [
                "t",
                "expected_x",
                "expected_y",
                "actual_x",
                "actual_y",
                "expected_error",
                "actual_error",
                "error_difference",
            ]
        )
        for idx, t_val in enumerate(odom_t):
            writer.writerow(
                [
                    t_val,
                    expected_points[idx][0],
                    expected_points[idx][1],
                    actual_points[idx][0],
                    actual_points[idx][1],
                    expected_error[idx],
                    actual_error[idx],
                    abs(expected_error[idx] - actual_error[idx]),
                ]
            )


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    if len(argv) != 1:
        print("Usage: ros2 run boat_control metrics_report <run_dir>")
        return 1

    run_dir = Path(argv[0]).expanduser().resolve()
    analysis_dir = run_dir / "analysis"
    analysis_dir.mkdir(parents=True, exist_ok=True)

    try:
        ensure_input_files(run_dir)
        odom_rows = load_csv_rows(run_dir / "odom.csv")
        cmd_rows = load_csv_rows(run_dir / "cmd_vel.csv")
        disturbance_rows = load_csv_rows(run_dir / "disturbance.csv")
        waypoint_rows = load_csv_rows(run_dir / "waypoints.csv")

        planned_path = build_planned_path(odom_rows, waypoint_rows)
        planned_total_length = path_length(planned_path)
        planned_cum_lengths = cumulative_lengths(planned_path)

        odom_t_raw = to_float(odom_rows, "t")
        odom_t0 = odom_t_raw[0]
        odom_t = [t - odom_t0 for t in odom_t_raw]
        mission_duration = odom_t[-1] if len(odom_t) > 1 else 0.0

        actual_points = [
            (float(row["x"]), float(row["y"])) for row in odom_rows
        ]

        expected_points = []
        expected_error = []
        actual_error = []

        for idx, point in enumerate(actual_points):
            if mission_duration > 0.0:
                progress = odom_t[idx] / mission_duration
            else:
                progress = 1.0
            progress = max(0.0, min(1.0, progress))
            expected_point = interpolate_polyline(
                planned_path,
                planned_cum_lengths,
                progress * planned_total_length,
            )
            expected_points.append(expected_point)
            expected_error.append(math.hypot(point[0] - expected_point[0], point[1] - expected_point[1]))
            actual_error.append(distance_point_to_polyline(point, planned_path))

        cmd_t_raw = to_float(cmd_rows, "t")
        cmd_t0 = cmd_t_raw[0]
        cmd_t = [t - cmd_t0 for t in cmd_t_raw]
        cmd_linear_x = to_float(cmd_rows, "linear_x")
        cmd_angular_z = to_float(cmd_rows, "angular_z")

        disturbance_t_raw = to_float(disturbance_rows, "t")
        disturbance_t0 = disturbance_t_raw[0]
        disturbance_t = [t - disturbance_t0 for t in disturbance_t_raw]
        disturbance_vx = to_float(disturbance_rows, "vx")
        disturbance_vy = to_float(disturbance_rows, "vy")
        disturbance_mag = [
            math.hypot(vx, vy) for vx, vy in zip(disturbance_vx, disturbance_vy)
        ]

        summary = {
            "run_dir": str(run_dir),
            "odom_sample_count": len(odom_rows),
            "cmd_sample_count": len(cmd_rows),
            "disturbance_sample_count": len(disturbance_rows),
            "waypoint_count": max(0, len(planned_path) - 1),
            "mission_duration_sec": mission_duration,
            "planned_path_length": planned_total_length,
            "actual_path_length": path_length(actual_points),
            "mean_expected_error": mean(expected_error),
            "max_expected_error": max(expected_error) if expected_error else 0.0,
            "mean_actual_error": mean(actual_error),
            "max_actual_error": max(actual_error) if actual_error else 0.0,
            "mean_difference_expected_vs_actual_error": mean(
                [abs(a - b) for a, b in zip(expected_error, actual_error)]
            ),
            "signed_mean_difference_expected_minus_actual_error": mean(
                [a - b for a, b in zip(expected_error, actual_error)]
            ),
            "mean_cmd_vel_linear_x": mean(cmd_linear_x),
            "max_cmd_vel_linear_x": max(cmd_linear_x) if cmd_linear_x else 0.0,
            "mean_abs_cmd_vel_angular_z": mean([abs(val) for val in cmd_angular_z]),
            "max_abs_cmd_vel_angular_z": max([abs(val) for val in cmd_angular_z]) if cmd_angular_z else 0.0,
            "mean_disturbance_magnitude": mean(disturbance_mag),
            "max_disturbance_magnitude": max(disturbance_mag) if disturbance_mag else 0.0,
        }

        save_trajectory_plot(analysis_dir, planned_path, expected_points, actual_points)
        save_cmd_plots(analysis_dir, cmd_t, cmd_linear_x, cmd_angular_z)
        save_disturbance_plot(analysis_dir, disturbance_t, disturbance_vx, disturbance_vy)
        save_error_plot(analysis_dir, odom_t, expected_error, actual_error)
        write_error_series_csv(
            analysis_dir,
            odom_t,
            expected_points,
            actual_points,
            expected_error,
            actual_error,
        )

        with (analysis_dir / "metrics_summary.json").open("w", encoding="utf-8") as file_obj:
            json.dump(summary, file_obj, indent=2)

        print(f"Metrics written to: {analysis_dir}")
        return 0
    except Exception as exc:
        print(f"metrics_report failed: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
