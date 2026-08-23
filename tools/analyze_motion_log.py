#!/usr/bin/env python3

import argparse
import csv
import math
from pathlib import Path
from statistics import fmean


def finite_values(rows, key):
    values = []
    for row in rows:
        try:
            value = float(row[key])
        except (KeyError, TypeError, ValueError):
            continue
        if math.isfinite(value):
            values.append(value)
    return values


def percentile(values, percent):
    if not values:
        return math.nan
    ordered = sorted(values)
    position = (len(ordered) - 1) * percent / 100.0
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def joint_names(fieldnames):
    suffix = ".command_position_rad"
    return [
        name[: -len(suffix)]
        for name in fieldnames
        if name.endswith(suffix)
    ]


def rms(values):
    if not values:
        return math.nan
    return math.sqrt(fmean(value * value for value in values))


def print_summary(path, rows, joints):
    dt = [value for value in finite_values(rows, "dt_us") if value > 0.0]
    lateness = finite_values(rows, "lateness_us")
    missed = sum(int(row.get("deadline_missed", "0")) for row in rows)
    actual_time = finite_values(rows, "actual_time_us")

    print(f"Motion log: {path}")
    print(f"Samples: {len(rows)}")
    if actual_time:
        print(f"Duration: {(max(actual_time) - min(actual_time)) / 1e6:.3f} s")
    if dt:
        print(
            "Loop dt [ms]: "
            f"mean={fmean(dt) / 1000.0:.3f}, "
            f"p95={percentile(dt, 95) / 1000.0:.3f}, "
            f"p99={percentile(dt, 99) / 1000.0:.3f}, "
            f"max={max(dt) / 1000.0:.3f}"
        )
    if lateness:
        print(
            "Wake lateness [ms]: "
            f"p95={percentile(lateness, 95) / 1000.0:.3f}, "
            f"p99={percentile(lateness, 99) / 1000.0:.3f}, "
            f"max={max(lateness) / 1000.0:.3f}"
        )
    ratio = 100.0 * missed / len(rows) if rows else 0.0
    print(f"Deadline misses: {missed} ({ratio:.2f}%)")

    for joint in joints:
        errors = finite_values(rows, f"{joint}.position_error_rad")
        p_torque = finite_values(rows, f"{joint}.estimated_p_torque_nm")
        d_torque = finite_values(rows, f"{joint}.estimated_d_torque_nm")
        raw_gravity = finite_values(rows, f"{joint}.raw_gravity_torque_nm")
        limited_gravity = finite_values(
            rows, f"{joint}.limited_gravity_torque_nm"
        )
        sent_feedforward = finite_values(
            rows, f"{joint}.sent_feedforward_torque_nm"
        )
        control_torque = finite_values(
            rows, f"{joint}.estimated_control_torque_nm"
        )
        measured_torque = finite_values(rows, f"{joint}.measured_torque_nm")
        temperatures = finite_values(
            rows, f"{joint}.motor_temperature_celsius"
        )
        ages = finite_values(rows, f"{joint}.feedback_age_ms")
        operation_ages = finite_values(
            rows, f"{joint}.operation_feedback_age_ms"
        )
        print(f"\n{joint}")
        if errors:
            print(
                "  position error [deg]: "
                f"rms={math.degrees(rms(errors)):.3f}, "
                f"max={math.degrees(max(abs(value) for value in errors)):.3f}"
            )
        if p_torque:
            print(
                "  estimated |P torque| max: "
                f"{max(abs(value) for value in p_torque):.3f} N.m"
            )
        if d_torque:
            print(
                "  estimated |D torque| max: "
                f"{max(abs(value) for value in d_torque):.3f} N.m"
            )
        if raw_gravity:
            clamp_count = sum(
                int(row.get(f"{joint}.gravity_torque_clamped", "0"))
                for row in rows
            )
            valid_count = sum(
                int(row.get(f"{joint}.gravity_input_valid", "0"))
                for row in rows
            )
            print(
                "  gravity |raw/limited/sent| max [N.m]: "
                f"{max(abs(value) for value in raw_gravity):.3f} / "
                f"{max(abs(value) for value in limited_gravity):.3f} / "
                f"{max(abs(value) for value in sent_feedforward):.3f}"
            )
            print(
                "  gravity valid/clamped samples: "
                f"{valid_count}/{clamp_count} of {len(rows)}"
            )
        if control_torque:
            print(
                "  estimated |control torque| max: "
                f"{max(abs(value) for value in control_torque):.3f} N.m"
            )
        if measured_torque:
            print(
                "  Type 2 measured torque [N.m]: "
                f"mean={fmean(measured_torque):.3f}, "
                f"max_abs={max(abs(value) for value in measured_torque):.3f}"
            )
        if temperatures:
            print(
                "  Type 2 motor temperature [C]: "
                f"mean={fmean(temperatures):.1f}, "
                f"max={max(temperatures):.1f}"
            )
        if ages:
            print(
                "  feedback age [ms]: "
                f"median={percentile(ages, 50):.3f}, "
                f"p99={percentile(ages, 99):.3f}, "
                f"max={max(ages):.3f}"
            )
        if operation_ages:
            print(
                "  Type 2 feedback age [ms]: "
                f"median={percentile(operation_ages, 50):.3f}, "
                f"p99={percentile(operation_ages, 99):.3f}, "
                f"max={max(operation_ages):.3f}"
            )


def save_plot(path, rows, joints, output_path, show):
    try:
        import matplotlib.pyplot as plt
    except ImportError as error:
        raise SystemExit(
            "matplotlib is required for --plot; the text summary is still valid"
        ) from error

    time_s = [float(row["actual_time_us"]) / 1e6 for row in rows]
    figure, axes = plt.subplots(
        len(joints) + 1,
        2,
        figsize=(15, 3.0 * (len(joints) + 1)),
        sharex=True,
    )

    for row_index, joint in enumerate(joints):
        position_axis = axes[row_index][0]
        velocity_axis = axes[row_index][1]
        command = [
            math.degrees(float(row[f"{joint}.command_position_rad"]))
            for row in rows
        ]
        actual = [
            math.degrees(float(row[f"{joint}.actual_position_rad"]))
            for row in rows
        ]
        trajectory_velocity = [
            float(row[f"{joint}.trajectory_velocity_rad_s"])
            for row in rows
        ]
        sent_velocity = [
            float(row[f"{joint}.sent_velocity_rad_s"])
            for row in rows
        ]
        actual_velocity = [
            float(row[f"{joint}.actual_velocity_rad_s"])
            for row in rows
        ]
        position_axis.plot(
            time_s, command, label="command", linewidth=1.4
        )
        position_axis.plot(
            time_s, actual, label="encoder", linewidth=1.0
        )
        position_axis.set_ylabel("angle [deg]")
        position_axis.set_title(f"{joint} position")
        position_axis.grid(True, alpha=0.3)
        position_axis.legend(loc="best")

        velocity_axis.plot(
            time_s,
            trajectory_velocity,
            label="trajectory",
            linewidth=1.2,
        )
        velocity_axis.plot(
            time_s, sent_velocity, label="sent", linewidth=1.0
        )
        velocity_axis.plot(
            time_s,
            actual_velocity,
            label=(
                "Type 2 measured"
                if f"{joint}.operation_feedback_valid" in rows[0]
                else "encoder estimate"
            ),
            linewidth=1.0,
        )
        velocity_axis.set_ylabel("velocity [rad/s]")
        velocity_axis.set_title(f"{joint} velocity")
        velocity_axis.grid(True, alpha=0.3)
        velocity_axis.legend(loc="best")

    timing_axis = axes[-1][0]
    dt_ms = [float(row["dt_us"]) / 1000.0 for row in rows]
    lateness_ms = [float(row["lateness_us"]) / 1000.0 for row in rows]
    timing_axis.plot(time_s, dt_ms, label="loop dt", linewidth=1.0)
    timing_axis.plot(
        time_s,
        lateness_ms,
        label="wake lateness",
        linewidth=1.0,
    )
    timing_axis.set_ylabel("time [ms]")
    timing_axis.set_xlabel("elapsed time [s]")
    timing_axis.set_title("Control-loop timing")
    timing_axis.grid(True, alpha=0.3)
    timing_axis.legend(loc="best")

    torque_axis = axes[-1][1]
    for joint in joints:
        control_torque = [
            float(row[f"{joint}.estimated_control_torque_nm"])
            for row in rows
        ]
        torque_axis.plot(
            time_s,
            control_torque,
            label=joint,
            linewidth=1.0,
        )
        measured_key = f"{joint}.measured_torque_nm"
        if measured_key in rows[0]:
            measured_torque = [float(row[measured_key]) for row in rows]
            torque_axis.plot(
                time_s,
                measured_torque,
                label=f"{joint} measured",
                linewidth=0.9,
                linestyle="--",
            )
    torque_axis.set_ylabel("torque [N.m]")
    torque_axis.set_xlabel("elapsed time [s]")
    torque_axis.set_title("Estimated command and Type 2 measured torque")
    torque_axis.grid(True, alpha=0.3)
    torque_axis.legend(loc="best")

    figure.suptitle(path.name)
    figure.tight_layout()
    figure.savefig(output_path, dpi=160)
    print(f"Plot saved: {output_path}")
    if show:
        plt.show()


def parse_arguments():
    parser = argparse.ArgumentParser(
        description="Summarize and plot a RAVEN coordinated-motion CSV log"
    )
    parser.add_argument("csv_path", type=Path)
    parser.add_argument(
        "--plot",
        nargs="?",
        const="",
        metavar="PNG_PATH",
        help="save a PNG plot; omit the path to use <csv>_analysis.png",
    )
    parser.add_argument(
        "--show",
        action="store_true",
        help="show the matplotlib window after saving the plot",
    )
    return parser.parse_args()


def main():
    arguments = parse_arguments()
    with arguments.csv_path.open(newline="") as input_file:
        reader = csv.DictReader(input_file)
        rows = list(reader)
        fields = reader.fieldnames or []

    if not rows:
        raise SystemExit(f"motion log is empty: {arguments.csv_path}")

    joints = joint_names(fields)
    if not joints:
        raise SystemExit("no joint command columns found in motion log")

    print_summary(arguments.csv_path, rows, joints)
    if arguments.plot is not None:
        output_path = (
            Path(arguments.plot)
            if arguments.plot
            else arguments.csv_path.with_name(
                arguments.csv_path.stem + "_analysis.png"
            )
        )
        save_plot(
            arguments.csv_path,
            rows,
            joints,
            output_path,
            arguments.show,
        )


if __name__ == "__main__":
    main()
