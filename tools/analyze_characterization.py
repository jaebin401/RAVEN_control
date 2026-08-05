#!/usr/bin/env python3

import argparse
import csv
import math
from collections import defaultdict
from pathlib import Path
from statistics import fmean


def finite(row, key):
    try:
        value = float(row[key])
    except (KeyError, TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


def rms(values):
    if not values:
        return math.nan
    return math.sqrt(fmean(value * value for value in values))


def sample_std(values):
    if len(values) < 2:
        return 0.0 if values else math.nan
    mean = fmean(values)
    return math.sqrt(
        sum((value - mean) ** 2 for value in values) / (len(values) - 1)
    )


def yaml_string(value):
    escaped = str(value).replace("\\", "\\\\").replace('"', '\\"')
    return f'"{escaped}"'


def yaml_number(value):
    return "null" if not math.isfinite(value) else f"{value:.9g}"


def joint_names(fieldnames):
    suffix = ".command_position_rad"
    return [
        field[: -len(suffix)]
        for field in fieldnames
        if field.endswith(suffix)
    ]


def phase_metrics(rows, joints):
    grouped = defaultdict(list)
    for row in rows:
        phase = row.get("phase", "")
        if "/analysis/" in phase or phase.endswith("/analysis"):
            grouped[phase].append(row)

    result = {}
    for phase, phase_rows in sorted(grouped.items()):
        joint_result = {}
        for joint in joints:
            errors = [
                value
                for row in phase_rows
                if (value := finite(row, f"{joint}.position_error_rad"))
                is not None
            ]
            positions = [
                value
                for row in phase_rows
                if (value := finite(row, f"{joint}.actual_position_rad"))
                is not None
            ]
            velocities = [
                value
                for row in phase_rows
                if (value := finite(row, f"{joint}.actual_velocity_rad_s"))
                is not None
            ]
            torques = [
                value
                for row in phase_rows
                if (value := finite(row, f"{joint}.measured_torque_nm"))
                is not None
            ]
            joint_result[joint] = {
                "error_rms_deg": math.degrees(rms(errors)),
                "error_peak_deg": (
                    math.degrees(max(abs(value) for value in errors))
                    if errors
                    else math.nan
                ),
                "drift_deg": (
                    math.degrees(positions[-1] - positions[0])
                    if len(positions) >= 2
                    else math.nan
                ),
                "velocity_mean_rad_s": (
                    fmean(velocities) if velocities else math.nan
                ),
                "torque_mean_nm": fmean(torques) if torques else math.nan,
                "torque_std_nm": sample_std(torques),
                "torque_peak_abs_nm": (
                    max(abs(value) for value in torques)
                    if torques
                    else math.nan
                ),
            }
        result[phase] = {"samples": len(phase_rows), "joints": joint_result}
    return result


def global_metrics(rows, joints):
    times = [
        value
        for row in rows
        if (value := finite(row, "actual_time_us")) is not None
    ]
    missed = sum(int(row.get("deadline_missed", "0")) for row in rows)
    all_vbus = []
    all_temperatures = []
    fault_samples = 0
    invalid_feedback_samples = 0
    for row in rows:
        row_fault = False
        row_invalid = False
        is_analysis = "/analysis/" in row.get("phase", "")
        for joint in joints:
            voltage = finite(row, f"{joint}.bus_voltage_v")
            if voltage is not None:
                all_vbus.append(voltage)
            temperature = finite(row, f"{joint}.motor_temperature_celsius")
            if temperature is not None:
                all_temperatures.append(temperature)
            try:
                row_fault = row_fault or int(
                    row.get(f"{joint}.motor_fault_flags", "0")
                ) != 0
            except ValueError:
                row_fault = True
            if is_analysis:
                row_invalid = row_invalid or row.get(
                    f"{joint}.operation_feedback_valid", "0"
                ) != "1"
        fault_samples += int(row_fault)
        invalid_feedback_samples += int(row_invalid)

    return {
        "samples": len(rows),
        "duration_s": (
            (max(times) - min(times)) / 1e6 if times else math.nan
        ),
        "deadline_misses": missed,
        "deadline_miss_percent": 100.0 * missed / len(rows) if rows else 0.0,
        "fault_samples": fault_samples,
        "invalid_feedback_samples": invalid_feedback_samples,
        "vbus_min_v": min(all_vbus) if all_vbus else math.nan,
        "vbus_max_v": max(all_vbus) if all_vbus else math.nan,
        "temperature_max_c": (
            max(all_temperatures) if all_temperatures else math.nan
        ),
    }


def read_run_status(run_directory):
    path = run_directory / "run_status.yaml"
    if not path.is_file():
        return "missing"
    for line in path.read_text().splitlines():
        if line.startswith("status:"):
            return line.split(":", 1)[1].strip()
    return "missing"


def evaluate_acceptance(global_result, phases, run_status, arguments):
    checks = {
        "run_completed": run_status == "completed",
        "no_motor_faults": global_result["fault_samples"] == 0,
        "all_operation_feedback_valid": (
            global_result["invalid_feedback_samples"] == 0
        ),
    }
    if arguments.max_deadline_miss_percent is not None:
        checks["deadline_miss_percent"] = (
            global_result["deadline_miss_percent"]
            <= arguments.max_deadline_miss_percent
        )
    if arguments.max_vbus_v is not None:
        checks["vbus_max_v"] = (
            math.isfinite(global_result["vbus_max_v"])
            and global_result["vbus_max_v"] <= arguments.max_vbus_v
        )
    if arguments.max_temperature_c is not None:
        checks["temperature_max_c"] = (
            math.isfinite(global_result["temperature_max_c"])
            and global_result["temperature_max_c"]
            <= arguments.max_temperature_c
        )

    error_rms = []
    error_peak = []
    for phase in phases.values():
        for joint in phase["joints"].values():
            if math.isfinite(joint["error_rms_deg"]):
                error_rms.append(joint["error_rms_deg"])
            if math.isfinite(joint["error_peak_deg"]):
                error_peak.append(joint["error_peak_deg"])
    if arguments.max_rms_error_deg is not None:
        checks["max_rms_error_deg"] = bool(error_rms) and max(error_rms) <= (
            arguments.max_rms_error_deg
        )
    if arguments.max_peak_error_deg is not None:
        checks["max_peak_error_deg"] = bool(error_peak) and max(error_peak) <= (
            arguments.max_peak_error_deg
        )
    return checks, all(checks.values())


def write_summary(
    path, global_result, phases, run_status, checks, accepted
):
    with path.open("w") as output:
        output.write("schema_version: 1\n")
        output.write(f"run_status: {yaml_string(run_status)}\n")
        output.write(f"accepted: {'true' if accepted else 'false'}\n")
        output.write("acceptance_checks:\n")
        for name, passed in checks.items():
            output.write(f"  {name}: {'true' if passed else 'false'}\n")
        output.write("global:\n")
        for name, value in global_result.items():
            if isinstance(value, int):
                output.write(f"  {name}: {value}\n")
            else:
                output.write(f"  {name}: {yaml_number(value)}\n")
        output.write("analysis_phases:\n")
        for phase_name, phase in phases.items():
            output.write(f"  {yaml_string(phase_name)}:\n")
            output.write(f"    samples: {phase['samples']}\n")
            output.write("    joints:\n")
            for joint_name, metrics in phase["joints"].items():
                output.write(f"      {joint_name}:\n")
                for name, value in metrics.items():
                    output.write(f"        {name}: {yaml_number(value)}\n")


def parse_arguments():
    parser = argparse.ArgumentParser(
        description="Create summary.yaml from a RAVEN characterization run"
    )
    parser.add_argument("run_directory", type=Path)
    parser.add_argument("--max-rms-error-deg", type=float)
    parser.add_argument("--max-peak-error-deg", type=float)
    parser.add_argument("--max-vbus-v", type=float)
    parser.add_argument("--max-temperature-c", type=float)
    parser.add_argument("--max-deadline-miss-percent", type=float)
    return parser.parse_args()


def main():
    arguments = parse_arguments()
    csv_path = arguments.run_directory / "raw.csv"
    if not csv_path.is_file():
        raise SystemExit(f"missing raw.csv: {csv_path}")
    with csv_path.open(newline="") as input_file:
        reader = csv.DictReader(input_file)
        rows = list(reader)
        fields = reader.fieldnames or []
    if not rows:
        raise SystemExit(f"raw.csv is empty: {csv_path}")
    joints = joint_names(fields)
    if not joints:
        raise SystemExit("raw.csv contains no joint columns")

    global_result = global_metrics(rows, joints)
    phases = phase_metrics(rows, joints)
    run_status = read_run_status(arguments.run_directory)
    checks, accepted = evaluate_acceptance(
        global_result, phases, run_status, arguments
    )
    output_path = arguments.run_directory / "summary.yaml"
    write_summary(
        output_path,
        global_result,
        phases,
        run_status,
        checks,
        accepted,
    )
    print(f"Summary saved: {output_path}")
    print(f"Accepted: {'yes' if accepted else 'no'}")


if __name__ == "__main__":
    main()
