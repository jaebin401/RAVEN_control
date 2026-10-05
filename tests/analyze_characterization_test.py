#!/usr/bin/env python3

import importlib.util
import math
import sys
import tempfile
from pathlib import Path


def load_analyzer(path):
    spec = importlib.util.spec_from_file_location(
        "analyze_characterization", path
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def metrics(kp, kd, rms_error, peak_error, d_torque_step):
    return {
        "kp": kp,
        "kd": kd,
        "error_rms_deg": rms_error,
        "error_peak_deg": peak_error,
        "drift_deg": 0.0,
        "overshoot_deg": 0.1,
        "settling_time_s": 0.4,
        "velocity_mean_rad_s": 0.0,
        "velocity_reversal_percent": 10.0,
        "velocity_peak_abs_rad_s": 0.1,
        "torque_mean_nm": 0.0,
        "torque_std_nm": 0.0,
        "torque_peak_abs_nm": 0.0,
        "d_torque_std_nm": 0.01,
        "d_torque_step_p95_nm": d_torque_step,
        "d_torque_sign_flip_percent": 10.0,
        "duration_s": 3.0,
    }


def main():
    analyzer = load_analyzer(Path(sys.argv[1]))
    phases = {
        "r1/low/analysis/hold": {
            "samples": 10,
            "joints": {"joint": metrics(4.0, 0.1, 0.8, 1.5, 0.1)},
        },
        "r1/best/analysis/hold": {
            "samples": 10,
            "joints": {"joint": metrics(8.0, 0.15, 0.3, 0.8, 0.1)},
        },
        "r1/unsafe/analysis/hold": {
            "samples": 10,
            "joints": {"joint": metrics(12.0, 0.25, 0.2, 2.5, 0.1)},
        },
    }
    recommendations = analyzer.tuning_recommendations(phases, ["joint"])
    selected = recommendations["joint"]["selected"]
    assert selected is not None
    assert math.isclose(selected["kp"], 8.0)
    assert math.isclose(selected["kd"], 0.15)

    with tempfile.TemporaryDirectory() as directory:
        output = Path(directory) / "recommended_gains.yaml"
        analyzer.write_recommendations(output, recommendations)
        text = output.read_text()
        assert "recommended_kp: 8" in text
        assert "recommended_kd: 0.15" in text
        assert "safe: false" in text

    print("All characterization analyzer tests passed")


if __name__ == "__main__":
    main()
