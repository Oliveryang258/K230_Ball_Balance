#!/usr/bin/env python3
"""PC-only servo-to-rail modelling and calibration helper.

This file is NOT K230 MicroPython runtime code.  It intentionally uses NumPy,
pandas, and matplotlib for offline analysis on Computer A/B.

The built-in mechanism equation is only a first-order approximation.  Fitted
models describe the supplied samples and must not be treated as validated plant
models without repeatable forward/reverse calibration data.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

try:
    import numpy as np
    import pandas as pd
    import matplotlib
except ImportError as exc:  # pragma: no cover - depends on the PC environment
    raise SystemExit(
        "Missing PC dependency: {}. Install numpy, pandas and matplotlib on the PC; "
        "do not install them on the K230.".format(exc.name)
    )


DEFAULT_ANGLES_DEG = [0.0, 30.0, 60.0, 90.0]
REQUIRED_CSV_COLUMNS = {
    "direction",
    "servo_angle_deg",
    "pwm_us",
    "rail_angle_deg",
    "measurement_method",
    "notes",
}


def approximate_rail_angle_deg(
    servo_angle_deg,
    r_servo_cm=1.5,
    l_drive_cm=24.8,
    servo_zero_deg=0.0,
    rail_zero_offset_deg=0.0,
):
    """Return the first-order rail-angle prediction in degrees.

    theta = rail_zero_offset + atan(r_servo * sin(alpha) / L_drive)
    alpha is the commanded servo angle relative to servo_zero_deg.
    """
    if l_drive_cm <= 0.0:
        raise ValueError("l_drive_cm must be greater than zero")
    if r_servo_cm < 0.0:
        raise ValueError("r_servo_cm must not be negative")

    servo_angles = np.asarray(servo_angle_deg, dtype=float)
    relative_rad = np.deg2rad(servo_angles - float(servo_zero_deg))
    rail_rad = np.arctan(float(r_servo_cm) * np.sin(relative_rad) / float(l_drive_cm))
    return float(rail_zero_offset_deg) + np.rad2deg(rail_rad)


def load_measurements(csv_path):
    """Load CSV rows that contain numeric servo and rail angles."""
    frame = pd.read_csv(csv_path)
    missing = REQUIRED_CSV_COLUMNS.difference(frame.columns)
    if missing:
        raise ValueError("CSV missing columns: {}".format(", ".join(sorted(missing))))

    frame = frame.copy()
    frame["servo_angle_deg"] = pd.to_numeric(frame["servo_angle_deg"], errors="coerce")
    frame["rail_angle_deg"] = pd.to_numeric(frame["rail_angle_deg"], errors="coerce")
    frame["direction"] = frame["direction"].fillna("unknown").astype(str)
    return frame.dropna(subset=["servo_angle_deg", "rail_angle_deg"])


def _least_squares(name, design, target, coefficient_names):
    """Fit one linear-in-parameters model and report basic diagnostics."""
    coefficients, _, rank, _ = np.linalg.lstsq(design, target, rcond=None)
    if rank < design.shape[1]:
        return None

    predicted = design @ coefficients
    residual = target - predicted
    rmse = float(np.sqrt(np.mean(residual * residual)))
    centered = target - np.mean(target)
    total_square = float(np.sum(centered * centered))
    if total_square > 0.0:
        r_squared = 1.0 - float(np.sum(residual * residual)) / total_square
    else:
        r_squared = float("nan")

    return {
        "name": name,
        "coefficients": coefficients,
        "coefficient_names": coefficient_names,
        "rmse_deg": rmse,
        "r_squared": r_squared,
    }


def fit_models(measurements, servo_zero_deg=0.0, polynomial_degree=2):
    """Fit linear, sine-scale, and low-order polynomial models."""
    x = measurements["servo_angle_deg"].to_numpy(dtype=float)
    y = measurements["rail_angle_deg"].to_numpy(dtype=float)
    models = []

    if len(x) >= 2 and len(np.unique(x)) >= 2:
        linear = _least_squares(
            "linear",
            np.column_stack((np.ones_like(x), x)),
            y,
            ["b0", "b1"],
        )
        if linear is not None:
            models.append(linear)

        relative_rad = np.deg2rad(x - float(servo_zero_deg))
        sine = _least_squares(
            "sine_scale",
            np.column_stack((np.ones_like(x), np.sin(relative_rad))),
            y,
            ["b0", "k_sin"],
        )
        if sine is not None:
            models.append(sine)

    required_points = polynomial_degree + 1
    if len(x) >= required_points and len(np.unique(x)) >= required_points:
        polynomial = _least_squares(
            "polynomial_degree_{}".format(polynomial_degree),
            np.vander(x, N=polynomial_degree + 1, increasing=True),
            y,
            ["c{}".format(index) for index in range(polynomial_degree + 1)],
        )
        if polynomial is not None:
            polynomial["degree"] = polynomial_degree
            models.append(polynomial)

    return models


def predict_fitted_model(model, servo_angles_deg, servo_zero_deg=0.0):
    """Evaluate a model returned by fit_models()."""
    x = np.asarray(servo_angles_deg, dtype=float)
    coefficients = model["coefficients"]

    if model["name"] == "linear":
        return coefficients[0] + coefficients[1] * x
    if model["name"] == "sine_scale":
        return coefficients[0] + coefficients[1] * np.sin(
            np.deg2rad(x - float(servo_zero_deg))
        )
    if model["name"].startswith("polynomial_degree_"):
        return np.vander(x, N=model["degree"] + 1, increasing=True) @ coefficients
    raise ValueError("unsupported fitted model: {}".format(model["name"]))


def print_prediction_table(angles, predictions):
    """Print a compact theory table suitable for experiment notes."""
    table = pd.DataFrame(
        {
            "servo_angle_deg": np.asarray(angles, dtype=float),
            "model_rail_angle_deg": np.asarray(predictions, dtype=float),
        }
    )
    print("\nFirst-order theoretical prediction (not calibrated):")
    print(table.to_string(index=False, float_format=lambda value: "{:.4f}".format(value)))


def print_fit_summary(models, sample_count):
    """Print coefficients and errors without claiming physical validation."""
    print("\nValid measured samples: {}".format(sample_count))
    if not models:
        print("Not enough complete, independent samples for fitting.")
        return

    print("Fitted models describe this CSV only; they are not validated control models.")
    for model in models:
        terms = []
        for name, value in zip(model["coefficient_names"], model["coefficients"]):
            terms.append("{}={:.8g}".format(name, value))
        r_squared = model["r_squared"]
        r_squared_text = "nan" if np.isnan(r_squared) else "{:.6f}".format(r_squared)
        print(
            "- {}: {}; RMSE={:.6f} deg; R^2={}".format(
                model["name"], ", ".join(terms), model["rmse_deg"], r_squared_text
            )
        )


def draw_plot(
    grid_angles,
    theory_angles,
    measurements,
    models,
    servo_zero_deg,
    output_path=None,
    show=True,
):
    """Draw theory, measurements, and available fitted curves."""
    if not show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    figure, axis = plt.subplots(figsize=(9, 5.5))
    axis.plot(grid_angles, theory_angles, label="first-order theory", linewidth=2.0)

    if measurements is not None and not measurements.empty:
        for direction, group in measurements.groupby("direction", dropna=False):
            axis.scatter(
                group["servo_angle_deg"],
                group["rail_angle_deg"],
                s=34,
                label="measured: {}".format(direction),
            )

    for model in models:
        axis.plot(
            grid_angles,
            predict_fitted_model(model, grid_angles, servo_zero_deg),
            linestyle="--",
            label="fit: {}".format(model["name"]),
        )

    axis.axhline(0.0, color="black", linewidth=0.8, alpha=0.5)
    axis.axvline(float(servo_zero_deg), color="black", linewidth=0.8, alpha=0.5)
    axis.set_xlabel("Servo angle (deg)")
    axis.set_ylabel("Rail angle (deg)")
    axis.set_title("Servo angle vs. rail angle (unvalidated model/calibration)")
    axis.grid(True, alpha=0.3)
    axis.legend()
    figure.tight_layout()

    if output_path is not None:
        output_path = Path(output_path)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        figure.savefig(output_path, dpi=160)
        print("Plot saved to: {}".format(output_path.resolve()))
    if show:
        plt.show()
    plt.close(figure)


def build_argument_parser():
    parser = argparse.ArgumentParser(
        description="PC-only Ball-and-Beam mechanism approximation and calibration fitter."
    )
    parser.add_argument(
        "--angles",
        nargs="*",
        type=float,
        default=DEFAULT_ANGLES_DEG,
        help="servo angles for the printed theoretical prediction table",
    )
    parser.add_argument("--csv", type=Path, help="optional calibration CSV")
    parser.add_argument("--r-servo-cm", type=float, default=1.5)
    parser.add_argument("--l-drive-cm", type=float, default=24.8)
    parser.add_argument("--servo-zero-deg", type=float, default=0.0)
    parser.add_argument("--rail-zero-offset-deg", type=float, default=0.0)
    parser.add_argument("--servo-limit-deg", type=float, default=90.0)
    parser.add_argument("--poly-degree", type=int, default=2, choices=range(2, 6))
    parser.add_argument("--plot-output", type=Path, help="optional PNG/PDF output path")
    parser.add_argument("--no-show", action="store_true", help="do not open a plot window")
    parser.add_argument("--no-plot", action="store_true", help="skip plot generation")
    return parser


def main(argv=None):
    args = build_argument_parser().parse_args(argv)
    if args.servo_limit_deg <= 0.0:
        raise SystemExit("--servo-limit-deg must be greater than zero")

    angles = args.angles if args.angles else DEFAULT_ANGLES_DEG
    predictions = approximate_rail_angle_deg(
        angles,
        r_servo_cm=args.r_servo_cm,
        l_drive_cm=args.l_drive_cm,
        servo_zero_deg=args.servo_zero_deg,
        rail_zero_offset_deg=args.rail_zero_offset_deg,
    )
    print_prediction_table(angles, predictions)

    measurements = None
    models = []
    if args.csv is not None:
        try:
            measurements = load_measurements(args.csv)
        except (OSError, ValueError, pd.errors.ParserError) as exc:
            raise SystemExit("Cannot load calibration CSV: {}".format(exc))
        models = fit_models(measurements, args.servo_zero_deg, args.poly_degree)
        print_fit_summary(models, len(measurements))

    if not args.no_plot:
        limit = float(args.servo_limit_deg)
        grid_angles = np.linspace(-limit, limit, 361)
        grid_theory = approximate_rail_angle_deg(
            grid_angles,
            r_servo_cm=args.r_servo_cm,
            l_drive_cm=args.l_drive_cm,
            servo_zero_deg=args.servo_zero_deg,
            rail_zero_offset_deg=args.rail_zero_offset_deg,
        )
        draw_plot(
            grid_angles,
            grid_theory,
            measurements,
            models,
            args.servo_zero_deg,
            output_path=args.plot_output,
            show=not args.no_show,
        )
    return 0


if __name__ == "__main__":
    sys.exit(main())
