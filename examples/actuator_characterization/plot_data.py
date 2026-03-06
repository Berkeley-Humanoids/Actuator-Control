"""
Plot characterization test results.
"""

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


def _get_results_list(data: np.lib.npyio.NpzFile):
    """If NPZ has 'results', return list of (t, q_cmd, theta_out, omega_out, label). Raises if missing."""
    if "results" not in data:
        raise ValueError('NPZ must contain "results" (from run_erob_characterization.py)')
    results = data["results"]
    if results.ndim == 0:
        results = [results.item()]
    else:
        results = list(results.flatten())
    out = []
    for r in results:
        t = np.asarray(r["times"]).flatten()
        q_cmd = np.asarray(r["target_positions"]).flatten()
        theta_out = np.asarray(r["measured_positions"]).flatten()
        omega_out = np.asarray(r["measured_velocities"]).flatten()
        bt = r.get("brake_torque", "")
        idx = r.get("config_index", len(out))
        label = f"config {idx}" + (f" (brake={bt} Nm)" if bt != "" else "")
        out.append((t, q_cmd, theta_out, omega_out, label))
    return out


def plot_characterization(
    data_path: Path,
    output_path: Path | None = None,
    show: bool = True,
    dpi: int = 150,
    config_index: int | None = None,
) -> None:
    data = np.load(data_path, allow_pickle=True)
    results_list = _get_results_list(data)
    data.close()

    if config_index is not None:
        if not (0 <= config_index < len(results_list)):
            raise ValueError(f"--config {config_index} out of range [0, {len(results_list)})")
        results_list = [results_list[config_index]]

    n = len(results_list[0][0])
    if n == 0:
        raise ValueError(f"No samples in {data_path}")

    colors = plt.cm.tab10(np.linspace(0, 1, max(len(results_list), 1)))
    fig, axes = plt.subplots(3, 1, sharex=True, figsize=(10, 8))

    for k, (t, q_cmd, theta_out, omega_out, label) in enumerate(results_list):
        c = colors[k % len(colors)]
        error = theta_out - q_cmd
        ax0, ax1, ax2 = axes[0], axes[1], axes[2]
        ax0.plot(t, q_cmd, color=c, linewidth=0.8, alpha=0.7)
        ax0.plot(t, theta_out, color=c, linewidth=0.6, linestyle="--", alpha=0.9, label=label)
        ax1.plot(t, omega_out, color=c, linewidth=0.8, label=label)
        ax2.plot(t, error, color=c, linewidth=0.8, label=label)
    axes[0].set_ylabel("Position (rad)")
    axes[0].set_title("Tracking: $q_{cmd}$ (solid) vs $\\theta_{out}$ (dashed)")
    axes[0].legend(loc="upper right")
    axes[0].grid(True)
    axes[1].set_ylabel("Velocity (rad/s)")
    axes[1].set_title("Measured velocity $\\omega_{out}$")
    axes[1].legend(loc="upper right")
    axes[1].grid(True)
    axes[2].axhline(0, color="k", linewidth=0.5, linestyle="--")
    axes[2].set_xlabel("Time (s)")
    axes[2].set_ylabel("Error (rad)")
    axes[2].set_title("Position error $\\theta_{out} - q_{cmd}$")
    axes[2].legend(loc="upper right")
    axes[2].grid(True)

    fig.tight_layout()

    if output_path is not None:
        output_path = Path(output_path)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(output_path, dpi=dpi)
        plt.close(fig)
        print(f"Saved plot to {output_path}")
    if show:
        plt.show()
    elif output_path is None:
        plt.close(fig)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Plot characterization test results.")
    parser.add_argument(
        "data",
        type=Path,
        nargs="?",
        default=Path("./data/characterization_data.npz"),
        help="Path to characterization NPZ (default: ./data/characterization_data.npz)",
    )
    parser.add_argument(
        "--output", "-o",
        type=Path,
        default=None,
        help="Save figure to path (e.g. ./data/characterization_plot.png)",
    )
    parser.add_argument("--no-show", action="store_true", help="Do not show interactive plot (only save if -o given)")
    parser.add_argument("--dpi", type=int, default=150, help="DPI for saved figure")
    parser.add_argument("--config", type=int, default=None, help="If NPZ has multiple configs, plot only this result index")
    args = parser.parse_args()

    plot_characterization(
        data_path=args.data,
        output_path=args.output,
        show=not args.no_show,
        dpi=args.dpi,
        config_index=args.config,
    )
