#!/usr/bin/env python3
"""Reproduce landing figures from the GTSAM 4.3 tag and published timings.

Run with the py312 environment:
  python scripts/build_release_figures.py --gtsam-repo ../gtsam
Notebook images are extracted unchanged; numerical plots are generated here.
"""
import argparse
import base64
import json
import os
from pathlib import Path
import subprocess
import tempfile

os.environ.setdefault("MPLCONFIGDIR", tempfile.mkdtemp(prefix="gtsam-figures-"))
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
import numpy as np
import plotly.graph_objects as go

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "assets/images/release-43"
REF = "97b7a5e3b8edce204320397f00b502f57952fb38"  # GTSAM 4.3.0


def source(repo, path):
    return subprocess.check_output(["git", "-C", str(repo), "show", f"{REF}:{path}"])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--gtsam-repo", type=Path, required=True)
    args = parser.parse_args()
    OUT.mkdir(parents=True, exist_ok=True)
    notebook = json.loads(source(args.gtsam_repo,
        "python/gtsam/examples/slam/Pose2SLAMExample.ipynb"))
    plot_cell = next(cell for cell in notebook["cells"]
                    if "Optimized Poses with Covariance Ellipses" in "".join(cell.get("source", [])))
    image = next(output["data"]["image/png"] for output in plot_cell["outputs"]
                 if "image/png" in output.get("data", {}))
    OUT.joinpath("pose2-notebook.png").write_bytes(base64.b64decode(
        "".join(image) if isinstance(image, list) else image))
    range_notebook = json.loads(source(args.gtsam_repo,
        "python/gtsam/examples/slam/RangeSLAMExample_plaza2.ipynb"))
    range_plot = next(output["data"]["application/vnd.plotly.v1+json"]
                      for cell in reversed(range_notebook["cells"])
                      for output in cell.get("outputs", [])
                      if "application/vnd.plotly.v1+json" in output.get("data", {}))
    # Plotly 6 removed this unused trace type from older default templates.
    range_plot["layout"]["template"]["data"].pop("heatmapgl", None)
    # Render the notebook's saved final figure without changing its data or styling.
    go.Figure(range_plot).write_image(OUT / "plaza2-notebook.png",
                                      width=900, height=650, scale=2)
    OUT.joinpath("hybrid-bayes-tree.svg").write_bytes(source(args.gtsam_repo,
        "gtsam/hybrid/doc/figures/DataAssociationHybridBayesTree.svg"))

    plt.rcParams.update({
        "font.family": "sans-serif", "font.size": 12,
        "text.color": "#332f3b", "axes.labelcolor": "#655d70",
        "xtick.color": "#655d70", "ytick.color": "#332f3b",
        "svg.fonttype": "none", "svg.hashsalt": "gtsam-release-43",
    })
    cuda_chart()
    cuda_chart(mobile=True)
    solver_chart()
    print(f"Generated six release assets in {OUT}")


def save(fig, name):
    fig.savefig(OUT / name, facecolor="white", metadata={"Date": None})
    plt.close(fig)


def cuda_chart(mobile=False):
    # Values reported in _posts/2026-08-20-cuda-backend.md.
    # Reported speedups are retained, rather than recomputed from rounded times.
    groups = [
        ("General CUDA · best reported speedups", ["2D pose graphs", "3D pose graphs",
          "Stereo SLAM / VO", "BAL SfM"], [3.64, 4.70, 3.58, 6.08], "#7525c6"),
        ("GPU-resident SfM · dense Schur", ["BAL · 16 cameras", "BAL · 88 cameras",
          "BAL · 135 cameras"], [9.56, 8.72, 7.95], "#a32c82"),
    ]
    fig, axes = plt.subplots(2, 1, figsize=(4.2, 7.4) if mobile else (8.2, 6.8), sharex=True,
                             gridspec_kw={"height_ratios": [4, 3]})
    fig.subplots_adjust(left=.07 if mobile else .28, right=.88 if mobile else .91,
                        top=.76 if mobile else .78, bottom=.12 if mobile else .10,
                        hspace=.78 if mobile else .72)
    fig.suptitle("CUDA speedup over CPU", x=.05, y=.98, ha="left",
                 fontsize=16 if mobile else 20, fontweight="semibold", color="#321852")
    subtitle = "NVIDIA A100\nComplete optimizer wall time" if mobile else "NVIDIA A100 · complete optimizer wall time"
    fig.text(.05, .89 if mobile else .90, subtitle, fontsize=11 if mobile else 12,
             color="#655d70")
    for ax, (title, labels, values, color) in zip(axes, groups):
        if mobile:
            title = title.replace(" · ", "\n")
        ax.set_title(title, loc="left", pad=15, fontsize=12, fontweight="semibold")
        positions = np.arange(len(values)) * (1.6 if mobile else 1)
        ax.barh(positions, values, color=color, height=.55, zorder=2)
        if mobile:
            ax.set_yticks([])
            ax.set_ylim(positions[-1] + .65, -.95)
            for y, label in zip(positions, labels):
                ax.text(0, y - .42, label, va="bottom", fontsize=11)
        else:
            ax.set_yticks(positions, labels)
            ax.invert_yaxis()
        ax.set_xlim(0, 10.8)
        for y, value in zip(positions, values):
            ax.text(value + .12, y, f"{value:.2f}×", va="center",
                    fontsize=11 if mobile else 12, fontweight="semibold")
        ax.axvline(1, color="#95899f", lw=1, ls=(0, (3, 3)), zorder=1)
        ax.spines[["top", "right", "left"]].set_visible(False)
        ax.spines["bottom"].set_color("#ddd6e5")
        ax.tick_params(axis="y", length=0, pad=10)
        ax.tick_params(axis="x", length=3)
        ax.set_xticks([0, 2, 4, 6, 8, 10])
    axes[0].tick_params(axis="x", labelbottom=True)
    axes[1].set_xlabel("Speedup (×)\nDashed line = CPU parity" if mobile else
                      "Speedup (×); dashed line = CPU parity", labelpad=8)
    save(fig, "cuda-speedups-mobile.svg" if mobile else "cuda-speedups.svg")


def solver_chart():
    # The unit-noise chain in MultifrontalSolver.ipynb:
    # x0 = 0, x1 - x0 = 1, x2 - x1 = 1, x3 - x2 = 1.
    A = np.eye(4)
    A[np.arange(1, 4), np.arange(3)] = -1
    b = np.array([0., 1., 1., 1.])
    H = A.T @ A
    R = np.linalg.cholesky(H).T
    np.testing.assert_allclose(R.T @ R, H, atol=1e-12)
    np.testing.assert_allclose(np.linalg.solve(A, b), np.arange(4.))
    fig, axes = plt.subplots(1, 2, figsize=(8.2, 4.2))
    fig.subplots_adjust(left=.08, right=.97, top=.74, bottom=.13, wspace=.3)
    fig.suptitle("Sparsity in a Gaussian chain", x=.05, y=.97, ha="left",
                 fontsize=20, fontweight="semibold", color="#321852")
    fig.text(.05, .86, "Four scalar states · unit noise · ordering x₀, x₁, x₂, x₃",
             fontsize=12, color="#655d70")
    cmap = ListedColormap(["#ffffff", "#7525c6"])
    for ax, matrix, title in zip(axes, [H, R], ["H = AᵀA", "R   (H = RᵀR)"]):
        ax.imshow(np.abs(matrix) > 1e-12, cmap=cmap, vmin=0, vmax=1)
        ax.set_title(title, fontsize=15, pad=10)
        ax.set_xticks(range(4), ["x₀", "x₁", "x₂", "x₃"])
        ax.set_yticks(range(4), ["x₀", "x₁", "x₂", "x₃"])
        ax.tick_params(length=0)
        for (i, j), value in np.ndenumerate(matrix):
            if abs(value) > 1e-12:
                label = f"{value:g}" if matrix is H else f"{value:.2f}"
                ax.text(j, i, label, ha="center", va="center", color="white", fontsize=12)
        for spine in ax.spines.values():
            spine.set_color("#ddd6e5")
    save(fig, "multifrontal-chain.svg")


if __name__ == "__main__":
    main()
