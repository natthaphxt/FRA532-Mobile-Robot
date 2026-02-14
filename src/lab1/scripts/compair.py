import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import os


def load_and_process_data(file_path):
    """Reads CSV and splits into segments if timestamps reset."""
    if not os.path.exists(file_path):
        print(f"[ERROR] File not found: {file_path}")
        return []
    try:
        df = pd.read_csv(file_path)
        time = df["timestamp"].to_numpy()
        x = df["x"].to_numpy()
        y = df["y"].to_numpy()

        reset_indices = np.where(np.diff(time) < -1.0)[0] + 1
        segments = []
        start_idx = 0
        split_points = list(reset_indices) + [len(time)]

        for end_idx in split_points:
            seg_x = x[start_idx:end_idx]
            seg_y = y[start_idx:end_idx]
            seg_t = time[start_idx:end_idx]
            if len(seg_x) > 10:
                segments.append({"x": seg_x, "y": seg_y, "time": seg_t})
            start_idx = end_idx
        return segments
    except Exception as e:
        print(f"[ERROR] Loading {file_path}: {e}")
        return []


def calculate_errors(ref_seg, test_seg):
    """Calculates Euclidean distance error relative to a reference (SLAM as ground truth)."""
    errors = []
    for tx, ty in zip(test_seg["x"], test_seg["y"]):
        # Find minimum distance to any point on the reference path
        dist = np.sqrt((ref_seg["x"] - tx) ** 2 + (ref_seg["y"] - ty) ** 2)
        errors.append(np.min(dist))
    return np.array(errors)


def plot_comprehensive_analysis():
    base_path = "/home/natthaphxt/mobile_ws/"

    # 1. LOAD DATA
    raw_segs = load_and_process_data(os.path.join(base_path, "raw_trajectory.csv"))
    ekf_segs = load_and_process_data(os.path.join(base_path, "ekf_trajectory.csv"))
    icp_segs = load_and_process_data(os.path.join(base_path, "icp_trajectory.csv"))
    slam_segs = load_and_process_data(os.path.join(base_path, "slam_path.csv"))

    if not raw_segs or not ekf_segs or not icp_segs:
        print("[ERROR] Could not load all required data. Check file paths.")
        return

    if not slam_segs:
        print("[ERROR] SLAM data not found. SLAM is required as ground truth.")
        print("[INFO] Please ensure 'slam_path.csv' exists in the base path.")
        return

    print(f"[INFO] Loaded SLAM data as GROUND TRUTH: {len(slam_segs[0]['x'])} points")
    print(f"[INFO] Loaded Raw Odometry: {len(raw_segs[0]['x'])} points")
    print(f"[INFO] Loaded EKF Fused: {len(ekf_segs[0]['x'])} points")
    print(f"[INFO] Loaded ICP LiDAR: {len(icp_segs[0]['x'])} points")

    # 2. TRAJECTORY VISUALIZATION (Consistent Scaling)
    all_x, all_y = [], []
    data_lists = [raw_segs, ekf_segs, icp_segs, slam_segs]

    for s_list in data_lists:
        for s in s_list:
            all_x.extend(s["x"])
            all_y.extend(s["y"])

    x_margin = (max(all_x) - min(all_x)) * 0.1
    y_margin = (max(all_y) - min(all_y)) * 0.1
    xlim = (min(all_x) - x_margin, max(all_x) + x_margin)
    ylim = (min(all_y) - y_margin, max(all_y) + y_margin)

    # Updated subplot layout: 2 rows x 3 columns (wider figure for external legends)
    fig1, axes = plt.subplots(2, 3, figsize=(22, 12))
    fig1.subplots_adjust(
        left=0.05, right=0.95, top=0.94, bottom=0.06, wspace=0.25, hspace=0.20
    )
    axes = axes.flatten()

    titles = [
        "Raw Odometry",
        "EKF Fused",
        "ICP Lidar",
        "SLAM Toolbox (Ground Truth)",
        "ALL PATHS OVERLAY",
        "Comparison vs SLAM Ground Truth",
    ]
    colors = ["darkred", "darkblue", "darkgreen", "darkorange"]

    def style_ax(ax, title):
        ax.set_title(title, fontsize=14, fontweight="bold")
        ax.set_xlim(xlim)
        ax.set_ylim(ylim)
        ax.set_aspect("equal", adjustable="box")
        ax.grid(True, linestyle="--", alpha=0.5)
        ax.set_xlabel("X (m)")
        ax.set_ylabel("Y (m)")

    # Plot individual trajectories
    data_to_plot = [raw_segs, ekf_segs, icp_segs, slam_segs]

    for idx, (segs, col, title) in enumerate(zip(data_to_plot, colors, titles[:4])):
        ax = axes[idx]
        style_ax(ax, title)
        for s in segs:
            ax.plot(s["x"], s["y"], color=col, linewidth=2)

            # Get start and stop coordinates
            start_x, start_y = s["x"][0], s["y"][0]
            stop_x, stop_y = s["x"][-1], s["y"][-1]

            # Smart positioning: check relative positions to avoid overlap
            if start_x < stop_x:
                # Start is to the left of Stop
                start_offset = (-40, 0)
                stop_offset = (8, 0)
            else:
                # Start is to the right of Stop
                start_offset = (8, 0)
                stop_offset = (-35, 0)

            # If they're close vertically, offset them more
            if abs(start_y - stop_y) < 2 and abs(start_x - stop_x) < 3:
                start_offset = (-40, 15)
                stop_offset = (8, -15)

            # Start point (black circle)
            ax.scatter(start_x, start_y, color="black", s=60, zorder=5)
            ax.annotate(
                "Start",
                xy=(start_x, start_y),
                xytext=start_offset,
                textcoords="offset points",
                fontsize=9,
                fontweight="bold",
                color="black",
            )
            # Stop point (red square)
            ax.scatter(stop_x, stop_y, color="red", marker="s", s=60, zorder=5)
            ax.annotate(
                "Stop",
                xy=(stop_x, stop_y),
                xytext=stop_offset,
                textcoords="offset points",
                fontsize=9,
                fontweight="bold",
                color="red",
            )
        # Highlight SLAM as ground truth
        if idx == 3:
            ax.annotate(
                "GROUND TRUTH",
                xy=(0.5, 0.02),
                xycoords="axes fraction",
                ha="center",
                fontsize=12,
                color="darkorange",
                fontweight="bold",
                bbox=dict(boxstyle="round", facecolor="white", alpha=0.8),
            )

    # ALL PATHS OVERLAY - Legend in lower right, smaller size
    style_ax(axes[4], titles[4])
    axes[4].plot(
        slam_segs[0]["x"],
        slam_segs[0]["y"],
        color="orange",
        linewidth=3,
        label="SLAM (GT)",
        zorder=5,
    )
    axes[4].plot(
        raw_segs[0]["x"],
        raw_segs[0]["y"],
        color="red",
        linestyle="--",
        alpha=0.5,
        linewidth=1.5,
        label="Raw Odom",
    )
    axes[4].plot(
        ekf_segs[0]["x"],
        ekf_segs[0]["y"],
        color="blue",
        linewidth=2,
        alpha=0.7,
        label="EKF Fused",
    )
    axes[4].plot(
        icp_segs[0]["x"],
        icp_segs[0]["y"],
        color="green",
        linestyle="-.",
        linewidth=2,
        label="ICP LiDAR",
    )
    # Legend in lower right corner, compact size
    axes[4].legend(
        loc="lower right", fontsize=8, frameon=True, framealpha=0.9, edgecolor="gray"
    )

    # Comparison vs SLAM Ground Truth - Legend in lower right, smaller size
    style_ax(axes[5], titles[5])
    axes[5].plot(
        slam_segs[0]["x"],
        slam_segs[0]["y"],
        color="orange",
        linewidth=3,
        label="SLAM (GT)",
        zorder=5,
    )
    axes[5].plot(
        ekf_segs[0]["x"],
        ekf_segs[0]["y"],
        color="blue",
        linewidth=2,
        alpha=0.7,
        label="EKF",
    )
    axes[5].plot(
        icp_segs[0]["x"],
        icp_segs[0]["y"],
        color="green",
        linestyle="-.",
        linewidth=2,
        label="ICP (LiDAR)",
    )
    # Legend in lower right corner, compact size
    axes[5].legend(
        loc="lower right", fontsize=8, frameon=True, framealpha=0.9, edgecolor="gray"
    )

    # 3. ERROR COMPARISON (Using SLAM as Ground Truth)
    ref = slam_segs[0]  # SLAM as ground truth
    err_raw = calculate_errors(ref, raw_segs[0])
    err_ekf = calculate_errors(ref, ekf_segs[0])
    err_icp = calculate_errors(ref, icp_segs[0])

    fig2, axes2 = plt.subplots(1, 3, figsize=(22, 6), constrained_layout=True)

    ax_err = axes2[0]
    ax_bar = axes2[1]
    ax_box = axes2[2]

    # Error vs Samples
    ax_err.plot(
        err_raw,
        color="red",
        alpha=0.5,
        linewidth=1.5,
        label=f"Raw Error (Mean: {np.mean(err_raw):.3f}m)",
    )
    ax_err.plot(
        err_ekf,
        color="blue",
        linewidth=2,
        label=f"EKF Error (Mean: {np.mean(err_ekf):.3f}m)",
    )
    ax_err.plot(
        err_icp,
        color="green",
        linewidth=2,
        linestyle="-.",
        label=f"ICP Error (Mean: {np.mean(err_icp):.3f}m)",
    )
    ax_err.set_title(
        "Drift Over Time (Reference: SLAM Toolbox)", fontsize=14, fontweight="bold"
    )
    ax_err.set_xlabel("Samples")
    ax_err.set_ylabel("Error (meters)")
    ax_err.grid(True, alpha=0.3)
    ax_err.legend(loc="upper left")

    # Bar Chart for Summary
    methods = ["Raw Odometry", "EKF Fused", "ICP LiDAR"]
    means = [np.mean(err_raw), np.mean(err_ekf), np.mean(err_icp)]
    maxs = [np.max(err_raw), np.max(err_ekf), np.max(err_icp)]
    stds = [np.std(err_raw), np.std(err_ekf), np.std(err_icp)]

    x_axis = np.arange(len(methods))
    width = 0.25

    ax_bar.bar(
        x_axis - width,
        means,
        width,
        label="Mean Error",
        color="skyblue",
        edgecolor="black",
    )
    ax_bar.bar(
        x_axis, maxs, width, label="Max Error", color="orange", edgecolor="black"
    )
    ax_bar.bar(
        x_axis + width,
        stds,
        width,
        label="Std Dev",
        color="lightgreen",
        edgecolor="black",
    )
    ax_bar.set_xticks(x_axis)
    ax_bar.set_xticklabels(methods, rotation=15, ha="right")
    ax_bar.set_ylabel("Error (meters)")
    ax_bar.set_title(
        "Statistical Accuracy Comparison (vs SLAM)", fontsize=14, fontweight="bold"
    )
    ax_bar.legend(loc="upper right")
    ax_bar.grid(axis="y", alpha=0.3)

    # Box Plot
    error_data = [err_raw, err_ekf, err_icp]
    bp = ax_box.boxplot(
        error_data, labels=methods, patch_artist=True, notch=True, showmeans=True
    )
    colors_box = ["lightcoral", "lightblue", "lightgreen"]
    for patch, color in zip(bp["boxes"], colors_box):
        patch.set_facecolor(color)

    ax_box.set_title(
        "Error Distribution (Reference: SLAM)", fontsize=14, fontweight="bold"
    )
    ax_box.set_ylabel("Error (meters)")
    ax_box.grid(axis="y", alpha=0.3)
    ax_box.set_xticklabels(methods, rotation=15, ha="right")

    # 4. PRINT STATISTICS
    print("\n" + "=" * 70)
    print("TRAJECTORY COMPARISON STATISTICS (Ground Truth: SLAM Toolbox)")
    print("=" * 70)

    print(f"\n{'Method':<20} {'Mean (m)':<12} {'Max (m)':<12} {'Std Dev (m)':<12}")
    print("-" * 70)
    print(
        f"{'Raw Odometry':<20} {np.mean(err_raw):<12.4f} {np.max(err_raw):<12.4f} {np.std(err_raw):<12.4f}"
    )
    print(
        f"{'EKF Fused':<20} {np.mean(err_ekf):<12.4f} {np.max(err_ekf):<12.4f} {np.std(err_ekf):<12.4f}"
    )
    print(
        f"{'ICP LiDAR':<20} {np.mean(err_icp):<12.4f} {np.max(err_icp):<12.4f} {np.std(err_icp):<12.4f}"
    )

    # Improvement Analysis
    print("\n" + "-" * 70)
    print("IMPROVEMENT ANALYSIS (vs Raw Odometry):")
    print("-" * 70)
    ekf_improvement = ((np.mean(err_raw) - np.mean(err_ekf)) / np.mean(err_raw)) * 100
    icp_improvement = ((np.mean(err_raw) - np.mean(err_icp)) / np.mean(err_raw)) * 100

    print(f"EKF improvement over Raw:  {ekf_improvement:>6.2f}%")
    print(f"ICP improvement over Raw:  {icp_improvement:>6.2f}%")

    print("\n" + "-" * 70)
    print("METHOD COMPARISON:")
    print("-" * 70)
    icp_vs_ekf = ((np.mean(err_ekf) - np.mean(err_icp)) / np.mean(err_ekf)) * 100
    print(f"ICP vs EKF difference:     {icp_vs_ekf:>6.2f}%")

    # Rank methods by accuracy
    print("\n" + "-" * 70)
    print("ACCURACY RANKING (Best to Worst):")
    print("-" * 70)
    rankings = sorted(
        [
            ("Raw Odometry", np.mean(err_raw)),
            ("EKF Fused", np.mean(err_ekf)),
            ("ICP LiDAR", np.mean(err_icp)),
        ],
        key=lambda x: x[1],
    )
    for rank, (method, error) in enumerate(rankings, 1):
        print(f"  {rank}. {method:<20} Mean Error: {error:.4f} m")

    print("=" * 70 + "\n")

    plt.show()


if __name__ == "__main__":
    plot_comprehensive_analysis()
