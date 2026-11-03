import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

# ============================================================
# Trial-by-trial data
# ============================================================

data = [
    # speed, trial, tau_cam, tau_gt, trigger_class, physical_outcome
    [1.0, 1,  0.384, 0.425, "In window", "Success"],
    [1.0, 2,  0.383, 0.424, "In window", "Success"],
    [1.0, 3,  0.375, 0.399, "In window", "Success"],
    [1.0, 4,  0.381, 0.449, "In window", "Success"],
    [1.0, 5,  0.375, 0.393, "In window", "Success"],
    [1.0, 6,  0.376, 0.499, "Early",     "Fail (non-contact)"],
    [1.0, 7,  0.384, 0.407, "In window", "Success"],
    [1.0, 8,  0.378, 0.432, "In window", "Success"],
    [1.0, 9,  0.384, 0.428, "In window", "Success"],
    [1.0, 10, 0.384, 0.381, "In window", "Success"],

    [0.9, 1,  0.375, 0.399, "In window", "Success"],
    [0.9, 2,  0.384, 0.515, "Early",     "Fail (non-contact)"],
    [0.9, 3,  0.384, 0.413, "In window", "Success"],
    [0.9, 4,  0.384, 0.430, "In window", "Success"],
    [0.9, 5,  0.378, 0.363, "Late",      "Success"],
    [0.9, 6,  0.375, 0.448, "In window", "Success"],
    [0.9, 7,  0.375, 0.430, "In window", "Success"],
    [0.9, 8,  0.375, 0.413, "In window", "Success"],
    [0.9, 9,  0.377, 0.547, "Early",     "Fail (non-contact)"],
    [0.9, 10, 0.377, 0.439, "In window", "Success"],

    [0.8, 1,  0.376, 0.405, "In window", "Success"],
    [0.8, 2,  0.380, 0.502, "Early",     "Success"],
    [0.8, 3,  0.384, 0.441, "In window", "Success"],
    [0.8, 4,  0.380, 0.437, "In window", "Success"],
    [0.8, 5,  0.382, 0.454, "In window", "Success"],
    [0.8, 6,  0.380, 0.420, "In window", "Success"],
    [0.8, 7,  0.381, 0.610, "Early",     "Fail (non-contact)"],
    [0.8, 8,  0.377, 0.443, "In window", "Success"],
    [0.8, 9,  0.384, 0.422, "In window", "Success"],
    [0.8, 10, 0.375, 0.386, "In window", "Fail (detached)"],

    [0.7, 1,  0.378, 0.446, "In window", "Success"],
    [0.7, 2,  0.381, 0.452, "In window", "Success"],
    [0.7, 3,  0.378, 0.517, "Early",     "Success"],
    [0.7, 4,  0.381, 0.415, "In window", "Success"],
    [0.7, 5,  0.376, 0.610, "Early",     "Fail (non-contact)"],
    [0.7, 6,  0.382, 0.459, "In window", "Success"],
    [0.7, 7,  0.384, 0.406, "In window", "Success"],
    [0.7, 8,  0.378, 0.432, "In window", "Success"],
    [0.7, 9,  0.375, 0.634, "Early",     "Fail (non-contact)"],
    [0.7, 10, 0.381, 0.488, "Early",     "Success"],

    [0.6, 1,  np.nan, np.nan, "No trigger", "No flip/abort"],
    [0.6, 2,  0.375, 0.458, "In window", "Success"],
    [0.6, 3,  0.379, 0.464, "Early",     "Success"],
    [0.6, 4,  0.375, 0.885, "Early",     "Fail (non-contact)"],
    [0.6, 5,  0.379, 0.382, "In window", "Fail"],
    [0.6, 6,  0.381, 0.441, "In window", "Success"],
    [0.6, 7,  0.384, 0.552, "Early",     "Success"],
    [0.6, 8,  0.382, 0.490, "Early",     "Success"],
    [0.6, 9,  0.380, 0.502, "Early",     "Success"],
    [0.6, 10, 0.381, 0.434, "In window", "Success"],

    [0.5, 1,  0.376, 0.447, "In window", "Success"],
    [0.5, 2,  0.384, 0.471, "Early",     "Fail (detached)"],
    [0.5, 3,  0.375, 0.499, "Early",     "Success"],
    [0.5, 4,  0.378, 0.391, "In window", "Fail"],
    [0.5, 5,  np.nan, np.nan, "No trigger", "No flip/abort"],
    [0.5, 6,  0.377, 0.384, "In window", "Fail"],
    [0.5, 7,  0.384, 0.624, "Early",     "Success"],
    [0.5, 8,  0.378, 0.438, "In window", "Success"],
    [0.5, 9,  0.379, 0.751, "Early",     "Fail (non-contact)"],
    [0.5, 10, 0.375, 0.527, "Early",     "Success"],
]

df = pd.DataFrame(
    data,
    columns=[
        "speed",
        "trial",
        "tau_cam",
        "tau_gt",
        "trigger_class",
        "physical_outcome",
    ],
)

# ============================================================
# Plot settings
# ============================================================

TARGET_LOW = 0.38
TARGET_HIGH = 0.46
NO_TRIGGER_Y = 0.335

speed_ticks = [0.5, 0.6, 0.7, 0.8, 0.9, 1.0]

# Fixed trial-based jitter for reproducibility.
trial_offsets = np.linspace(-0.028, 0.028, 10)
df["x_jitter"] = df["speed"] + df["trial"].apply(lambda t: trial_offsets[int(t) - 1])

# Physical outcome colors
outcome_colors = {
    "Success": "tab:green",
    "Fail": "tab:red",                 # plain Fail = red
    "Fail (non-contact)": "tab:blue",  # non-contact Fail = blue
    "Fail (detached)": "tab:orange",
    "No flip/abort": "0.45",
}

# Trigger-class markers
trigger_markers = {
    "In window": "o",
    "Early": "^",
    "Late": "v",
    "No trigger": "X",
}

plt.rcParams.update({
    "font.size": 9,
    "axes.labelsize": 10,
    "xtick.labelsize": 9,
    "ytick.labelsize": 9,
    "legend.fontsize": 8,
    "axes.linewidth": 0.8,
})


# ============================================================
# Legend helpers
# ============================================================

def get_outcome_handles():
    return [
        Line2D(
            [0], [0],
            marker="o",
            linestyle="None",
            markersize=7,
            markerfacecolor=outcome_colors["Success"],
            markeredgecolor="black",
            label="Success",
        ),
        Line2D(
            [0], [0],
            marker="o",
            linestyle="None",
            markersize=7,
            markerfacecolor=outcome_colors["Fail"],
            markeredgecolor="black",
            label="Fail",
        ),
        Line2D(
            [0], [0],
            marker="o",
            linestyle="None",
            markersize=7,
            markerfacecolor=outcome_colors["Fail (non-contact)"],
            markeredgecolor="black",
            label="Fail (non-contact)",
        ),
        Line2D(
            [0], [0],
            marker="o",
            linestyle="None",
            markersize=7,
            markerfacecolor=outcome_colors["Fail (detached)"],
            markeredgecolor="black",
            label="Fail (detached)",
        ),
        Line2D(
            [0], [0],
            marker="X",
            linestyle="None",
            markersize=7,
            markerfacecolor=outcome_colors["No flip/abort"],
            markeredgecolor="black",
            color="black",
            label="No flip/abort",
        ),
    ]


def get_trigger_handles():
    return [
        Line2D(
            [0], [0],
            marker="o",
            linestyle="None",
            markersize=7,
            markerfacecolor="white",
            markeredgecolor="black",
            color="black",
            label="In window",
        ),
        Line2D(
            [0], [0],
            marker="^",
            linestyle="None",
            markersize=7,
            markerfacecolor="white",
            markeredgecolor="black",
            color="black",
            label="Early",
        ),
        Line2D(
            [0], [0],
            marker="v",
            linestyle="None",
            markersize=7,
            markerfacecolor="white",
            markeredgecolor="black",
            color="black",
            label="Late",
        ),
        Line2D(
            [0], [0],
            marker="X",
            linestyle="None",
            markersize=7,
            markerfacecolor="white",
            markeredgecolor="black",
            color="black",
            label="No trigger",
        ),
    ]


def add_inside_legends(ax):
    leg1 = ax.legend(
        handles=get_outcome_handles(),
        title="Physical outcome",
        loc="upper left",
        frameon=True,
        fontsize=7,
        title_fontsize=8,
    )
    ax.add_artist(leg1)

    ax.legend(
        handles=get_trigger_handles(),
        title="Trigger class",
        loc="upper right",
        frameon=True,
        fontsize=7,
        title_fontsize=8,
    )


def save_figure(fig, png_name, pdf_name):
    fig.savefig(
        png_name,
        dpi=300,
        bbox_inches="tight",
        pad_inches=0.05,
    )
    fig.savefig(
        pdf_name,
        bbox_inches="tight",
        pad_inches=0.05,
    )


# ============================================================
# Option 1:
# Speed vs tau_gt scatter
# ============================================================

fig, ax = plt.subplots(figsize=(7.4, 4.6), dpi=300)
fig.subplots_adjust(left=0.12, right=0.98, bottom=0.16, top=0.96)

# Target TTC band
ax.axhspan(
    TARGET_LOW,
    TARGET_HIGH,
    alpha=0.18,
    zorder=0,
)

ax.axhline(TARGET_LOW, linestyle="--", linewidth=1.0, zorder=1)
ax.axhline(TARGET_HIGH, linestyle="--", linewidth=1.0, zorder=1)

# No-trigger reference line
ax.axhline(NO_TRIGGER_Y, linestyle=":", linewidth=1.2, color="0.25", zorder=1)
ax.text(
    1.045,
    NO_TRIGGER_Y,
    "No trigger",
    va="center",
    ha="right",
    fontsize=9,
    color="0.15",
)

for _, row in df.iterrows():
    color = outcome_colors[row["physical_outcome"]]
    marker = trigger_markers[row["trigger_class"]]

    # Draw no-trigger cases separately.
    if row["trigger_class"] == "No trigger":
        ax.scatter(
            row["x_jitter"],
            NO_TRIGGER_Y,
            s=95,
            marker="X",
            facecolors=color,
            edgecolors="black",
            linewidths=0.9,
            zorder=7,
            clip_on=False,
        )
        continue

    ax.scatter(
        row["x_jitter"],
        row["tau_gt"],
        s=58,
        marker=marker,
        facecolors=color,
        edgecolors="black",
        linewidths=0.8,
        zorder=4,
    )

ax.set_xlabel("Approach speed [m/s]")
ax.set_ylabel(r"Ground-truth TTC at trigger, $\tau_{\mathrm{gt}}$ [s]")
ax.set_xticks(speed_ticks)
ax.set_xlim(0.45, 1.05)
ax.set_ylim(0.30, 0.92)
ax.grid(True, alpha=0.25)

add_inside_legends(ax)

save_figure(
    fig,
    "option1_speed_vs_tau_gt_scatter.png",
    "option1_speed_vs_tau_gt_scatter.pdf",
)

plt.show()


# ============================================================
# Option 2:
# Paired tau_cam vs tau_gt plot
# ============================================================

fig, ax = plt.subplots(figsize=(7.4, 4.8), dpi=300)
fig.subplots_adjust(left=0.12, right=0.98, bottom=0.16, top=0.96)

# Target TTC band
ax.axhspan(
    TARGET_LOW,
    TARGET_HIGH,
    alpha=0.18,
    zorder=0,
)

ax.axhline(TARGET_LOW, linestyle="--", linewidth=1.0, zorder=1)
ax.axhline(TARGET_HIGH, linestyle="--", linewidth=1.0, zorder=1)

# No-trigger reference line
ax.axhline(NO_TRIGGER_Y, linestyle=":", linewidth=1.2, color="0.25", zorder=1)
ax.text(
    1.045,
    NO_TRIGGER_Y,
    "No trigger",
    va="center",
    ha="right",
    fontsize=9,
    color="0.15",
)

for _, row in df.iterrows():
    color = outcome_colors[row["physical_outcome"]]
    marker = trigger_markers[row["trigger_class"]]
    x = row["x_jitter"]

    # Draw no-trigger cases separately.
    if row["trigger_class"] == "No trigger":
        ax.scatter(
            x,
            NO_TRIGGER_Y,
            s=95,
            marker="X",
            facecolors=color,
            edgecolors="black",
            linewidths=0.9,
            zorder=7,
            clip_on=False,
        )
        continue

    # Same-trial connection between tau_cam and tau_gt
    ax.plot(
        [x, x],
        [row["tau_cam"], row["tau_gt"]],
        color="0.65",
        linewidth=0.8,
        zorder=2,
    )

    # tau_cam: open circle
    ax.scatter(
        x,
        row["tau_cam"],
        s=44,
        marker="o",
        facecolors="white",
        edgecolors=color,
        linewidths=1.2,
        zorder=4,
    )

    # tau_gt: filled marker using trigger class
    ax.scatter(
        x,
        row["tau_gt"],
        s=58,
        marker=marker,
        facecolors=color,
        edgecolors="black",
        linewidths=0.8,
        zorder=5,
    )

ax.set_xlabel("Approach speed [m/s]")
ax.set_ylabel(r"TTC at trigger [s]")
ax.set_xticks(speed_ticks)
ax.set_xlim(0.45, 1.05)
ax.set_ylim(0.30, 0.92)
ax.grid(True, alpha=0.25)

# First legend: tau_cam and tau_gt
tau_handles = [
    Line2D(
        [0], [0],
        marker="o",
        linestyle="None",
        markersize=7,
        markerfacecolor="white",
        markeredgecolor="black",
        color="black",
        label=r"$\tau_{\mathrm{cam}}$",
    ),
    Line2D(
        [0], [0],
        marker="o",
        linestyle="None",
        markersize=7,
        markerfacecolor="black",
        markeredgecolor="black",
        color="black",
        label=r"$\tau_{\mathrm{gt}}$",
    ),
    Line2D(
        [0], [0],
        color="0.65",
        linewidth=1.0,
        label="Same trial",
    ),
]

leg0 = ax.legend(
    handles=tau_handles,
    title="TTC values",
    loc="upper center",
    frameon=True,
    fontsize=7,
    title_fontsize=8,
    ncol=3,
)
ax.add_artist(leg0)

# Second and third legends
leg1 = ax.legend(
    handles=get_outcome_handles(),
    title="Physical outcome",
    loc="upper left",
    frameon=True,
    fontsize=7,
    title_fontsize=8,
)
ax.add_artist(leg1)

ax.legend(
    handles=get_trigger_handles(),
    title="Trigger class",
    loc="upper right",
    frameon=True,
    fontsize=7,
    title_fontsize=8,
)

save_figure(
    fig,
    "option2_tau_cam_tau_gt_paired.png",
    "option2_tau_cam_tau_gt_paired.pdf",
)

plt.show()