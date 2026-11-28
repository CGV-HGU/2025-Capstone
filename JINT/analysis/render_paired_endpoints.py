"""Draw endpoint distances for the started pairs in a validated field log."""

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D


def render(pairs, sites, destination):
    by_site = {site: {} for site in sites}
    for pair_id, arms in pairs.items():
        site = next(iter(arms.values()))["site"]
        by_site[site][pair_id] = {
            arm: float(row["end_distance_measured_m"]) for arm, row in arms.items()
            if row["end_distance_measured_m"]
        }
    if not any(len(measured) == 2 for rows in by_site.values() for measured in rows.values()):
        destination.unlink(missing_ok=True)
        return
    active = [site for site, rows in by_site.items() if rows]
    max_rows = max(len(by_site[site]) for site in active)
    upper = max(.55, max(value for rows in by_site.values() for pair in rows.values()
                          for value in pair.values())) * 1.08
    styles = {"V-LiDAR": ("o", "#166968", -.09), "Comparator": ("D", "#904d23", .09)}
    with plt.rc_context({"pdf.fonttype": 42, "font.family": "DejaVu Sans"}):
        height = 1.50 + .24 * max_rows
        fig, axes = plt.subplots(1, len(active), squeeze=False, sharex=True, figsize=(5.13, height))
        fig.subplots_adjust(left=.11, right=.98, bottom=.52 / height, top=1-.68 / height, wspace=.38)
        for ax, site in zip(axes[0], active):
            ax.axvline(.50, color="#566672", linestyle=(0, (3, 3)), lw=.95)
            labels = []
            for index, (pair_id, values) in enumerate(sorted(by_site[site].items())):
                suffix = pair_id.rsplit("_", 1)[-1]
                labels.append("Pair " + suffix if suffix.isdecimal() else pair_id)
                if len(values) == 2:
                    ax.plot([values[arm] for arm in styles], [index + style[2] for style in styles.values()],
                            lw=1, color="#8c9ba2", zorder=2)
                if not values:
                    ax.text(.025, index, "—", transform=ax.get_yaxis_transform(),
                            ha="left", va="center", fontsize=8, color="#66747c")
                for arm, value in values.items():
                    marker, color, offset = styles[arm]
                    ax.plot(value, index + offset, marker=marker, ls="none", ms=5,
                            markerfacecolor=color if arm == "V-LiDAR" else "white",
                            markeredgecolor=color, markeredgewidth=1.15, clip_on=False, zorder=4)
            ax.set(xlim=(0, upper), ylim=(max_rows-.5, -.5), yticks=range(len(labels)), yticklabels=labels)
            ax.set_title(sites[site], fontsize=9, loc="left", fontweight="semibold")
            ax.locator_params(axis="x", nbins=4)
            ax.set_axisbelow(True)
            ax.grid(axis="y", color="#e0e6e8", lw=.65)
            ax.spines[["left", "top", "right"]].set_visible(False)
            ax.spines["bottom"].set_color("#9da9ad")
            ax.tick_params(axis="y", labelsize=8.1, length=0)
            ax.tick_params(axis="x", labelsize=8.1, length=3)
        fig.supxlabel("Measured endpoint error (m)", fontsize=8.2)
        handles = [Line2D([], [], marker=m, color=c, markerfacecolor=c if arm == "V-LiDAR" else "white",
                          ls="none", markersize=5, label=arm) for arm, (m, c, _) in styles.items()]
        handles.append(Line2D([], [], ls="--", color="#566672", lw=1, label="0.50 m radius"))
        fig.legend(handles=handles, ncol=3, loc="upper center", frameon=False,
                   fontsize=8.1, columnspacing=1.5, handlelength=1.3, handletextpad=.4)
        fig.savefig(destination, metadata={"Title": "Paired field endpoint measurements", "CreationDate": None})
        plt.close(fig)
