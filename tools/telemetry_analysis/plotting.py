from __future__ import annotations

from pathlib import Path
from typing import Any

import matplotlib

matplotlib.use("Agg")

from matplotlib import pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402

from .frames import canonicalize_telemetry_frame


MAX_PLOT_POINTS = 2000


def _plot_frame(frame: pd.DataFrame) -> pd.DataFrame:
    if len(frame) <= MAX_PLOT_POINTS:
        return frame
    indices = np.linspace(0, len(frame) - 1, MAX_PLOT_POINTS, dtype=int)
    return frame.iloc[np.unique(indices)]


def write_multiplot_svg(
    path: str | Path,
    frame: pd.DataFrame,
    title: str,
    panels: list[dict[str, Any]],
    *,
    x_label: str = "Time (s)",
) -> None:
    data = _plot_frame(canonicalize_telemetry_frame(frame))
    figure, axes = plt.subplots(
        len(panels),
        1,
        sharex=True,
        figsize=(12, 3.0 * len(panels)),
        constrained_layout=True,
        squeeze=False,
    )
    figure.suptitle(title)
    time = data.get("t_sec", pd.Series(index=data.index, dtype=float))

    for axis, panel in zip(axes[:, 0], panels):
        plotted = False
        for key, color, label in panel["series"]:
            if key not in data.columns:
                continue
            available = time.notna() & data[key].notna()
            if not available.any():
                continue
            axis.plot(time[available], data.loc[available, key], color=color, label=label)
            plotted = True

        axis.set_title(panel["title"], loc="left")
        axis.set_ylabel(panel["y_label"])
        axis.grid(True, color="#E5E7EB", linewidth=0.8)
        axis.axhline(0.0, color="#9CA3AF", linewidth=0.8, linestyle="--")
        if panel.get("center_zero") and plotted:
            limits = axis.get_ylim()
            bound = max(abs(limits[0]), abs(limits[1]))
            axis.set_ylim(-bound, bound)
        if plotted:
            axis.legend(loc="upper right", ncols=min(3, len(panel["series"])))

    axes[-1, 0].set_xlabel(x_label)
    figure.savefig(path, format="svg")
    plt.close(figure)
