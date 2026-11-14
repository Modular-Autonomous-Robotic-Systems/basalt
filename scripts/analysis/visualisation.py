import numpy as np
import plotly.graph_objects as go
from bokeh.layouts import column
from bokeh.models import CustomJS, DataRange1d, Div, FactorRange
from bokeh.palettes import Category20_20
from bokeh.plotting import figure as bokeh_figure
from plotly.subplots import make_subplots

LEGEND_TITLE = "Experiment"

_TOOLS = "pan,wheel_zoom,box_zoom,reset,save"
_MARKERS = ["circle", "square", "triangle", "diamond", "inverted_triangle", "cross", "x", "star"]

_GROUND_TRUTH_COLOUR = "green"
_RUN_COLOURS: dict[str, str] = {}

# Plotly's own default legend sits inside the plot frame, top right, over
# whatever data happens to be there; x=1.02 (just past the right edge of
# the data area, which ends at x=1 in paper coordinates) moves it into the
# figure's own margin instead, no overlap, and still part of the same
# figure Plotly's own camera icon rasterises to PNG, so a downloaded image
# keeps its legend.
_OUTSIDE_LEGEND = dict(x=1.02, y=1, xanchor="left", yanchor="top")


def _legend_outside(fig: bokeh_figure) -> None:
    """Bokeh's own documented way to move a legend from its default
    position, inside the plot frame, into a side panel outside the data
    area: add_layout re-parents the Legend annotation into a margin region
    rather than overlaying it on the central frame. A side panel is still
    part of the same figure, so the toolbar's Save tool (PNG export)
    includes it exactly as if it had stayed inside. A figure given no runs
    has no legend, and is left as it is."""
    if not fig.legend:
        return
    fig.add_layout(fig.legend[0], "right")


def _colour_for(run: str) -> str:
    """Deterministic, so the same experiment id is drawn in the same colour
    on every plot across the whole dashboard, independent of which other
    experiments happen to be present in any one given plot's own data,
    essential both for a legend that means the same thing on every tab and
    for figures exported for publication. Colours are assigned in first
    discovery order and cached at module scope for the lifetime of this
    process; panel serve re-executes dashboard.py per browser session but
    --logdir is fixed for the life of one server process, so every session
    discovers runs in the same deterministic order (DataReader.experiment_ids
    is sorted once, every _analyse_* loop iterates it in that same order)
    and ends up with the same mapping regardless of which session happens to
    populate the cache first. "ground truth" is excluded from the experiment
    cycle and always drawn in the same fixed colour instead, so it never
    collides with a real run's colour."""
    if run == "ground truth":
        return _GROUND_TRUTH_COLOUR
    if run not in _RUN_COLOURS:
        _RUN_COLOURS[run] = Category20_20[len(_RUN_COLOURS) % len(Category20_20)]
    return _RUN_COLOURS[run]


def seed_run_colours(experiment_ids: list[str]) -> None:
    """Populates _RUN_COLOURS for every id up front, in the given (sorted)
    order, before any thread or process pool work starts. _colour_for's own
    lazy, first-touch population is a race under concurrency, a plain dict
    with no lock under threads, and a cache that would diverge per process
    under multiprocessing; calling this first turns it into a read-only
    cache for the rest of the run regardless of what touches it afterwards."""
    for exp in experiment_ids:
        _colour_for(exp)


def _size(fig: go.Figure, width: int | None, height: int | None) -> go.Figure:
    return fig.update_layout(width=width, height=height)


def time_series(t_ns_per_run: dict[str, list], y_per_run: dict[str, list], title: str,
                 xaxis_title: str, yaxis_title: str, log_y: bool = False,
                 marker_index_per_run: dict[str, np.ndarray] | None = None,
                 x_range: DataRange1d | None = None, width: int | None = None,
                 height: int | None = None) -> bokeh_figure:
    """Bokeh, not Plotly, specifically so x_range can be the SAME DataRange1d
    instance class Analysis passes in for every linked plot on the page,
    including ones on other tabs, Bokeh's own linked panning mechanism, a
    figure whose x_range is another figure's x_range object pans and zooms
    together with it with no JavaScript of our own. Passing None (the 5 call
    sites whose x axis is a global iteration index, not t_ns) builds this
    figure its own fresh, independent DataRange1d instead, the same unlinked
    behaviour those call sites always had; bokeh_figure's own x_range
    property rejects a literal None, it only defaults sensibly when the
    kwarg is omitted entirely, so None can't simply be passed through.
    output_backend="webgl" is Bokeh's equivalent of Plotly's Scattergl, one
    canvas per renderer regardless of point count, for the same
    several-thousand-sample-per-run reason Scattergl was chosen originally.
    marker_index_per_run optionally overlays a second, marker only series at
    the given indices. color is explicit (_colour_for), unlike Plotly, Bokeh
    does not auto cycle a colour per scatter() call, every renderer defaults
    to the same colour otherwise."""
    fig = bokeh_figure(title=title, x_axis_label=xaxis_title, y_axis_label=yaxis_title,
                       x_range=x_range or DataRange1d(), y_axis_type="log" if log_y else "linear",
                       output_backend="webgl", width=width or 700, height=height or 450,
                       tools=_TOOLS, active_scroll="wheel_zoom")
    for run, y in y_per_run.items():
        fig.scatter(t_ns_per_run[run], y, size=4, legend_label=run, color=_colour_for(run))
        if marker_index_per_run and run in marker_index_per_run:
            idx = marker_index_per_run[run]
            fig.scatter(np.asarray(t_ns_per_run[run])[idx], np.asarray(y)[idx],
                        size=8, marker="circle_x", legend_label=f"{run} marker", color=_colour_for(run))
    fig.legend.title = LEGEND_TITLE
    fig.legend.click_policy = "hide"
    _legend_outside(fig)
    return fig


def multi_axis_time_series(t_ns_per_run: dict[str, list], quantities: dict[str, dict[str, list]],
                            title: str, xaxis_title: str, yaxis_title: str,
                            x_range: DataRange1d | None = None, width: int | None = None,
                            height: int | None = None):
    """One Bokeh figure per quantity, stacked in a column under a Div
    carrying the overall title, Bokeh layouts have no title of their own the
    way a single Plotly figure does. Every subplot is given the SAME
    x_range, the one class Analysis passed in, which used to need
    make_subplots(shared_xaxes=True) for linking only the subplots of ONE
    quantity group; sharing the single cross-tab DataRange1d here gets both
    the within-figure linking and the cross-tab linking from one mechanism.
    Only the top subplot's legend is shown (click_policy="hide"), but each
    subplot is its own independent Bokeh figure with its own renderers,
    unlike Plotly's make_subplots where every subplot still belonged to one
    figure and one legendgroup already spanned all of them, so clicking the
    top legend alone would only hide that subplot's own renderer. The
    CustomJS callbacks below broadcast the top subplot's own renderer
    visibility to the same run's renderer in every other subplot; an
    explicit CustomJS on js_on_change("visible", ...), not the js_link
    convenience wrapper, which multiple Bokeh discourse threads report as
    unreliable for exactly this cross-figure visibility sync, the community
    confirmed working recipe is the explicit callback used here. One
    direction is enough since only the visible legend's clicks ever change
    .visible in the first place."""
    names = list(quantities)
    x_range = x_range or DataRange1d()
    per_fig_height = (height or min(300 * len(names), 2400)) // len(names)
    figs = []
    renderers_by_run: dict[str, list] = {}
    for i, name in enumerate(names):
        f = bokeh_figure(title=name, y_axis_label=yaxis_title, x_range=x_range,
                         x_axis_label=xaxis_title if i == len(names) - 1 else None,
                         output_backend="webgl", width=width or 1080, height=per_fig_height,
                         tools=_TOOLS, active_scroll="wheel_zoom")
        for run, y in quantities[name].items():
            r = f.scatter(t_ns_per_run[run], y, size=4, legend_label=run, color=_colour_for(run))
            renderers_by_run.setdefault(run, []).append(r)
        f.legend.click_policy = "hide"
        _legend_outside(f)
        if i > 0:
            f.legend.visible = False
        figs.append(f)
    for renderers in renderers_by_run.values():
        if len(renderers) < 2:
            continue
        leader, *followers = renderers
        leader.js_on_change("visible", CustomJS(
            args=dict(followers=followers),
            code="for (const r of followers) { r.visible = cb_obj.visible; }",
        ))
    return column(Div(text=f"<b>{title}</b>"), *figs, sizing_mode="stretch_width")


def multi_axis_histogram(quantities: dict[str, dict[str, list]], title: str, xaxis_title: str,
                          width: int | None = None, height: int | None = None) -> go.Figure:
    """One subplot per quantity, one histogram trace per experiment within it."""
    names = list(quantities)
    fig = make_subplots(rows=len(names), cols=1, subplot_titles=names, vertical_spacing=0.1)
    for i, name in enumerate(names, start=1):
        for run, values in quantities[name].items():
            fig.add_trace(go.Histogram(x=values, name=run, opacity=0.6, marker_color=_colour_for(run),
                                        legendgroup=run, showlegend=(i == 1)), row=i, col=1)
    fig.update_xaxes(title_text=xaxis_title, row=len(names), col=1)
    fig.update_yaxes(title_text="count")
    fig.update_layout(title=title, barmode="overlay",
                       legend=dict(title_text=LEGEND_TITLE, **_OUTSIDE_LEGEND))
    return _size(fig, width, height or min(350 * len(names), 2400))


def line(x_per_run: dict[str, list], y_per_run: dict[str, list], title: str,
         xaxis_title: str, yaxis_title: str, width: int | None = None, height: int | None = None) -> go.Figure:
    fig = go.Figure()
    for run, y in y_per_run.items():
        colour = _colour_for(run)
        fig.add_trace(go.Scatter(x=x_per_run[run], y=y, name=run, mode="lines+markers",
                                  line=dict(color=colour), marker=dict(color=colour)))
    fig.update_layout(title=title, xaxis_title=xaxis_title, yaxis_title=yaxis_title,
                       legend=dict(title_text=LEGEND_TITLE, **_OUTSIDE_LEGEND))
    return _size(fig, width, height)


def line_with_band(x_per_run: dict[str, np.ndarray], mean_per_run: dict[str, np.ndarray],
                    std_per_run: dict[str, np.ndarray], title: str, xaxis_title: str,
                    yaxis_title: str, width: int | None = None, height: int | None = None) -> go.Figure:
    fig = go.Figure()
    for run in mean_per_run:
        x, mean, std = np.asarray(x_per_run[run]), np.asarray(mean_per_run[run]), np.asarray(std_per_run[run])
        colour = _colour_for(run)
        fig.add_trace(go.Scatter(x=x, y=mean, name=run, mode="lines+markers", line=dict(color=colour)))
        fig.add_trace(go.Scatter(x=np.concatenate([x, x[::-1]]),
                                  y=np.concatenate([mean + std, (mean - std)[::-1]]),
                                  fill="toself", opacity=0.2, line=dict(width=0), fillcolor=colour,
                                  name=f"{run}, +/- 1 std", showlegend=False))
    fig.update_layout(title=title, xaxis_title=xaxis_title, yaxis_title=yaxis_title,
                       legend=dict(title_text=LEGEND_TITLE, **_OUTSIDE_LEGEND))
    return _size(fig, width, height)


def histogram(values_per_run: dict[str, list], title: str, xaxis_title: str, bins: int = 30,
              width: int | None = None, height: int | None = None) -> go.Figure:
    fig = go.Figure()
    for run, values in values_per_run.items():
        fig.add_trace(go.Histogram(x=values, name=run, nbinsx=bins, opacity=0.6, marker_color=_colour_for(run)))
    fig.update_layout(title=title, xaxis_title=xaxis_title, yaxis_title="count",
                       barmode="overlay", legend=dict(title_text=LEGEND_TITLE, **_OUTSIDE_LEGEND))
    return _size(fig, width, height)


def histogram_prebinned(bin_labels: list[str], counts_per_run: dict[str, list], title: str,
                         xaxis_title: str, width: int | None = None, height: int | None = None) -> go.Figure:
    """Histograms already binned at logging time, whose bin edges are fixed
    by the C++ logger, hence a grouped bar rather than go.Histogram."""
    fig = go.Figure()
    for run, counts in counts_per_run.items():
        fig.add_trace(go.Bar(x=bin_labels, y=counts, name=run, marker_color=_colour_for(run)))
    fig.update_layout(title=title, xaxis_title=xaxis_title, yaxis_title="count",
                       barmode="group", legend=dict(title_text=LEGEND_TITLE, **_OUTSIDE_LEGEND))
    return _size(fig, width, height)


def stacked_bar(counts_per_run: dict[str, dict], title: str, xaxis_title: str = "category",
                 width: int | None = None, height: int | None = None) -> go.Figure:
    fig = go.Figure()
    for run, counts in counts_per_run.items():
        fig.add_trace(go.Bar(x=list(counts.keys()), y=list(counts.values()), name=run,
                              marker_color=_colour_for(run)))
    fig.update_layout(title=title, xaxis_title=xaxis_title, yaxis_title="count",
                       barmode="stack", legend=dict(title_text=LEGEND_TITLE, **_OUTSIDE_LEGEND))
    return _size(fig, width, height)


def categorical_timeline(t_ns_per_run: dict[str, list], category_per_run: dict[str, list],
                          title: str, xaxis_title: str, x_range: DataRange1d | None = None,
                          width: int | None = None, height: int | None = None) -> bokeh_figure:
    """One row per run on a categorical y axis (bokeh.models.FactorRange).
    Colour is per run (_colour_for), consistent with every other plot on the
    dashboard; a run's distinct category values (0/1 flags typically) are
    told apart by marker shape instead of colour, cycling _MARKERS, so a
    run's identity reads the same way here as everywhere else. x_range
    shared the same way as time_series/multi_axis_time_series above, None
    building a fresh, independent DataRange1d instead of being passed
    through, bokeh_figure's own x_range property rejects a literal None."""
    runs = list(category_per_run)
    fig = bokeh_figure(title=title, x_axis_label=xaxis_title, y_axis_label="experiment",
                       x_range=x_range or DataRange1d(), y_range=FactorRange(*runs), output_backend="webgl",
                       width=width or 1080, height=height or 400,
                       tools=_TOOLS, active_scroll="wheel_zoom")
    for run, categories in category_per_run.items():
        categories = np.asarray(categories)
        values = sorted(set(categories.tolist()))
        for vi, value in enumerate(values):
            mask = categories == value
            fig.scatter(np.asarray(t_ns_per_run[run])[mask], [run] * int(mask.sum()),
                        size=5, marker=_MARKERS[vi % len(_MARKERS)], color=_colour_for(run),
                        legend_label=f"{run}, {value}")
    fig.legend.title = LEGEND_TITLE
    fig.legend.click_policy = "hide"
    _legend_outside(fig)
    return fig


def line_3d(xyz_per_run: dict[str, np.ndarray], title: str,
            colour_per_run: dict[str, np.ndarray] | None = None,
            width: int | None = None, height: int | None = None) -> go.Figure:
    """colour_per_run mode colours each run's own markers continuously along
    its length by its own per point ATE residual; showscale is set on only
    the first trace so N runs draw one colorbar, not N stacked ones. The
    discrete per run legend is always kept, moved to a horizontal strip
    below the plot rather than Plotly's own default top right corner inside
    the scene, which in colour_per_run mode would otherwise collide with
    the colorbar, and a 3D scene's own wide aspect leaves more room below
    than to either side; without colour_per_run each run is a flat line in
    its own colour, _colour_for(run), matching every other plot."""
    fig = go.Figure()
    for i, (run, xyz) in enumerate(xyz_per_run.items()):
        if colour_per_run:
            marker = dict(color=colour_per_run[run], colorscale="Viridis", size=2,
                          showscale=(i == 0), colorbar=dict(title="ATE (m)"))
            fig.add_trace(go.Scatter3d(x=xyz[:, 0], y=xyz[:, 1], z=xyz[:, 2], name=run,
                                        mode="markers", marker=marker))
        else:
            fig.add_trace(go.Scatter3d(x=xyz[:, 0], y=xyz[:, 1], z=xyz[:, 2], name=run,
                                        mode="lines", line=dict(color=_colour_for(run))))
    fig.update_layout(title=title,
                       legend=dict(title_text=LEGEND_TITLE, orientation="h",
                                   yanchor="top", y=-0.02, xanchor="left", x=0),
                       scene=dict(xaxis_title="x (m)", yaxis_title="y (m)", zaxis_title="z (m)"))
    return _size(fig, width, height)


def trajectory_3d_overlay(gt_xyz: np.ndarray, est_xyz_per_run: dict[str, np.ndarray], title: str,
                           width: int | None = None, height: int | None = None) -> go.Figure:
    """gt_xyz is the single combined ground truth trajectory, always drawn
    in _GROUND_TRUTH_COLOUR; est_xyz_per_run gets one line per experiment in
    its own _colour_for colour, consistent with every other plot."""
    fig = go.Figure()
    fig.add_trace(go.Scatter3d(x=gt_xyz[:, 0], y=gt_xyz[:, 1], z=gt_xyz[:, 2],
                                name="ground truth", mode="lines",
                                line=dict(color=_colour_for("ground truth"), width=4)))
    for run, xyz in est_xyz_per_run.items():
        fig.add_trace(go.Scatter3d(x=xyz[:, 0], y=xyz[:, 1], z=xyz[:, 2], name=run, mode="lines",
                                    line=dict(color=_colour_for(run))))
    fig.update_layout(title=title, legend=dict(title_text=LEGEND_TITLE, **_OUTSIDE_LEGEND),
                       scene=dict(xaxis_title="x (m)", yaxis_title="y (m)", zaxis_title="z (m)"))
    return _size(fig, width, height)


def heatmap(matrix_per_run: dict[str, np.ndarray], title: str, width: int | None = None,
            height: int | None = None) -> go.Figure:
    """One run at a time, a heatmap has no legend of its own to distinguish
    overlays."""
    run, matrix = next(iter(matrix_per_run.items()))
    fig = go.Figure(go.Heatmap(z=matrix, colorbar=dict(title="shared landmarks")))
    fig.update_layout(title=f"{title}, {run}", xaxis_title="keyframe index", yaxis_title="keyframe index")
    return _size(fig, width, height)


def diff_table(diff_df, title: str, width: int | None = None, height: int | None = None) -> go.Figure:
    fig = go.Figure(go.Table(
        header=dict(values=list(diff_df.columns)),
        cells=dict(values=[diff_df[c] for c in diff_df.columns]),
    ))
    fig.update_layout(title=title)
    return _size(fig, width, height)


def box_violin(values_per_run: dict[str, list], title: str, yaxis_title: str, kind: str = "box",
               width: int | None = None, height: int | None = None) -> go.Figure:
    fig = go.Figure()
    trace = go.Box if kind == "box" else go.Violin
    for run, values in values_per_run.items():
        fig.add_trace(trace(y=values, name=run, marker_color=_colour_for(run)))
    fig.update_layout(title=title, yaxis_title=yaxis_title, legend=dict(title_text=LEGEND_TITLE, **_OUTSIDE_LEGEND))
    return _size(fig, width, height)
