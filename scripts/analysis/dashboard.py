import argparse
import json
import os

import panel as pn
from bokeh.models import LayoutDOM

import metrics
from analysis import Analysis
from data_reader import DataReader

pn.extension("plotly", "tabulator")


def _center(component):
    """The pn.HSpacer() sandwich, Panel's own reliable centering idiom, a
    flexible empty space on each side of a fixed width component inside a
    row that itself stretches to the page width, rather than relying on
    align="center" alone, which does nothing when the component's parent is
    not already wider than the component."""
    return pn.Row(pn.HSpacer(), component, pn.HSpacer(), sizing_mode="stretch_width")


def _tab_pane(fig):
    """fig is whatever one _analyse_<tab> entry produced, a bokeh figure or
    layout for time_series/multi_axis_time_series/categorical_timeline,
    a plotly.graph_objects.Figure for every other shape. LayoutDOM is the
    common Bokeh base both a bare figure() and a layouts.column(...) share,
    so this dispatches correctly for multi_axis_time_series's own column of
    stacked subplots, not only a single figure."""
    return pn.pane.Bokeh(fig) if isinstance(fig, LayoutDOM) else pn.pane.Plotly(fig)


def _build_tab(figs: list[tuple[str, object]]) -> pn.Column:
    """Wrapped per tab by pn.param.ParamFunction(..., lazy=True) in _build,
    so this only runs, and only then attaches each figure's model to the
    live document, the first time that tab is selected. This is Panel's own
    documented pattern for genuinely on-demand Tabs content under
    dynamic=True, not pn.bind, which the Tabs reference does not use for
    this."""
    return pn.Column(*[_center(_tab_pane(fig)) for _, fig in figs], sizing_mode="stretch_width")


def _build(tabs: dict, metrics_rows: list[dict]) -> pn.Column:
    metrics_df = metrics.pivot_metrics(metrics_rows)
    categories = sorted(metrics_df["category"].unique().tolist()) if not metrics_df.empty else []
    header_filters = {"category": {"type": "list", "values": categories}}
    table = pn.widgets.Tabulator(metrics_df, header_filters=header_filters,
                                  layout="fit_data_table", pagination=None)
    tab_items = [("scalars", pn.Column(_center(table), sizing_mode="stretch_width"))]
    tab_items += [
        (tab.replace("_", " "), pn.param.ParamFunction(lambda figs=figs: _build_tab(figs),
                                                        lazy=True, name=tab))
        for tab, figs in tabs.items() if figs
    ]
    return pn.Column(pn.Tabs(*tab_items, dynamic=True, sizing_mode="stretch_width"),
                      sizing_mode="stretch_width")


def _parse_args():
    ap = argparse.ArgumentParser()
    ap.add_argument("--logdir", required=True)
    ap.add_argument("--config", default=os.path.join(os.path.dirname(__file__), "config.json"))
    ap.add_argument(
        "--workers", type=int, default=None,
        help="Thread/process pool size for per-experiment CSV loading and for the "
             "trajectory/excitation-covariance/BoW-PR-curve tabs' per-experiment compute. "
             "Default: min(experiments, os.cpu_count()). Pass 1 to disable pooling entirely. "
             "On a host where cupy is importable (analysis_container with --gpus), "
             "a value above 1 means every pooled worker process independently acquires "
             "its own CUDA context for the trajectory tab's RPE sweep, which can contend "
             "for a single GPU; see plans/analysis_module.md section 18.",
    )
    return ap.parse_args()


# Module level, not behind if __name__ == "__main__": panel serve/bokeh
# serve execute this whole script once per new browser session rather than
# importing it and calling a named entry point, the standard Bokeh server
# app model; --args passes --logdir/--config through to sys.argv unchanged.
# Analysis.run() still runs once, eagerly, synchronously, before _build is
# called, so metrics_rows and figures/<tab>/*.html are exactly as complete
# as they were when dashboard.py saved a static file; only _build_tab's own
# body, deferred by ParamFunction(lazy=True) above, waits for its tab to be
# selected.
_args = _parse_args()
with open(_args.config) as _fh:
    _config = json.load(_fh)
_reader = DataReader(_args.logdir, workers=_args.workers,
                     time_reference=_config.get("time_reference", "mission"),
                     liftoff_threshold_m=_config.get("liftoff_threshold_m", 0.15))
_analysis = Analysis(_reader, _config, _args.logdir, workers=_args.workers)
_analysis.run()
_build(_analysis.tabs, _analysis.metrics_rows).servable(title=f"SLAM analysis, {_args.logdir}")
