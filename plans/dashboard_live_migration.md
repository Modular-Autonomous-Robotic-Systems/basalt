# Live Bokeh Dashboard Migration

## 1. Motivation

The dashboard built in `plans/analysis_module.md` and `scripts/analysis/` renders every tab's figures into one static `dashboard.html` via `panel.save(..., resources=INLINE)`. That file grew slow to open as tabs were added (most recently `stationary_detection`), because `pn.Tabs` without a live server keeps every tab's content fully rendered in the DOM at once, merely hidden with `display:none`. The immediately preceding change addressed load time alone by splitting each tab into its own self-contained `tabs/<tab>.html`, loaded into an `<iframe>` only when that tab first becomes visible (`IntersectionObserver` in `_LAZY_TAB_SCRIPT`), with `_LINKED_ZOOM_SCRIPT` re-injected per sub-page so within-tab cross-figure zoom kept working. That traded away the one property the dashboard exists for during a debugging session, the ability to zoom one time series and have every other tab's time series, not just the current tab's own, jump to the same window, since each tab is now a separate document and the existing script only ever crawled one document's DOM.

This plan restores cross-tab linked zoom and keeps genuine lazy loading, by accepting the one tradeoff surfaced when the options were compared: giving up the static, mail-anywhere single HTML file in favour of a live `panel serve` process for the length of a debugging session. The two features stop being in tension once there is a live server, Bokeh's own linked-panning mechanism (one `DataRange1d` object shared across every figure's `x_range`, including figures in different tabs) requires nothing hand-rolled, and `pn.Tabs(dynamic=True)` finally does what it was tried for already, defer a tab's content until it is first opened, because this time there is a server left alive to build it on demand.

This supersedes `plans/analysis_module.md` §2.3's library verdict (Plotly + Panel + static `.save()`, chosen there for `requirement 16`'s own ease-of-use/install-size/offline criteria) and §10's `dashboard.py` design built on it, and it removes the iframe/`postMessage`-shaped infrastructure the immediately preceding turn had just added, before it was ever committed. Both are correct records of what they decided at the time; they are superseded here because the actual use of this tool, interactively comparing values across tabs at the same timestamp window while debugging a live SLAM run, needs a capability neither of those designs can give at once, not because either prior decision was wrong on its own terms.

## 2. Why no static-export option gives both features at once

Two static-friendly options were considered and rejected, recorded here so neither is silently reproposed later.

**`pn.Tabs(dynamic=True)` against a saved file.** The option's own documentation states it defers a tab's render only while a live Bokeh server is attached; against a plain `.save()`'d page there is no server left once the browser opens it, so every tab but whichever was active at save time stays permanently blank. This is why the previous turn built the iframe/`IntersectionObserver` scheme by hand instead, there was no working static equivalent.

**A `postMessage` hub between the already-split iframes.** Each tab's own script could report its pan/zoom range to the parent shell, which relays it to every other loaded iframe and replays it to any tab not yet opened the moment it signals ready. This keeps the static, open-anywhere file and was judged workable, `file://` pages get an unstable, often-opaque origin which rules out `BroadcastChannel`, but plain `postMessage` addressed via `iframe.contentWindow` and verified on receipt via `event.source` rather than `event.origin` does not depend on that origin at all. It was not chosen because it only ever gives range sync, a hand-written, iframe-boundary-crossing mechanism with no path to the richer cross-filtering a debugging session eventually wants (click a glitch in one tab, highlight the same frame everywhere), and because once a live server is accepted as unavoidable for genuine lazy loading, Bokeh's native shared `DataRange1d` gives the same sync with no hand-written JS at all, and with zero per-interaction server round trips, unlike re-deriving the sync through Python callbacks against a live Plotly server would.

## 3. High-level design

Only the plot shapes that ever carried the existing `link_zoom` flag move to Bokeh; every other shape is untouched.

`plans/analysis_module.md` §15 already establishes, and a grep against the live `scripts/analysis/analysis.py` confirms, that `link_zoom` is only ever passed by `visualisation.time_series`, `visualisation.multi_axis_time_series` and `visualisation.categorical_timeline`, the three shapes that plot against the shared `t_ns` axis. Every other shape, `histogram`, `stacked_bar`, `heatmap`, `line_3d`, `trajectory_3d_overlay`, `diff_table`, `box_violin`, `multi_axis_histogram`, `histogram_prebinned`, `line`, `line_with_band`, never needed cross-figure linking and stays exactly as it is, still a `plotly.graph_objects.Figure`, still rendered by `pn.pane.Plotly`. This confines the whole migration to three functions in `visualisation.py` and the call sites in `analysis.py` that pass `link_zoom=True` to them, currently 41 of the 46 total calls to those three functions (the other 5 plot a global solver iteration index, not `t_ns`, and were never linked either, see §4.2).

```
class Analysis (unchanged shape)
  __init__ gains self.shared_t_range = bokeh.models.DataRange1d()
  run() still eagerly calls every _analyse_<tab>(), still populates
        self.metrics_rows completely, still writes figures/<tab>/*.html
        for every figure, Plotly or Bokeh alike (see §4.2's _write_figure)
      |
      v
  self.tabs: dict[str, list[tuple[str, Figure]]]   Figure is now
      Plotly go.Figure (most shapes) or bokeh.plotting.figure
      (time_series/multi_axis_time_series/categorical_timeline)
      |
      v
dashboard.py, now a panel serve app, not a .save() script
  scalars tab built eagerly from analysis.metrics_rows, as today
  every other tab wrapped in pn.param.ParamFunction(fn, lazy=True),
      fn a closure returning pn.Column(pn.pane.Bokeh(fig) or
      pn.pane.Plotly(fig) for that tab's already-built figures)
  pn.Tabs(*tab_items, dynamic=True) only attaches a lazy tab's
      content to the live page the first time it is selected
```

The key property this relies on is that `Analysis.run()` itself stays exactly as eager as it is today, every tab's figures, every scalar metric and every `figures/<tab>/*.html` export are still produced in one pass, with nothing deferred. What `dynamic=True` plus the `lazy=True` wrapper defer is only the later, separate step of attaching an already-built figure's Bokeh model into the live page the browser is rendering, which is the step that actually costs browser load time, not the pandas/NumPy/figure-construction work, which was always comparatively fast and already paid once per session regardless. This is why lazy loading and complete, deterministic `metrics.csv`/`figures/` output are not in conflict here, unlike an earlier design considered and rejected in §6.

## 4. Detailed changes

### 4.1 `visualisation.py`, three functions ported to Bokeh

Add imports:

```python
from bokeh.plotting import figure
from bokeh.layouts import column
from bokeh.models import DataRange1d, Div, FactorRange
```

`DataRange1d` is imported here only for the type hint on the new `x_range` parameter; the one shared instance itself is created once by `class Analysis`, §4.2, and merely threaded through.

```python
def time_series(t_ns_per_run: dict[str, list], y_per_run: dict[str, list], title: str,
                 xaxis_title: str, yaxis_title: str, log_y: bool = False,
                 marker_index_per_run: dict[str, np.ndarray] | None = None,
                 x_range: DataRange1d | None = None, width: int | None = None,
                 height: int | None = None):
    """Bokeh replaces Plotly here specifically so x_range can be the SAME
    DataRange1d instance passed in from class Analysis for every linked plot on
    the page, including ones in other tabs, Bokeh's own linked panning
    mechanism, a figure whose x_range is another figure's x_range object
    pans and zooms together with it with no JavaScript of our own. Passing
    None (the 5 call sites whose x axis is a global iteration index, not
    t_ns) builds this figure its own fresh, independent DataRange1d instead,
    reproducing today's unlinked behaviour exactly; figure()'s own x_range
    property rejects a literal None outright (`ValueError: failed to
    validate FigureOptions(...).x_range`, hit live against the first of
    those 5 call sites, cost_curve, 2026-10-22), it only defaults sensibly
    when the kwarg is omitted entirely, which passing None through is not,
    so `x_range or DataRange1d()` is required at the call, not `x_range` alone.
    output_backend="webgl" is Bokeh's equivalent of Plotly's Scattergl, one
    canvas per renderer regardless of point count, for the same
    several-thousand-sample-per-run reason Scattergl was chosen originally."""
    fig = figure(title=title, x_axis_label=xaxis_title, y_axis_label=yaxis_title,
                 x_range=x_range or DataRange1d(), y_axis_type="log" if log_y else "linear",
                 output_backend="webgl", width=width or 700, height=height or 450,
                 tools="pan,wheel_zoom,box_zoom,reset,save", active_scroll="wheel_zoom")
    for run, y in y_per_run.items():
        fig.scatter(t_ns_per_run[run], y, size=4, legend_label=run)
        if marker_index_per_run and run in marker_index_per_run:
            idx = marker_index_per_run[run]
            fig.scatter(np.asarray(t_ns_per_run[run])[idx], np.asarray(y)[idx],
                        size=8, marker="circle_x", legend_label=f"{run} marker")
    fig.legend.title = LEGEND_TITLE
    fig.legend.click_policy = "hide"
    return fig


def multi_axis_time_series(t_ns_per_run: dict[str, list], quantities: dict[str, dict[str, list]],
                            title: str, xaxis_title: str, yaxis_title: str,
                            x_range: DataRange1d | None = None, width: int | None = None,
                            height: int | None = None):
    """One Bokeh figure per quantity, stacked in a column under a Div
    carrying the overall title (Bokeh layouts have no title of their own,
    unlike a single Plotly figure). Every subplot is given the SAME x_range,
    the one class Analysis passed in, which is what used to need
    make_subplots(shared_xaxes=True) for linking only the subplots of ONE
    quantity group; sharing the single cross-tab DataRange1d here gets both the
    within-figure linking and the cross-tab linking from one mechanism."""
    names = list(quantities)
    x_range = x_range or DataRange1d()
    per_fig_height = (height or min(300 * len(names), 2400)) // len(names)
    figs = []
    for i, name in enumerate(names):
        f = figure(title=name, y_axis_label=yaxis_title, x_range=x_range,
                   x_axis_label=xaxis_title if i == len(names) - 1 else None,
                   output_backend="webgl", width=width or 1080, height=per_fig_height,
                   tools="pan,wheel_zoom,box_zoom,reset,save", active_scroll="wheel_zoom")
        for run, y in quantities[name].items():
            f.scatter(t_ns_per_run[run], y, size=4, legend_label=run)
        f.legend.click_policy = "hide"
        if i > 0:
            f.legend.visible = False
        figs.append(f)
    return column(Div(text=f"<b>{title}</b>"), *figs, sizing_mode="stretch_width")


def categorical_timeline(t_ns_per_run: dict[str, list], category_per_run: dict[str, list],
                          title: str, xaxis_title: str, x_range: DataRange1d | None = None,
                          width: int | None = None, height: int | None = None):
    """One row per run on a categorical y axis (bokeh.models.FactorRange),
    one coloured marker series per distinct category value within it, the
    same shape the Plotly version drew. x_range shared the same way as the
    two functions above, `x_range or DataRange1d()` for the same reason."""
    runs = list(category_per_run)
    fig = figure(title=title, x_axis_label=xaxis_title, y_axis_label="experiment",
                 x_range=x_range or DataRange1d(), y_range=FactorRange(*runs), output_backend="webgl",
                 width=width or 1080, height=height or 400,
                 tools="pan,wheel_zoom,box_zoom,reset,save", active_scroll="wheel_zoom")
    for run, categories in category_per_run.items():
        categories = np.asarray(categories)
        for value in sorted(set(categories.tolist())):
            mask = categories == value
            fig.scatter(np.asarray(t_ns_per_run[run])[mask], [run] * int(mask.sum()),
                        size=5, legend_label=f"{run}, {value}")
    fig.legend.title = LEGEND_TITLE
    fig.legend.click_policy = "hide"
    return fig
```

The pre-existing `_size`/`LEGEND_TITLE` helper and every other Plotly-producing function (`line`, `line_with_band`, `histogram`, `histogram_prebinned`, `stacked_bar`, `trajectory_3d_overlay`, `heatmap`, `diff_table`, `box_violin`, `multi_axis_histogram`) are untouched. `line_3d` is the one exception, carrying an unrelated pre-existing fix from the same debugging pass, §4.6.

**Correction, 2026-10-22.** Every `Range1d` above was originally written as `bokeh.models.Range1d`, confirmed live-broken: `Range1d` is a fixed, manually bounded range, constructed with no `start`/`end` it does not auto-fit to data at all, rendering as a degenerate near-empty window (every linked plot came up blank, axis range roughly -3 to 0 regardless of the real `t_ns` domain). `bokeh.models.DataRange1d` is the type Bokeh itself creates by default whenever `x_range` is omitted from `figure()` entirely, auto-fitting to the union of every renderer attached to it, including every other figure sharing the same instance, which is what linked panning actually needs; every `Range1d` reference above, the import, the type hints, the `x_range or Range1d()` fallbacks and `class Analysis.shared_t_range`'s own construction, is `DataRange1d` in the code as implemented.

### 4.6 `line_3d`, an unrelated pre-existing bug surfaced by the same testing pass

`visualisation.line_3d`'s `color_by_error` mode (`trajectory_3d.color_by_error` in `config.json`) sets `marker.colorbar` on every trace, one `go.Scatter3d` per experiment, so N experiments drew N stacked colorbars overlapping the ordinary per-experiment legend in the same corner, illegible once more than two or three experiments are compared (reported directly against `/ws/logs/slam/`, roughly ten experiments at once). Pre-existing in the original Plotly design, unrelated to this migration, surfaced now only because comparing every logged experiment at once is a heavier real use than earlier testing exercised. Fixed by setting `showscale=True` only on the first trace, so one colorbar renders rather than N. **Correction, 2026-10-22.** The first fix also set `showlegend=False` on every trace in this mode, removing the legend entirely rather than repositioning it; reported back as a regression (the discrete legend is still wanted alongside the colorbar). Current fix keeps the legend always on and instead moves it to a horizontal strip below the plot (`legend=dict(orientation="h", y=-0.02, ...)`) only when `color_by_error` is active, so it no longer collides with the colorbar's own default top right corner; the ordinary flat-per-run-colour mode is unaffected, legend stays in its default position there.

### 4.8 Colour consistency across every plot, and cross-subplot legend toggling

Reported directly against `/ws/logs/slam/`: every experiment rendered in the same single colour on the new Bokeh figures, and `multi_axis_time_series`'s own legend, shown only on its top subplot, did not hide the same run's renderer in the other subplots when clicked. Two separate, unrelated causes. Bokeh does not auto cycle a colour per `scatter()`/glyph call the way Plotly auto cycles a colour per trace, every renderer left without an explicit `color=` defaults to the same one; fixed with `visualisation._colour_for(run)`, a module level, deterministic, first discovery order mapping cached for the life of the process (stable across `panel serve`'s per-session re-exec of `dashboard.py` because `--logdir` is fixed for that process's whole life, so every session discovers runs in the same order from the same sorted `DataReader.experiment_ids`), applied at every one of the 16 places across `visualisation.py` that colour by run, Bokeh and Plotly figures alike, so the same experiment id is the same colour on every tab, which the user flagged as required for consistent publication figures. `"ground truth"` is excluded from the experiment cycle and always drawn in a fixed colour instead, so it never collides with a real run's assigned colour. The cross-subplot legend issue is structural: Plotly's `make_subplots` kept every subplot inside one figure with one shared legend/trace list, so one `legendgroup` already spanned every subplot; `multi_axis_time_series`'s Bokeh replacement builds one independent `figure()` per subplot, each with its own renderers and its own legend, so toggling the one visible legend (top subplot only, the rest have `legend.visible = False`) only ever changed that subplot's own renderer. Fixed by capturing every `GlyphRenderer` `scatter()` returns, grouped by run across every subplot. **Correction, 2026-10-22.** The first fix wired this with `leader.js_link("visible", follower, "visible")`; reported back as still not propagating a click on the one visible legend to the other subplots. `js_link` is Bokeh's own convenience wrapper around `js_on_change` plus a `CustomJS`, and multiple Bokeh discourse threads report it as unreliable for exactly this cross-figure visibility sync case specifically, one poster's own words, "I did try to use js_link but without any success", the community's confirmed working recipe for this same problem is an explicit `CustomJS` attached via `js_on_change` instead. Current fix: `leader.js_on_change("visible", CustomJS(args=dict(followers=followers), code="for (const r of followers) { r.visible = cb_obj.visible; }"))`, `cb_obj` is Bokeh's own name, inside a `CustomJS`, for whichever object the callback fired on, the leader renderer itself here. One direction is enough since only the visible legend's clicks ever change `.visible` in the first place. `categorical_timeline` also moved from colouring by `(run, value)` pair to colouring by run alone via the same `_colour_for`, telling a run's own distinct category values apart by marker shape (`_MARKERS`, cycled) instead, so its legend swatches stay consistent with every other plot too.

### 4.9 `DataReader._scan_epochs`, a pre-existing bug that made per-experiment zeroing a no-op

Reported as "timestamps need to start from 0 so experiments are aligned" and confirmed, directly against `/ws/logs/slam/`, not to be a data problem, every experiment's own `t_ns`, across every one of its own CSVs, already zeroes correctly to within 108.8 to 185.5 seconds when `DataReader`'s documented algorithm is replicated by hand against the files on disk. `_scan_epochs`'s own `pd.read_csv` call, found carrying `header=None, skiprows=1` alongside `usecols=["t_ns"]`, is the actual fault, not something this migration touched, `data_reader.py` was never edited before this finding. `header=None` tells pandas there is no header row at all, so its columns become a plain integer `RangeIndex`, `0, 1, 2, ...`; `usecols=["t_ns"]` then asks pandas to select a column by that string name, which can never resolve against integer columns, so pandas raises `ValueError` on every single CSV read inside this function, caught and silently swallowed by the surrounding `except (ValueError, pd.errors.EmptyDataError): continue`. The result, `mins` stays empty for every experiment, `self._epoch_t_ns` ends up `{}` in its entirety, and `load()`'s own `exp in self._epoch_t_ns` guard is false for every experiment, so the per-experiment subtraction never runs at all, every experiment keeps its raw, real epoch-nanosecond `t_ns`, scattered across real historical recording time exactly as the fragmented, non-zero-starting axes reported. Fixed by removing `header=None, skiprows=1`, restoring the plain `pd.read_csv(path, usecols=["t_ns"])["t_ns"]` the rest of this plan, and `plans/analysis_module.md` §5, always documented.

### 4.10 `metrics.combine_ground_truth`, near duplicate strands instead of one reference curve

Reported as the plotted "ground truth" showing multiple trajectories instead of one, once §4.9's fix let every experiment's own ground truth actually reach its correct, zeroed position on a shared time axis. Confirmed directly against `/ws/logs/slam/`, every experiment genuinely does replay the same mission, ground truth positions from different experiments agree within a few metres at matching mission relative times throughout the flight (sampled at t=30/50/70/90/120/150s). The combination itself was the fault: concatenating every experiment's own ground truth and deduplicating on an exact `t_ns` match almost never actually drops anything, independently logged runs do not tick on the same nanosecond grid, so near identical but not exactly equal samples from every experiment all survive and render as several near identical strands braided together rather than one clean curve. Every experiment here already covers the full ~180 second mission with near complete coverage (774 to 829 samples each, no gaps one experiment's data would need to fill from another), so `combine_ground_truth` no longer attempts a merge at all, it returns the single experiment with the most logged ground truth samples, `max(t_ns_per_run, key=lambda exp: len(t_ns_per_run[exp]))`, deterministic and sufficient for data with this little real divergence between experiments. A fuzzy, time bucketed merge was considered and rejected, it would only earn its complexity back if some experiment genuinely needed another's samples to fill a gap, which is not the case for any experiment logged here.

### 4.11 Legend placement, every plot

Reported as the legend covering part of the plot, and that downloaded plot images must still carry the legend. Every `visualisation.py` function previously left its legend at each library's own default position, inside the plot frame, over whatever data happened to be there. Bokeh figures (`time_series`, `multi_axis_time_series`, `categorical_timeline`) now call `fig.add_layout(fig.legend[0], "right")`, Bokeh's own documented way to re-parent the Legend annotation into a side panel outside the central data frame rather than overlaid on it; a side panel is still part of the same figure, so the toolbar's Save tool (PNG export) still includes it. Every Plotly function moves its legend to `x=1.02, y=1, xanchor="left", yanchor="top"` (module constant `_OUTSIDE_LEGEND`), just past the right edge of the data area in paper coordinates, same reasoning, still part of the one figure Plotly's own camera icon rasterises. `line_3d`'s legend, which already had to dodge its own colorbar in `color_by_error` mode (§4.6), is unified to the same horizontal strip below the plot in both branches instead of only one, consistent behaviour regardless of mode and more headroom than a side panel would leave next to a 3D scene's own wide aspect.

### 4.2 `analysis.py`, the shared range and the export path

`__init__` gains the one shared range instance, created fresh per `Analysis` instance, which under `panel serve`'s per-session execution model (§4.3) means fresh per browser session, never reused across two concurrent sessions, which Bokeh models must not be:

```python
from bokeh.io import save as bokeh_save
from bokeh.models import DataRange1d, LayoutDOM
from bokeh.resources import INLINE

class Analysis:
    def __init__(self, reader: DataReader, config: dict, out_dir: str):
        self.reader, self.config, self.out_dir = reader, config, out_dir
        self.metrics_rows: list[dict] = []
        self.tabs: dict[str, list[tuple[str, object]]] = {}
        self.shared_t_range = DataRange1d()
```

Every call site currently passing `link_zoom=True` to `visualisation.time_series`/`multi_axis_time_series`/`categorical_timeline` has that one kwarg renamed to `x_range=self.shared_t_range`, a uniform textual substitution (`sed -i 's/link_zoom=True,/x_range=self.shared_t_range,/' analysis.py`), not 41 individually-designed edits, verified after the fact by `grep -c link_zoom analysis.py` returning 0 and `grep -c x_range=self.shared_t_range analysis.py` returning 41. The 5 call sites with no `link_zoom` kwarg at all (`cost_curve`, `damping_trace`, `cost_split`, `mapper_cost_curve`, `mapper_damping_accept_reject`, every one plotting a global solver iteration index rather than `t_ns`, named already in `plans/analysis_module.md` §15) are untouched, they pass no `x_range` either, so Bokeh gives each its own independent range, reproducing their current unlinked behaviour exactly.

`run()`'s figure-writing loop needs one type dispatch added, since `self.tabs` now holds a mix of Plotly and Bokeh objects and only the former has `.write_html()`. The dispatch is on `bokeh.models.LayoutDOM`, not `bokeh.plotting.figure`, because `multi_axis_time_series` returns a `bokeh.layouts.column(...)` of several figures under a `Div`, a `Column` instance, not a bare `figure`; `LayoutDOM` is the base class both share, so checking against it (rather than `figure` alone, which would silently fall through to the Plotly branch for every `multi_axis_time_series` output and call `.write_html()` on a Bokeh layout, an `AttributeError`) catches every Bokeh shape `visualisation.py` can now return:

```python
def _write_figure(self, fig, path: str, title: str) -> None:
    if isinstance(fig, LayoutDOM):
        bokeh_save(fig, filename=path, resources=INLINE, title=title)
    else:
        fig.write_html(path, include_plotlyjs=True)
```

called in place of the existing bare `fig.write_html(...)` line inside `run()`'s `for title, fig in figs:` loop. Every other line of `run()` is unchanged, `self.metrics_rows` and `figures/<tab>/*.html` are populated exactly as completely and deterministically as today, see §3's closing paragraph for why that matters.

### 4.3 `dashboard.py`, rewritten as a `panel serve` app

The `.save()`/`_inject_linked_zoom`/`_inject_lazy_tab_script`/`_lazy_placeholder`/`tabs/` splitting the previous turn added are all removed, none of it is needed once `pn.Tabs(dynamic=True)` can do the deferring itself against a live server.

```python
import argparse
import json
import os

import panel as pn

import metrics
from analysis import Analysis
from data_reader import DataReader

pn.extension("plotly", "tabulator")
# Not "bokeh": Panel loads Bokeh model support unconditionally, it isn't a
# named extension bundle the way "plotly"/"tabulator" are (those load their
# own JS); passing "bokeh" only logs "bokeh extension not recognized and
# will be skipped." (hit live, 2026-10-22), harmless on its own but removed
# since it asserted a capability that doesn't exist.


def _center(component):
    """Unchanged from the previous design, the pn.HSpacer() sandwich."""
    return pn.Row(pn.HSpacer(), component, pn.HSpacer(), sizing_mode="stretch_width")


def _tab_pane(fig):
    from bokeh.models import LayoutDOM
    return pn.pane.Bokeh(fig) if isinstance(fig, LayoutDOM) else pn.pane.Plotly(fig)


def _build_tab(figs: list[tuple[str, object]]) -> pn.Column:
    """The function pn.param.ParamFunction(..., lazy=True) wraps per tab,
    so it only runs, and only then attaches each figure's Bokeh model to
    the live document, the first time that tab is selected. Panel's own
    Tabs reference documents exactly this ParamFunction(fn, lazy=True)
    pattern as the way to get genuine on-demand tab content under
    dynamic=True; it is not pn.bind, which panel's own Tabs example
    explicitly does not use for this."""
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
    return ap.parse_args()


args = _parse_args()
with open(args.config) as fh:
    _config = json.load(fh)
_reader = DataReader(args.logdir)
_analysis = Analysis(_reader, _config, args.logdir)
_analysis.run()
_build(_analysis.tabs, _analysis.metrics_rows).servable(title=f"SLAM analysis, {args.logdir}")
```

This is module level code, not a `main()` behind `if __name__ == "__main__":`, because `panel serve`/`bokeh serve` execute the whole script once per new browser session rather than importing it and calling a named entry point, the standard Bokeh server app model. `--logdir`/`--config` still parse from `sys.argv`, `bokeh serve`'s (and `panel serve`'s, a superset of its CLI) own `--args` flag passes everything after it through verbatim, so `panel serve dashboard.py --args --logdir /path` reaches `argparse` exactly as before.

`Analysis(...).run()` still runs once, synchronously, before `_build` is ever called, so `metrics_rows` is complete and `figures/<tab>/*.html` is fully written before the page is handed to the browser, exactly the property §3 relies on. Only `_build_tab`'s own body, wrapped by `pn.param.ParamFunction(..., lazy=True)`, is deferred, and only the Panel-pane-wrapping/model-attachment step it performs, not the figure objects themselves, which already exist by the time it runs.

### 4.4 `djinn`/Docker

`/ws/scripts/start.sh:42` already runs every container, `analysis_container` included, with `--net=host`, confirmed directly from the script, so a port `panel serve` binds to inside the container is already reachable on the host at the same port with no `-p` mapping to add anywhere. The only change is `/ws/djinn`'s `slam-analysis` case, `iexec` already runs `docker exec -it`, so a long-running foreground server blocks the calling shell exactly the way the existing `basalt`/gdb `djinn start` entry already does, no new blocking/backgrounding concern.

```bash
elif [[ "$2" == "slam-analysis" ]]; then
    if [[ -z "$3" ]]; then
        echo "Usage: djinn start slam-analysis <absolute logdir path inside the container>"
    else
        echo "Starting SLAM analysis dashboard for $3 at http://localhost:5006"
        spawn_container_if_needed analysis_container "shandilya1998/nrt:analysis"
        iexec analysis_container "cd /ws/ros_ws/src/slam/ext/basalt && python3 -m panel serve scripts/analysis/dashboard.py --port 5006 --address 0.0.0.0 --allow-websocket-origin=\"*\" --admin --args --logdir \"$3\""
    fi
```

`--allow-websocket-origin="*"` is needed because Bokeh servers refuse WebSocket connections from an origin they were not told to expect by default, and the host's own address (`localhost:5006`, or whatever hostname a remote viewer uses) is not known in advance here the way it would be for a fixed deployment; this is the direct, Bokeh-documented reason this flag exists and is the standard way to run a Bokeh/Panel server behind an unknown or varying front end address. `--admin` (added 2026-10-22, during the same debugging pass as §4.6) exposes a live session/performance panel at `/admin`, memory, CPU and per-request latency, essentially free when not visited. No Dockerfile change, `panel` already pulls in `bokeh` as a hard dependency (`plans/analysis_module.md` §13), confirmed present at `/ws/docker/analysis/Dockerfile:132`'s `pip install ... panel`.

This changed workflow is recorded in `context/djinn-workflows.md` per `working-conventions.md`'s mandatory rule, call chain, invocation, side effects (a long-lived foreground server process bound to a port on the container's, and therefore the host's, network namespace) and a validation strategy, since tests cannot run inside this dev container and are authored from that catalogue.

### 4.5 Backward compatibility

`config.json`'s schema is unchanged, `Analysis._wh()` still splats `width`/`height` straight into whichever figure constructor is called, Bokeh's `figure(width=, height=)` accepts the same two keys Plotly's `update_layout` did. Every non-time-series plot function, and every call site that builds one, is untouched. `metrics.csv`, the `scalars` tab and `figures/<tab>/*.html` keep exactly their current completeness guarantee, see §3. The one deliberate, user-approved behaviour change is that no `dashboard.html` is written at all any more, `--logdir`'s own directory no longer gets a single static file that can be copied or emailed elsewhere and still opens on its own, viewing the dashboard now means `analysis_container` (or any machine with `panel`/`bokeh` installed) running `panel serve` for the length of the session, reached over a live connection instead.

### 4.7 `class Analysis.run()` timing, added during the same debugging pass

`Analysis.run()` is the one piece of work that cannot be made lazy, §3's own closing paragraph, so it is also the one piece of work whose wall time a reader actually waiting on the page needs visibility into, Bokeh/Tornado's own `--log-level debug` is mostly protocol and session handshake chatter, not application logic, and would not show where this time actually goes. `run()` now logs, at the default `INFO` level so no `--log-level` override is needed, one line per `_analyse_<tab>` call with its own wall time and figure count, one line for the `figures/<tab>/*.html` write pass, and one line for the whole call; `logging.getLogger(__name__)` propagates to the root handler Bokeh's own server already installs (confirmed live, its own `Starting Bokeh server…` lines use the same root handler), so no additional `logging.basicConfig()` call is needed or added. This is the first thing to read whenever a load feels slow, before reaching for `--log-level debug`.

## 5. Validation strategy

| Check | How |
|---|---|
| `link_zoom` no longer appears anywhere in `analysis.py`/`visualisation.py` | `grep -rn link_zoom scripts/analysis/`, zero matches |
| Every former `link_zoom=True` site now carries the shared range | `grep -c "x_range=self.shared_t_range" analysis.py` equals 41, the count established in §4.2 before the edit |
| The 5 iteration-index plots stay unlinked | Zoom one of `cost_curve`/`damping_trace`/`cost_split`/`mapper_cost_curve`/`mapper_damping_accept_reject` and confirm no other figure's range moves |
| Cross-tab linking actually works | Open two different tabs (forcing both lazy `ParamFunction`s to fire), zoom a `t_ns` plot in one, confirm a `t_ns` plot in the other tab, once selected, shows the same window; this needs the live server and cannot be checked from a saved file |
| Tabs are genuinely lazy | Open the dashboard, confirm via the browser's network/performance panel that a tab's figures are not constructed until that tab is first clicked |
| `metrics.csv`/`figures/<tab>/*.html` unaffected | Diff their contents for one log directory against a pre-migration run of the same directory |
| `figures/<tab>/*.html` still opens standalone for a Bokeh-backed plot | Open a saved `figures/vio_optimisation/state_trace_v_norm.html` (or equivalent) with no server running |
| `djinn start slam-analysis` still reaches the dashboard | Run it against a real `/ws/logs/slam/<run>` directory from the host, confirm `http://localhost:5006` loads given `--net=host` |

None of this can be executed from the dev container `panel`/`bokeh` run in, confirmed earlier in this session (`python3 -c "import panel"` fails here), it has to be run inside `analysis_container` once implemented.

## 6. Deliberately not done

The 10 non-time-series plot functions are not migrated to Bokeh, cross-figure linking was never a requirement on them (`plans/analysis_module.md` §15), and rewriting them would be pure risk with no behavioural gain. The `postMessage`/iframe hub from §2 is not built, Bokeh's native shared `DataRange1d` supersedes it entirely once a live server is accepted. Per-session result caching across multiple simultaneous browser connections is not implemented, `panel serve`'s default per-session script execution means every new browser tab re-runs `Analysis.run()` from scratch; acceptable for a single engineer's own debugging session, the same work `Analysis.run()` already did once per `djinn start` invocation under the static design, merely relocated to page-open time rather than avoided or duplicated. A custom `HoverTool` matching Plotly's implicit hover-on-marker behaviour is not added to the new Bokeh figures, not something either design asked for, Bokeh's default toolbar (pan/wheel zoom/box zoom/reset/save) is kept instead; worth adding later if the lack of hover values is actually felt during debugging.
