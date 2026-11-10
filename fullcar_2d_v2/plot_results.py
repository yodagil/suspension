#!/usr/bin/env python3
"""Plot V2 Full-car CDC maps in their computed absolute (vc, r) plane.

This is visualization only: it never moves samples, constructs a convex hull,
fills missing results, or edits simulation MAT files.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import itertools
import json
import os
from pathlib import Path
import tempfile
import textwrap
import time

os.environ.setdefault('MPLCONFIGDIR', str(Path(tempfile.gettempdir()) / 'fullcar_v2_matplotlib_cache'))
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap, BoundaryNorm
from matplotlib.lines import Line2D
from matplotlib.patches import Patch
import numpy as np
from scipy.io import loadmat


COLORS = ['#ffffff', '#d8d8d8', '#68b879']
CMAP = ListedColormap(COLORS)
NORM = BoundaryNorm(np.arange(-.5, 3, 1), CMAP.N)
MAT_VARIABLE = 'STABILITY2D'


def sha(path: Path) -> str:
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def json_clean(value):
    if isinstance(value, dict):
        return {str(k): json_clean(v) for k, v in value.items()}
    if isinstance(value, (list, tuple, np.ndarray)):
        return [json_clean(v) for v in value]
    if isinstance(value, (np.integer,)):
        return int(value)
    if isinstance(value, (np.floating, float)):
        return float(value) if np.isfinite(value) else None
    if isinstance(value, np.bool_):
        return bool(value)
    return value


def atomic_json(path: Path, value):
    temp = path.with_name(path.name + '.tmp')
    temp.write_text(json.dumps(json_clean(value), indent=2, allow_nan=False) + '\n')
    temp.replace(path)


def as_cases(value):
    if isinstance(value, dict):
        return [value]
    if isinstance(value, np.ndarray):
        return list(value.reshape(-1))
    return list(value)


def scalar(value) -> float:
    return float(np.asarray(value).reshape(-1)[0])


def mask(value, nv, nr, nh):
    a = np.asarray(value, dtype=bool)
    if a.shape == (nv, nr) and nh == 1:
        return a[:, :, None]
    if a.shape != (nv, nr, nh):
        raise ValueError(f'Unexpected mask dimensions {a.shape}; expected {(nv, nr, nh)}')
    return a


def read_result(path: Path):
    data = loadmat(path, simplify_cells=True)
    if MAT_VARIABLE not in data:
        raise ValueError(f'{path}: missing {MAT_VARIABLE}; this plotter accepts 2-D results only')
    s = data[MAT_VARIABLE]
    meta, cfg = s['meta'], s['cfg']
    horizons = np.atleast_1d(meta['horizons_s']).astype(float)
    if str(meta.get('version', '')) != 'cdc-fullcar-2d-v2':
        raise ValueError('This plotter accepts V2 Full-car results only')
    slice_mode = str(cfg['slice_mode'])
    if not bool(meta['roll_dynamics_retained']) or bool(meta['is_projection_of_3d']):
        raise ValueError('Expected a V2 Full-car initial-state section, not a projected region')
    result = dict(path=path, S=s, meta=meta, cfg=cfg, horizons=horizons,
                  speed=scalar(meta['speed_kmh']), mu=scalar(meta['mu_road']),
                  steer=scalar(cfg['steer_deg']), slice_mode=slice_mode,
                  initialization=str(cfg['initialization']), cases=[])
    for c in as_cases(s['CASE']):
        vc = np.atleast_1d(c['grid']['vc']).astype(float)
        r = np.atleast_1d(c['grid']['r']).astype(float)
        if vc.ndim != 1 or r.ndim != 1 or len(vc) < 2 or len(r) < 2:
            raise ValueError('Expected two nontrivial increasing grid axes')
        if not (np.diff(vc) > 0).all() or not (np.diff(r) > 0).all():
            raise ValueError('Grid axes must increase')
        nv, nr, nh = len(vc), len(r), len(horizons)
        masks = {name: mask(c['masks'][name], nv, nr, nh)
                 for name in ['safe', 'yaw_policy', 'guarded_safe', 'recoverable', 'guarded_recoverable', 'safe_unresolved']}
        selected = masks['recoverable'].copy()
        if bool(cfg['use_yaw_policy_for_region']):
            selected &= masks['yaw_policy']
        if not np.array_equal(np.asarray(c['stable_mask'], dtype=bool), selected[:, :, -1]):
            raise ValueError('Saved selected mask disagrees with its declared selection policy')
        if not np.array_equal(masks['guarded_safe'], masks['safe'] & masks['yaw_policy']):
            raise ValueError('guarded_safe mask is inconsistent')
        if not np.array_equal(masks['guarded_recoverable'], masks['recoverable'] & masks['yaw_policy']):
            raise ValueError('guarded_recoverable mask is inconsistent')
        if not np.array_equal(masks['safe_unresolved'], masks['safe'] & ~masks['recoverable']):
            raise ValueError('safe_unresolved mask is inconsistent')
        if np.any(masks['recoverable'] & ~masks['safe']):
            raise ValueError('A recovered sample must also satisfy the physical/domain constraints')
        result['cases'].append(dict(raw=c, vc=vc, r=r, masks=masks, selected=selected,
                                    wear=np.asarray(c['wear']).reshape(-1).astype(int),
                                    initial_roll=scalar(c['fixed_initial_roll_rad'])))
    return result


def contour(ax, vc, r, values, *, color, linestyle='-', linewidth=1.2):
    values = np.asarray(values, dtype=bool)
    # There is no boundary inside the sampled domain for an empty/full mask.
    # Never pad the exterior to manufacture a closed contour.
    if values.any() and (~values).any():
        ax.contour(vc, r, values.T.astype(float), levels=[.5], colors=[color],
                   linestyles=[linestyle], linewidths=[linewidth])


def trim_xy(case):
    t = case['raw']['trim']
    x = np.asarray(t['x'], dtype=float).reshape(-1)
    if bool(t['root_found']) and len(x) == 32 and np.isfinite(x[:2]).all():
        return x[:2], bool(t['usable'])
    return None, False


def view_limits(cases, which='selected'):
    vc, r = cases[0]['vc'], cases[0]['r']
    points = []
    for c in cases:
        a = c['selected'][:, :, -1] if which == 'selected' else c['masks'][which][:, :, -1]
        ii, jj = np.nonzero(a)
        if len(ii):
            points.extend(np.column_stack([c['vc'][ii], c['r'][jj]]))
        xy, _ = trim_xy(c)
        if xy is not None and vc[0] <= xy[0] <= vc[-1] and r[0] <= xy[1] <= r[-1]:
            points.append(xy)
    if not points:
        return (vc[0], vc[-1]), (r[0], r[-1])
    a = np.asarray(points)
    low, high = np.min(a, axis=0), np.max(a, axis=0)
    center = (low + high) / 2
    half = np.maximum((high - low) * .58, [1., .06])
    return ((max(vc[0], center[0]-half[0]), min(vc[-1], center[0]+half[0])),
            (max(r[0], center[1]-half[1]), min(r[-1], center[1]+half[1])))


def finish_axes(ax, vc, r, xlim=None, ylim=None):
    ax.set_xlim(xlim or (vc[0], vc[-1]))
    ax.set_ylim(ylim or (r[0], r[-1]))
    ax.set_xlabel(r'$v_{y,0}$ [m/s]')
    ax.set_ylabel(r'$r_0$ [rad/s]')
    ax.grid(alpha=.2, linewidth=.5)
    ax.tick_params(labelsize=8)


def save_figure(fig, path):
    temp = path.with_name(path.name + '.tmp')
    fig.savefig(temp, format='png', dpi=145, facecolor='white')
    plt.close(fig)
    temp.replace(path)


def plot_condition(result, output):
    cases, horizons = result['cases'], result['horizons']
    nw = len(cases)
    fig, axes = plt.subplots(2, nw, figsize=(max(10, 4.1*nw), 8.6), squeeze=False)
    zoom_x, zoom_y = view_limits(cases)
    summaries = []
    for iw, c in enumerate(cases):
        safe, recovered, selected = c['masks']['safe'][:, :, -1], c['masks']['recoverable'][:, :, -1], c['selected'][:, :, -1]
        categories = np.zeros(safe.shape, dtype=np.uint8)
        categories[safe] = 1
        categories[recovered] = 2
        touch = bool(np.asarray(c['raw']['summary']['touches_grid']).any())
        area = scalar(c['raw']['region']['sampled_area'])
        xy, usable = trim_xy(c)
        t = c['raw']['trim']
        wear = ' '.join(str(x) for x in c['wear'])
        first_count = int(c['selected'][:, :, 0].sum())
        selected_count = int(selected.sum())
        for row in range(2):
            ax = axes[row, iw]
            ax.pcolormesh(c['vc'], c['r'], categories.T, cmap=CMAP, norm=NORM, shading='nearest', rasterized=True)
            if len(horizons) > 1:
                contour(ax, c['vc'], c['r'], c['selected'][:, :, 0], color='#c77c22', linestyle='--', linewidth=1.1)
            contour(ax, c['vc'], c['r'], selected, color='#135426', linewidth=1.15)
            contour(ax, c['vc'], c['r'], c['masks']['guarded_recoverable'][:, :, -1],
                    color='#773e96', linestyle=':', linewidth=1.2)
            if xy is not None:
                ax.plot(xy[0], xy[1], 'x', color='black' if usable else '#a32222', ms=6, mew=1.3)
            finish_axes(ax, c['vc'], c['r'], zoom_x if row else None, zoom_y if row else None)
            if row == 0:
                ax.set_title(f'Wear {iw+1}: [{wear}]\nselected {horizons[0]:g}s/{horizons[-1]:g}s: {first_count}/{selected_count}\nphi(0)={c["initial_roll"]:+.5g} rad', fontsize=10)
            else:
                ax.set_title(f'Common absolute-coordinate zoom; area={area:.5g}\ntrim root={int(bool(t["root_found"]))}, usable={int(bool(t["usable"]))}' + ('; grid-truncated' if touch else ''), fontsize=9,
                             color='#a32222' if touch else '#222222')
        summaries.append(dict(wear_index=iw+1, wear=c['wear'], initial_roll_rad=c['initial_roll'],
                              guarded_recoverable_count=int(c['masks']['guarded_recoverable'][:, :, -1].sum()),
                              safe_count=int(safe.sum()),
                              recoverable_count=int(recovered.sum()), selected_count=selected_count,
                              selected_count_first_horizon=first_count, sampled_cell_area=area,
                              selected_touches_grid=touch, trim_root=bool(t['root_found']), trim_usable=bool(t['usable'])))
    smoke_note = '\nSHORT-HORIZON SCHEMA SMOKE ONLY; not a production stability result' if horizons[-1] < 5 else ''
    policy_label = 'recoverable + absolute yaw policy' if bool(result['cfg']['use_yaw_policy_for_region']) else 'recoverable; absolute yaw policy separate'
    fig.suptitle(f'CDC Full-car V2 initial-state section | {result["speed"]:g} km/h | mu={result["mu"]:g} | front steer={result["steer"]:+g} deg\n'
                 f'slice={result["slice_mode"]}; full roll dynamics retained; selected={policy_label}' + smoke_note, fontsize=13, y=.995)
    handles = [Patch(facecolor=COLORS[0], edgecolor='#aaaaaa', label='Constraint/domain violation'),
               Patch(facecolor=COLORS[1], label='Safe; recovery unresolved'),
               Patch(facecolor=COLORS[2], label='Recovered (physical path constraints passed)'),
               Line2D([], [], color='#c77c22', linestyle='--', label=f'Selected boundary at {horizons[0]:g}s'),
               Line2D([], [], color='#135426', label=f'Selected boundary at {horizons[-1]:g}s'),
               Line2D([], [], color='#773e96', linestyle=':', label=f'Recovered + absolute yaw policy at {horizons[-1]:g}s'),
               Line2D([], [], color='black', marker='x', linestyle='None', label='Computed trim; red if unusable')]
    fig.legend(handles=handles, loc='lower center', ncol=2 if nw < 3 else 4, frameon=False, fontsize=8.5, bbox_to_anchor=(.5, .04))
    caption = 'Colors classify grid samples; contour interpolation is visualization only. Cell area uses four passed vertices and tested centers. No shift or convex hull is applied.'
    fig.text(.5, .005, textwrap.fill(caption, 110 if nw < 3 else 220), ha='center', va='bottom', fontsize=7.5)
    fig.tight_layout(rect=[0, .205 if nw < 3 else .14, 1, .90 if smoke_note else .925])
    path = output / (result['path'].stem + '.png')
    save_figure(fig, path)
    return dict(source_mat=result['path'].name, figure=path.name,
                speed_kmh=result['speed'], mu_road=result['mu'], steer_deg=result['steer'],
                slice_mode=result['slice_mode'], initialization=result['initialization'],
                selected_uses_yaw_policy=bool(result['cfg']['use_yaw_policy_for_region']), cases=summaries)


def group_key(result):
    return result['speed'], result['mu'], result['initialization'], result['slice_mode']


def plot_overview(results, output, kind='selected'):
    results = sorted(results, key=lambda r: r['steer'])
    first = results[0]
    nw = len(first['cases'])
    if any(len(r['cases']) != nw for r in results):
        raise ValueError('Overview members have different tire-wear catalogs')
    for result in results:
        if not np.array_equal(result['horizons'], first['horizons']):
            raise ValueError('Overview members must have identical horizon judgments')
        if bool(result['cfg']['use_yaw_policy_for_region']) != bool(first['cfg']['use_yaw_policy_for_region']):
            raise ValueError('Overview members must use the same representative-region policy')
        for a, b in zip(result['cases'], first['cases']):
            if not (np.array_equal(a['vc'], b['vc']) and np.array_equal(a['r'], b['r']) and np.array_equal(a['wear'], b['wear'])):
                raise ValueError('Overview members must have the same absolute grid and wear catalog')
    all_cases = list(itertools.chain.from_iterable(r['cases'] for r in results))
    zx, zy = view_limits(all_cases, kind)
    fig, axes = plt.subplots(2, nw, figsize=(max(10, 4.1*nw), 8.1), squeeze=False)
    colors = plt.get_cmap('turbo')(np.linspace(.05, .9, len(results)))
    for iw in range(nw):
        for row in range(2):
            ax = axes[row, iw]
            empty = []
            for s, color in zip(results, colors):
                c = s['cases'][iw]
                values = c['selected'][:, :, -1] if kind == 'selected' else c['masks'][kind][:, :, -1]
                contour(ax, c['vc'], c['r'], values, color=color, linewidth=1.2)
                if not values.any():
                    empty.append(f'{s["steer"]:g}')
                xy, usable = trim_xy(c)
                if xy is not None:
                    ax.plot(xy[0], xy[1], 'o' if usable else 'x', color=color, markersize=3, linewidth=.8)
            c = first['cases'][iw]
            finish_axes(ax, c['vc'], c['r'], zx if row else None, zy if row else None)
            wear = ' '.join(str(x) for x in c['wear'])
            ax.set_title(f'Wear {iw+1}: [{wear}]' + ('\nCommon absolute-coordinate zoom' if row else ''), fontsize=10)
            if empty:
                ax.text(.015, .025, 'Empty at deg: ' + ', '.join(empty), transform=ax.transAxes, fontsize=7, va='bottom')
    final_horizon = first['horizons'][-1]
    selected_label = 'Recovered + absolute yaw policy' if bool(first['cfg']['use_yaw_policy_for_region']) else 'Recovered; absolute yaw policy separate'
    label = {'selected': selected_label, 'recoverable': 'Recovered without yaw policy',
             'guarded_recoverable': 'Recovered + absolute yaw policy',
             'safe': 'Constraint-accepted (recovery not required)', 'guarded_safe': 'Constraint-accepted + yaw policy'}[kind]
    fig.suptitle(f'{label} | Full-car V2 | {first["speed"]:g} km/h | mu={first["mu"]:g} | t={final_horizon:g}s\n'
                 f'slice={first["slice_mode"]}; absolute coordinates; phi(0) follows each case slice', fontsize=13, y=.995)
    handles = [Line2D([], [], color=c, label=f'delta={s["steer"]:g} deg') for s, c in zip(results, colors)]
    fig.legend(handles=handles, loc='lower center', ncol=min(len(handles), 7), frameon=False, bbox_to_anchor=(.5, .025))
    caption = 'Contours are computed sampled-mask interpolations. No exterior padding, forced inclusion, artificial translation or convex hull is used.'
    fig.text(.5, .005, textwrap.fill(caption, 110 if nw < 3 else 220), ha='center', va='bottom', fontsize=7.5)
    fig.tight_layout(rect=[0, .09, 1, .925])
    mu_tag = f'{first["mu"]:g}'.replace('.', 'p')
    path = output / f'overview_{kind}_{first["speed"]:g}kph_mu{mu_tag}_{first["initialization"]}_{first["slice_mode"]}.png'
    save_figure(fig, path)
    return path.name


def find_inputs(path):
    if path.is_file():
        return [path]
    return sorted(p for p in path.glob('fullcar2d_v2_*.mat') if 'checkpoint' not in p.name)


def write_csv(path, records):
    fields = ['source_mat', 'speed_kmh', 'mu_road', 'steer_deg', 'wear_index', 'safe_count',
              'recoverable_count', 'guarded_recoverable_count', 'selected_count', 'initial_roll_rad', 'sampled_cell_area', 'selected_touches_grid',
              'trim_root', 'trim_usable']
    temp = path.with_name(path.name + '.tmp')
    with temp.open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for record in records:
            for case in record['cases']:
                row = {key: record.get(key, case.get(key, '')) for key in fields}
                writer.writerow(row)
    temp.replace(path)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', type=Path, required=True, help='A STABILITY2D MAT or final-results directory')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--watch', action='store_true')
    parser.add_argument('--interval', type=float, default=15)
    parser.add_argument('--overview', action='store_true')
    parser.add_argument('--overview-kind', choices=['selected', 'recoverable', 'guarded_recoverable', 'safe', 'guarded_safe'], default='selected')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    state_path = args.output / 'plot_index.json'
    source_hash = sha(Path(__file__))
    old = json.loads(state_path.read_text()) if state_path.exists() else {}
    cached = old.get('records', {}) if old.get('plotter_sha256') == source_hash else {}
    result_cache = {}
    group_cache = {}
    while True:
        files = find_inputs(args.input)
        for path in files:
            key = path.name
            info = [path.stat().st_size, path.stat().st_mtime_ns]
            previous = cached.get(key, {})
            if previous.get('source_stat') == info and (args.output / previous.get('figure', '__missing__')).exists():
                continue
            result = read_result(path)
            record = plot_condition(result, args.output)
            record.update(source_stat=info, source_sha256=sha(path))
            cached[key] = record
            result_cache[key] = result
            print(f'PLOTTED {len(cached)} condition files: {key}', flush=True)
        overview_files = []
        if args.overview:
            groups = {}
            for path in files:
                s = result_cache.get(path.name)
                if s is None:
                    s = read_result(path)
                    result_cache[path.name] = s
                groups.setdefault(group_key(s), []).append(s)
            for key, group in groups.items():
                # The full study contains seven steering angles; incomplete
                # groups stay unplotted in watch mode instead of implying completion.
                if len(group) < 7 and args.watch:
                    continue
                if len(group) < 2:
                    continue
                fingerprint = tuple((x['path'].name, cached[x['path'].name]['source_sha256']) for x in sorted(group, key=lambda x: x['steer']))
                if group_cache.get(key) != fingerprint:
                    name = plot_overview(group, args.output, args.overview_kind)
                    group_cache[key] = fingerprint
                else:
                    mu_tag = f'{key[1]:g}'.replace('.', 'p')
                    name = f'overview_{args.overview_kind}_{key[0]:g}kph_mu{mu_tag}_{key[2]}_{key[3]}.png'
                overview_files.append(name)
        records = [cached[path.name] for path in files if path.name in cached]
        atomic_json(state_path, dict(plotter_sha256=source_hash, completed_condition_figures=len(records),
                                    records={r['source_mat']: r for r in records}, overview_figures=overview_files,
                                    visualization_only=True, coordinates='absolute vc0/r0; V2 slice mode preserves per-case trim roll; no shifts'))
        write_csv(args.output / 'plot_summary.csv', records)
        if not args.watch:
            if not files:
                raise SystemExit('No completed fullcar2d_v2_*.mat results found')
            break
        progress_path = args.input / 'progress.json' if args.input.is_dir() else args.input.parent / 'progress.json'
        if progress_path.exists():
            progress = json.loads(progress_path.read_text())
            phase = progress.get('phase', '')
            if phase in {'failed', 'cancelled', 'packaging_failed'}:
                raise SystemExit(f'Simulation phase={phase}; completed figures have been preserved')
            expected = int(progress.get('total_scenarios', 0))
            done = int(progress.get('completed_scenarios', 0))
            if phase in {'completed', 'mat_completed', 'simulations_completed'} and expected > 0 and done == expected and len(records) >= expected and (not args.overview or len(overview_files) >= (30 if expected == 210 else 0)):
                print(f'ALL {len(records)} CONDITION FIGURES COMPLETE', flush=True)
                break
        time.sleep(max(1., args.interval))


if __name__ == '__main__':
    main()
