#!/usr/bin/env python3
"""Package measured full-car steady states into an explicitly indexed MATLAB LUT.

No simulation or interpolation is performed here.  Every completed row must
come from the companion native engine, with the exact input catalog below.
"""
from __future__ import annotations

import argparse
from collections import Counter
from datetime import datetime, timezone
import hashlib
import itertools
import json
from pathlib import Path
import subprocess
import sys

import numpy as np
from scipy.io import loadmat, savemat

ROOT = Path(__file__).resolve().parents[1]
SPEEDS = np.array([50., 60., 70., 80., 90., 100.])
MUS = np.array([.9, .8, .7, .6, .5])  # Engine row-id order; published LUT reverses this axis.
STEERS = np.array([0., .5, 1., 1.5, 2., 2.5, 3.])
WEARS = np.array([[0, 0, 0, 0], [25, 25, 0, 0], [50, 50, 25, 25],
                  [0, 0, 25, 25], [25, 25, 50, 50]], dtype=np.uint8)
MODES = [''.join(x) for x in itertools.product('nsh', repeat=4)]
SPRINGS_FRONT = {'n': 30790., 's': 22161., 'h': 36086.}
SPRINGS_REAR = {'n': 30790., 's': 23632., 'h': 35007.}
SPRINGS = np.array([[SPRINGS_FRONT[x[0]], SPRINGS_FRONT[x[1]],
                     SPRINGS_REAR[x[2]], SPRINGS_REAR[x[3]]] for x in MODES])
SHAPE = (6, 5, 7, 5, 81)
TOTAL = int(np.prod(SHAPE))
AXES = ['speed_kmh', 'mu_road', 'front_wheel_steer_deg', 'wear_case', 'suspension_mode']
STATUS = {'not_simulated': 0, 'converged': 1,
          'not_converged_time_limit': 2, 'numerical_failure': 3, 'hard_guard': 4}
STATE_NAMES = ['vc', 'r', 'z_body', 'phi', 'theta',
               'zu_FL', 'zu_FR', 'zu_RL', 'zu_RR',
               'z_body_dot', 'phi_dot', 'theta_dot',
               'zu_dot_FL', 'zu_dot_FR', 'zu_dot_RL', 'zu_dot_RR',
               'kappa_FL', 'kappa_FR', 'kappa_RL', 'kappa_RR',
               'alpha_FL', 'alpha_FR', 'alpha_RL', 'alpha_RR',
               'omega_FL', 'omega_FR', 'omega_RL', 'omega_RR',
               'ay_load', 'heading', 'world_X', 'world_Y']
STATE_UNITS = ['m/s', 'rad/s', 'm', 'rad', 'rad', 'm', 'm', 'm', 'm',
               'm/s', 'rad/s', 'rad/s', 'm/s', 'm/s', 'm/s', 'm/s',
               '1', '1', '1', '1', 'rad', 'rad', 'rad', 'rad',
               'rad/s', 'rad/s', 'rad/s', 'rad/s', 'm/s^2', 'rad', 'm', 'm']


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def get(row, *names, default=None, required=True):
    found = [name for name in names if name in row]
    if not found:
        if required:
            raise ValueError(f'Row {row.get("row_id", row.get("rowid", "?"))}: missing {names[0]}')
        return default
    return row[found[0]]


def vector(value, size, name, *, allow_nonfinite=False):
    a = np.asarray([np.nan if x is None else x for x in value], dtype=np.float64)
    if a.shape != (size,) or (not allow_nonfinite and not np.isfinite(a).all()):
        raise ValueError(f'{name}: expected {size} finite numeric values')
    return a


def finite_scalar(value, name, *, allow_nan=False):
    x = np.nan if value is None else float(value)
    if not np.isfinite(x) and not (allow_nan and np.isnan(x)):
        raise ValueError(f'{name}: expected a finite value')
    return x


def condition(rowid):
    i, j, k, w, mode = np.unravel_index(rowid, SHAPE, order='C')
    return i, j, k, w, mode


def checked_reason(value, name):
    result = int(value)
    if result != value or not 0 <= result <= 31:
        raise ValueError(f'{name}: unknown reason bits')
    return result


def read_rows(paths):
    rows = {}
    for path in paths:
        with Path(path).open() as stream:
            for line_number, line in enumerate(stream, 1):
                if not line.strip():
                    continue
                row = json.loads(line)
                raw_rowid = get(row, 'row_id', 'rowid')
                rowid = int(raw_rowid)
                if rowid != raw_rowid:
                    raise ValueError(f'{path}:{line_number}: row id must be an integer')
                if not 0 <= rowid < TOTAL:
                    raise ValueError(f'{path}:{line_number}: row id outside 0..{TOTAL-1}')
                if rowid in rows:
                    raise ValueError(f'{path}:{line_number}: duplicate row {rowid}')
                rows[rowid] = row
    return rows


def make_provenance(metadata):
    frozen = ROOT.parent / '.matlab-env/reference09_full210_code'
    manifest = frozen / 'source_fingerprint.json'
    if manifest.exists():
        fingerprints = json.loads(manifest.read_text())
        if not all(sha(frozen / name) == value for name, value in fingerprints.items()):
            raise ValueError('Original frozen model fingerprint mismatch')
        verification_scope = 'All seven original private frozen source files verified'
    else:
        manifest = ROOT / 'steady_state_lookup/model/frozen_source_fingerprint.json'
        if not manifest.exists():
            raise ValueError('Missing original source fingerprint manifest')
        fingerprints = json.loads(manifest.read_text())
        if sha(ROOT / 'steady_state_lookup/model/native_accel.cpp') != fingerprints['native_accel.cpp']:
            raise ValueError('Shipped native model does not match the original source fingerprint')
        verification_scope = 'Shipped native model verified; original seven-file historical fingerprints retained'
    try:
        commit = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True,
                                         stderr=subprocess.DEVNULL).strip()
    except subprocess.CalledProcessError:
        commit = 'not_available_in_export'
    sources = []
    for rel in ['steady_state_lookup/native_steady_lookup.cpp',
                'steady_state_lookup/model/native_accel.cpp',
                'steady_state_lookup/build_lookup.py']:
        path = ROOT / rel
        if path.exists():
            sources.append((rel, sha(path)))
    for key, rel in [('adapter_sha256', 'steady_state_lookup/native_steady_lookup.cpp'),
                     ('frozen_model_sha256', 'steady_state_lookup/model/native_accel.cpp')]:
        if key in metadata and sha(ROOT / rel) != metadata[key]:
            raise ValueError(f'Run fingerprint does not match {rel}')
    if 'Ts_s' in metadata and metadata['Ts_s'] != .01:
        raise ValueError('Run metadata sampling time differs from .01 s')
    if 'RK4_substeps' in metadata and metadata['RK4_substeps'] != 20:
        raise ValueError('Run metadata RK4 substeps differ from 20')
    return {
        'original_sweep_run_version': 'mu-sweep-v3-reference09-matlabgrid',
        'original_sweep_source_files': np.array(list(fingerprints), dtype=object),
        'original_sweep_sha256': np.array(list(fingerprints.values()), dtype=object),
        'source_verification_scope': verification_scope,
        'lookup_source_files': np.array([x[0] for x in sources], dtype=object),
        'lookup_source_sha256': np.array([x[1] for x in sources], dtype=object),
        'source_commit_before_delivery': commit,
        'build_flags': '-O3 -std=c++17 -fopenmp; no fast-math',
        'initial_condition_vc_r_phi': np.zeros(3),
        'initialization': 'Original direct-start: relaxed alpha=-wheel_steer; omega=max(ux*cos(wheel_steer),0)/Rw; remaining states zero',
        'longitudinal_speed': 'Held at speed_kmh/3.6 throughout each trajectory',
        'road_friction_scaling': 'Original MF longitudinal/lateral force scaling mu/0.9',
        'Ts_s': .01, 'RK4_substeps': 20,
        'convergence_state_indices_matlab': np.arange(1, 30, dtype=np.uint8),
        'excluded_pose_state_indices_matlab': np.array([30, 31, 32], dtype=np.uint8),
        'convergence_scope': 'One zero-initial-body-state trajectory per operating condition; does not prove unique/global/asymptotic attraction',
        'convergence_basis': 'Numerical tail-window state peak-to-peak and maximum absolute raw RHS; thresholds are numerical choices, not paper criteria',
        'run_metadata_json': json.dumps(metadata, sort_keys=True, ensure_ascii=False),
    }


def build(args):
    rows = read_rows(args.input_jsonl)
    if len(rows) != TOTAL and not args.allow_partial:
        raise ValueError(f'Only {len(rows)}/{TOTAL} completed rows; use --allow-partial only for an explicitly incomplete checkpoint')
    if not rows:
        raise ValueError('No simulation rows supplied')
    nan = lambda: np.full(TOTAL, np.nan)
    outputs = {name: nan() for name in ['vc_ss', 'r_ss', 'phi_ss']}
    flags = {name: np.zeros(TOTAL, dtype=np.uint8) for name in
             ['computed', 'converged', 'tail_constraint_safe', 'all_observed_samples_constraint_safe',
              'usable', 'first5s_complete', 'reason_total', 'reason_tail', 'reason_first5s', 'status']}
    metrics = {name: nan() for name in ['end_time_s', 'max_scaled_ptp', 'max_scaled_rhs', 'max_scaled_mean_drift',
                                        'clipping_total', 'clipping_tail', 'consecutive_good_windows']}
    flags['window_complete'] = np.zeros(TOTAL, dtype=np.uint8)
    final_state = np.full((TOTAL, 32), np.nan)
    tol_state = None
    tol_rhs = None
    windows = set()
    max_times = set()
    for rowid, row in rows.items():
        i, j, k, w, mode = condition(rowid)
        for field, expected in [('speed_kmh', SPEEDS[i]), ('mu_road', MUS[j]), ('steer_deg', STEERS[k])]:
            actual = finite_scalar(get(row, field, 'front_steer_deg' if field == 'steer_deg' else field), field)
            if abs(actual - expected) > 1e-10:
                raise ValueError(f'Row {rowid}: {field} disagrees with row id')
        if get(row, 'mode', 'mode_name') != MODES[mode]:
            raise ValueError(f'Row {rowid}: suspension mode order mismatch')
        if int(get(row, 'wear_id')) != w + 1 or int(get(row, 'mode_id')) != mode + 1:
            raise ValueError(f'Row {rowid}: one-based wear/mode index mismatch')
        wear = vector(get(row, 'wear'), 4, 'wear')
        if not np.array_equal(wear, WEARS[w]):
            raise ValueError(f'Row {rowid}: tire wear mismatch')
        spring = vector(get(row, 'k_spring_N_m'), 4, 'k_spring_N_m')
        current = vector(get(row, 'I_mA', 'cdc_current_mA'), 4, 'I_mA')
        if not np.array_equal(spring, SPRINGS[mode]) or not np.array_equal(current, np.full(4, 950.)):
            raise ValueError(f'Row {rowid}: spring stiffness/CDC input mismatch')
        raw_status = get(row, 'status')
        status = STATUS[raw_status] if isinstance(raw_status, str) else int(raw_status)
        if status not in [1, 2, 3, 4]:
            raise ValueError(f'Row {rowid}: invalid completed status {raw_status!r}')
        converged = bool(get(row, 'converged'))
        if converged != (status == STATUS['converged']):
            raise ValueError(f'Row {rowid}: convergence/status disagreement')
        x = vector(get(row, 'state_final32', 'state_final', 'final_state'), 32, 'final state', allow_nonfinite=True)
        mean = vector(get(row, 'state_mean29', 'state_mean', 'tail_mean'), 29, 'tail mean', allow_nonfinite=not converged)
        ptp = vector(get(row, 'p2p29', 'state_ptp29', 'tail_ptp'), 29, 'tail peak-to-peak', allow_nonfinite=not converged)
        rhs = vector(get(row, 'rhs_max_abs29', 'tail_rhs_max_abs'), 29, 'tail raw RHS maximum', allow_nonfinite=not converged)
        state_tol = vector(get(row, 'state_tolerance29', 'tolerances29', 'state_tolerances29', 'ptp_tolerance'), 29, 'state tolerance')
        rhs_tol = vector(get(row, 'rhs_tolerance29', 'rhs_tolerances29', 'rhs_tolerance'), 29, 'RHS tolerance')
        if np.any(state_tol <= 0) or np.any(rhs_tol <= 0):
            raise ValueError('Convergence tolerances must be positive')
        if tol_state is None:
            tol_state, tol_rhs = state_tol, rhs_tol
        elif not np.array_equal(tol_state, state_tol) or not np.array_equal(tol_rhs, rhs_tol):
            raise ValueError('Mixed convergence tolerances are not allowed in one lookup')
        if np.any(ptp[np.isfinite(ptp)] < 0) or np.any(rhs[np.isfinite(rhs)] < 0):
            raise ValueError(f'Row {rowid}: negative convergence metric')
        scaled_ptp = float(np.max(ptp / state_tol)) if np.isfinite(ptp).all() else np.nan
        scaled_rhs = float(np.max(rhs / rhs_tol)) if np.isfinite(rhs).all() else np.nan
        drift_vector = vector(get(row, 'adjacent_mean_difference29'), 29, 'adjacent-window mean difference', allow_nonfinite=not converged)
        scaled_drift = float(np.max(drift_vector / state_tol)) if np.isfinite(drift_vector).all() else np.nan
        window_complete = bool(get(row, 'window_complete'))
        good_windows = int(get(row, 'consecutive_good_windows'))
        clipping_total = int(get(row, 'clipping_total'))
        clipping_tail = int(get(row, 'clipping_lastwindow'))
        if min(good_windows, clipping_total, clipping_tail) < 0:
            raise ValueError(f'Row {rowid}: negative diagnostic count')
        if converged and (not np.isfinite(x).all() or not window_complete or
                          good_windows < 2 or clipping_tail != 0 or
                          not np.isfinite(scaled_ptp + scaled_rhs + scaled_drift) or
                          scaled_ptp > 1.00000001 or scaled_rhs > 1.00000001 or scaled_drift > .50000001):
            raise ValueError(f'Row {rowid}: convergence certificate exceeds its thresholds')
        reason_total = checked_reason(get(row, 'reasons_total', 'reason_total'), 'total reason')
        reason_tail = checked_reason(get(row, 'lastwindow_reason', 'reasons_last_window', 'reason_tail', 'reasons_tail'), 'tail reason')
        reason_first5 = checked_reason(get(row, 'first5s_reason', 'reason_first5s'), 'first-five-second reason')
        first5_complete = bool(get(row, 'first5s_complete'))
        tail_safe = reason_tail == 0 and window_complete
        all_safe = reason_total == 0
        flags['computed'][rowid] = 1
        flags['status'][rowid] = status
        flags['converged'][rowid] = converged
        flags['tail_constraint_safe'][rowid] = tail_safe
        flags['all_observed_samples_constraint_safe'][rowid] = all_safe
        flags['usable'][rowid] = converged and tail_safe
        flags['reason_total'][rowid] = reason_total
        flags['reason_tail'][rowid] = reason_tail
        flags['reason_first5s'][rowid] = reason_first5
        flags['first5s_complete'][rowid] = first5_complete
        flags['window_complete'][rowid] = window_complete
        if converged:
            for name, state_index in [('vc_ss', 0), ('r_ss', 1), ('phi_ss', 3)]:
                outputs[name][rowid] = mean[state_index]
        metrics['end_time_s'][rowid] = finite_scalar(get(row, 'end_time_s', 'endtime', 'time_s'), 'end time')
        metrics['max_scaled_ptp'][rowid] = scaled_ptp
        metrics['max_scaled_rhs'][rowid] = scaled_rhs
        metrics['max_scaled_mean_drift'][rowid] = scaled_drift
        metrics['clipping_total'][rowid] = clipping_total
        metrics['clipping_tail'][rowid] = clipping_tail
        metrics['consecutive_good_windows'][rowid] = good_windows
        final_state[rowid] = x
        win = get(row, 'window_s', default=None, required=False)
        if win is not None:
            windows.add(float(win))
        limit = get(row, 'max_time_s', default=None, required=False)
        if limit is not None:
            max_times.add(float(limit))
    # Numeric LUT breakpoints are ascending. Preserve native descending-mu
    # row ids only in the separate flat diagnostic rows and row-id map.
    reshape = lambda a: a.reshape(SHAPE, order='C')[:, ::-1, :, :, :].copy()
    complete = len(rows) == TOTAL
    metadata = json.loads(Path(args.run_metadata).read_text()) if args.run_metadata else {}
    for value in outputs.values():
        if not np.isfinite(value[flags['converged'] != 0]).all() or not np.isnan(value[flags['converged'] == 0]).all():
            raise ValueError('Steady-state output validity disagrees with convergence flags')
    lut = {
        'schema_version': 'fullcar-steady-state-lut-v1',
        'description': 'Simulated steady-state map for 210 operating conditions x 5 tire-wear cases x 81 corner spring modes',
        'complete': np.uint8(complete), 'completed_count': np.uint32(len(rows)),
        'expected_count': np.uint32(TOTAL),
        'generated_utc': datetime.now(timezone.utc).isoformat(),
        'axes_order': np.array(AXES, dtype=object), 'array_size': np.array(SHAPE, dtype=np.uint32),
        'speed_kmh': SPEEDS, 'mu_road': MUS[::-1].copy(), 'front_wheel_steer_deg': STEERS,
        'engine_row_order_mu_road': MUS,
        'row_id_zero_based_map': reshape(np.arange(TOTAL, dtype=np.uint32)),
        'wear_case_index_matlab': np.arange(1, 6, dtype=np.uint8), 'wear_cases': WEARS,
        'suspension_mode_index_matlab': np.arange(1, 82, dtype=np.uint8),
        'suspension_modes': np.array(MODES, dtype=object),
        'corner_order': np.array(['FL', 'FR', 'RL', 'RR'], dtype=object),
        'corner_mode_order': np.array(['n', 's', 'h'], dtype=object),
        'k_spring_N_m': SPRINGS, 'I_mA': np.full((81, 4), 950.),
        'status_code': np.array(list(STATUS.values()), dtype=np.uint8),
        'status_name': np.array(list(STATUS), dtype=object),
        'reason_bit_values': np.array([1, 2, 4, 8, 16], dtype=np.uint8),
        'reason_bit_names': np.array(['yaw_rate_limit', 'lateral_acceleration_limit', 'LTR_limit',
                                      'nonfinite_numeric', 'hard_state_guard'], dtype=object),
        'output_names': np.array(['vc_ss', 'r_ss', 'phi_ss'], dtype=object),
        'output_units': np.array(['m/s', 'rad/s', 'rad'], dtype=object),
        'state_names': np.array(STATE_NAMES, dtype=object),
        'state_units': np.array(STATE_UNITS, dtype=object),
        'convergence_state_ptp_tolerance': tol_state,
        'convergence_rhs_abs_tolerance': tol_rhs,
        'convergence_window_s': np.array(sorted(windows)) if windows else np.array([2.]),
        'convergence_required_good_windows': np.uint8(2),
        'convergence_check_interval_s': 2.,
        'convergence_earliest_time_s': 7.,
        'convergence_mean_drift_scaled_limit': .5,
        'simulation_time_limit_s': np.array(sorted(max_times)) if max_times else np.array([metadata.get('max_time_s', np.nan)]),
        'provenance': make_provenance(metadata),
        'diagnostics': {
            'row_id_zero_based': np.arange(TOTAL, dtype=np.uint32),
            'row_id_order': 'C order: mode fastest, then wear, steer ascending, mu descending [0.9 0.8 0.7 0.6 0.5], speed ascending',
            'final_state': final_state,
        },
    }
    for name, value in outputs.items():
        lut[name] = reshape(value)
        safe = value.copy()
        safe[flags['usable'] == 0] = np.nan
        lut['safe_' + name] = reshape(safe)
    for name, value in flags.items():
        lut[name] = reshape(value)
    for name, value in metrics.items():
        lut[name] = reshape(value)
    path = Path(args.output_mat)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + '.tmp')
    savemat(tmp, {'STEADY_STATE_LUT': lut}, do_compression=True, long_field_names=True,
            appendmat=False, oned_as='row')
    if tmp.stat().st_size >= 90 * 1024 * 1024:
        tmp.unlink()
        raise ValueError('MAT exceeds the 90 MiB publication budget')
    check = loadmat(tmp, simplify_cells=True)['STEADY_STATE_LUT']
    for name in [*outputs, *flags, *metrics]:
        expected = lut[name]
        actual = check[name]
        if actual.shape != SHAPE or not np.array_equal(actual, expected, equal_nan=True):
            raise ValueError(f'MAT read-back mismatch in {name}')
    if not np.array_equal(check['diagnostics']['final_state'], final_state, equal_nan=True):
        raise ValueError('MAT final-state read-back mismatch')
    tmp.replace(path)
    counts = Counter(int(x) for x in flags['status'])
    report = {
        'schema_version': 'fullcar-steady-state-lut-validation-v1',
        'complete': complete, 'completed_count': len(rows), 'expected_count': TOTAL,
        'all_checks_passed': True, 'array_size': list(SHAPE),
        'status_counts': {name: counts[code] for name, code in STATUS.items()},
        'converged_count': int(flags['converged'].sum()),
        'usable_safe_steady_count': int(flags['usable'].sum()),
        'input_jsonl': [{'name': Path(p).name, 'sha256': sha(p)} for p in args.input_jsonl],
        'output_file': path.name, 'output_bytes': path.stat().st_size, 'output_sha256': sha(path),
        'checks': ['unique bounded row ids', 'row-id-to-condition mapping',
                   'all 81 n/s/h mode stiffness vectors', '950 mA CDC currents',
                   'five tire-wear vectors', 'convergence certificate thresholds',
                   'nonconverged/missing outputs remain NaN', 'MAT round-trip exact arrays',
                   'MAT final state round-trip', 'file under 90 MiB'],
    }
    path.with_suffix('.validation.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--input-jsonl', '--inputjsonl', '--input', nargs='+', required=True)
    p.add_argument('--output-mat', '--outputmat', '--output', default=str(ROOT / 'results/steady_state_lookup/steady_state_lookup_210x5x81.mat'))
    p.add_argument('--run-metadata')
    p.add_argument('--allow-partial', action='store_true',
                   help='Write an explicitly incomplete checkpoint; missing rows stay status=0 and NaN')
    build(p.parse_args())


if __name__ == '__main__':
    try:
        main()
    except (ValueError, KeyError, TypeError, OSError) as exc:
        print(f'Lookup build failed: {exc}', file=sys.stderr)
        raise SystemExit(2)
