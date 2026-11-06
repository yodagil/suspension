#!/usr/bin/env python3
"""Repeat only time-limited rows with a longer unchanged-physics trajectory."""
import argparse
from collections import Counter
from concurrent.futures import ThreadPoolExecutor, as_completed
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def atomic(path, obj):
    temp = path.with_name(path.name + '.tmp')
    temp.write_text(json.dumps(obj, indent=2, allow_nan=False) + '\n')
    temp.replace(path)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--max-time', type=float, default=300)
    parser.add_argument('--base-time', type=float, default=60)
    parser.add_argument('--threads', type=int, default=4)
    parser.add_argument('--engine', default=str(ROOT / 'validation/steady_state_lookup/native_steady_lookup'))
    args = parser.parse_args()
    source, output, engine = Path(args.input), Path(args.output), Path(args.engine)
    assert source.resolve() != output.resolve(), 'Preserve the original 60-second results'
    assert args.max_time > args.base_time >= 7 and args.threads > 0
    work = source.parent / 'extended_time_limits'
    work.mkdir(parents=True, exist_ok=True)
    public = ROOT / 'results/steady_state_lookup'
    rows = {}
    with source.open() as stream:
        for line in stream:
            row = json.loads(line)
            assert row['row_id'] not in rows
            rows[row['row_id']] = row
    assert len(rows) == 85050 and set(rows) == set(range(85050))
    selected = [row for row in rows.values() if row['status'] == 'not_converged_time_limit']
    original_counts = dict(Counter(row['status'] for row in rows.values()))
    fingerprint = dict(input_sha256=sha(source), engine_sha256=sha(engine),
                       base_time_s=args.base_time, extended_time_s=args.max_time,
                       selected_row_ids=[row['row_id'] for row in selected])
    manifest = work / 'fingerprint.json'
    if manifest.exists():
        assert json.loads(manifest.read_text()) == fingerprint, 'Extension cache provenance changed'
    else:
        atomic(manifest, fingerprint)
    previous = json.loads((public / 'progress.json').read_text())
    completed = 0
    final_counts = Counter(original_counts)

    def progress(phase):
        data = dict(previous)
        data.update(phase=phase, updated_utc=datetime.now(timezone.utc).isoformat(),
                    supervisor_pid=os.getpid(), engine_pid=None,
                    completed_scenarios=210, completed_simulations=85050,
                    current=None, extension_completed=completed,
                    extension_total=len(selected), extension_time_limit_s=args.max_time,
                    status_counts=dict(final_counts))
        atomic(public / 'progress.json', data)

    def execute(old):
        row_id = old['row_id']
        path = work / f'row_{row_id:05d}.json'
        if path.exists():
            result = json.loads(path.read_text())
        else:
            command = [str(engine.resolve()), '--case', str(old['speed_kmh']),
                       str(old['mu_road']), str(old['steer_deg']),
                       str(old['wear_id']), old['mode'], '--max-time', str(args.max_time)]
            result = json.loads(subprocess.check_output(command, text=True,
                                env=dict(os.environ, OPENBLAS_NUM_THREADS='1')))
            result['max_time_s'] = args.max_time
            result['base_max_time_s'] = args.base_time
            result['extended_horizon'] = True
            atomic(path, result)
        assert result['row_id'] == row_id and result['max_time_s'] == args.max_time
        assert all(result[key] == old[key] for key in
                   ['speed_kmh', 'mu_road', 'steer_deg', 'wear_id', 'mode_id',
                    'mode', 'wear', 'k_spring_N_m', 'I_mA', 'state_tolerance29', 'rhs_tolerance29'])
        return row_id, result

    progress('extending_time_limits')
    with ThreadPoolExecutor(max_workers=args.threads) as pool:
        futures = [pool.submit(execute, row) for row in selected]
        for future in as_completed(futures):
            row_id, result = future.result()
            final_counts[rows[row_id]['status']] -= 1
            final_counts[result['status']] += 1
            rows[row_id] = result
            completed += 1
            progress('extending_time_limits')
            if completed % 25 == 0 or completed == len(selected):
                print(f'Extended convergence checks {completed}/{len(selected)}', flush=True)
    temp = output.with_name(output.name + '.tmp')
    with temp.open('w') as stream:
        for row_id in range(85050):
            row = rows[row_id]
            row.setdefault('max_time_s', args.base_time)
            row.setdefault('extended_horizon', False)
            stream.write(json.dumps(row, separators=(',', ':'), allow_nan=False) + '\n')
    temp.replace(output)
    metadata = json.loads((source.parent / 'run_fingerprint.json').read_text())
    metadata.update(base_max_time_s=args.base_time, max_time_s=args.max_time,
                    simulation_time_limits_s=[args.base_time, args.max_time],
                    additional_simulation_count=len(selected),
                    extension_fingerprint=fingerprint)
    atomic(source.parent / 'final_run_metadata.json', metadata)
    report = dict(base_status_counts=original_counts,
                  final_status_counts=dict(Counter(row['status'] for row in rows.values())),
                  base_simulation_count=85050, additional_simulation_count=len(selected),
                  extended_time_limit_s=args.max_time, final_rows_sha256=sha(output),
                  completed_utc=datetime.now(timezone.utc).isoformat())
    atomic(public / 'extended_convergence_checks.json', report)
    progress('simulations_completed')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
