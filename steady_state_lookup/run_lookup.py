#!/usr/bin/env python3
"""Resume the requested 210 x 5 x 81 actual steady-state simulations."""
import argparse
import collections
from datetime import datetime, timezone
import fcntl
import hashlib
import itertools
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
WORK = ROOT / 'validation/steady_state_lookup/run'
PUBLIC = ROOT / 'results/steady_state_lookup'
TOTAL = 85050
PER_SCENARIO = 405
COMBINATIONS = list(itertools.product([50, 60, 70, 80, 90, 100],
                                     [.9, .8, .7, .6, .5],
                                     [0, .5, 1, 1.5, 2, 2.5, 3]))
child = None


def utc():
    return datetime.now(timezone.utc).isoformat()


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def atomic_json(path, data):
    temp = path.with_name(path.name + '.tmp')
    temp.write_text(json.dumps(data, indent=2, allow_nan=False) + '\n')
    temp.replace(path)


def read_rows(path, start, stop, allow_incomplete=False):
    result = {}
    if not path.exists():
        if allow_incomplete:
            return result
        raise ValueError(f'Missing chunk: {path}')
    with path.open() as stream:
        for line in stream:
            try:
                row = json.loads(line)
            except json.JSONDecodeError:
                if allow_incomplete:
                    break
                raise
            row_id = row['row_id']
            assert isinstance(row_id, int) and start <= row_id < stop
            assert row_id not in result, 'Duplicate completed simulation'
            assert row['status'] in {'converged', 'not_converged_time_limit',
                                     'numerical_failure', 'hard_guard'}
            result[row_id] = row
    if not allow_incomplete:
        assert len(result) == stop - start, f'Incomplete chunk: {path}'
    return result


def shutdown(signum, _frame):
    if child is not None and child.poll() is None:
        os.killpg(child.pid, signal.SIGTERM)
        try:
            child.wait(timeout=15)
        except subprocess.TimeoutExpired:
            os.killpg(child.pid, signal.SIGKILL)
            child.wait()
    raise KeyboardInterrupt(f'Stopped by signal {signum}')


def run(args):
    global child
    WORK.mkdir(parents=True, exist_ok=True)
    PUBLIC.mkdir(parents=True, exist_ok=True)
    chunks = WORK / 'chunks'
    chunks.mkdir(exist_ok=True)
    lock = (WORK / 'run.lock').open('w')
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    lock.write(str(os.getpid()))
    lock.flush()
    engine = Path(args.engine).resolve()
    source = ROOT / 'steady_state_lookup/native_steady_lookup.cpp'
    model = ROOT / 'steady_state_lookup/model/native_accel.cpp'
    assert engine.is_file() and source.is_file() and model.is_file()
    original = json.loads((model.parent / 'frozen_source_fingerprint.json').read_text())
    assert digest(model) == original['native_accel.cpp']
    frozen = Path('/workspace/.matlab-env/reference09_full210_code')
    if frozen.is_dir():
        assert all(digest(frozen / name) == value for name, value in original.items())
    fingerprint = dict(engine_sha256=digest(engine), adapter_sha256=digest(source),
                       frozen_model_sha256=digest(model), max_time_s=args.max_time,
                       Ts_s=.01, RK4_substeps=20, total=TOTAL,
                       mode_order='n,s,h; FL FR RL RR; RR varies fastest')
    manifest_path = WORK / 'run_fingerprint.json'
    if manifest_path.exists():
        assert json.loads(manifest_path.read_text()) == fingerprint, 'Run provenance changed'
    else:
        atomic_json(manifest_path, fingerprint)
    started = utc()
    complete = 0
    scenarios_done = 0
    counts = collections.Counter()

    def status(phase, partial=None, scenario=None, **extra):
        p = partial or {}
        current = None
        if scenario is not None:
            speed, mu, steer = COMBINATIONS[scenario]
            for wear in range(1, 6):
                modes_done = sum(row['wear_id'] == wear for row in p.values())
                if modes_done < 81:
                    current = dict(scenario_index=scenario + 1, speed_kmh=speed,
                                   mu_road=mu, front_steer_deg=steer,
                                   wear_case=wear, wear_total=5,
                                   modes_completed=modes_done, modes_total=81)
                    break
        data = dict(phase=phase, supervisor_pid=os.getpid(),
                    engine_pid=child.pid if child is not None and child.poll() is None else None,
                    started_utc=started, updated_utc=utc(),
                    completed_scenarios=scenarios_done, total_scenarios=210,
                    completed_simulations=complete + len(p), total_simulations=TOTAL,
                    status_counts=dict(counts + collections.Counter(row['status'] for row in p.values())),
                    current=current, max_time_s=args.max_time, **extra)
        atomic_json(PUBLIC / 'progress.json', data)
        return data

    for sig in (signal.SIGTERM, signal.SIGINT):
        signal.signal(sig, shutdown)
    try:
        for index, _ in enumerate(COMBINATIONS):
            start, stop = index * PER_SCENARIO, (index + 1) * PER_SCENARIO
            target = chunks / f'scenario_{index+1:03d}.jsonl'
            if target.exists():
                rows = read_rows(target, start, stop)
            else:
                temporary = target.with_suffix('.jsonl.tmp')
                if temporary.exists():
                    temporary.rename(temporary.with_name(temporary.name + f'.interrupted.{time.time_ns()}'))
                assert digest(engine) == fingerprint['engine_sha256']
                command = [str(engine), '--start', str(start), '--stop', str(stop),
                           '--threads', str(args.threads), '--max-time', str(args.max_time),
                           '--output', str(temporary)]
                env = os.environ.copy()
                env.update(OPENBLAS_NUM_THREADS='1', OMP_DYNAMIC='FALSE')
                with (WORK / 'native.log').open('ab', buffering=0) as log:
                    child = subprocess.Popen(command, cwd=ROOT, stdin=subprocess.DEVNULL,
                                             stdout=log, stderr=subprocess.STDOUT,
                                             start_new_session=True, env=env)
                    status('running', scenario=index)
                    while child.poll() is None:
                        status('running', read_rows(temporary, start, stop, True), index)
                        try:
                            child.wait(timeout=3)
                        except subprocess.TimeoutExpired:
                            pass
                if child.returncode != 0:
                    raise RuntimeError(f'Engine exited {child.returncode} in scenario {index+1}')
                rows = read_rows(temporary, start, stop)
                temporary.replace(target)
            complete += len(rows)
            scenarios_done += 1
            counts.update(row['status'] for row in rows.values())
            status('running', scenario=index+1 if index+1 < 210 else None)
            print(f'{scenarios_done}/210; simulations {complete}/{TOTAL}; {dict(counts)}', flush=True)
        merged = WORK / 'rows.jsonl'
        temporary = merged.with_suffix('.jsonl.tmp')
        with temporary.open('w') as stream:
            for index in range(210):
                rows = read_rows(chunks / f'scenario_{index+1:03d}.jsonl',
                                 index * PER_SCENARIO, (index+1) * PER_SCENARIO)
                for row_id in sorted(rows):
                    stream.write(json.dumps(rows[row_id], separators=(',', ':'), allow_nan=False) + '\n')
        temporary.replace(merged)
        status('simulations_completed', raw_rows=str(merged))
        if args.build:
            status('building_mat', raw_rows=str(merged))
            command = [sys.executable, str(ROOT / 'steady_state_lookup/build_lookup.py'),
                       '--input', str(merged), '--output',
                       str(PUBLIC / 'steady_state_lookup_210x5x81.mat'),
                       '--run-metadata', str(manifest_path)]
            subprocess.run(command, cwd=ROOT, check=True,
                           env=dict(os.environ, OPENBLAS_NUM_THREADS='1'))
            status('mat_completed', raw_rows=str(merged))
        return 0
    except BaseException as exc:
        status('cancelled' if isinstance(exc, KeyboardInterrupt) else 'failed', error=str(exc))
        raise


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--engine', default=str(ROOT / 'validation/steady_state_lookup/native_steady_lookup'))
    parser.add_argument('--max-time', type=float, default=60)
    parser.add_argument('--threads', type=int, default=4)
    parser.add_argument('--build', action='store_true')
    args = parser.parse_args()
    return run(args)


if __name__ == '__main__':
    sys.exit(main())
