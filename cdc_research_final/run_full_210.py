#!/usr/bin/env python3
"""Run the final research H-preserving CDC map on its trim-aligned 2-D slice."""
import argparse
from datetime import datetime, timezone
import fcntl
import hashlib
import itertools
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import time

ROOT = Path(__file__).resolve().parents[1]
COMBOS = list(itertools.product([50, 60, 70, 80, 90, 100], [.9, .8, .7, .6, .5],
                               [0, .5, 1, 1.5, 2, 2.5, 3]))
TAGS = [f'research_{s:g}kph_mu{mu:g}_delta{a:+g}_I950_950_950_950_trim' for s, mu, a in COMBOS]
child = None


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def utc():
    return datetime.now(timezone.utc).isoformat()


def atomic(path, data):
    temp = path.with_name(path.name + '.tmp')
    temp.write_text(json.dumps(data, indent=2, allow_nan=False) + '\n')
    temp.replace(path)


def cancel(signum, _frame):
    if child is not None and child.poll() is None:
        os.killpg(child.pid, signal.SIGTERM)
        try:
            child.wait(timeout=20)
        except subprocess.TimeoutExpired:
            os.killpg(child.pid, signal.SIGKILL)
            child.wait()
    raise KeyboardInterrupt(f'Cancelled by signal {signum}')


def main():
    global child
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--native', required=True)
    parser.add_argument('--octave', default='/workspace/.matlab-env/bin/octave')
    parser.add_argument('--threads', type=int, default=4)
    parser.add_argument('--output', default=str(ROOT/'results/cdc_research_final'))
    parser.add_argument('--frozen-dir', default=str(ROOT/'validation/cdc_research_final/frozen_code'))
    args = parser.parse_args()
    out = Path(args.output).resolve()
    frozen = Path(args.frozen_dir).resolve()
    out.mkdir(parents=True, exist_ok=True)
    frozen.mkdir(parents=True, exist_ok=True)
    lock = (out/'producer.lock').open('w')
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    lock.write(str(os.getpid()))
    lock.flush()
    sources = ROOT/'cdc_research_final'
    source_files = {str(path.relative_to(sources)):path for path in sources.glob('*.m')}
    source_files.update({str(path.relative_to(sources)):path for path in (sources/'native').glob('*.cpp')})
    source_files.update({str(path.relative_to(sources)):path for path in (sources/'native').glob('*.hpp')})
    source_files['native/cdc_research_native'] = Path(args.native).resolve()
    hashes = {name:sha(path) for name,path in source_files.items()}
    manifest = frozen/'source_fingerprint.json'
    if manifest.exists():
        assert json.loads(manifest.read_text()) == hashes, 'Changed code: choose a new output and frozen directory'
        assert all(sha(frozen/name) == value for name,value in hashes.items())
    else:
        for name,path in source_files.items():
            (frozen/name).parent.mkdir(parents=True,exist_ok=True)
            shutil.copy2(path,frozen/name)
        atomic(manifest,hashes)
    quote = lambda value: "'" + str(value).replace("'", "''") + "'"
    expression = (
        f"addpath({quote(frozen)});v=struct();v.engine='native';"
        f"v.native_executable={quote(frozen/'native/cdc_research_native')};"
        f"v.native_threads={args.threads};v.output_dir={quote(out)};"
        "v.trace_stride=0;v.verbose=true;R=run_cdc_research('full',v);"
        f"save({quote(out/'completed_manifest.mat')},'R','-v7');")
    (frozen/'production_expression.m').write_text(expression + '\n')
    state = dict(phase='starting', supervisor_pid=os.getpid(), started_utc=utc(),
                 total_scenarios=210, total_wear_cases=5, grid_size=[61,121],
                 slice_mode='trim',initial_roll_mode='per-case computed trim phi_eq',
                 main_mask="final", horizons_s=[5,10], dt_s=.0005,
                 geometric_grid_candidates=7750050, expected_H_initial_trajectories=841284,
                 preliminary_estimate_hours=[10,15], source_sha256=hashes)

    def status(phase, **extra):
        completed = sum((out/(tag+'.mat')).is_file() for tag in TAGS)
        current = None
        for index, tag in enumerate(TAGS):
            if not (out/(tag+'.mat')).is_file():
                done = sum((out/f'{tag}_wear{i}_checkpoint.mat').is_file() for i in range(1,6))
                speed, mu, steer = COMBOS[index]
                current = dict(scenario_index=index+1, speed_kmh=speed, mu_road=mu,
                               front_steer_deg=steer, completed_wear_cases=done,
                               current_wear_case=min(done+1,5), wear_total=5)
                break
        state.update(phase=phase, updated_utc=utc(), completed_scenarios=completed,
                     current=current, **extra)
        atomic(out/'progress.json',state)
        return completed

    for sig in (signal.SIGTERM, signal.SIGINT):
        signal.signal(sig,cancel)
    try:
        env=dict(os.environ,OPENBLAS_NUM_THREADS='1',OMP_DYNAMIC='FALSE')
        command=[args.octave,'--quiet','--eval',expression]
        with (out/'simulation.log').open('ab',buffering=0) as log:
            child=subprocess.Popen(command,cwd=ROOT,env=env,stdin=subprocess.DEVNULL,
                                   stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            status('running',producer_pid=child.pid)
            while child.poll() is None:
                assert all(sha(frozen/name)==value for name,value in hashes.items()), 'Frozen code changed during calculation'
                status('running')
                try:child.wait(timeout=5)
                except subprocess.TimeoutExpired:pass
        count=sum((out/(tag+'.mat')).is_file() for tag in TAGS)
        success=child.returncode==0 and count==210
        status('completed' if success else 'failed',producer_returncode=child.returncode)
        assert success, 'The 210-condition sweep is incomplete'
    except BaseException as exc:
        if child is not None and child.poll() is None:
            os.killpg(child.pid,signal.SIGTERM)
        status('cancelled' if isinstance(exc,KeyboardInterrupt) else 'failed',error=str(exc))
        raise


if __name__=='__main__':
    main()
