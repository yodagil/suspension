#!/usr/bin/env python3
"""Run the V2 full-car CDC safety/recovery map on its trim-aligned 2-D slice."""
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
TAGS = [f'fullcar2d_v2_{s:g}kph_mu{mu:g}_delta{a:+g}_trim_trim' for s, mu, a in COMBOS]
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
    parser.add_argument('--output', default=str(ROOT/'results/fullcar_2d_v2'))
    parser.add_argument('--frozen-dir', default=str(ROOT/'validation/fullcar_2d_v2/frozen_code'))
    args = parser.parse_args()
    out = Path(args.output).resolve()
    frozen = Path(args.frozen_dir).resolve()
    out.mkdir(parents=True, exist_ok=True)
    frozen.mkdir(parents=True, exist_ok=True)
    lock = (out/'producer.lock').open('w')
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    lock.write(str(os.getpid()))
    lock.flush()
    sources = ROOT/'fullcar_2d_v2/matlab'
    files = ['stability_fullcar_2d_v2.m', 'cdc2d_native_simulate.m', 'export_cdc_native_case.m']
    hashes = {name:sha(sources/name) for name in files}
    native = Path(args.native).resolve()
    hashes['fullcar2d_v2_native'] = sha(native)
    manifest = frozen/'source_fingerprint.json'
    if manifest.exists():
        assert json.loads(manifest.read_text()) == hashes, 'Changed code: choose a new output and frozen directory'
        assert all(sha(frozen/name) == value for name, value in hashes.items())
    else:
        for name in files:
            shutil.copy2(sources/name, frozen/name)
        shutil.copy2(native, frozen/'fullcar2d_v2_native')
        atomic(manifest, hashes)
    quote = lambda value: "'" + str(value).replace("'", "''") + "'"
    expression = (
        f"addpath({quote(frozen)});o=stability_fullcar_2d_v2('defaults');"
        "o.speeds=[50 60 70 80 90 100];o.mus=[.9 .8 .7 .6 .5];o.steers=0:.5:3;"
        "o.wear_cases=[0 0 0 0;25 25 0 0;50 50 25 25;0 0 25 25;25 25 50 50];"
        "o.grid=struct('vc',-6:.25:6,'r',-1.2:.03:1.2);o.slice_mode='trim';o.use_yaw_policy_for_region=false;"
        "o.horizons=[5 10];o.engine='native';"
        f"o.native_executable={quote(frozen/'fullcar2d_v2_native')};o.native_threads={args.threads};"
        f"o.output_dir={quote(out)};R=stability_fullcar_2d_v2(o);"
        f"save({quote(out/'completed_manifest.mat')},'R','-v7');")
    (frozen/'production_expression.m').write_text(expression + '\n')
    state = dict(phase='starting', supervisor_pid=os.getpid(), started_utc=utc(),
                 total_scenarios=210, total_wear_cases=5, grid_size=[49,81],
                 slice_mode='trim',initial_roll_mode='per-case computed trim phi_eq',
                 use_yaw_policy_for_region=False, horizons_s=[5,10], dt_s=.0005,
                 primary_grid_trajectories=4167450, source_sha256=hashes)

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
