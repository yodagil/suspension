#!/usr/bin/env python3
"""Three fixed-speed quasistatic Eq.11 comparison runs, preserving the fine grid."""
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor, as_completed
import hashlib
import json
import os
import subprocess
ROOT=Path('/workspace/suspension')
CODE=ROOT/'paper_model_fig6/matlab'
OUT=ROOT/'validation/fig6_quasistatic_roll_full_grid'
OUT.mkdir(exist_ok=True)
hashes={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in CODE.glob('*.m')}
plan=dict(speed_kmh=40,mu=.85,angles_deg=[0,3.5,7],speed_mode='fixed',
          model='Original quasistatic Eq.11; fixed-speed approximation and missing-parameter closure',
          ms=1300,hphi=.54,Ks=1,burckhardt_shape=[1.28,23.99,.52],
          grid_shape=[41,55,31],states_per_angle=69905,dt=.01,horizon_s=5,
          initial_roll_rate=0,source_sha256=hashes)
(OUT/'plan.json').write_text(json.dumps(plan,indent=2)+'\n')
env=os.environ.copy();env.update(OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1')

def run(angle):
    folder=OUT/f'angle_{angle:g}';folder.mkdir(exist_ok=True)
    target=OUT/f'classification_{angle:g}deg.mat'
    script=folder/'run.m'
    script.write_text(f"""addpath('{CODE}');
cfg=paper_config();cfg.speed_mode='fixed';cfg.speed_kmh=40;cfg.mu=.85;
cfg.ms=1300;cfg.hphi=.54;cfg.Ks=1;cfg.burckhardt_shape=[1.28 23.99 .52];
cfg.steer_deg={angle:g};cfg.dt=.01;cfg.horizon_s=5;cfg.batch_size=3500;
cfg.progress_path='{folder}/progress.json';
result=paper_simulate_grid(cfg);
meta=struct('model','Original quasistatic Eq.11; fixed-speed approximation and missing-parameter closure', ...
    'paper_exact_reproduction',false,'inner_region_algorithm','pending', ...
    'engine','GNU Octave MATLAB-compatible M-code', ...
    'initial_roll_rate_rad_s',0,'grid_shape',[41 55 31]);
PAPER_STABILITY3D=struct('meta',meta,'cfg',cfg,'result',result);
save('{target}','PAPER_STABILITY3D','-v7');
fprintf('FULL GRID SAVED steer=%g stable=%d total=%d nonfinite=%d\\n',cfg.steer_deg,result.stable_count,result.total_count,result.nonfinite_failure_count);
""")
    with (folder/'run.log').open('w') as log:
        p=subprocess.run(['/workspace/.matlab-env/bin/octave','--quiet',str(script)],
                         cwd='/tmp',env=env,stdout=log,stderr=subprocess.STDOUT)
    return dict(angle_deg=angle,returncode=p.returncode,output=str(target),
                exists=target.exists())

cases=[]
with ThreadPoolExecutor(max_workers=3) as pool:
    fs={pool.submit(run,a):a for a in [0,3.5,7]}
    for f in as_completed(fs):
        cases.append(f.result())
        (OUT/'status.json').write_text(json.dumps(dict(completed=len(cases),total=3,
            cases=cases,complete=len(cases)==3),indent=2)+'\n')
        print('COMPLETED',len(cases),'/3',cases[-1],flush=True)
assert all(c['returncode']==0 and c['exists'] for c in cases)
after={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in CODE.glob('*.m')}
assert hashes==after,'Source changed during calculation'
print('ALL THREE FULL-GRID RUNS COMPLETED; source hashes unchanged',flush=True)
