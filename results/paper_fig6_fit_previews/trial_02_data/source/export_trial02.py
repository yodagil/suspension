#!/usr/bin/env python3
"""Export the three actual fine-grid runs with an explicit inner-region label."""
from pathlib import Path
import hashlib,json
import numpy as np
from scipy.io import loadmat,savemat
from scipy.spatial import ConvexHull
ROOT=Path('/workspace/suspension')
SOURCE=ROOT/'validation/fig6_dynamic_roll_full_grid'
OUT=ROOT/'results/paper_fig6_fit_previews/trial_02_data'
OUT.mkdir(parents=True,exist_ok=True)
regions=json.loads((ROOT/'validation/fig6_dynamic_roll/equilibrium_sheared_full.json').read_text())
plan=json.loads((SOURCE/'plan.json').read_text())
checks=[]
for region in regions['cases']:
    angle=region['angle_deg'];raw_path=SOURCE/f'classification_{angle:g}deg.mat'
    s=loadmat(raw_path,simplify_cells=True)['PAPER_STABILITY3D']
    cfg=s['cfg'];r=s['result'];v=np.asarray(region['vertices']);p=r['initial_points']
    stable=np.asarray(r['stable_mask'],bool).ravel(order='F')
    assert r['total_count']==69905 and r['nonfinite_failure_count']==0 and r['low_speed_domain_guard_count']==0
    assert cfg['speed_mode']=='fixed' and cfg['speed_kmh']==40 and cfg['mu']==.85
    assert cfg['load_transfer_mode']=='dynamic_roll_reaction'
    assert cfg['dt']==.01 and cfg['horizon_s']==5 and cfg['ms']==1300 and cfg['hphi']==.35 and cfg['Ks']==1
    assert np.allclose(cfg['burckhardt_shape'],[1.28,23.99,.52])
    ids=np.asarray(region['vertex_grid_indices_1based'],int)-1
    assert np.all(stable[ids]) and np.allclose(p[ids],v,atol=1e-12,rtol=0)
    scale=np.ptp(p,axis=0);h=ConvexHull(v/scale)
    assert len(h.vertices)==8
    in_hull=np.all((p/scale)@h.equations[:,:3].T+h.equations[:,3]<=1e-10,axis=1)
    assert not np.any(in_hull&~stable)
    assert abs(h.volume*np.prod(scale)-region['volume'])<1e-9
    eq=np.asarray(region['equilibrium'])
    assert np.all((eq/scale)@h.equations[:,:3].T+h.equations[:,3]<-1e-10)
    s['region']=region
    s['meta'].update(inner_region_algorithm=region['method'],
        model='Dynamic roll-reaction load extension, not literal Eq.11',
        paper_exact_reproduction=False,
        equilibrium_centered_inner_approximation=True,
        continuous_stability_proof=False,
        full_stable_basin_not_equal_to_inner_approximation=True,
        source_mat_sha256=hashlib.sha256(raw_path.read_bytes()).hexdigest(),
        simulation_source_files=list(plan['source_sha256']),
        simulation_source_sha256=list(plan['source_sha256'].values()),
        inertia_convention_assumption='Ixx=700 is used in Eq.4 and the lumped global roll-reaction term; paper inertia reference unspecified',
        grid_guarantee='No unstable original-grid sample on or inside the eight-vertex inner approximation')
    target=OUT/f'paper_dynamic_roll_40kph_0.85mu_{angle:g}deg.mat'
    savemat(target,{'PAPER_STABILITY3D':s},do_compression=True,long_field_names=True)
    verified=loadmat(target,simplify_cells=True)['PAPER_STABILITY3D']
    assert np.array_equal(verified['result']['stable_mask'],r['stable_mask'])
    assert np.allclose(verified['region']['vertices'],v)
    checks.append(dict(angle_deg=angle,initial_states=69905,stable_count=int(r['stable_count']),
        inner_volume=region['volume'],inner_roll_bounds=[float(v[:,2].min()),float(v[:,2].max())],
        inner_axis_spans=np.ptp(v,axis=0).tolist(),
        raw_stable_roll_bounds=region['raw_stable_roll_bounds'],
        unstable_original_grid_samples_in_or_on_region=0,
        all_eight_vertices_actual_stable_grid_nodes=True,
        exact_equilibrium_strictly_inside=True,
        nonfinite_failure_count=0,mat_file=target.name,
        exported_mat_sha256=hashlib.sha256(target.read_bytes()).hexdigest()))
    print('EXPORTED',target.name,checks[-1]['inner_roll_bounds'],flush=True)
summary=dict(trial=2,speed_kmh=40,mu=.85,angles_deg=[0,3.5,7],speed_mode='fixed',
    initial_grid_shape=[41,55,31],initial_conditions_per_scenario=69905,horizon_s=5,dt=.01,
    simulation_engine='GNU Octave, MATLAB-compatible simulation M-code',
    model='Explicit dynamic roll-reaction load extension, not literal Eq.11',
    estimated_parameters=dict(ms=1300,hphi=.35,Ks=1,burckhardt_shape=[1.28,23.99,.52]),
    published_table1_constants_unchanged=True,
    model_extension='D=m*h*ay + ms*g*hphi*sin(phi) - Ixx*pdot; zero active suspension',
    inertia_reference_is_assumed=True,
    geometry='Common maximum-volume equilibrium-centered grid parallelepiped candidate family',
    continuous_stability_proof=False,paper_exact_reproduction=False,
    raw_stable_set_distinct_from_conservative_inner_approximation=True,
    plotted_coordinates_manually_scaled=False,cases=checks)
(OUT.parent/'trial_02.json').write_text(json.dumps(summary,indent=2)+'\n')
