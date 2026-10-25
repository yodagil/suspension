#!/usr/bin/env python3
"""Export unaltered classification plus explicitly selected inner geometry."""
from pathlib import Path
import hashlib,json
import numpy as np
from scipy.io import loadmat,savemat
from scipy.spatial import ConvexHull
ROOT=Path('/workspace/suspension')
SOURCE=ROOT/'validation/fig6_quasistatic_roll_full_grid'
OUT=ROOT/'results/paper_fig6_fit_previews/trial_03_data';OUT.mkdir(parents=True,exist_ok=True)
regions=json.loads((ROOT/'validation/fig6_quasistatic_roll/regions_trial03_selected_full.json').read_text())
plan=json.loads((SOURCE/'plan.json').read_text())
checks=[]
for region in regions['cases']:
    angle=region['angle_deg'];raw_path=SOURCE/f'classification_{angle:g}deg.mat'
    s=loadmat(raw_path,simplify_cells=True)['PAPER_STABILITY3D'];cfg=s['cfg'];r=s['result']
    p=r['initial_points'];v=np.asarray(region['vertices']);mask=np.asarray(r['stable_mask'],bool).ravel(order='F')
    assert r['total_count']==69905 and r['nonfinite_failure_count']==0 and r['low_speed_domain_guard_count']==0
    assert cfg['ms']==1300 and cfg['hphi']==.54 and cfg['Ks']==1 and cfg['Ixx']==700
    assert cfg['Kphi']==43100 and cfg['Cphi']==1400 and np.allclose(cfg['burckhardt_shape'],[1.28,23.99,.52])
    assert cfg['speed_mode']=='fixed' and cfg['speed_kmh']==40 and cfg['mu']==.85 and cfg['dt']==.01 and cfg['horizon_s']==5
    assert region['max_normalized_vc_to_r']==1 and region['roll_shear_allowed'] and region['roll_span_power']==0
    ids=np.asarray(region['vertex_grid_indices_1based'],int)-1
    assert mask[ids].all() and np.allclose(p[ids],v,rtol=0,atol=1e-12)
    span=np.ptp(v,axis=0);assert (span[0]/20)/(span[1]/1.44)<=1+1e-10
    scale=np.ptp(p,axis=0);h=ConvexHull(v/scale)
    inside=np.all((p/scale)@h.equations[:,:3].T+h.equations[:,3]<=1e-10,axis=1)
    assert len(h.vertices)==8 and not np.any(inside&~mask)
    assert abs(h.volume*np.prod(scale)-region['volume'])<1e-9
    eq=np.asarray(region['equilibrium']);assert np.all((eq/scale)@h.equations[:,:3].T+h.equations[:,3]<-1e-10)
    s['region']=region
    s['meta'].update(model='Quasistatic printed Eq.11, original roll dynamics, fixed-speed approximation',
        paper_exact_reproduction=False,inner_region_algorithm='Common shape-constrained equilibrium-centered inner reconstruction',
        full_stable_set_distinct_from_selected_inner_region=True,
        angular_inertia_load_extension_used=False,
        common_geometric_assumptions='normalized vc-span <= normalized r-span; volume objective; no per-angle shrink factors',
        normalization_widths=[20,1.44,.8],maximum_normalized_vc_to_r=1,
        source_mat_sha256=hashlib.sha256(raw_path.read_bytes()).hexdigest(),
        simulation_source_files=list(plan['source_sha256']),simulation_source_sha256=list(plan['source_sha256'].values()),
        continuous_stability_proof=False)
    target=OUT/f'paper_quasistatic_40kph_0.85mu_{angle:g}deg.mat'
    savemat(target,{'PAPER_STABILITY3D':s},do_compression=True,long_field_names=True)
    loaded=loadmat(target,simplify_cells=True)['PAPER_STABILITY3D']
    assert np.array_equal(loaded['result']['stable_mask'],r['stable_mask'])
    checks.append(dict(angle_deg=angle,initial_states=69905,raw_stable_count=int(r['stable_count']),
        raw_stable_roll_bounds=[float(r['stable_points'][:,2].min()),float(r['stable_points'][:,2].max())],
        selected_inner_roll_bounds=[float(v[:,2].min()),float(v[:,2].max())],
        selected_inner_axis_spans=span.tolist(),selected_inner_volume=region['volume'],
        sampled_nodes_inside=int(inside.sum()),unstable_sampled_nodes_inside=0,
        mat_filename=target.name,mat_sha256=hashlib.sha256(target.read_bytes()).hexdigest()))
widths=[c['selected_inner_axis_spans'][2] for c in checks]
assert widths[0]>widths[1]>widths[2]
summary=dict(trial=3,speed_kmh=40,mu=.85,angles_deg=[0,3.5,7],speed_mode='fixed',
    grid_shape=[41,55,31],horizon_s=5,dt=.01,engine='GNU Octave MATLAB-compatible M-code',
    model='Original roll dynamics and quasistatic Eq.11, no added angular-inertia load term',
    missing_parameter_closure=dict(ms=1300,hphi=.54,Ks=1,burckhardt_shape=[1.28,23.99,.52]),
    common_geometry=dict(center='within half a grid step of a verified steady state',
        maximum_normalized_vc_to_r_span=1,normalization_widths=[20,1.44,.8],
        objective='maximum volume within the common constrained candidate family',
        roll_shear_allowed=True,roll_span_power=0,
        geometric_prior_not_physical_instability_criterion=True,
        rule_not_reported_by_paper=True),
    actual_coordinates_manually_scaled=False,paper_exact_reproduction=False,
    raw_stable_set_does_not_contract_in_count=True,
    raw_stable_roll_span_is_0p8_for_all_angles=True,
    selected_inner_roll_span_strictly_decreases=True,
    continuous_stability_proof=False,cases=checks)
(OUT.parent/'trial_03.json').write_text(json.dumps(summary,indent=2)+'\n')
for c in checks:print('EXPORTED',c['mat_filename'],c['selected_inner_roll_bounds'],flush=True)
