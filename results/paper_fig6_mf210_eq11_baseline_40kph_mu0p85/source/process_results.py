#!/usr/bin/env python3
"""Apply declared paper Section3.1 sequence to independent printed-Eq11 runs."""
from pathlib import Path
import argparse,hashlib,importlib.util,json,math
import numpy as np
from scipy.io import loadmat,savemat
from scipy.spatial import ConvexHull
HERE=Path(__file__).resolve().parent
GEOMETRY_PATH=HERE/'geometry/paper_sec31_region.py'
if not GEOMETRY_PATH.exists():GEOMETRY_PATH=HERE/'paper_sec31_region.py'
spec=importlib.util.spec_from_file_location('sec31_baseline',GEOMETRY_PATH)
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def clean(x):
    if isinstance(x,dict):return {k:clean(v) for k,v in x.items()}
    if isinstance(x,(list,tuple)):return [clean(v) for v in x]
    if isinstance(x,np.ndarray):return clean(x.tolist())
    if isinstance(x,np.generic):return clean(x.item())
    if isinstance(x,float) and not math.isfinite(x):return None
    return x

def main():
    ap=argparse.ArgumentParser();ap.add_argument('classification_dir',type=Path)
    ap.add_argument('output_dir',type=Path);args=ap.parse_args();args.output_dir.mkdir(parents=True,exist_ok=True)
    cases=[]
    for angle in [0,3.5,7]:
        path=args.classification_dir/f'classification_{angle:g}deg.mat'
        if not path.exists():continue
        saved=loadmat(path,simplify_cells=True)['PAPER_STABILITY3D'];r=saved['result'];g=saved['cfg']['grid']
        mask=np.asarray(r['stable_mask'],bool);geometry=module.paper_sec31_region(mask,g)
        region=dict(geometry['inner_candidate']);region['candidate_volume']=region['volume']
        region['candidate_vertices']=region['vertices'].copy()
        region['inner_volume']=region['volume'] if region['valid'] else np.nan
        region['inner_vertices']=region['vertices'].copy() if region['valid'] else np.empty((0,3))
        if not region['valid']:region['volume']=np.nan
        region.update(method=geometry['method'],angle_deg=angle,axis_order=['vc','r','phi'],
            author_exact_algorithm_claimed=False,max_volume_objective_used=False,equilibrium_anchor_used=False,
            nesting_constraints_used=False,roll_scaling_or_caps_applied=False,
            physical_roll_boundary_resolved=geometry['physical_roll_boundary_resolved'])
        oi=tuple(np.flatnonzero(np.abs(np.asarray(g[k]))<1e-12)[0] for k in ['vc','r','phi'])
        origin=bool(mask[oi]);origin_inside=None;physical_inside=unknown_inside=None
        if len(region['vertices'])==8:
            hull=ConvexHull(region['vertices']);eq=hull.equations
            origin_inside=bool(np.all(eq[:,3]<=1e-10))
            counts=[]
            for key in ['unstable_points','unresolved_points']:
                p=np.asarray(r.get(key,np.empty((0,3)))).reshape(-1,3)
                counts.append(int(np.count_nonzero(np.all(p@eq[:,:3].T+eq[:,3]<=1e-10,axis=1))) if len(p) else 0)
            physical_inside,unknown_inside=counts
            if region['valid']:assert physical_inside==0 and unknown_inside==0
        saved['meta'].update(geometry_not_yet_extracted=False,
            geometry_method=geometry['method'],geometry_author_exact_algorithm_claimed=False,
            geometry_numeric_completion='Boundary vc bands; declared extreme yaw caps and cardinal vc/phi extrema',
            geometry_max_volume_objective_used=False,geometry_equilibrium_anchor_used=False,
            geometry_source_sha256=sha(GEOMETRY_PATH))
        filename=f'stable_point_40kph_0.85mu_{angle:g}deg.mat';target=args.output_dir/filename
        savemat(target,dict(PAPER_STABILITY3D=saved,SECTION31=geometry,INNER_REGION=region),
                long_field_names=True,do_compression=True)
        back=loadmat(target,simplify_cells=True)
        for key in ['stable_mask','reason_mask','physical_unstable_mask','numerical_unresolved_mask','initial_points']:
            assert np.array_equal(back['PAPER_STABILITY3D']['result'][key],r[key]),key
        phi=np.asarray(g['phi']);vc=np.asarray(g['vc']);yaw=np.asarray(g['r'])
        stable=np.asarray(r['stable_points']).reshape(-1,3)
        case=dict(angle_deg=angle,file=filename,file_sha256=sha(target),
            classification_source_sha256=sha(path),initial_states=int(mask.size),stable_count=int(mask.sum()),
            physical_unstable_count=int(r['physical_unstable_count']),unresolved_count=int(r['unresolved_count']),
            raw_stable_bounds=np.column_stack([stable.min(0),stable.max(0)]).tolist() if len(stable) else None,
            origin_raw_stable=origin,origin_inside_extreme_candidate=origin_inside,
            accepted_inner=bool(region['valid']),accepted_volume=float(region['volume']),
            candidate_volume=float(region['candidate_volume']),candidate_vertices=region['candidate_vertices'],
            candidate_faces=region['faces'],candidate_extreme_vertices=int(region.get('extreme_vertices',0)),
            candidate_planar_face_count=int(region.get('planar_face_count',0)),
            candidate_physical_unstable_inside=physical_inside,candidate_numerical_unresolved_inside=unknown_inside,
            unavailable_reason=region.get('unavailable_reason',''),
            roll_domain_censored_slices=geometry['roll_domain_censored_slices'],
            stable_vc_yaw_slices=geometry['stable_vc_yaw_slices'],
            physical_roll_boundary_resolved=geometry['physical_roll_boundary_resolved'],
            branch_counts=geometry['branch_counts'],disconnected_roll_slices=geometry['disconnected_roll_slices'])
        cases.append(case);print(json.dumps(clean(case)),flush=True)
    summary=dict(model='Original MF with printed Eq11 and paper Sec3.1 extreme-point sequence',
        speed_kmh=40,mu=.85,angles_deg=[0,3.5,7],completed=len(cases),total=3,
        all_simulations_complete=len(cases)==3,accepted_inner_count=sum(c['accepted_inner'] for c in cases),
        forced_nesting=False,roll_scaling_or_caps=False,max_volume_objective=False,equilibrium_anchor=False,
        author_exact_corner_algorithm_claimed=False,continuous_stability_certificate=False,cases=cases)
    (args.output_dir/'region_summary.json').write_text(json.dumps(clean(summary),indent=2,allow_nan=False)+'\n')

if __name__=='__main__':main()
