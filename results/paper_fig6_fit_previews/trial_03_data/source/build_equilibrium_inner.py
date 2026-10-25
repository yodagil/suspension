#!/usr/bin/env python3
"""Largest sampled all-stable box centered near a verified steady state.

This is an explicit conservative reconstruction assumption, not the authors'
unpublished Fig.4/5 vertex-selection code. Center error is at most half one
original grid step per axis. The exact equilibrium must lie strictly inside.
All vertices are actual stable grid nodes; every sampled node of the box is
stable. Thus this describes a neighborhood of the operating equilibrium,
not the entire basin of stable initial conditions.
"""
from pathlib import Path
import argparse, hashlib, json
import numpy as np
from scipy.io import loadmat
from scipy.spatial import ConvexHull


def centered_inner(cfg, result, equilibrium, max_normalized_vc_to_r=None, roll_span_power=0):
    axes=[np.asarray(cfg['grid'][k],float).ravel() for k in ['vc','r','phi']]
    eq=np.asarray(equilibrium,float)
    mask=np.asarray(result['stable_mask'],bool)
    pairs=[]
    for values, target in zip(axes,eq):
        delta=float(np.min(np.diff(values)))
        pp=[]
        for lo in range(len(values)-1):
            for hi in range(lo+1,len(values)):
                # A tracking neighborhood must contain the exact steady state.
                if not values[lo] < target < values[hi]:
                    continue
                center=(values[lo]+values[hi])/2
                if abs(center-target)<=.5*delta+1e-10:
                    pp.append((lo,hi,values[hi]-values[lo]))
        pairs.append(pp)
    best=0.;best_score=0.;bounds=None;valid_candidates=0
    for pl,ph,dp in pairs[2]:
        sheet=np.all(mask[:,:,pl:ph+1],axis=2)
        for vl,vh,dv in pairs[0]:
            row=np.all(sheet[vl:vh+1,:],axis=0)
            for rl,rh,dr in pairs[1]:
                if max_normalized_vc_to_r is not None:
                    aspect=(dv / np.ptp(axes[0])) / (dr / np.ptp(axes[1]))
                    if aspect > max_normalized_vc_to_r + 1e-12:
                        continue
                vol=float(dv*dr*dp)
                score=vol*(dp/np.ptp(axes[2]))**roll_span_power
                if score<=best_score+1e-12:
                    continue
                if np.all(row[rl:rh+1]):
                    best=vol;best_score=score;bounds=((vl,vh),(rl,rh),(pl,ph));valid_candidates+=1
    if bounds is None:
        return dict(valid=False,volume=None,
                    status='No centered three-dimensional sampled inner box found; not zero stability')
    ix=np.array([[v,r,p] for p in bounds[2] for v,r in
                 [(bounds[0][0],bounds[1][0]),(bounds[0][1],bounds[1][0]),
                  (bounds[0][1],bounds[1][1]),(bounds[0][0],bounds[1][1])]],int)
    v=np.column_stack([axes[j][ix[:,j]] for j in range(3)])
    ids=np.ravel_multi_index(ix.T,mask.shape,order='F')
    assert mask.ravel(order='F')[ids].all()
    lo=v.min(0);hi=v.max(0)
    points=np.asarray(result['initial_points'],float)
    inside=np.all((points>=lo-1e-10)&(points<=hi+1e-10),axis=1)
    bad=~mask.ravel(order='F')
    assert np.count_nonzero(inside&bad)==0
    assert np.all(lo<eq)&np.all(eq<hi)
    h=ConvexHull(v)
    assert len(h.vertices)==8 and abs(h.volume-best)<1e-10
    return dict(valid=True,volume=best,selection_score=best_score,roll_span_power=roll_span_power,vertices=v.tolist(),faces=(h.simplices+1).tolist(),
                vertex_grid_indices_1based=(ids+1).tolist(),grid_unstable_inside=0,
                sampled_nodes_inside=int(inside.sum()),center=v.mean(0).tolist(),
                equilibrium=eq.tolist(),center_error=(v.mean(0)-eq).tolist(),
                axis_spans=np.ptp(v,axis=0).tolist(),vertex_min=lo.tolist(),vertex_max=hi.tolist(),
                method='maximum-volume all-stable grid box centered within half a grid step of the verified steady state',
                exact_equilibrium_strictly_inside=True,
                center_tolerance_in_grid_steps=.5,
                objective='physical volume times normalized roll span to a common power, within the same candidate family',
                angle_dependent_shrink_factors=False,
                sampled_grid_only=True,paper_exact_vertex_rule=False,max_normalized_vc_to_r=max_normalized_vc_to_r,
                continuous_stability_proof=False,valid_volume_improvements=valid_candidates)


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('source',type=Path)
    ap.add_argument('equilibria',type=Path)
    ap.add_argument('output',type=Path)
    a=ap.parse_args()
    eqs=loadmat(a.equilibria,simplify_cells=True)['equilibrium_cases']
    cases=[]
    for angle,e in zip([0,3.5,7],eqs):
        assert e['locally_asymptotically_stable'] and e['physical_constraints_valid']
        assert e['max_abs_residual']<1e-9
        p=a.source/f'classification_{angle:g}deg.mat'
        if not p.exists():p=a.source/f'screen_{angle:g}deg.mat'
        raw=loadmat(p,simplify_cells=True)
        s=raw.get('PAPER_STABILITY3D',raw.get('PAPER_SCREEN'))
        r=centered_inner(s['cfg'],s['result'],np.asarray(e['state_u_v_r_phi_p'])[[1,2,3]])
        r.update(angle_deg=angle,stable_count=int(s['result']['stable_count']),
                 total_count=int(s['result']['total_count']),
                 source_mat_sha256=hashlib.sha256(p.read_bytes()).hexdigest(),
                 source_mat_filename=p.name,
                 raw_stable_roll_bounds=[float(np.min(s['result']['stable_points'][:,2])),
                                         float(np.max(s['result']['stable_points'][:,2]))])
        cases.append(r)
        print(angle,r['valid'],r.get('volume'),r.get('axis_spans'),flush=True)
    a.output.parent.mkdir(parents=True,exist_ok=True)
    a.output.write_text(json.dumps(dict(cases=cases,
        reconstruction='Conservative equilibrium-centered sampled inner approximation; not author vertex rule',
        equilibrium_source_sha256=hashlib.sha256(a.equilibria.read_bytes()).hexdigest()),indent=2)+'\n')


if __name__=='__main__':main()
