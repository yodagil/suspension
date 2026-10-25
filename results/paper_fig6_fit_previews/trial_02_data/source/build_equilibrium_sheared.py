#!/usr/bin/env python3
"""Equilibrium-centered, sheared eight-vertex sampled inner regions.

All candidate vertices are grid samples. A single fixed candidate family
and maximum-volume objective are used for every steering angle. The shear
models correlations of vc/r/phi without scaling any simulation coordinate.
The method is an explicit inner-reconstruction assumption, not the paper's
unpublished numerical vertex algorithm or a continuous stability proof.
"""
from pathlib import Path
import argparse, hashlib, json, os
os.environ.setdefault('OPENBLAS_NUM_THREADS','1')
import numpy as np
from scipy.io import loadmat
from scipy.spatial import ConvexHull
from build_equilibrium_inner import centered_inner


def runs(mask):
    d=np.diff(np.r_[False,mask,False].astype(int))
    return [(int(a),int(b-1)) for a,b in zip(np.flatnonzero(d==1),np.flatnonzero(d==-1)) if b-a>=3]


def build(cfg,result,equilibrium):
    axes=[np.asarray(cfg['grid'][k],float).ravel() for k in ['vc','r','phi']]
    mask=np.asarray(result['stable_mask'],bool);nv,nr,nphi=mask.shape
    eq=np.asarray(equilibrium,float)
    delta=np.array([np.diff(a)[0] for a in axes])
    q=np.array([int(np.argmin(abs(a-v))) for a,v in zip(axes,eq)])
    center=np.array([a[i] for a,i in zip(axes,q)])
    initial=centered_inner(cfg,result,eq)
    best=initial if initial['valid'] else None
    best_volume=best['volume'] if best else 0.
    points=np.asarray(result['initial_points'],float)
    stable=mask.ravel(order='F');bad=points[~stable]
    opposite_phi=2*q[2]-np.arange(nphi)
    valid_phi=(opposite_phi>=0)&(opposite_phi<nphi)
    pair_cache={};checked=0;improvements=0

    def pair_intervals(c0,c1):
        key=tuple(c0)+tuple(c1)
        if key not in pair_cache:
            m=np.zeros(nphi,bool)
            m[valid_phi]=mask[c0[0],c0[1],valid_phi]&mask[c1[0],c1[1],opposite_phi[valid_phi]]
            pair_cache[key]=runs(m)
        return pair_cache[key]

    for av in range(1,min(q[0],nv-1-q[0])+1):
        for ar in range(-(nr-1),nr):
            max_br=min(q[1]-abs(ar),nr-1-q[1]-abs(ar))
            if max_br<1:continue
            for br in range(1,max_br+1):
                columns=np.array([[q[0]-av,q[1]-ar-br],
                                  [q[0]+av,q[1]+ar-br],
                                  [q[0]+av,q[1]+ar+br],
                                  [q[0]-av,q[1]-ar+br]],int)
                if any(not mask[v,r,:].any() for v,r in columns):continue
                p0=pair_intervals(columns[0],columns[2])
                p1=pair_intervals(columns[1],columns[3])
                for l0,h0 in p0:
                    for l1,h1 in p1:
                        cp_max=min((h0-l0)//2,(h1-l1)//2)
                        for cp in range(cp_max,0,-1):
                            volume=float(8*av*br*cp*np.prod(delta))
                            if volume<=best_volume+1e-12:break
                            for pc0 in range(l0+cp,h0-cp+1):
                                for pc1 in range(l1+cp,h1-cp+1):
                                    t0=int(pc0-q[2]);t1=int(pc1-q[2])
                                    if (t1-t0)%2:continue
                                    ap=(t1-t0)//2;bp=-(t0+t1)//2
                                    A=np.column_stack([np.array([av,ar,ap])*delta,
                                                       np.array([0,br,bp])*delta,
                                                       np.array([0,0,cp])*delta])
                                    inv=np.linalg.inv(A)
                                    if np.any(np.abs((eq-center)@inv.T)>=1-1e-9):continue
                                    pc=np.array([pc0,pc1,2*q[2]-pc0,2*q[2]-pc1])
                                    ids0=np.column_stack([columns,pc-cp])
                                    ids1=np.column_stack([columns,pc+cp])
                                    ix=np.vstack([ids0,ids1])
                                    if np.any(ix<0) or np.any(ix>=np.array(mask.shape)):continue
                                    ids=np.ravel_multi_index(ix.T,mask.shape,order='F')
                                    if not np.all(stable[ids]):continue
                                    vertices=points[ids]
                                    lo=vertices.min(0);hi=vertices.max(0)
                                    bb=np.all((bad>=lo-1e-10)&(bad<=hi+1e-10),axis=1)
                                    checked+=1
                                    if np.any(np.all(np.abs((bad[bb]-center)@inv.T)<=1+1e-9,axis=1)):continue
                                    h=ConvexHull(vertices)
                                    assert len(h.vertices)==8 and abs(h.volume-volume)<1e-9
                                    improvements+=1;best_volume=volume
                                    best=dict(valid=True,volume=volume,vertices=vertices.tolist(),
                                        faces=(h.simplices+1).tolist(),
                                        vertex_grid_indices_1based=(ids+1).tolist(),
                                        grid_unstable_inside=0,center=center.tolist(),
                                        equilibrium=eq.tolist(),center_error=(center-eq).tolist(),
                                        axis_spans=np.ptp(vertices,axis=0).tolist(),
                                        vertex_min=lo.tolist(),vertex_max=hi.tolist(),
                                        edge_vectors=A.T.tolist(),
                                        index_half_edges=[[av,ar,ap],[0,br,bp],[0,0,cp]],
                                        method='maximum-volume equilibrium-centered sheared grid parallelepiped',
                                        exact_equilibrium_strictly_inside=True,
                                        center_tolerance_in_grid_steps=.5,
                                        objective='largest volume among common centered grid candidates',
                                        angle_dependent_shrink_factors=False,sampled_grid_only=True,
                                        paper_exact_vertex_rule=False,continuous_stability_proof=False)
    if best is None:return dict(valid=False,volume=None)
    best['candidate_checks']=checked;best['sheared_improvements']=improvements
    best['reconstruction_assumption']='Conservative neighborhood of steady state; not entire initial-condition basin'
    # Independent inclusive ConvexHull verification in normalized coordinates.
    v=np.asarray(best['vertices']);scale=np.array([np.ptp(a) for a in axes]);h=ConvexHull(v/scale)
    bad_in=np.all((bad>=v.min(0)-1e-10)&(bad<=v.max(0)+1e-10),axis=1)
    assert not np.any(np.all((bad[bad_in]/scale)@h.equations[:,:3].T+h.equations[:,3]<=1e-10,axis=1))
    return best


def main():
    ap=argparse.ArgumentParser();ap.add_argument('source',type=Path)
    ap.add_argument('equilibria',type=Path);ap.add_argument('output',type=Path);a=ap.parse_args()
    es=loadmat(a.equilibria,simplify_cells=True)['equilibrium_cases'];cases=[]
    for angle,e in zip([0,3.5,7],es):
        assert e['physical_constraints_valid'] and e['locally_asymptotically_stable'] and e['max_abs_residual']<1e-9
        f=a.source/f'classification_{angle:g}deg.mat'
        if not f.exists():f=a.source/f'screen_{angle:g}deg.mat'
        raw=loadmat(f,simplify_cells=True);s=raw.get('PAPER_STABILITY3D',raw.get('PAPER_SCREEN'))
        r=build(s['cfg'],s['result'],np.asarray(e['state_u_v_r_phi_p'])[[1,2,3]])
        r.update(angle_deg=angle,source_mat_filename=f.name,
                 source_mat_sha256=hashlib.sha256(f.read_bytes()).hexdigest(),
                 stable_count=int(s['result']['stable_count']),total_count=int(s['result']['total_count']),
                 raw_stable_roll_bounds=[float(np.min(s['result']['stable_points'][:,2])),
                                         float(np.max(s['result']['stable_points'][:,2]))])
        cases.append(r);a.output.parent.mkdir(parents=True,exist_ok=True)
        a.output.write_text(json.dumps(dict(cases=cases,complete=len(cases)==3,
           reconstruction='Explicit equilibrium-centered sampled inner approximation; not author vertex rule'),indent=2)+'\n')
        print(angle,r['valid'],r.get('volume'),r.get('axis_spans'),r.get('index_half_edges'),flush=True)

if __name__=='__main__':main()
