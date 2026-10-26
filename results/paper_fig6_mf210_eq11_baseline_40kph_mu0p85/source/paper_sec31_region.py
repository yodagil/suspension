#!/usr/bin/env python3
"""Declared discrete completion of Xiao2022 Sec3.1, without rank/volume fitting.

The paper publishes an extreme-point sequence, but not executable branch or
eight-corner tie rules. This module follows that sequence and records those
remaining numerical choices. Invalid candidates are retained as diagnostics,
never replaced by a cuboid, an inward rank, or a cross-angle intersection.
"""
from __future__ import annotations
from pathlib import Path
import argparse,json
import numpy as np
from scipy.spatial import ConvexHull,QhullError
from scipy.io import loadmat,savemat

TOL=1e-10
AXES=('vc','r','phi')

def coords(indices,g):
    a=np.asarray(indices,int).reshape(-1,3)
    return np.column_stack([g[k][a[:,i]] for i,k in enumerate(AXES)]) if len(a) else np.empty((0,3))

def lexrows(a):
    return np.unique(np.asarray(a).reshape(-1,3),axis=0) if len(a) else np.empty((0,3),int)

def canonical_polygon(points):
    """Convex hull CCW, starting at lexicographically smallest (vc,phi)."""
    h=ConvexHull(points);q=np.asarray(points)[h.vertices]
    area=np.sum(q[:,0]*np.roll(q[:,1],-1)-q[:,1]*np.roll(q[:,0],-1))
    if area<0:q=q[::-1]
    i=np.lexsort((q[:,1],q[:,0]))[0]
    return np.roll(q,-int(i),axis=0)

def section_four(points):
    q=np.unique(np.asarray(points,float).reshape(-1,2),axis=0)
    if len(q)<4 or np.linalg.matrix_rank(q-q.mean(0),tol=1e-11)<2:return None,'fewer_than_four_nondegenerate_points'
    try:h=canonical_polygon(q)
    except QhullError:return None,'degenerate_section'
    if len(h)==4:return h,'exact_four_vertex_section_hull'
    # Four cardinal extrema, with physical-coordinate midpoint tie rule.
    # No geometric translation or centering is applied to returned points.
    selected=[]
    for axis,sign in [(0,-1),(1,1),(0,1),(1,-1)]:
        value=(q[:,axis]*sign).max();hits=q[np.abs(q[:,axis]*sign-value)<=1e-12]
        other=1-axis;mid=(q[:,other].min()+q[:,other].max())/2
        order=np.lexsort((hits[:,1],hits[:,0],np.abs(hits[:,other]-mid)))
        selected.append(hits[order[0]])
    selected=np.unique(np.array(selected),axis=0)
    if len(selected)!=4:return None,'cardinal_extrema_not_four_distinct_points'
    try:h=canonical_polygon(selected)
    except QhullError:return None,'cardinal_extrema_degenerate'
    if len(h)!=4:return None,'cardinal_extrema_not_convex_four_vertices'
    return h,'four_cardinal_vc_phi_extrema'

def canonical_cycle(indices):
    q=list(map(int,indices));i=q.index(min(q));q=q[i:]+q[:i]
    if q[1]>q[-1]:q=[q[0]]+q[:0:-1]
    return q

def hull_audit(vertices,bad):
    v=np.asarray(vertices,float).reshape(-1,3)
    out=dict(valid=False,vertices=v,faces=np.empty((0,4),int),volume=np.nan,
             extreme_vertices=0,planar_face_count=0,unstable_nodes_inside_inclusive=-1,
             excluded_not_stable_nodes_inside_inclusive=-1,
             excluded_node_definition='Every node not classified stable; may include physically unstable and numerically unresolved nodes')
    if len(v)!=8 or len(np.unique(v,axis=0))!=8:return out
    try:h=ConvexHull(v)
    except QhullError:return out
    planes=[]
    for eq in h.equations:
        if not any(np.max(abs(eq-p))<1e-9 for p in planes):planes.append(eq)
    out.update(volume=float(h.volume),extreme_vertices=len(h.vertices),planar_face_count=len(planes),
               hull_triangles_1based=h.simplices+1)
    bad=np.asarray(bad,float).reshape(-1,3)
    count=int(np.count_nonzero(np.all(bad@h.equations[:,:3].T+h.equations[:,3]<=TOL,axis=1))) if len(bad) else 0
    out['unstable_nodes_inside_inclusive']=count
    out['excluded_not_stable_nodes_inside_inclusive']=count
    faces=[]
    for eq in planes:
        ids=np.flatnonzero(abs(v@eq[:3]+eq[3])<1e-9)
        if len(ids)!=4:continue
        pts=v[ids];center=pts.mean(0);u=pts[0]-center;u/=np.linalg.norm(u);w=np.cross(eq[:3],u)
        ang=np.arctan2((pts-center)@w,(pts-center)@u)
        faces.append(canonical_cycle(ids[np.argsort(ang)]+1))
    faces=sorted(faces)
    if len(faces)==6:out['faces']=np.array(faces,int)
    out['valid']=bool(len(h.vertices)==8 and len(planes)==6 and len(faces)==6 and count==0 and h.volume>1e-14)
    return out

def eight_from_yaw_sections(indices,g,bad):
    indices=lexrows(indices)
    sections=[]
    for r in np.unique(indices[:,1]) if len(indices) else []:
        a=indices[indices[:,1]==r];p=np.column_stack([g['vc'][a[:,0]],g['phi'][a[:,2]]])
        four,rule=section_four(p)
        if four is not None:sections.append((int(r),four,rule))
    empty=dict(valid=False,vertices=np.empty((0,3)),faces=np.empty((0,4),int),volume=np.nan,
               unavailable_reason='Fewer than two yaw sections admit four distinct convex extrema')
    if len(sections)<2:return empty
    lo,hi=sections[0],sections[-1]
    v=np.vstack([np.column_stack([s[1][:,0],np.full(4,g['r'][s[0]]),s[1][:,1]]) for s in [lo,hi]])
    out=hull_audit(v,bad)
    out.update(yaw_section_indices_zero_based=[lo[0],hi[0]],
               yaw_section_selection='First and last yaw sections admitting four nondegenerate extrema; no inward search',
               cap_selection_rules=[lo[2],hi[2]])
    if not out['valid']:out['unavailable_reason']='Extreme-point candidate fails eight-vertex/six-quad/zero-unstable audit'
    return out

def paper_sec31_region(stable_mask,grid):
    m=np.asarray(stable_mask,bool);g={k:np.asarray(grid[k],float).ravel() for k in AXES}
    assert m.shape==tuple(len(g[k]) for k in AXES)
    assert all(np.all(np.isfinite(g[k])) and np.all(np.diff(g[k])>0) for k in AXES)
    lower=[];upper=[];censored=disconnected=0
    for v in range(m.shape[0]):
        for r in range(m.shape[1]):
            ps=np.flatnonzero(m[v,r])
            if not len(ps):continue
            lower.append((v,r,int(ps[0])));upper.append((v,r,int(ps[-1])))
            censored+=int(ps[0]==0 or ps[-1]==m.shape[2]-1)
            disconnected+=int(np.any(np.diff(ps)>1))
    limit=lexrows(lower+upper);outer=[];inner=[]
    counts=dict(one_band_coincident=0,two_bands_central_pair=0,multiple_bands_ambiguous_excluded=0,
                central_interval_contains_unstable_excluded=0)
    rows=[]
    for r,p in sorted(set(map(tuple,limit[:,[1,2]]))):
        vs=np.sort(limit[(limit[:,1]==r)&(limit[:,2]==p),0]);cuts=np.flatnonzero(np.diff(vs)>1)+1
        bands=np.split(vs,cuts);outer.extend([(vs[0],r,p),(vs[-1],r,p)])
        row=dict(r_index=int(r),phi_index=int(p),band_count=len(bands))
        if len(bands)==1:
            pair=[int(vs[0]),int(vs[-1])];counts['one_band_coincident']+=1
            row['status']='one_band_inner_outer_coincident_at_grid_resolution'
        elif len(bands)==2:
            pair=[int(bands[0][-1]),int(bands[1][0])]
            if not np.all(m[pair[0]:pair[1]+1,r,p]):
                pair=[];counts['central_interval_contains_unstable_excluded']+=1
                row['status']='central_gap_not_all_stable_excluded'
            else:counts['two_bands_central_pair']+=1;row['status']='facing_endpoints_of_two_branches'
        else:
            pair=[];counts['multiple_bands_ambiguous_excluded']+=1
            row['status']='multiple_boundary_branches_ambiguous_excluded'
        row['inner_vc_indices']=pair
        inner.extend([(v,r,p) for v in pair]);rows.append(row)
    outer=lexrows(outer);inner=lexrows(inner);bad=coords(np.argwhere(~m),g)
    a=eight_from_yaw_sections(inner,g,bad);b=eight_from_yaw_sections(outer,g,bad)
    return dict(method='Paper Sec3.1 extreme-point sequence with declared discrete branch/corner completion',
        author_exact_algorithm_claimed=False,max_volume_objective_used=False,equilibrium_anchor_used=False,
        excluded_node_definition='Every node not classified stable; may include physically unstable and numerically unresolved nodes',
        nesting_constraints_used=False,roll_scaling_or_caps_applied=False,
        limit_roll_points=coords(limit,g),lower_roll_points=coords(lower,g),upper_roll_points=coords(upper,g),
        outer_limit_points=coords(outer,g),inner_limit_points=coords(inner,g),
        lower_roll_indices_zero_based=np.asarray(lower,int).reshape(-1,3),
        upper_roll_indices_zero_based=np.asarray(upper,int).reshape(-1,3),
        limit_roll_indices_zero_based=limit,outer_limit_indices_zero_based=outer,inner_limit_indices_zero_based=inner,
        slice_diagnostics=rows,branch_counts=counts,
        stable_vc_yaw_slices=len(lower),roll_domain_censored_slices=censored,
        disconnected_roll_slices=disconnected,physical_roll_boundary_resolved=(len(lower)>0 and censored==0),
        inner_candidate=a,outer_candidate=b,is_valid_inner=a['valid'],
        continuous_stability_certificate=False)

def serializable(x):
    if isinstance(x,dict):return {k:serializable(v) for k,v in x.items()}
    if isinstance(x,list):return [serializable(v) for v in x]
    if isinstance(x,np.ndarray):return x.tolist()
    if isinstance(x,np.generic):return x.item()
    return x

def main():
    p=argparse.ArgumentParser();p.add_argument('classification',type=Path);p.add_argument('output',type=Path);a=p.parse_args()
    s=loadmat(a.classification,simplify_cells=True)['PAPER_STABILITY3D'];r=paper_sec31_region(s['result']['stable_mask'],s['cfg']['grid'])
    a.output.parent.mkdir(parents=True,exist_ok=True)
    a.output.with_suffix('.json').write_text(json.dumps(serializable(r),indent=2)+'\n')
    savemat(a.output.with_suffix('.mat'),{'SEC31_REGION':r},do_compression=True,long_field_names=True)
    print(json.dumps({k:serializable(r[k]) for k in ['is_valid_inner','branch_counts','roll_domain_censored_slices']}))

if __name__=='__main__':main()
