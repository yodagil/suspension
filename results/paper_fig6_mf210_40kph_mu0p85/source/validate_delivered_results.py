#!/usr/bin/env python3
"""Recheck the three delivered MATs without altering masks or geometry."""
from pathlib import Path
import hashlib, json
import numpy as np
from scipy.io import loadmat
from scipy.spatial import ConvexHull

root=Path(__file__).resolve().parents[1]
report={'passed':True,'nesting_constraints_used':False,'cases':[],
        'containment':{},'raw_mask_inclusion':{}}
cases={}
for angle in (0,3.5,7):
    path=root/f'stable_point_40kph_0.85mu_{angle:g}deg.mat'
    loaded=loadmat(path,simplify_cells=True)
    scenario=loaded['PAPER_STABILITY3D'];raw=scenario['result']
    region=loaded['REGION210']['region'];v=np.asarray(region['inner_vertices'])
    mask=np.asarray(raw['stable_mask'],dtype=bool).ravel(order='F')
    points=np.asarray(raw['initial_points']);h=ConvexHull(v)
    unstable=points[~mask]
    inside=np.all(unstable@h.equations[:,:3].T+h.equations[:,3]<=1e-10,axis=1)
    actual=all(np.any(np.all(points[mask]==p,axis=1)) for p in v)
    valid=bool(region['is_valid_inner']) and actual and not inside.any()
    cfg=scenario['cfg']
    assert cfg['speed_kmh']==40 and cfg['speed_mode']=='fixed' and cfg['mu']==.85
    assert np.array_equal(points[mask],raw['stable_points'])
    assert len(points)==138105 and mask.sum()==raw['stable_count']
    assert scenario['meta']['nesting_constraints_used']==0
    report['cases'].append({'angle_deg':angle,'sha256':hashlib.sha256(path.read_bytes()).hexdigest(),
        'stable_count':int(mask.sum()),'zero_unstable_grid_nodes_inside':not bool(inside.any()),
        'all_vertices_actual_stable_grid_points':actual,'volume':float(h.volume),
        'valid':valid,'numerical_failures':int(raw['nonfinite_failure_count'])})
    report['passed'] &= valid and int(raw['nonfinite_failure_count'])==0
    cases[angle]=(points,mask,v,h.equations)
for child,parent in ((3.5,0),(7,3.5),(7,0)):
    cp,cm,cv,_=cases[child];pp,pm,_,eq=cases[parent]
    assert np.array_equal(cp,pp)
    key=f'{child:g}deg_in_{parent:g}deg'
    residual=cv@eq[:,:3].T+eq[:,3]
    report['containment'][key]={'contained':bool(np.max(residual)<=1e-10),
        'max_halfspace_residual':float(np.max(residual))}
    outside=cm&~pm
    report['raw_mask_inclusion'][key]={'included':not bool(outside.any()),
        'nonincluded_stable_states':int(outside.sum())}
report['naturally_nested']=all(x['contained'] for x in report['containment'].values())
report['raw_masks_nested']=all(x['included'] for x in report['raw_mask_inclusion'].values())
(root/'delivery_validation.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
assert report['passed']
