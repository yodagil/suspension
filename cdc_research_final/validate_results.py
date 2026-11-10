#!/usr/bin/env python3
"""Validate actual H-preserving research results, including stored cell tests."""
import hashlib
import json
from pathlib import Path
import numpy as np
from scipy.io import loadmat

WEARS=[[0,0,0,0],[25,25,0,0],[50,50,25,25],[0,0,25,25],[25,25,50,50]]
SOURCE_NAMES=['cdc_research_generate.m','cdc_research_simulate.m','cdc_fullcar_model.m',
 'cdc_handling_reference.m','cdc_dynamic_slope.m','cdc_research_codes.m',
 'cdc_run_points.m','cdc_research_fields.m','cdc_research_native.m']

def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def need(ok,message):
    if not ok:raise ValueError(message)
def finite(x):return float(x) if np.isfinite(x) else None
def cases_of(value):return [value] if isinstance(value,dict) else list(value)

def raw_check(raw,n,usable):
    keys=['safe','model_safe','yaw_ok','model_recovered','recoverable','guarded_recoverable',
          'handling_preserved_recoverable','guarded_handling_preserved_recoverable',
          'prepeak_handling_preserved_recoverable','stayed_in_handling']
    a={k:np.asarray(raw[k]).reshape(n,2) for k in keys}
    for k,x in a.items():need(np.isin(x,[0,1]).all(),'Invalid boolean '+k)
    reason=np.asarray(raw['reason']).reshape(n,2)
    need(np.isfinite(reason).all() and np.all((reason>=0)&(reason<=255)&(reason==np.floor(reason))),'Invalid reason bits')
    reason=reason.astype(np.uint16)
    need(np.array_equal(a['model_safe'],(reason&252)==0),'Hard-domain safety mismatch')
    need(np.array_equal(a['safe'],(a['model_safe']!=0)&((reason&2)==0)),'Physical policy safety mismatch')
    need(np.array_equal(a['yaw_ok'],(reason&1)==0),'Yaw policy mismatch')
    need(np.all(a['model_recovered']<=a['model_safe']),'Recovery after hard failure')
    need(np.array_equal(a['recoverable'],a['safe']*a['model_recovered']),'Safe recovery mismatch')
    need(np.array_equal(a['guarded_recoverable'],a['recoverable']*a['yaw_ok']),'Guarded recovery mismatch')
    need(np.array_equal(a['handling_preserved_recoverable'],a['recoverable']*a['stayed_in_handling']),'H preservation mismatch')
    need(np.array_equal(a['guarded_handling_preserved_recoverable'],a['handling_preserved_recoverable']*a['yaw_ok']),'Guarded final mismatch')
    need(np.all(a['prepeak_handling_preserved_recoverable']<=a['handling_preserved_recoverable']),'Prepeak not subset')
    for key in ['safe','model_safe','yaw_ok','stayed_in_handling']:
        need(np.all(a[key][:,1]<=a[key][:,0]),'Cumulative path property grows: '+key)
    need(np.all((reason[:,0]&reason[:,1])==reason[:,0]),'Cumulative reason decreases')
    rec=a['model_recovered'].astype(bool)
    err=np.asarray(raw['terminal_error']).reshape(n,2);der=np.asarray(raw['terminal_derivative']).reshape(n,2)
    need(np.all(err[rec]<=1+1e-9) and np.all(der[rec]<=1+1e-9),'Recovery tolerance failure')
    if not usable:need(not rec.any(),'Recovery to unchecked trim')
    first=np.asarray(raw['first_handling_exit']).reshape(n,2)
    need(np.array_equal(np.isinf(first),a['stayed_in_handling']!=0),'H exit diagnostic mismatch')
    observed=np.asarray(raw['observed_until']).reshape(n,2)
    need(np.isfinite(observed).all() and np.all(observed>=0) and np.all(observed<=np.array([5,10])+1e-9),'Invalid observed duration')
    need(np.array_equal(observed[a['model_safe'].astype(bool)],np.broadcast_to([5,10],(n,2))[a['model_safe'].astype(bool)]),'Safe path not fully observed')
    return a

def validate_file(path,expected_sha_dict=None,expected_wears=WEARS):
    path=Path(path);data=loadmat(path,simplify_cells=True);need('RESEARCH2D' in data,'Wrong MAT variable')
    S=data['RESEARCH2D'];m=S['meta'];c=S['cfg'];cases=cases_of(S['CASE'])
    need(m['version']=='CDC_RESEARCH_FINAL_1' and m['engine']=='native','Wrong research version/engine')
    need(m['initialization']=='trim/trim' and not m['feedback_enabled'],'Wrong hidden-state/controller configuration')
    need(np.array_equal(m['horizons_s'],[5,10]) and m['dt_s']==.0005,'Timing changed')
    need(np.array_equal(m['currents_mA'],[950]*4) and np.array_equal(c['k_spring_N_m'],[30790]*4),'Suspension command changed')
    need(not m['certified_invariant'],'Unsubstantiated invariant claim')
    need(len(cases)==len(expected_wears),'Incomplete tire-wear cases')
    if expected_sha_dict:
        mapping={name.replace('.','_'):name for name in SOURCE_NAMES}
        mapping.update(native_driver='native/cdc_research_native.cpp',native_physics='native/cdc_physics.hpp')
        for key,name in mapping.items():
            need(hashlib.sha256(S['source'][key].encode()).hexdigest()==expected_sha_dict[name],'Embedded source mismatch: '+name)
    v=np.asarray(c['grid']['vc']);r=np.asarray(c['grid']['r']);shape=(61,121)
    need(v.shape==(61,) and r.shape==(121,) and np.allclose(v,np.linspace(-3,3,61),rtol=0,atol=1e-13) and np.allclose(r,np.linspace(-1.5,1.5,121),rtol=0,atol=1e-13),'Unexpected grid')
    V,R=np.meshgrid(v,r,indexing='ij');points=np.column_stack([V.ravel(order='F'),R.ravel(order='F')])
    summaries=[]
    for i,(C,wear) in enumerate(zip(cases,expected_wears),1):
        need(np.array_equal(C['wear'],wear),'Tire wear order mismatch')
        M=C['masks'];ref=np.asarray(M['reference']).astype(bool);need(ref.shape==shape,'H mask dimensions')
        need(np.array_equal(ref,np.asarray(C['handling']['mask']).astype(bool)),'Reference mismatch')
        ids=np.flatnonzero(ref.ravel(order='F'));raw=C['raw'];n=len(ids);need(n>0,'Empty H')
        need(np.array_equal(np.asarray(raw['initial_grid_linear_index_1based']).reshape(-1),ids+1),'H grid indexing mismatch')
        need(np.array_equal(np.asarray(raw['initial_points']).reshape(n,2),points[ids]),'H initial coordinates mismatch')
        T=C['trim'];need(T['root_found'],'Missing checked root')
        a=raw_check(raw,n,bool(T['usable']))
        mapping={'recoverable_on_reference':'recoverable','guarded_recoverable_on_reference':'guarded_recoverable',
          'final':'handling_preserved_recoverable','guarded_final':'guarded_handling_preserved_recoverable','prepeak_final':'prepeak_handling_preserved_recoverable'}
        for key,field in mapping.items():
            actual=np.asarray(M[key]).astype(bool);need(actual.shape==(61,121,2),'Mask dimensions '+key)
            expected=np.zeros((7381,2),bool);expected[ids]=a[field].astype(bool)
            if key in ['final','guarded_final','prepeak_final'] and not C['trim_in_reference']:expected[:]=False
            need(np.array_equal(actual,expected.reshape((61,121,2),order='F')),'Mask/raw mapping '+key)
        final=np.asarray(M['final'])[:,:,1].astype(bool)
        candidate=final[:-1,:-1]&final[1:,:-1]&final[:-1,1:]&final[1:,1:]
        reg=C['region'];cells=np.asarray(reg['cell_mask']).astype(bool)
        need(cells.shape==(60,120) and np.all(cells<=candidate),'Cell vertices did not pass')
        ci=np.flatnonzero(candidate.ravel(order='F'));iv,ir=np.unravel_index(ci,candidate.shape,order='F')
        cp=np.column_stack([(v[iv]+v[iv+1])/2,(r[ir]+r[ir+1])/2])
        need(int(reg['candidate_cells'])==len(ci) and int(reg['accepted_cells'])==int(cells.sum()),'Cell counts mismatch')
        need(np.array_equal(np.asarray(reg['center_points']).reshape(-1,2),cp),'Cell center coordinates mismatch')
        passed=np.asarray(reg['center_pass']).reshape(-1).astype(bool)
        if len(ci):
            ca=raw_check(reg['center_raw'],len(ci),bool(T['usable']))
            need(np.array_equal(passed,ca['handling_preserved_recoverable'][:,1].astype(bool)),'Cell center simulation failed')
        need(np.array_equal(cells.ravel(order='F')[ci],passed),'Cell acceptance mismatch')
        area=np.diff(v)[:,None]*np.diff(r)[None,:]
        need(np.isclose(float(reg['cell_area']),float(area[cells].sum()),rtol=1e-10,atol=1e-12),'Cell area mismatch')
        need(not reg['certified_continuous'],'Unsubstantiated continuous-region claim')
        summaries.append(dict(wear_case=i,wear=list(map(int,wear)),reference_count=n,
            recovered_counts=np.asarray(M['recoverable_on_reference']).sum(axis=(0,1)).astype(int).tolist(),
            final_counts=np.asarray(M['final']).sum(axis=(0,1)).astype(int).tolist(),
            guarded_counts=np.asarray(M['guarded_final']).sum(axis=(0,1)).astype(int).tolist(),
            prepeak_counts=np.asarray(M['prepeak_final']).sum(axis=(0,1)).astype(int).tolist(),
            sampled_area=float(reg['cell_area']),accepted_cells=int(cells.sum()),center_points=len(ci),
            trim_usable=bool(T['usable']),trim_in_reference=bool(C['trim_in_reference']),
            initial_roll_rad=finite(T['x'][3]),grid_truncated=bool(C['grid_truncated']),reference_grid_truncated=bool(C['reference_grid_truncated'])))
    return dict(all_checks_passed=True,filename=path.name,sha256=sha(path),bytes=path.stat().st_size,
        speed_kmh=float(m['speed_kmh']),mu_road=float(m['mu_road']),front_steer_deg=float(m['wheel_steer_deg'][0]),
        version=m['version'],case_summaries=summaries)

if __name__=='__main__':
    import argparse
    p=argparse.ArgumentParser();p.add_argument('file');a=p.parse_args();print(json.dumps(validate_file(a.file),indent=2))
