#!/usr/bin/env python3
"""Strict read-only validation of uploaded-model CDC 2-D production MATs.

validate_file(path, expected_sha_dict=None) returns a small JSON-safe summary.
A supplied SHA dictionary may use frozen filenames or metadata SHA field names.
The checker preserves legitimate empty regions and failed/unusable trim states.
"""
import argparse
from collections import Counter
import hashlib
import itertools
import json
from pathlib import Path
import re

import numpy as np
from scipy.io import loadmat

WEAR=np.array([[0,0,0,0],[25,25,0,0],[50,50,25,25],[0,0,25,25],[25,25,50,50]])
COMBINATIONS=list(itertools.product([50,60,70,80,90,100],[.9,.8,.7,.6,.5],[0,.5,1,1.5,2,2.5,3]))
VC=np.arange(-20,21)*.5
RR=np.arange(-15,16)*(.8/30)
ALIASES={
 'stability_shift_cdc_2d.m':'main_source_sha256',
 'cdc2d_native':'native_executable_sha256',
 'cdc2d_native_simulate.m':'native_bridge_sha256',
 'export_cdc_native_case.m':'native_exporter_sha256',
}


def assert_equal(actual,expected,description):
    assert np.array_equal(actual,expected,equal_nan=True),description


def json_safe(value):
    if isinstance(value,dict):return {key:json_safe(item) for key,item in value.items()}
    if isinstance(value,(list,tuple,np.ndarray)):return [json_safe(item) for item in value]
    if isinstance(value,(np.bool_,bool)):return bool(value)
    if isinstance(value,(np.integer,int)):return int(value)
    if isinstance(value,(np.floating,float)):return float(value) if np.isfinite(value) else None
    return value


def same(a,b):
    if isinstance(a,dict) or isinstance(b,dict):
        return isinstance(a,dict) and isinstance(b,dict) and set(a)==set(b) and all(same(a[k],b[k]) for k in a)
    try:return bool(np.array_equal(a,b,equal_nan=True))
    except TypeError:return bool(np.array_equal(a,b))


def validate_file(path, expected_sha_dict=None):
    path=Path(path)
    before=path.stat()
    found={};stats=Counter()
    data=loadmat(path,simplify_cells=True)
    assert 'STABILITY2D' in data,path
    S=data['STABILITY2D'];c=S['cfg'];meta=S['meta']
    combo=(int(c['speed_kmh']),round(float(c['mu']),1),round(float(c['steer_deg']),10))
    np.testing.assert_allclose(meta['wheel_steer_deg'],[combo[2],combo[2],0,0],rtol=0,atol=1e-12)
    assert combo in COMBINATIONS and combo not in found,(path,combo)
    assert meta['version']=='cdc-shift-2d-fixed-phi-v1'
    assert meta['fixed_initial_roll_rad']==0 and meta['roll_dynamics_retained']
    assert not meta['is_projection_of_3d'] and not meta['certified_invariant']
    assert list(meta['initial_axes'])==['vc_m_s','yaw_rate_rad_s']
    assert_equal(np.asarray(meta['horizons_s']),np.array([5,10]),'horizons')
    assert meta['dt_s']==.01/20
    assert c['fixed_phi0']==0 and c['Ts']==.01 and c['Nsub']==20
    assert c['initialization']=='trim' and bool(c['use_yaw_policy_for_region'])
    assert c['reject_force_clipping'] and c['check_cell_centers'] and c['random_points_per_cell']==0
    assert_equal(c['I_mA'],np.full(4,950),'CDC current')
    assert_equal(c['k_spring_N_m'],np.full(4,30790),'spring settings')
    G=S['grid_coarse'];assert set(G)=={'vc','r'}
    assert_equal(G['vc'],VC,'vc grid')
    assert_equal(G['r'],RR,'r grid')
    assert_equal(S['wear_cases'],WEAR,'wear order')
    assert len(S['CASE'])==5 and len(S['Summary'])==5
    v,r=np.meshgrid(VC,RR,indexing='ij')
    points=np.column_stack((v.ravel(order='F'),r.ravel(order='F')))
    one=[]
    for iw,C in enumerate(S['CASE']):
        assert_equal(C['wear'],WEAR[iw],'wear vector')
        assert C['fixed_initial_roll_rad']==0
        assert C['total_count']==1271
        assert C['stable_mask'].shape==(41,31)
        raw=C['raw'];assert np.asarray(raw['reason']).dtype==np.uint16,'Raw reason mask must be uint16'
        reason=np.asarray(raw['reason'],dtype=np.uint16)
        assert reason.shape==(1271,2) and reason.max()<=255
        safe=np.asarray(raw['safe'],dtype=bool);yaw=np.asarray(raw['yaw_ok'],dtype=bool)
        recovered=np.asarray(raw['recovered'],dtype=bool)
        assert_equal(safe,(reason&254)==0,'physical safe vs reason bits')
        assert_equal(yaw,(reason&1)==0,'yaw policy vs reason bits')
        assert np.all(safe[:,1]<=safe[:,0]) and np.all(yaw[:,1]<=yaw[:,0])
        assert_equal(reason[:,1]&reason[:,0],reason[:,0],'cumulative reasons')
        assert np.all(recovered<=safe)
        T=C['trim']
        if T['usable']:
            assert T['root_found'] and T['admissible'] and T['locally_stable_test']
            assert T['residual_norm']<c['trim_residual_tol']
            assert T['max_real_eigenvalue'] < -c['eigenvalue_tolerance']
            eig=np.linalg.eigvals(T['jacobian'])
            assert np.isclose(eig.real.max(),T['max_real_eigenvalue'],rtol=1e-8,atol=1e-8)
            stats['usable_trims']+=1
        else:
            assert not recovered.any()
            stats['unusable_trims']+=1
        assert np.all(np.asarray(raw['terminal_error'])[recovered]<=1)
        assert np.all(np.asarray(raw['terminal_derivative'])[recovered]<=1)
        times=np.asarray(raw['first_physical_violation'])
        assert_equal(safe,times[:,None]>np.array([5,10]),'first failure vs horizons')
        assert_equal(np.asarray(raw['status']),safe.astype(np.uint8)+recovered.astype(np.uint8),'raw status')
        for name,expected in [('safe',safe),('yaw_policy',yaw),('guarded_safe',safe&yaw),
                              ('recoverable',recovered),('guarded_recoverable',recovered&yaw),
                              ('safe_unresolved',safe&~recovered)]:
            assert C['masks'][name].shape==(41,31,2)
            assert_equal(C['masks'][name],expected.reshape((41,31,2),order='F'),name)
        selected=recovered[:,-1]&yaw[:,-1]
        mask=selected.reshape((41,31),order='F')
        assert_equal(C['stable_mask'],mask,'selected 2D mask')
        stable=np.asarray(C['stable_points']).reshape(-1,2)
        nonselected=np.asarray(C['nonselected_points']).reshape(-1,2)
        assert_equal(stable,points[selected],'absolute stable coordinates; no shift')
        assert_equal(nonselected,points[~selected],'nonselected coordinates')
        assert C['stable_count']==selected.sum()
        assert C['stable_fraction']==selected.sum()/1271
        region=C['region'];cell=np.asarray(region['cell_mask'],dtype=bool)
        assert cell.shape==(40,30)
        candidate=mask[:-1,:-1]&mask[1:,:-1]&mask[:-1,1:]&mask[1:,1:]
        assert np.all(cell<=candidate)
        assert region['candidate_cells']==candidate.sum()
        assert region['accepted_cells']==cell.sum()
        assert region['interior_points_tested']==candidate.sum()
        assert region['failed_cells']==candidate.sum()-cell.sum()
        dv,dr=np.meshgrid(np.diff(VC),np.diff(RR),indexing='ij')
        area=float((dv*dr)[cell].sum())
        assert np.isclose(area,region['sampled_area'],rtol=1e-13,atol=1e-13)
        if area>0:
            cv,cr=np.meshgrid((VC[:-1]+VC[1:])/2,(RR[:-1]+RR[1:])/2,indexing='ij')
            ctr=np.array([(cv*dv*dr)[cell].sum(),(cr*dv*dr)[cell].sum()])/area
            np.testing.assert_allclose(region['cell_area_centroid'],ctr,rtol=1e-12,atol=1e-12)
        else:assert np.isnan(region['cell_area_centroid']).all()
        assert not region['certified_continuous']
        stats['wear_cases']+=1;stats['initial_points']+=1271
        stats['center_tests']+=int(region['interior_points_tested'])
        stats['selected_at_10s']+=int(selected.sum())
        stats['safe_at_10s']+=int(safe[:,-1].sum())
        stats['recovered_at_10s']+=int(recovered[:,-1].sum())
        one.append(dict(wear_case=iw+1,wear=WEAR[iw].tolist(),
                safe_counts=safe.sum(axis=0).astype(int).tolist(),
                guarded_safe_counts=(safe&yaw).sum(axis=0).astype(int).tolist(),
                recovered_counts=recovered.sum(axis=0).astype(int).tolist(),
                selected_counts=(recovered&yaw).sum(axis=0).astype(int).tolist(),
                safe_unresolved_counts=(safe&~recovered).sum(axis=0).astype(int).tolist(),
                selected_count=int(selected.sum()),sampled_area=float(region['sampled_area']),
                selected_centroid=C['summary']['selected_centroid'],
                area_centroid=region['cell_area_centroid'],
                trim_root_found=bool(T['root_found']),trim_usable=bool(T['usable']),
                trim_admissible=bool(T['admissible']),trim_yaw_admissible=bool(T['yaw_admissible']),
                trim_locally_stable_test=bool(T['locally_stable_test']),
                initialization_used=C['initialization_used'],
                trim_residual_norm=T['residual_norm'],max_real_eigenvalue=T['max_real_eigenvalue'],
                touches_grid=np.asarray(C['summary']['touches_grid'],dtype=bool),
                candidate_cells=int(region['candidate_cells']),accepted_cells=int(region['accepted_cells']),
                interior_points_tested=int(region['interior_points_tested'])))

    assert path.name==f'cdc2d_{combo[0]:g}kph_mu{combo[1]:g}_delta{combo[2]:+g}_trim_phi00.mat','Unexpected production filename'
    assert meta['speed_kmh']==c['speed_kmh'] and meta['mu_road']==c['mu'],'Operating-condition metadata mismatch'
    assert c['ux']==c['speed_kmh']/3.6 and meta['initialization']==c['initialization']
    np.testing.assert_allclose(c['delta'],np.deg2rad([combo[2],combo[2],0,0]),rtol=0,atol=1e-15)
    assert_equal(meta['motion_ratios'],np.array([.8613,.797]),'Motion ratios')
    assert meta['engine']=='native' and c['engine']=='native'
    assert c['terminal_window']==1 and c['derivative_time_scale']==1 and c['batch_size']==256
    assert np.asarray(c['hidden_variants']).shape in [(4,),(1,4)]
    assert np.all(np.asarray(c['hidden_variants'])==0)
    assert c['yaw_factor']==.85 and c['ay_factor']==1 and c['LTR_limit']==1
    expected_tol=np.array([.1,.005,.002,np.deg2rad(.2),np.deg2rad(.2),
                          *([.002]*4),*([.01]*7),*([.001]*4),*([.002]*4),*([.2]*4),.05])
    np.testing.assert_allclose(c['state_tolerance'],expected_tol,rtol=0,atol=1e-16)
    expected_guard=dict(vc=30,r=2,phi=np.deg2rad(40),kappa=.5,alpha=1.2,omega=300)
    for key,value in expected_guard.items():assert c['guard'][key]==value,('Guard mismatch',key)
    assert same(S['signature']['options'],c),'Signature options differ from saved cfg'
    assert same(S['signature']['P'],S['parameters']),'Signature parameters differ'
    provenance=meta['provenance']
    assert same(provenance,S['signature']['provenance']),'Metadata/signature provenance mismatch'
    assert set(provenance)==set(ALIASES.values()),'Missing source fingerprints'
    for field,value in provenance.items():assert isinstance(value,str) and re.fullmatch('[0-9a-f]{64}',value),(field,'bad SHA')
    assert hashlib.sha256(S['signature']['source_text'].encode('utf-8')).hexdigest()==provenance['main_source_sha256'],'Embedded source fingerprint mismatch'
    if expected_sha_dict is not None:
        expected={ALIASES.get(key,key):value for key,value in expected_sha_dict.items()}
        assert set(expected)==set(ALIASES.values()),'Expected SHA dictionary needs the four frozen sources'
        assert provenance==expected,'Result does not match the frozen source fingerprints'
    for iw,C in enumerate(S['CASE']):
        T=C['trim'];summary=C['summary'];raw=C['raw']
        assert C['initialization_requested']=='trim'
        assert C['initialization_used']==('trim' if T['root_found'] else 'kinematic_zero')
        assert_equal(C['reason_mask'],np.asarray(raw['reason'][:,-1]).reshape((41,31),order='F'),'Final reason mask')
        assert same(summary,S['Summary'][iw]),'Case/top-level summary differs'
        assert summary['safe_count']==np.count_nonzero(raw['safe'][:,-1])
        assert summary['guarded_safe_count']==np.count_nonzero(raw['safe'][:,-1]&raw['yaw_ok'][:,-1])
        assert summary['recovered_count']==np.count_nonzero(raw['recovered'][:,-1])
        assert summary['selected_count']==C['stable_count']
        assert summary['safe_unresolved_count']==np.count_nonzero(raw['safe'][:,-1]&~np.asarray(raw['recovered'][:,-1],dtype=bool))
        assert summary['sampled_cell_area']==C['region']['sampled_area']
        mask=np.asarray(C['stable_mask'],dtype=bool)
        touch=np.array([mask[0,:].any(),mask[-1,:].any(),mask[:,0].any(),mask[:,-1].any()])
        assert_equal(summary['touches_grid'],touch,'Truncation flags')
        pts=np.asarray(C['stable_points']).reshape(-1,2)
        centroid=pts.mean(axis=0) if len(pts) else np.full(2,np.nan)
        np.testing.assert_allclose(summary['selected_centroid'],centroid,rtol=1e-13,atol=1e-13,equal_nan=True)
        assert np.asarray(summary['constraint_counts']).shape==(8,)
        for bit in range(8):assert summary['constraint_counts'][bit]==np.count_nonzero(np.asarray(raw['reason'][:,-1],dtype=np.uint16)&(1<<bit))
        if T['root_found']:
            assert np.isfinite(T['x']).all() and np.asarray(T['x']).shape==(32,)
            assert T['residual_norm']<c['trim_residual_tol']
        else:
            assert not T['usable']
    checksum=hashlib.sha256(path.read_bytes()).hexdigest()
    after=path.stat()
    assert (before.st_size,before.st_mtime_ns)==(after.st_size,after.st_mtime_ns),'MAT changed during validation'
    result=dict(all_checks_passed=True,filename=path.name,sha256=checksum,
                bytes=after.st_size,speed_kmh=combo[0],mu_road=combo[1],front_steer_deg=combo[2],
                fixed_initial_roll_rad=0,horizons_s=[5,10],grid_shape=[41,31],wear_cases=5,
                case_summaries=one,statistics=dict(stats),provenance=provenance)
    return json_safe(result)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--inputfolder','--results',type=Path,required=True)
    parser.add_argument('--report',type=Path,required=True)
    parser.add_argument('--expected-sha',type=Path)
    parser.add_argument('--allow-partial',action='store_true')
    args=parser.parse_args()
    expected=json.loads(args.expected_sha.read_text()) if args.expected_sha else None
    results=[];errors={}
    for path in sorted(args.inputfolder.glob('cdc2d_*_trim_phi00.mat')):
        try:results.append(validate_file(path,expected))
        except Exception as exc:errors[path.name]=str(exc) or type(exc).__name__
    complete=len(results)==210 and not errors
    if not args.allow_partial and not complete:errors['_complete']=f'{len(results)}/210 validated results'
    report=dict(all_checks_passed=not errors,complete=complete,completed_scenarios=len(results),
                total_scenarios=210,errors=errors,scenarios=results)
    args.report.parent.mkdir(parents=True,exist_ok=True)
    args.report.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print(json.dumps({key:value for key,value in report.items() if key!='scenarios'},indent=2))
    return 0 if not errors else 1


if __name__=='__main__':raise SystemExit(main())
