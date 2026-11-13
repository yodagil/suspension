#!/usr/bin/env python3
"""Cross-check independent HMC raw trajectories and 2D cell bookkeeping."""
from pathlib import Path
import hashlib
import numpy as np
from scipy.io import loadmat
WEARS=[[0,0,0,0],[25,25,0,0],[50,50,25,25],[0,0,25,25],[25,25,50,50]]
def cases_of(x):return [x] if isinstance(x,dict) else list(x)
def need(ok,msg):
 if not ok:raise ValueError(msg)
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def validate_file(path,expected_sha_dict=None,expected_wears=WEARS):
 path=Path(path);S=loadmat(path,simplify_cells=True)['RESEARCH2D'];m=S['meta'];o=S['options'];C=cases_of(S['CASE'])
 need(m['version']=='HMC_INDEPENDENT_1' and m['not_carsim_engine'] and m['not_tno_swift_belt_model'],'Wrong model scope')
 need(m['engine']=='native' and m['state_count']==34 and m['longitudinal_speed']=='dynamic PI speed control','Wrong engine')
 need(m['dt_s'] in [.0005,.001] and np.array_equal(m['horizons_s'],[5,10]),'Timing mismatch')
 need(np.array_equal(m['currents_mA'],[950]*4),'Damper command changed')
 need(len(C)==len(expected_wears),'Incomplete wear cases')
 if expected_sha_dict:
  for key,value in S['source'].items():
   if not isinstance(value,str):continue
   names=[n for n in expected_sha_dict if n.replace('/','_').replace('.','_')==key or Path(n).name.replace('.','_')==key]
   if names:need(hashlib.sha256(value.encode()).hexdigest()==expected_sha_dict[names[0]],'Source changed: '+key)
 v=np.asarray(o['grid']['vc']);r=np.asarray(o['grid']['r']);need(v.shape==(61,) and r.shape==(121,),'Wrong grid')
 V,R=np.meshgrid(v,r,indexing='ij');points=np.c_[V.ravel(order='F'),R.ravel(order='F')];summaries=[]
 for iw,(c,wear) in enumerate(zip(C,expected_wears),1):
  need(np.array_equal(c['wear'],wear),'Wear order changed');T=c['trim'];need(T['root_found'] and np.isfinite(T['x']).all(),'Unchecked trim')
  need(np.asarray(T['x']).size==34 and T['residual_norm']<1e-6,'Bad trim residual')
  M=c['masks'];H=np.asarray(M['reference'],bool);need(H.shape==(61,121),'Wrong H shape');ids=np.flatnonzero(H.ravel(order='F'));n=len(ids);raw=c['raw']
  need(np.array_equal(raw['initial_grid_linear_index_1based'],ids+1),'H index mismatch')
  need(np.array_equal(np.asarray(raw['initial_points']).reshape(n,2),points[ids]),'Initial coordinates mismatch')
  for k in ['passed','recoverable','model_safe','guarded_final','stayed_in_handling','yaw_ok','ay_ok']:
   a=np.asarray(raw[k]).reshape(n,2);need(np.isin(a,[0,1]).all(),'Nonboolean '+k)
  p=np.asarray(raw['passed']).reshape(n,2);rec=np.asarray(raw['recoverable']).reshape(n,2);safe=np.asarray(raw['model_safe']).reshape(n,2)
  inH=np.asarray(raw['stayed_in_handling']).reshape(n,2);yo=np.asarray(raw['yaw_ok']).reshape(n,2);ao=np.asarray(raw['ay_ok']).reshape(n,2)
  expected=rec*inH
  if not c['trim_in_reference']:expected[:]=False
  need(np.array_equal(p,expected),'Final != recoverable & H')
  need(np.array_equal(raw['guarded_final'],p*yo*ao),'Guarded final mismatch')
  need(np.all(rec<=safe),'Recovery without physical safety')
  err=np.asarray(raw['terminal_error']).reshape(n,2);need(np.all(err[rec.astype(bool)]<=1+1e-9),'Terminal tolerance failed')
  for key,field in [('final','passed'),('recoverable_on_reference','recoverable'),('guarded_final','guarded_final'),('model_safe','model_safe')]:
   full=np.zeros((7381,2),bool);full[ids]=np.asarray(raw[field]).reshape(n,2).astype(bool)
   need(np.array_equal(np.asarray(M[key],bool),full.reshape(61,121,2,order='F')),'Mask/raw mismatch '+key)
  F=np.asarray(M['final'],bool)[:,:,1];cand=F[:-1,:-1]&F[1:,:-1]&F[:-1,1:]&F[1:,1:];reg=c['region'];cells=np.asarray(reg['cell_mask'],bool)
  need(cells.shape==(60,120) and np.all(cells<=cand),'Invalid cell vertices');ci=np.flatnonzero(cand.ravel(order='F'));iv,ir=np.unravel_index(ci,cand.shape,order='F')
  cp=np.c_[(v[iv]+v[iv+1])/2,(r[ir]+r[ir+1])/2]
  need(np.array_equal(np.asarray(reg['center_points']).reshape(-1,2),cp),'Wrong center coordinates')
  if len(ci):
   passed=np.asarray(reg['center_raw']['passed']).reshape(len(ci),2)[:,1].astype(bool)
   need(np.array_equal(np.asarray(reg['center_pass']).reshape(-1),passed),'Center pass mismatch')
   need(np.array_equal(cells.ravel(order='F')[ci],passed),'Unvalidated centers')
  area=(np.diff(v)[:,None]*np.diff(r)[None,:])[cells].sum();need(np.isclose(reg['cell_area'],area,rtol=1e-10,atol=1e-12),'Wrong cell area')
  need(reg['accepted_cells']==cells.sum() and reg['candidate_cells']==cand.sum(),'Cell counts mismatch')
  summaries.append(dict(wear_case=iw,wear=wear,reference_count=n,final_counts=np.asarray(M['final']).sum(axis=(0,1)).astype(int).tolist(),recovered_counts=np.asarray(M['recoverable_on_reference']).sum(axis=(0,1)).astype(int).tolist(),guarded_counts=np.asarray(M['guarded_final']).sum(axis=(0,1)).astype(int).tolist(),sampled_area=float(area),accepted_cells=int(cells.sum()),center_points=len(ci),trim_usable=bool(T['usable']),trim_in_reference=bool(c['trim_in_reference']),initial_roll_rad=float(T['x'][3]),grid_truncated=bool(c['grid_truncated']),reference_grid_truncated=bool(c['reference_grid_truncated'])))
 return dict(all_checks_passed=True,filename=path.name,sha256=sha(path),bytes=path.stat().st_size,speed_kmh=float(m['speed_kmh']),mu_road=float(m['mu_road']),front_steer_deg=float(m['steer_target_deg']),version=m['version'],case_summaries=summaries)
