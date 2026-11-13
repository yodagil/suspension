#!/usr/bin/env python3
"""Compare saved independent output to supplied CarSim mask without shifting."""
from pathlib import Path
import argparse,json
import numpy as np
from scipy.io import loadmat

def compare(independent,reference_dir):
 S=loadmat(independent,simplify_cells=True)['RESEARCH2D'];C=S['CASE'];C=C if isinstance(C,dict) else C[0]
 baseline=json.loads((reference_dir/'carsim_reference_summary.json').read_text());R=np.load(reference_dir/'carsim_reference_masks.npz')
 for key in ['vc','r']:assert np.allclose(C['grid'][key],R[key],rtol=0,atol=1e-12)
 expected=baseline['condition'];meta=S['meta'];assert meta['speed_kmh']==expected['speed_kmh'] and meta['mu_road']==expected['mu'] and meta['steer_target_deg']==expected['steer_target_deg']
 H=np.asarray(C['masks']['reference'],bool);Hr=R['reference'].astype(bool);common=H&Hr;metrics={}
 for name in ['final','recoverable_on_reference','guarded_final','model_safe']:
  a=np.asarray(C['masks'][name],bool);b=R[name].astype(bool);rows=[]
  for k in range(2):
   aa=a[:,:,k];bb=b[:,:,k];union=int((aa|bb).sum());intersection=int((aa&bb).sum())
   rows.append(dict(horizon_s=[5,10][k],independent_count=int(aa.sum()),carsim_count=int(bb.sum()),intersection=intersection,union=union,jaccard=intersection/union if union else 1,common_H_disagreements=int(((aa!=bb)&common).sum())))
  metrics[name]=rows
 trimkeys=['vy_mps','r_radps','vx_kmh','roll_deg','pitch_deg','rollrate_degps','pitchrate_degps']
 trim={k:dict(independent=float(C['trim'][k]),carsim=float(baseline['trim'][k]),difference=float(C['trim'][k]-baseline['trim'][k])) for k in trimkeys}
 return dict(independent_file=str(independent),condition=expected,reference_H=int(Hr.sum()),independent_H=int(H.sum()),H_disagreements=int((H!=Hr).sum()),masks=metrics,trim=trim,independent_cell_area=float(C['region']['cell_area']),carsim_cell_area=baseline['cell_area'],independent_cells=int(C['region']['accepted_cells']),carsim_cells=baseline['accepted_cells'],not_carsim_engine=True,scope='Finite-time saved-mask comparison only; no time histories were supplied by CarSim')

if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('file',type=Path);p.add_argument('--reference-dir',type=Path,default=Path(__file__).resolve().parents[1]/'validation/hmc_portability/reference');p.add_argument('--output',type=Path);a=p.parse_args();v=compare(a.file,a.reference_dir);text=json.dumps(v,indent=2,allow_nan=False);print(text)
 if a.output:a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(text+'\n')
