#!/usr/bin/env python3
"""Export simulation outputs only; never publish embedded vehicle calibration."""
from pathlib import Path
import hashlib,json,os
import numpy as np
from scipy.io import loadmat,savemat

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def cases_of(x):return [x] if isinstance(x,dict) else list(x)
def public_export(source,target,hashes):
 source=Path(source);target=Path(target);target.parent.mkdir(parents=True,exist_ok=True)
 original_sha=sha(source);stamp=target.with_suffix('.export.json')
 if target.exists() and stamp.exists():
  old=json.loads(stamp.read_text())
  if old.get('input_sha256')==original_sha and old.get('exporter_sha256')==sha(Path(__file__)) and old.get('output_sha256')==sha(target):return old
 S=loadmat(source,simplify_cells=True)['RESEARCH2D']
 # None of the input-data container or original source attachment metadata
 # survives this explicit replacement. Only checksums and computed results.
 code_hashes=[[name,value] for name,value in sorted(hashes.items()) if name.endswith(('.m','.cpp','.hpp'))]
 S['source']={'publication_scope':'computed simulation results only',
  'private_model_data_excluded':True,'input_mat_sha256':original_sha,
  'model_source_sha256':np.asarray(code_hashes,dtype=object),
  'parameter_file_sha256':hashes['data/hmc_data.mat']}
 for options in [S['options']]+[c['configuration'] for c in cases_of(S['CASE'])]:
  for key in ['data_file','native_executable','output_dir']:options.pop(key,None)
 cells=cases_of(S['CASE']);arr=np.empty((len(cells),1),dtype=object)
 for i,c in enumerate(cells):arr[i,0]=c
 S['CASE']=arr
 forbidden={'derived_data','vehicle','tires','damper','tables','raw_scalars','data_manifest','targetTirePaths','tireFiles'}
 def check(obj):
  if isinstance(obj,dict):
   bad=forbidden&set(obj);assert not bad,'Private model data in public export: '+str(bad)
   for value in obj.values():check(value)
  elif isinstance(obj,np.ndarray) and obj.dtype==object:
   for value in obj.flat:check(value)
  elif isinstance(obj,(tuple,list)):
   for value in obj:check(value)
 check(S)
 temp=target.with_name(target.name+f'.{os.getpid()}.tmp')
 savemat(temp,{'RESEARCH2D':S},appendmat=False,do_compression=True,long_field_names=True)
 probe=loadmat(temp,simplify_cells=True)['RESEARCH2D'];check(probe)
 assert probe['source']['private_model_data_excluded']
 temp.replace(target)
 result=dict(input_sha256=original_sha,output_sha256=sha(target),exporter_sha256=sha(Path(__file__)),private_model_data_excluded=True)
 stamp.write_text(json.dumps(result,indent=2)+'\n')
 return result
