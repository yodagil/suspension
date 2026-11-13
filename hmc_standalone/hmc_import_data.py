#!/usr/bin/env python3
"""Import supplied HMC runtime data without executing CarSim or model callbacks.

Runtime echo is authoritative over the earlier assembly PAR: it contains the
actually used spring tables, controller, imports and geometry. Tables retain
CarSim interpolation labels; choosing an independent spline implementation is
an explicit solver approximation, not an implication of this data conversion.
"""
import argparse
import hashlib
import json
from pathlib import Path
import re
import zipfile

import numpy as np
from scipy.io import savemat

ROOT=Path(__file__).resolve().parent
ATTACH=Path('/workspace/attachments')
DEFAULTS={
 'echo':ATTACH/'dce0b795-d163-4541-9cdc-f05bd4ec24d9/trim_echo.par',
 'end':ATTACH/'c9b4a57f-0e24-4ee3-8890-cf474acd1a6c/trim_end.par',
 'tires':ATTACH/'e280dd4b-d3f0-4cc4-805e-420ffc4c444f/HMC_wear_model.zip',
 'slx':Path('/workspace/suspension/validation/hmc_portability/HMC_PHM_yohan.slx'),
}
CORNERS={'FL':(1,1),'FR':(1,2),'RL':(2,1),'RR':(2,2)}
SUFFIX={'FL':'L1','FR':'R1','RL':'L2','RR':'R2'}
NUM=r'[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eEdD][-+]?\d+)?'

def number(x):return float(x.replace('D','E').replace('d','e'))
def digest(data):return hashlib.sha256(data).hexdigest()
def parse_scalars(text):
 out={}
 for m in re.finditer(r'^([A-Z][A-Z0-9_]*(?:\([^\r\n]*?\))?)\s+('+NUM+r')(?=\s|$)',text,re.M):out[m[1]]=number(m[2])
 return out

def parse_tables(text):
 out={}
 pat=r'^([A-Z][A-Z0-9_]*(?:\([^\r\n]*?\))?)\s+(SPLINE|LINEAR_FLAT|LINEAR|CUBIC)[^\n]*\n(.*?)^ENDTABLE\s*$'
 for m in re.finditer(pat,text,re.M|re.S):
  a=[]
  for l in m[3].splitlines():
   l=l.split('!')[0].strip()
   if not l:continue
   v=re.fullmatch('('+NUM+r')\s*,\s*('+NUM+')',l)
   if not v:raise ValueError('Unexpected table row '+m[1]+': '+l)
   a.append([number(v[1]),number(v[2])])
  if len(a)<2:raise ValueError('Incomplete table '+m[1])
  assert np.all(np.diff(np.asarray(a)[:,0])>0),m[1]
  out[m[1]]={'interpolation':m[2],'native_values':a}
 return out

def matkey(s):
 s=re.sub('[^A-Za-z0-9_]','_',s)
 if len(s)>63:s=s[:54]+'_'+hashlib.sha1(s.encode()).hexdigest()[:8]
 return s

def get_array(s,key):return np.asarray([s[f'{key}({a},{b})'] for a,b in CORNERS.values()])
def jsonable(x):
 if isinstance(x,np.ndarray):return x.tolist()
 if isinstance(x,np.generic):return x.item()
 if isinstance(x,dict):return {k:jsonable(v) for k,v in x.items()}
 if isinstance(x,(list,tuple)):return [jsonable(v) for v in x]
 return x

def converted_table(table,xfactor,yfactor,xname,yname):
 a=np.asarray(table['native_values'])
 return {xname:a[:,0]*xfactor,yname:a[:,1]*yfactor,'interpolation':table['interpolation'],'native_values':a}

def parse_damper(slx):
 import xml.etree.ElementTree as ET
 with zipfile.ZipFile(slx) as z:
  script=ET.fromstring(z.read('simulink/stateflow/chart_28.xml')).find('.//P[@Name="script"]').text
 def arr(key):
  m=re.search(r'\b'+re.escape(key)+r'\s*=\s*\[(.*?)\]\s*;',script,re.S)
  assert m,key
  body=re.sub(r'%[^\n]*','',m[1]).replace('...','')
  rows=[r for r in body.split(';') if r.strip()]
  return np.asarray([[float(v) for v in re.findall(NUM,r)] for r in rows])
 current=arr('I_axis').reshape(-1);v=arr('v_axis').reshape(-1)
 front,rear=arr('F_front'),arr('F_rear')
 assert front.shape==rear.shape==(18,17) and current.size==18 and v.size==17
 return dict(current_mA=current,v_mm_s=v,front_N=front,rear_N=rear,
   current_nominal_mA=950.0,compression_sign=1.0,
   note='Active SLX integer maps. Positive velocity/force=compression. Clamp current and velocity; bilinear interpolation. No extra MR multiplication in forward block.')

def parse_tires(path):
 tires={};meta={}
 with zipfile.ZipFile(path) as z:
  for n in z.namelist():
   if not n.lower().endswith('.tir'):continue
   base=Path(n).name
   key='new' if '_New_' in base else ('front' if '_FR_' in base else 'rear')+('25' if '_25K_' in base else '50')
   t=z.read(n).decode('latin1');p={}
   for m in re.finditer(r'^\s*([A-Za-z][A-Za-z0-9_]*)\s*=\s*('+NUM+r')(?=\s|$)',t,re.M):p[m[1]]=number(m[2])
   assert p['FITTYP']==62 and p['FNOMIN']>0
   tires[key]=p;meta[key]={'filename':base,'sha256':digest(z.read(n)),'numeric_coefficients':len(p),'model':'MF-Tyre/MF-Swift 6.2 coefficient data; not executable TNO dynamics'}
 assert set(tires)=={'new','front25','front50','rear25','rear50'}
 return tires,meta

def main():
 pa=argparse.ArgumentParser(description=__doc__)
 for k,p in DEFAULTS.items():pa.add_argument('--'+k,type=Path,default=p)
 pa.add_argument('--output',type=Path,default=ROOT/'data');a=pa.parse_args()
 text=a.echo.read_text(errors='replace');s=parse_scalars(text);tables=parse_tables(text)
 end=a.end.read_text(errors='replace');es=parse_scalars(end)
 tires,tire_meta=parse_tires(a.tires);damper=parse_damper(a.slx)
 ms=s['M_SU'];mus=np.asarray([s['M_US(1)']/2]*2+[s['M_US(2)']/2]*2)
 L=(s['LX_AXLE(2)']-s['LX_AXLE(1)'])*.001;lf=(s['LX_CG_SU']-s['LX_AXLE(1)'])*.001;lr=L-lf
 # Standard gravity independently matches total supplied end Fz/m to1.3e-8.
 g=9.80665*s['R_GRAVITY']
 V=dict(ms=ms,m=ms+mus.sum(),g=g,Ixx=s['IXX_SU'],Iyy=s['IYY_SU'],Iz=s['IZZ_SU'],
    L=L,lf=lf,lr=lr,tf=s['L_TRACK(1)']*.001,tr=s['L_TRACK(2)']*.001,
    h_cg_sprung=s['H_CG_SU']*.001,h_cg=s['H_CG_SU']*.001,
    mu_u=mus,Iw=get_array(s,'IW')+get_array(s,'IT'),
    wheel_center_height_m=get_array(s,'H_WC')*.001,
    camber_static_rad=np.deg2rad(get_array(s,'A_CAMBER')),
    toe_static_rad=np.deg2rad(get_array(s,'A_TOE')),
    caster_static_rad=np.deg2rad(get_array(s,'A_CASTER')),
    kingpin_inclination_rad=np.deg2rad(get_array(s,'A_KPI')),
    kingpin_offset_m=get_array(s,'L_KPO')*.001,
    Karb_f=s['MX_AUX_COEFFICIENT(1)']*180/np.pi,Karb_r=s['MX_AUX_COEFFICIENT(2)']*180/np.pi,
    Carb_f=s['DAUX(1)']*180/np.pi,Carb_r=s['DAUX(2)']*180/np.pi,
    speed_KP=s['SPEED_KP'],speed_KI=s['SPEED_KI'],speed_KP3=s['SPEED_KP3'],
    max_power_W=s['PMAX_SC']*1000,rear_drive_fraction=s['R_REAR_DRIVE_SC'],
    steering_ratio=14.805,gear_rot_ratio=s['GEAR_ROT_COEFFICIENT(1)'],speed_control_mode=s['OPT_SC'],powertrain_mode=s['OPT_PT'],
    damper_motion_ratio=get_array(s,'CMP_DAMP_COEFFICIENT')*get_array(s,'CMP_DAMP_GAIN'),
    spring_motion_ratio=get_array(s,'CMP_SPR_SEAT_COEFFICIENT')*get_array(s,'CMP_SPR_SEAT_GAIN'),
    rho_air=s['D_AIR'],aero_area_m2=s['AREA_AERO'],aero_Cx=s['FX_AERO_SHAPING_CONSTANT'],
    corner_order=np.asarray(list(CORNERS),dtype=object),
    normal_spring_mode=2.0,CDC_current_mA=np.full(4,950.),
    tables={},raw_scalars={matkey(k):v for k,v in s.items()})
 V['Fz_static_N']=np.r_[np.full(2,ms*g*lr/(2*L)),np.full(2,ms*g*lf/(2*L))]+mus*g
 specs={
  'spring_comp':('FS_COMP_TABLE',.001,1,'x_m','y_N'),
  'spring_ext':('FS_EXT_TABLE',.001,1,'x_m','y_N'),
  'jounce_stop':('F_JNC_STOP_TABLE',.001,1,'x_m','y_N'),
  'rebound_stop':('F_REB_STOP_TABLE',.001,1,'x_m','y_N'),
  'camber':('CAMBER_TABLE',.001,np.pi/180,'x_m','y_rad'),
  'toe':('TOE_TABLE',.001,np.pi/180,'x_m','y_rad'),
  'dive':('SUSP_DIVE_TABLE',.001,np.pi/180,'x_m','y_rad'),
  'susp_x':('SUSP_X_TABLE',.001,.001,'x_m','y_m'),
  'susp_lat':('SUSP_LAT_TABLE',.001,.001,'x_m','y_m'),
 }
 for kind,(key,xf,yf,xn,yn) in specs.items():
  V['tables'][kind]={corner:converted_table(tables[f'{key}({aa},{bb})'],xf,yf,xn,yn) for corner,(aa,bb) in CORNERS.items()}
 V['tables']['steering']={corner:converted_table(tables[f'STEER_KIN_TABLE({aa},{bb})'],np.pi/180,np.pi/180,'x_rad','y_rad') for corner,(aa,bb) in list(CORNERS.items())[:2]}
 V['tables']['steering_wheel']=converted_table(tables['STEER_SW_TABLE'],1.,np.pi/180,'x_s','y_rad')
 V['tables']['steering_wheel']['note']='Inactive internal time-command table, replaced by external steering-wheel import. Not a gear map.'
 spring_slope=[];spring_intercept=[]
 for corner in CORNERS:
  q=V['tables']['spring_comp'][corner];x,y=q['x_m'],q['y_N']
  slope=np.diff(y)/np.diff(x);spring_slope.append(float(np.interp(0,(x[1:]+x[:-1])/2,slope)));spring_intercept.append(float(np.interp(0,x,y)))
 V['spring_rate_N_m']=np.asarray(spring_slope);V['spring_intercept_N']=np.asarray(spring_intercept)
 # Preserve every compliance coefficient, with explicit native and SI units.
 V['compliance']={}
 for key in ['CT_FX','CS_FY','CS_MZ','CC_FX','CI_FY','CI_MZ','C_LONG','C_LAT']:
  vals=get_array(s,key+'_COEFFICIENT')
  angle=key not in ['C_LONG','C_LAT']
  V['compliance'][key]={'native':vals,'SI':vals*(np.pi/180 if angle else .001),
    'units':'rad/N' if key in ['CT_FX','CS_FY','CC_FX','CI_FY'] else ('rad/(N*m)' if angle else 'm/N')}
 exports={}
 for m in re.finditer(r'^EXPORT\s+(\S+)\s+('+NUM+r')\s*!\s*#(\d+)\.',end,re.M):exports[int(m[3])]=(m[1],number(m[2]))
 assert sorted(exports)==list(range(1,73))
 e=np.asarray([exports[i][1] for i in range(1,73)])
 V['reference_trim']={
  'benchmark_only':1.,'time_s':es['TSTART'],'xIC_export72':e,
  'vx_m_s':e[2]/3.6,'vy_m_s':e[3]/3.6,'vz_m_s':e[4]/3.6,
  'r_rad_s':np.deg2rad(e[13]),'roll_rad':np.deg2rad(e[8]),'pitch_rad':np.deg2rad(e[9]),
  'rollrate_rad_s':np.deg2rad(e[11]),'pitchrate_rad_s':np.deg2rad(e[12]),'z_cg_m':e[66],
  'steer_rad':np.deg2rad(e[[53,54,55,56]]),'Fz_N':e[[22,24,23,25]],
  'Fx_N':e[[14,16,15,17]],'Fy_N':e[[18,20,19,21]],
  'spring_comp_m':e[[26,28,27,29]]*.001,'spring_N':e[[34,36,35,37]],'damper_N':e[[38,40,39,41]],
  'jounce_m':np.asarray([es['SV_JNC_'+suf] for suf in SUFFIX.values()]),
  'wheel_omega_rad_s':e[[67,68,69,70]]*np.pi/30,
  'alpha_rad':np.deg2rad(e[[42,44,43,45]]),'kappa':e[[57,59,58,60]],
  'note':'End-of12s source condition50km/h mu0.5 steer0 newtires. Reference values are not a fitted model law; do not reuse as all-condition trim.'}
 V['assumptions']={
  'gravity':'Standard9.80665m/s2 times supplied R_GRAVITY; independently consistent with exported static total load to1.3e-8m/s2.',
  'steering_ratio':'Active Simulink Constant delta_front_deg*14.805.',
  'antiroll_units':'Echo documents auxiliary coefficient Nm/deg; converted multiply180/pi toNm/rad.',
  'kinematic_conventions':'Table camber/toe describe mirrored side geometry. Applying left/right wheel frame signs is solver responsibility.',
  'interpolation':'Original table labels retained. Exact VS SPLINE interpolation algorithm is not supplied.',
  'speed_controller':'Echo KP/KI units retained asSI. Proprietary throttle/power law is not supplied by inputdata.',
  'tire_dynamics':'Coefficient data only; TNO proprietary Swift state evolution is not provided.',
  'spring_runtime':'Echo authoritative. Runtime springs are linear30790N/m, differing from earlier assemblyfile nonlinear curves.'}
 sources={k:{'filename':p.name,'path':str(p),'sha256':digest(p.read_bytes())} for k,p in [('echo',a.echo),('end',a.end),('tire_zip',a.tires),('slx',a.slx)]}
 source=dict(version='HMC_DATA_IMPORT_1',authority='actual runtime trim_echo.par + active Simulink CDCmaps + supplied5TIR',files=sources,tires=tire_meta,
    public_data=False,friction_mode=1114.,reference_speed_kmh=50.,reference_mu=.5,corner_order='FL FR RL RR',units='SI unless field name explicitly states mm_s/mA or native',no_stability_result_fitting=True)
 D={'vehicle':V,'tires':tires,'damper':damper,'source':source}
 a.output.mkdir(parents=True,exist_ok=True)
 for name,obj in [('vehicle',V),('tires',tires),('damper',damper),('source_manifest',source)]:
  (a.output/(name+'.json')).write_text(json.dumps(jsonable(obj),indent=2,ensure_ascii=False,allow_nan=False)+'\n')
 savemat(a.output/'hmc_data.mat',{'D':D},do_compression=True,long_field_names=True)
 print(json.dumps({'output':str(a.output),'files':5,'scalar_parameters':len(s),'runtime_tables':len(tables),'tire_coefficients':{k:len(v) for k,v in tires.items()},'spring_rate_N_m':spring_slope,'spring_intercept_N':spring_intercept,'damper_shape':list(damper['front_N'].shape),'m':V['m'],'g':g},indent=2))

if __name__=='__main__':main()
