#!/usr/bin/env python3
"""Plot stored H-preserving research samples and checked lattice cells."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import time
os.environ.setdefault('MPLCONFIGDIR','/tmp/cdc_research_matplotlib')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.colors import ListedColormap
from matplotlib.lines import Line2D
import numpy as np
from scipy.io import loadmat
from validate_results import cases_of

ROOT=Path(__file__).resolve().parents[1]
COLORS=['#0072b2','#e69f00','#009e73','#cc79a7','#d55e00','#56b4e9','#222222']

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def readjson(p):
    try:return json.loads(p.read_text())
    except (OSError,ValueError):return {}
def atomic_json(p,value):
    temp=p.with_name(p.name+'.tmp');temp.write_text(json.dumps(value,indent=2,allow_nan=False)+'\n');temp.replace(p)
def savefig(fig,p):
    temp=p.with_name(p.stem+'.tmp.png');fig.savefig(temp,dpi=145,bbox_inches='tight');plt.close(fig);temp.replace(p)
def edges(mask,v,r):
    segments=[]
    for i,j in np.argwhere(mask):
        if i==0 or not mask[i-1,j]:segments.append([(v[i],r[j]),(v[i],r[j+1])])
        if i==mask.shape[0]-1 or not mask[i+1,j]:segments.append([(v[i+1],r[j]),(v[i+1],r[j+1])])
        if j==0 or not mask[i,j-1]:segments.append([(v[i],r[j]),(v[i+1],r[j])])
        if j==mask.shape[1]-1 or not mask[i,j+1]:segments.append([(v[i],r[j+1]),(v[i+1],r[j+1])])
    return segments

def format_axis(ax,v,r):
    ax.set(xlim=(v[0],v[-1]),ylim=(r[0],r[-1]),xlabel='$v_{c,0}$ [m/s]',ylabel='$r_0$ [rad/s]')
    ax.grid(alpha=.15);ax.set_facecolor('#f3f3f3')

def load(p):return loadmat(p,simplify_cells=True)['RESEARCH2D']

def condition_plot(p,out):
    S=load(p);cases=cases_of(S['CASE']);v=np.asarray(S['cfg']['grid']['vc']);r=np.asarray(S['cfg']['grid']['r'])
    fig,axes=plt.subplots(1,5,figsize=(19,5),sharex=True,sharey=True)
    m=S['meta'];angle=np.asarray(m['wheel_steer_deg']).flat[0]
    fig.suptitle(f"H-preserving recovery | {m['speed_kmh']:g} km/h | mu={m['mu_road']:g} | front steer {angle:+g} deg | CDC 950 mA",fontsize=14)
    for ax,C in zip(axes,cases):
        H=np.asarray(C['masks']['reference'],bool);final=np.asarray(C['masks']['final'],bool)
        cells=np.asarray(C['region']['cell_mask'],bool)
        ax.pcolormesh(v,r,np.where(H.T,1,np.nan),cmap=ListedColormap(['#dddddd']),vmin=0,vmax=1,shading='nearest',rasterized=True)
        ax.pcolormesh(v,r,np.ma.masked_where(~cells.T,cells.T),cmap=ListedColormap(['#58ac83']),vmin=0,vmax=1,shading='flat',rasterized=True)
        if H.any() and not H.all():ax.contour(v,r,H.T.astype(float),levels=[.5],colors='#333333',linestyles='--',linewidths=.9)
        segments=edges(cells,v,r)
        if segments:ax.add_collection(LineCollection(segments,colors='#12623c',linewidths=1.1))
        iv,ir=np.where(final[:,:,1]);ax.scatter(v[iv],r[ir],s=2,c='#12623c',alpha=.6,rasterized=True)
        T=C['trim'];x=np.asarray(T['x']).reshape(-1)
        ax.plot(x[0],x[1],marker='x',ms=8,color='red',mew=1.7)
        wear=' '.join(str(int(x)) for x in np.asarray(C['wear']).reshape(-1))
        counts=final.sum(axis=(0,1));touch=bool(C['grid_truncated']) or bool(C['reference_grid_truncated'])
        ax.set_title(f"Wear [{wear}]\nfinal 5/10 s: {counts[0]}/{counts[1]}\ncell area={float(C['region']['cell_area']):.4g}; phi0={x[3]:.4g} rad"+('\nGRID TRUNCATED' if touch else ''),fontsize=9)
        format_axis(ax,v,r)
    legend=[Line2D([0],[0],color='#333333',ls='--',label='Static handling reference H'),Line2D([0],[0],color='#12623c',label='Checked-cell boundary (10 s)'),Line2D([0],[0],color='red',marker='x',ls='',label='Computed trim')]
    fig.legend(handles=legend,loc='lower center',ncol=3,bbox_to_anchor=(.5,.005))
    fig.text(.5,.10,'Gray: H seeds evaluated; outside H: not simulated. Green: final points + cells with all four vertices and center passing.\nFinite-time samples; no hull filling and no invariant-set claim.',ha='center',fontsize=9)
    fig.subplots_adjust(bottom=.30,top=.75,wspace=.12)
    savefig(fig,out)

def overview_plot(paths,out):
    entries=sorted([load(p) for p in paths],key=lambda S:float(np.asarray(S['meta']['wheel_steer_deg']).flat[0]))
    v=np.asarray(entries[0]['cfg']['grid']['vc']);r=np.asarray(entries[0]['cfg']['grid']['r'])
    fig,axes=plt.subplots(1,5,figsize=(19,4.5),sharex=True,sharey=True)
    for k,ax in enumerate(axes):
        for color,S in zip(COLORS,entries):
            C=cases_of(S['CASE'])[k];segments=edges(np.asarray(C['region']['cell_mask'],bool),v,r)
            if segments:ax.add_collection(LineCollection(segments,colors=color,linewidths=1.2))
        ax.set_title('Wear ['+' '.join(str(int(x)) for x in np.asarray(C['wear']).reshape(-1))+']',fontsize=10);format_axis(ax,v,r)
    m=entries[0]['meta'];fig.suptitle(f"H-preserving checked-cell boundaries at 10 s | {m['speed_kmh']:g} km/h | mu={m['mu_road']:g}")
    handles=[Line2D([0],[0],color=c,label=f"{float(np.asarray(S['meta']['wheel_steer_deg']).flat[0]):g} deg") for c,S in zip(COLORS,entries)]
    fig.legend(handles=handles,loc='lower center',ncol=7)
    fig.subplots_adjust(bottom=.19,top=.82,wspace=.12);savefig(fig,out)

def run_once(input_dir,output_dir,index):
    changed=False
    for p in sorted(input_dir.glob('research_*_trim.mat')):
        stamp=[p.stat().st_size,p.stat().st_mtime_ns];record=index['records'].get(p.name,{})
        target=output_dir/(p.stem+'.png')
        if record.get('stamp')==stamp and target.is_file():continue
        condition_plot(p,target)
        index['records'][p.name]={'source_mat':p.name,'source_sha256':sha(p),'figure':target.name,'stamp':stamp}
        print('PLOT '+p.name,flush=True);changed=True
    for s in [50,60,70,80,90,100]:
        for mu in [.9,.8,.7,.6,.5]:
            paths=[input_dir/f'research_{s:g}kph_mu{mu:g}_delta{a:+g}_I950_950_950_950_trim.mat' for a in [0,.5,1,1.5,2,2.5,3]]
            if not all(p.name in index['records'] for p in paths):continue
            name=f'overview_final_{s:g}kph_mu{mu:g}_trim.png'.replace(f'mu{mu:g}',f'mu{mu:g}'.replace('.','p'))
            signature=[index['records'][p.name]['source_sha256'] for p in paths]
            if index.setdefault('overview_sources',{}).get(name)==signature and (output_dir/name).is_file():continue
            overview_plot(paths,output_dir/name);index['overview_sources'][name]=signature
            if name not in index['overview_figures']:index['overview_figures'].append(name)
            print('OVERVIEW '+name,flush=True);changed=True
    if changed:
        index['updated_utc']=datetime.now(timezone.utc).isoformat();atomic_json(output_dir/'plot_index.json',index)
    return index

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--input',type=Path,default=ROOT/'results/cdc_research_final');parser.add_argument('--output',type=Path);parser.add_argument('--watch',action='store_true');parser.add_argument('--interval',type=float,default=15);a=parser.parse_args()
    out=a.output or a.input/'figures';out.mkdir(parents=True,exist_ok=True)
    index=readjson(out/'plot_index.json');fingerprint=sha(Path(__file__))
    if index.get('plotter_sha256')!=fingerprint:index={'records':{},'overview_figures':[],'plotter_sha256':fingerprint}
    while True:
        index=run_once(a.input,out,index)
        if not a.watch:return 0
        phase=readjson(a.input/'progress.json').get('phase')
        if phase=='completed' and len(index['records'])==210 and len(index['overview_figures'])==30:return 0
        if phase in ['failed','cancelled','stopped']:return 1
        time.sleep(max(1,a.interval))
if __name__=='__main__':raise SystemExit(main())
