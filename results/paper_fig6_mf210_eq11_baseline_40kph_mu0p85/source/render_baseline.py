#!/usr/bin/env python3
"""Actual Eq11 stable samples and Section3.1 stages, with invalidity visible."""
from pathlib import Path
import argparse,json,os
os.environ.setdefault('MPLCONFIGDIR','/tmp/mf210-eq11-mpl')
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.io import loadmat
from scipy.spatial import ConvexHull
from mpl_toolkits.mplot3d.art3d import Poly3DCollection,Line3DCollection

COLORS=['#389b51','#c5a414','#2445d9']

def scatter(ax,p,color,n=1800):
    p=np.asarray(p).reshape(-1,3)
    if not len(p):return
    ids=np.unique(np.rint(np.linspace(0,len(p)-1,min(len(p),n))).astype(int));v=p[ids][:,[1,0,2]]
    ax.scatter(*v.T,s=3,color=color,alpha=.28,linewidths=0,rasterized=True)

def axes(ax):
    ax.set(xlim=(.72,-.72),ylim=(-10,10),zlim=(-.4,.4))
    ax.set_xticks([.6,0,-.6]);ax.set_yticks([-10,0,10]);ax.set_zticks([-.4,0,.4])
    ax.set_xlabel(r'$r$ (rad/s)',labelpad=1);ax.set_ylabel(r'$v_c$ (m/s)',labelpad=1)
    ax.set_zlabel(r'$\phi$ (rad)',labelpad=1);ax.set_proj_type('ortho');ax.view_init(elev=11,azim=-72)
    ax.set_box_aspect((1.3,2.28,1.65));ax.tick_params(labelsize=7,pad=0)
    for axis in [ax.xaxis,ax.yaxis,ax.zaxis]:
        axis.pane.set_facecolor((1,1,1,0));axis.pane.set_edgecolor((1,1,1,0))
    ax.zaxis._axinfo['juggled']=(1,2,0)

def filled(ax,region,color):
    v=np.asarray(region['vertices'])[:,[1,0,2]];f=np.asarray(region['faces'],int)-1
    assert v.shape==(8,3) and f.shape==(6,4) and region['valid']
    ax.add_collection3d(Poly3DCollection(v[f],facecolors=color,alpha=.22,edgecolors='#333',linewidths=.6))

def main():
    ap=argparse.ArgumentParser();ap.add_argument('results_dir',type=Path);a=ap.parse_args()
    data=[]
    for angle in [0,3.5,7]:
        p=a.results_dir/f'stable_point_40kph_0.85mu_{angle:g}deg.mat'
        if p.exists():data.append((angle,loadmat(p,simplify_cells=True)))
    if not data:raise SystemExit('No completed MATs')
    fig=plt.figure(figsize=(14.5,5.5),facecolor='white')
    fig.subplots_adjust(left=.015,right=.98,bottom=.20,top=.81,wspace=.02)
    for i,(angle,s) in enumerate(data):
        ax=fig.add_subplot(1,3,i+1,projection='3d');r=s['PAPER_STABILITY3D']['result'];region=s['INNER_REGION']
        axes(ax);scatter(ax,r['stable_points'],COLORS[i]);ax.scatter([0],[0],[0],s=45,c='white',edgecolors='black',marker='*',depthshade=False)
        if region['valid']:filled(ax,region,COLORS[i]);status=f"Accepted inner V={region['volume']:.4g}"
        else:status='No validated inner hexahedron'
        p=np.asarray(r['stable_points']).reshape(-1,3);rb=f"[{p[:,2].min():.3f}, {p[:,2].max():.3f}]" if len(p) else 'empty'
        ax.set_title(f"{angle:g} deg | {r['stable_count']:,} stable\nRaw roll {rb} rad\n{status}",fontsize=10,pad=4)
    fig.suptitle('Printed Eq. (11) + original MF coefficients | 40 km/h fixed, '+r'$\mu=0.85$',fontsize=15,y=.96)
    fig.text(.5,.125,'Dots: actual five-second stable initial-state samples | Star: initial origin | Filled faces: validated inner candidate only',ha='center',fontsize=10)
    fig.text(.5,.075,'Roll grid remains +/-0.4 rad; touching its ends means the physical roll boundary has not been found.',ha='center',fontsize=10,color='#6b3f18')
    fig.text(.5,.03,'No dynamic roll-reaction subtraction, maximum-volume objective, equilibrium anchor, roll caps or forced nesting.',ha='center',fontsize=9,color='#444')
    fig.savefig(a.results_dir/'eq11_baseline_comparison.png',dpi=180);plt.close(fig)
    fig=plt.figure(figsize=(16,12),facecolor='white');fig.subplots_adjust(left=.02,right=.98,bottom=.07,top=.93,wspace=.03,hspace=.21)
    columns=['Raw stable samples','Step 1: roll extrema','Step 2: inner lateral extrema','Step 3: extreme-point candidate']
    for row,(angle,s) in enumerate(data):
        sec=s['SECTION31'];r=s['PAPER_STABILITY3D']['result'];reg=s['INNER_REGION']
        for col,key in enumerate(['stable_points','limit_roll_points','inner_limit_points','candidate_vertices']):
            ax=fig.add_subplot(3,4,row*4+col+1,projection='3d');axes(ax)
            if col==0:p=r[key]
            elif col<3:p=sec[key]
            else:p=reg[key]
            scatter(ax,p,COLORS[row],1800)
            if col==3:
                if reg['valid']:filled(ax,reg,COLORS[row])
                else:
                    p=np.asarray(p).reshape(-1,3)
                    if len(p):ax.scatter(p[:,1],p[:,0],p[:,2],s=25,c='#a00000',marker='x')
            subtitle=('accepted' if reg['valid'] else 'rejected / unavailable') if col==3 else ''
            ax.set_title(f'{angle:g} deg | {columns[col]}\n{subtitle}',fontsize=9,pad=1)
    fig.suptitle('Section 3.1 sequence with declared discrete branch/corner rules',fontsize=16,y=.98)
    fig.text(.5,.025,'Candidate rejection and grid censoring remain visible; an invalid candidate is never replaced by a different geometry.',ha='center',fontsize=11)
    fig.savefig(a.results_dir/'section31_extraction_stages.png',dpi=150);plt.close(fig)
    print('Rendered actual baseline and extraction stages for',len(data),'conditions')

if __name__=='__main__':main()
