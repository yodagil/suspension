#!/usr/bin/env python3
"""Shared camera, actual independent MF regions; no coordinate corrections."""
from pathlib import Path
import argparse,json,os
os.environ.setdefault('MPLCONFIGDIR','/tmp/mf210-final-mpl')
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Patch
from mpl_toolkits.mplot3d.art3d import Poly3DCollection,Line3DCollection
from scipy.spatial import ConvexHull

def main():
    ap=argparse.ArgumentParser();ap.add_argument('summary',type=Path);ap.add_argument('output',type=Path)
    ap.add_argument('--vc-limit',type=float,default=2);a=ap.parse_args()
    d=json.loads(a.summary.read_text());assert d['all_cases_valid']
    assert not d['nesting_constraints_applied'] and not d['coordinate_correction_applied']
    colors=['#a1e7ac','#f0ee31','#183fe5'];polys=[];facecolors=[]
    fig=plt.figure(figsize=(7.2,6.2),facecolor='white')
    ax=fig.add_axes([.04,.13,.94,.82],projection='3d');ax.set_proj_type('ortho')
    for c,col,alpha in zip(d['cases'],colors,[.26,.32,.58]):
        orig=np.array(c['vertices']);v=orig[:,[1,0,2]]
        h=ConvexHull(v);assert len(h.vertices)==8 and np.isclose(h.volume,c['volume'],rtol=1e-12)
        quads=np.array(c['faces'])-1;assert quads.shape==(6,4)
        polys.extend(v[quads]);facecolors.extend([matplotlib.colors.to_rgba(col,alpha)]*6)
        edges=set()
        for q in quads:
            for i,j in zip(q,np.roll(q,-1)):edges.add(tuple(sorted((i,j))))
        assert len(edges)==12
        ax.add_collection3d(Line3DCollection([v[list(e)] for e in edges],
            colors=('#1838a4' if c['angle_deg']==7 else '#454e45'),linewidths=.9,alpha=.9))
    ax.add_collection3d(Poly3DCollection(polys,facecolors=facecolors,edgecolor='none',zsort='average'))
    ax.set(xlim=(.72,-.72),ylim=(-a.vc_limit,a.vc_limit),zlim=(-.3,.3))
    ax.set_xticks([.6,.4,.2,0,-.2,-.4,-.6]);ax.set_yticks([-a.vc_limit,0,a.vc_limit])
    ax.set_zticks([-.3,-.2,-.1,0,.1,.2,.3]);ax.tick_params(labelsize=9,pad=1)
    ax.set_xlabel(r'$r$ (rad/s)',labelpad=8);ax.set_ylabel(r'$v_c$ (m/s)',labelpad=7)
    ax.set_zlabel(r'$\phi$ (rad)',labelpad=7)
    ax.view_init(elev=11,azim=-72);ax.set_box_aspect((1.3,2.28,1.65*.6/.84));ax.grid(False)
    for axis in [ax.xaxis,ax.yaxis,ax.zaxis]:
        axis.pane.set_facecolor((1,1,1,0));axis.pane.set_edgecolor((1,1,1,0))
    ax.zaxis._axinfo['juggled']=(1,2,0)
    handles=[Patch(facecolor=c,edgecolor='#444',alpha=.65,label=rf'$\delta={s:g}^\circ$')
        for c,s in zip(colors,[0,3.5,7])]
    ax.legend(handles=handles,loc='upper left',bbox_to_anchor=(.05,1.00),fontsize=11,
        edgecolor='#444',framealpha=1)
    fig.text(.5,.975,r'40 km/h fixed, $\mu=0.85$ | Original MF coefficients',
        fontsize=13,ha='center',va='top')
    spans=' > '.join(f"{c['roll_span']:.3f}" for c in d['cases'])
    fig.text(.5,.084,f'Selected inner roll spans: {spans} rad',ha='center',fontsize=10)
    fig.text(.5,.050,'Independent stable-cell inner regions; no forced nesting',ha='center',fontsize=9,color='#444')
    fig.text(.5,.022,'MF approximation with dynamic roll reaction; exact paper reproduction not established',
        ha='center',fontsize=8,color='#555')
    a.output.parent.mkdir(parents=True,exist_ok=True);fig.savefig(a.output,dpi=200,facecolor='white')
    fig.savefig(a.output.with_suffix('.svg'),facecolor='white');plt.close(fig)
    print(a.output)

if __name__=='__main__':main()
