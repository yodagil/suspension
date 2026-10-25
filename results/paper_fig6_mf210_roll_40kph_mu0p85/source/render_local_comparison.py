#!/usr/bin/env python3
"""Paper-camera comparison of unchanged independently optimized cuboids."""
from pathlib import Path
import argparse,json,os
os.environ.setdefault('MPLCONFIGDIR','/tmp/mf210-roll-mpl')
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Patch
from scipy.spatial import ConvexHull
from mpl_toolkits.mplot3d.art3d import Poly3DCollection,Line3DCollection


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('summary',type=Path);ap.add_argument('output',type=Path)
    a=ap.parse_args();data=json.loads(a.summary.read_text())
    assert data['all_cases_valid'] and not data['nesting_constraints_applied']
    assert not data['roll_scaling_or_caps_applied']
    assert data['strict_roll_span_narrowing_for_every_maximum_volume_tie']
    colors=['#a4eba8','#f3ed32','#173ce5'];faces=[];facecolors=[]
    fig=plt.figure(figsize=(7.3,7.3),facecolor='white')
    ax=fig.add_axes([.08,.15,.88,.79],projection='3d');ax.set_proj_type('ortho')
    for case,color,opacity in zip(data['cases'],colors,[.18,.28,.65]):
        v0=np.asarray(case['vertices']);v=v0[:,[1,0,2]]
        h=ConvexHull(v);assert len(h.vertices)==8 and abs(h.volume-case['volume'])<1e-10
        quads=np.asarray(case['faces'],int)-1
        assert len(quads)==6
        faces.extend(v[quads]);facecolors.extend([matplotlib.colors.to_rgba(color,opacity)]*6)
        edges=set()
        for q in quads:
            for p,s in zip(q,np.roll(q,-1)):edges.add(tuple(sorted((int(p),int(s)))))
        assert len(edges)==12
        ax.add_collection3d(Line3DCollection([v[list(e)] for e in edges],
             colors=('#173ce5' if case['angle_deg']==7 else '#435047'),linewidths=1.1,alpha=.9))
    ax.add_collection3d(Poly3DCollection(faces,facecolors=facecolors,edgecolor='none',zsort='average'))
    ax.set(xlim=(.72,-.72),ylim=(-2,2),zlim=(-.3,.3))
    ax.set_xticks([.6,.4,.2,0,-.2,-.4,-.6]);ax.set_yticks([-2,0,2]);ax.set_zticks([-.3,-.2,-.1,0,.1,.2,.3])
    ax.set_xlabel(r'$r$ (rad/s)',labelpad=9);ax.set_ylabel(r'$v_c$ (m/s)',labelpad=8)
    ax.set_zlabel(r'$\phi$ (rad)',labelpad=8)
    ax.view_init(elev=11,azim=-72);ax.set_box_aspect((1.3,2.28,1.65*.6/.84));ax.grid(False)
    for axis in [ax.xaxis,ax.yaxis,ax.zaxis]:
        axis.pane.set_facecolor((1,1,1,0));axis.pane.set_edgecolor((1,1,1,0))
    ax.zaxis._axinfo['juggled']=(1,2,0);ax.tick_params(labelsize=8,pad=1)
    labels=[Patch(facecolor=c,edgecolor='#444',label=r'$\delta=%s^\circ$'%d,alpha=.6)
            for c,d in zip(colors,[0,3.5,7])]
    ax.legend(handles=labels,loc='upper left',bbox_to_anchor=(.08,.98),frameon=True,edgecolor='#333',fontsize=11)
    fig.text(.5,.974,'40 km/h fixed, '+r'$\mu=0.85$'+' | MF210 coefficients',ha='center',va='top',fontsize=13)
    inset=fig.add_axes([.735,.765,.215,.16])
    for i,(case,col) in enumerate(zip(data['cases'],['#389d4a','#b7a500','#173ce5'])):
        lo,hi=case['roll_bounds'];inset.vlines(i,lo,hi,color=col,lw=7,alpha=.8)
        inset.hlines([lo,hi],i-.16,i+.16,color=col,lw=1)
        inset.text(i,.225,f'{hi-lo:.3f}',ha='center',fontsize=7,color=col)
    inset.set(xlim=(-.5,2.5),ylim=(-.23,.26));inset.set_xticks([0,1,2],['0','3.5','7'])
    inset.set_yticks([-.2,0,.2]);inset.tick_params(labelsize=7,pad=1)
    inset.axhline(0,color='#aaa',lw=.6,linestyle='--')
    inset.set_xlabel('Steer (deg)',fontsize=7,labelpad=1)
    inset.set_ylabel(r'$\phi$ (rad)',fontsize=7,labelpad=1)
    inset.set_title('Selected inner roll bounds / span',fontsize=7,pad=3)
    for sp in ['top','right']:inset.spines[sp].set_visible(False)
    fig.text(.5,.105,'Quasisteady MF + dynamic roll reaction; local inner hexahedra',ha='center',fontsize=9,color='#444')
    fig.text(.5,.080,'138,105 initial states / angle; 5 s; no steering-dependent roll caps or nesting constraints',ha='center',fontsize=8,color='#555')
    fig.text(.5,.055,'Roll spans narrow: 0.347 > 0.293 > 0.160 rad (selected inner regions).',ha='center',fontsize=8,color='#246234')
    fig.text(.5,.033,'Different load model and tire law from the paper; exact Figure 6 reproduction is not established.',ha='center',fontsize=7.5,color='#555')
    a.output.parent.mkdir(parents=True,exist_ok=True);fig.savefig(a.output,dpi=190,facecolor='white');plt.close(fig)
    print(a.output)


if __name__=='__main__':main()
