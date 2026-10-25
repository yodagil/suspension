#!/usr/bin/env python3
"""Common-camera plot of unscaled eight-vertex inner-region coordinates."""
from pathlib import Path
import argparse,json,os
os.environ['MPLCONFIGDIR']='/workspace/.matlab-env/matplotlib-cache'
import numpy as np
from scipy.io import loadmat
from scipy.spatial import ConvexHull
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Patch
from mpl_toolkits.mplot3d.art3d import Poly3DCollection,Line3DCollection

def main():
    ap=argparse.ArgumentParser();ap.add_argument('regions',type=Path);ap.add_argument('output',type=Path)
    ap.add_argument('--coarse',action='store_true');a=ap.parse_args()
    data=json.loads(a.regions.read_text())
    colors=['#a4eba8','#f3ed32','#173ce5']
    fig=plt.figure(figsize=(6.8,6.8),facecolor='white')
    all_faces=[];all_face_colors=[]
    ax=fig.add_axes([.08,.14,.88,.80],projection='3d');ax.set_proj_type('ortho')
    for c,color in zip(data['cases'],colors):
        assert c['valid'];v=np.asarray(c['vertices'])[:,[1,0,2]];h=ConvexHull(v)
        assert len(h.vertices)==8 and abs(h.volume-c['volume'])<1e-9
        opacity={0:.16,3.5:.24,7:.65}[c['angle_deg']]
        all_faces.extend(v[h.simplices])
        all_face_colors.extend([matplotlib.colors.to_rgba(color,opacity)]*len(h.simplices))
        edges={}
        for i,t in enumerate(h.simplices):
            for p,q in [(t[0],t[1]),(t[1],t[2]),(t[2],t[0])]:edges.setdefault(tuple(sorted((int(p),int(q)))),[]).append(i)
        lines=[]
        for e,fs in edges.items():
            if len(fs)==2:
                p,q=h.equations[fs]
                if np.max(abs(p-q))<1e-8:continue
            lines.append(v[list(e)])
        ax.add_collection3d(Line3DCollection(lines,colors=('#173ce5' if c['angle_deg']==7 else '#3e493e'),linewidths=.95,alpha=.85))
    ax.add_collection3d(Poly3DCollection(all_faces,facecolors=all_face_colors,edgecolor='none',zsort='average'))
    ax.set_xlim(.72,-.72);ax.set_ylim(-10,10);ax.set_zlim(-.45,.45)
    ax.set_xticks([.6,.4,.2,0,-.2,-.4,-.6]);ax.set_yticks([-10,0,10]);ax.set_zticks([-.4,-.2,0,.2,.4])
    ax.set_xlabel(r'$r$ (rad/s)',labelpad=9);ax.set_ylabel(r'$v_c$ (m/s)',labelpad=8);ax.set_zlabel(r'$\phi$ (rad)',labelpad=8)
    ax.view_init(elev=11,azim=-72);ax.set_box_aspect((1.3,2.28,1.65*.9/.84));ax.grid(False)
    for axis in [ax.xaxis,ax.yaxis,ax.zaxis]:
        axis.pane.set_facecolor((1,1,1,0));axis.pane.set_edgecolor((1,1,1,0))
    ax.zaxis._axinfo['juggled']=(1,2,0);ax.tick_params(axis='both',labelsize=8,pad=1)
    legend=[Patch(facecolor=c,edgecolor='#444',label=r'$\delta=%s^\circ$'%d,alpha=.55) for c,d in zip(colors,[0,3.5,7])]
    ax.legend(handles=legend,loc='upper left',bbox_to_anchor=(.08,.96),frameon=True,edgecolor='#333',fontsize=11)
    fig.text(.5,.965,'40 km/h fixed, '+r'$\mu=0.85$',ha='center',va='top',fontsize=13)
    fig.text(.5,.073,'Trial 03: quasistatic Eq. (11); shape-constrained inner regions',ha='center',fontsize=9,color='#444')
    label='Screening grid' if a.coarse else '69,905 states / angle; 5 s; not the full stable initial-state set'
    fig.text(.5,.047,label,ha='center',fontsize=8,color='#555')
    inset=fig.add_axes([.72,.755,.23,.155])
    for j,(case,col) in enumerate(zip(data['cases'],['#389d4a','#b8a900','#173ce5'])):
        verts=np.asarray(case['vertices']);lo=float(verts[:,2].min());hi=float(verts[:,2].max())
        inset.vlines(j,lo,hi,colors=col,linewidth=7,alpha=.7)
        inset.hlines([lo,hi],j-.16,j+.16,colors=col,linewidth=1.1)
    inset.set_xlim(-.5,2.5);inset.set_ylim(-.45,.45)
    inset.set_xticks([0,1,2],['0','3.5','7']);inset.set_yticks([-.4,0,.4])
    inset.tick_params(labelsize=7,pad=1)
    inset.axhline(0,color='#aaa',linewidth=.6,linestyle='--')
    inset.set_xlabel('Steer (deg)',fontsize=7,labelpad=1)
    inset.set_ylabel(r'$\phi$ (rad)',fontsize=7,labelpad=1)
    inset.set_title('Selected inner roll bounds',fontsize=7,pad=3)
    for sp in ['top','right']:inset.spines[sp].set_visible(False)
    a.output.parent.mkdir(parents=True,exist_ok=True);fig.savefig(a.output,dpi=170,facecolor='white');plt.close(fig)
    print(a.output)

if __name__=='__main__':main()
