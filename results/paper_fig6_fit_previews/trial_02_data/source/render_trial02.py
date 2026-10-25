#!/usr/bin/env python3
"""Render actual inner-region coordinates with the common paper-like view."""
from pathlib import Path
import os,json
os.environ['MPLCONFIGDIR']='/workspace/.matlab-env/matplotlib-cache'
import numpy as np
from scipy.io import loadmat
from scipy.spatial import ConvexHull
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Patch
from mpl_toolkits.mplot3d.art3d import Poly3DCollection,Line3DCollection
ROOT=Path('/workspace/suspension')
OUT=ROOT/'results/paper_fig6_fit_previews'
source=OUT/'trial_02_data'
colors=['#a4eba8','#f3ed32','#173ce5']
fig=plt.figure(figsize=(6.8,6.8),facecolor='white')
ax=fig.add_axes([.08,.14,.88,.80],projection='3d');ax.set_proj_type('ortho')
for angle,color in zip([0,3.5,7],colors):
    s=loadmat(source/f'paper_dynamic_roll_40kph_0.85mu_{angle:g}deg.mat',simplify_cells=True)['PAPER_STABILITY3D']
    reg=s['region'];v=np.asarray(reg['vertices'])[:,[1,0,2]];h=ConvexHull(v)
    assert reg['valid'] and len(h.vertices)==8
    assert abs(h.volume-reg['volume'])<1e-9
    ax.add_collection3d(Poly3DCollection(v[h.simplices],facecolor=color,edgecolor='none',alpha=.25))
    edge_faces={}
    for i,tri in enumerate(h.simplices):
        for a,b in [(tri[0],tri[1]),(tri[1],tri[2]),(tri[2],tri[0])]:
            edge_faces.setdefault(tuple(sorted((int(a),int(b)))),[]).append(i)
    lines=[]
    for edge,faces in edge_faces.items():
        if len(faces)==2:
            q,t=h.equations[faces]
            if np.max(abs(q-t))<1e-8:continue
        lines.append(v[list(edge)])
    ax.add_collection3d(Line3DCollection(lines,colors='#3e493e',linewidths=.85,alpha=.85))
ax.set_xlim(.72,-.72);ax.set_ylim(-10,10);ax.set_zlim(-.3,.3)
ax.set_xticks([.6,.4,.2,0,-.2,-.4,-.6]);ax.set_yticks([-10,0,10]);ax.set_zticks([-.3,-.2,-.1,0,.1,.2,.3])
ax.set_xlabel(r'$r$ (rad/s)',labelpad=9);ax.set_ylabel(r'$v_c$ (m/s)',labelpad=8);ax.set_zlabel(r'$\phi$ (rad)',labelpad=8)
ax.view_init(elev=11,azim=-72);ax.set_box_aspect((1.3,2.28,1.65*.6/.84));ax.grid(False)
for axis in [ax.xaxis,ax.yaxis,ax.zaxis]:
    axis.pane.set_facecolor((1,1,1,0));axis.pane.set_edgecolor((1,1,1,0))
ax.zaxis._axinfo['juggled']=(1,2,0)
ax.tick_params(axis='both',labelsize=8,pad=1)
legend=[Patch(facecolor=c,edgecolor='#444',label=r'$\delta=%s^\circ$'%a,alpha=.55) for c,a in zip(colors,[0,3.5,7])]
ax.legend(handles=legend,loc='upper left',bbox_to_anchor=(.08,.96),frameon=True,edgecolor='#333',fontsize=11)
fig.text(.5,.965,'40 km/h fixed, '+r'$\mu=0.85$',ha='center',va='top',fontsize=13)
fig.text(.5,.073,'Trial 02: dynamic roll-load extension; conservative inner regions',ha='center',fontsize=8.5,color='#444')
fig.text(.5,.047,'69,905 initial states / angle; 5 s; not literal paper Eq. (11)',ha='center',fontsize=8,color='#555')
fig.savefig(OUT/'trial_02.png',dpi=170,facecolor='white');plt.close(fig)
meta=json.loads((OUT/'trial_02.json').read_text())
meta['projection']=dict(kind='common orthographic camera for all three angles',elevation_deg=11,
    azimuth_deg=-72,r_limits=[.72,-.72],vc_limits=[-10,10],phi_limits=[-.3,.3],
    paper_axis_unit_ratios_approximately_preserved=True,all_vertices_visible=True,
    per_angle_coordinate_scaling_or_translation=False)
(OUT/'trial_02.json').write_text(json.dumps(meta,indent=2)+'\n')
print(OUT/'trial_02.png')
