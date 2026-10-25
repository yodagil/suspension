#!/usr/bin/env python3
"""Plot measured inner polytopes without changing any saved coordinates."""
from pathlib import Path
import argparse, json, os
os.environ.setdefault('MPLCONFIGDIR', '/tmp/mf210-mpl')
import numpy as np
from scipy.spatial import ConvexHull
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Patch
from mpl_toolkits.mplot3d.art3d import Poly3DCollection, Line3DCollection


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('summary', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    data = json.loads(args.summary.read_text())
    assert data['nesting_constraints_applied'] is False
    colors = ['#a4eba8', '#f3ed32', '#173ce5']
    fig = plt.figure(figsize=(7.1, 7.1), facecolor='white')
    ax = fig.add_axes([.08, .16, .88, .78], projection='3d')
    ax.set_proj_type('ortho')
    faces, facecolors, all_vertices = [], [], []
    for case, color, opacity in zip(data['cases'], colors, [.16, .24, .65]):
        assert case['valid']
        original = np.asarray(case['vertices'])
        v = original[:, [1, 0, 2]]
        hull = ConvexHull(v)
        assert abs(hull.volume-case['volume']) < 1e-9
        faces.extend(v[hull.simplices])
        facecolors.extend([matplotlib.colors.to_rgba(color, opacity)]*len(hull.simplices))
        adjacent = {}
        for i, tri in enumerate(hull.simplices):
            for a, b in zip(tri, np.roll(tri, -1)):
                adjacent.setdefault(tuple(sorted((int(a), int(b)))), []).append(i)
        edges = []
        for pair, indices in adjacent.items():
            if len(indices) == 2 and np.max(np.abs(hull.equations[indices[0]]-hull.equations[indices[1]])) < 1e-8:
                continue
            edges.append(v[list(pair)])
        ax.add_collection3d(Line3DCollection(edges, colors=('#173ce5' if case['angle_deg']==7 else '#3e493e'), linewidths=.95, alpha=.9))
        all_vertices.append(original)
    ax.add_collection3d(Poly3DCollection(faces, facecolors=facecolors, edgecolor='none', zsort='average'))
    vcall = np.concatenate(all_vertices)[:, 0]
    yrange = max(5., float(np.ceil(np.max(np.abs(vcall)))))
    ax.set(xlim=(.72, -.72), ylim=(-yrange, yrange), zlim=(-.45, .45))
    ax.set_xticks([.6, .4, .2, 0, -.2, -.4, -.6])
    ax.set_yticks([-yrange, 0, yrange])
    ax.set_zticks([-.4, -.2, 0, .2, .4])
    ax.set_xlabel(r'$r$ (rad/s)', labelpad=9)
    ax.set_ylabel(r'$v_c$ (m/s)', labelpad=8)
    ax.set_zlabel(r'$\phi$ (rad)', labelpad=8)
    ax.view_init(elev=11, azim=-72)
    ax.set_box_aspect((1.3, 2.28, 1.65*.9/.84))
    ax.grid(False)
    for axis in [ax.xaxis, ax.yaxis, ax.zaxis]:
        axis.pane.set_facecolor((1, 1, 1, 0))
        axis.pane.set_edgecolor((1, 1, 1, 0))
    ax.zaxis._axinfo['juggled'] = (1, 2, 0)
    ax.tick_params(labelsize=8, pad=1)
    ax.legend(handles=[Patch(facecolor=c, edgecolor='#444', label=r'$\delta=%s^\circ$'%d, alpha=.55)
                       for c,d in zip(colors, [0, 3.5, 7])], loc='upper left', bbox_to_anchor=(.08,.98), frameon=True, edgecolor='#333', fontsize=11)
    fig.text(.5, .972, '40 km/h fixed, '+r'$\mu=0.85$'+' | healthy original MF210 tires', ha='center', va='top', fontsize=12)
    fig.text(.5, .103, 'Paper body + original MF tire dynamics; independent inner regions', ha='center', fontsize=9, color='#444')
    fig.text(.5, .080, '138,105 initial states / angle; 5 s; zero unstable grid nodes inside', ha='center', fontsize=8, color='#555')
    fig.text(.5, .057, 'No containment constraints. Natural containment: '+('YES' if data['naturally_nested'] else 'NOT ACHIEVED'), ha='center', fontsize=9, color=('#206830' if data['naturally_nested'] else '#9a3c22'))
    fig.text(.5, .033, 'Finite-grid inner approximations; paper Figure 6 is not reproduced exactly.', ha='center', fontsize=8, color='#555')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=180, facecolor='white')
    plt.close(fig)
    print(args.output)


if __name__ == '__main__':
    main()
