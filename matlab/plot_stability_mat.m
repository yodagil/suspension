function h = plot_stability_mat(mat_file,case_index)
%PLOT_STABILITY_MAT Plot saved stable samples and the selected inner region.
% Example: plot_stability_mat('stable_point_80kph_0.9mu_2deg.mat')
% This viewer does not rerun vehicle simulations.
if nargin<2, case_index=1; end
D=load(mat_file,'STABILITY3D'); S=D.STABILITY3D;
assert(case_index>=1 && case_index<=numel(S.CASE),'Invalid case index.');
C=S.CASE(case_index); wear=C.wear(:).';
h=figure('Color','w'); ax=axes('Parent',h); hold(ax,'on');
P=C.stable_points;
scatter3(ax,P(:,2),P(:,3),P(:,1),7,[.25 .40 .65],'filled', ...
    'DisplayName','Stable initial conditions');
R=C.region;
if R.is_valid_inner && isfinite(R.inner_volume) && size(R.inner_vertices,1)>=4
    V=R.inner_vertices; K=convhulln(V);
    trisurf(K,V(:,2),V(:,3),V(:,1),'Parent',ax, ...
        'FaceColor',[.1 .7 .35],'FaceAlpha',.28,'EdgeColor',[.05 .35 .15], ...
        'DisplayName','Sample-validated inner region');
else
    % warning() requires scalar numeric formatting arguments, unlike a row vector.
    warning('Case %d [%g %g %g %g]: no valid inner region to plot.', ...
        case_index,wear(1),wear(2),wear(3),wear(4));
end
xlabel(ax,'r_0 [rad/s]'); ylabel(ax,'phi_0 [rad]'); zlabel(ax,'vc_0 [m/s]');
title(ax,sprintf('%g km/h, mu=%g, wheel steer=%g deg, case %d', ...
    S.meta.speed_kmh,S.meta.mu_road,S.meta.wheel_steer_deg(1),case_index));
grid(ax,'on'); view(ax,3); legend(ax,'show','Location','best');
end
