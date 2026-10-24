% Paper Figure 2's explicitly stated setup: u=80 km/h, delta=0, mu=0.85.
clear; clc; close all;
root=fileparts(mfilename('fullpath'));
addpath(root);
save_format='-v7.3';
if exist('OCTAVE_VERSION','builtin'), save_format='-v7'; end
cfg=paper_config();
cfg.speed_kmh=80;
cfg.mu=0.85;
cfg.steer_deg=0;
result=paper_simulate_grid(cfg);
region=paper_build_region(cfg,result.stable_mask);
PAPER_STABILITY3D=struct('cfg',cfg,'result',result,'region',region);
outdir=fullfile(root,'results_figure2');
if ~exist(outdir,'dir')
    mkdir(outdir);
end
save(fullfile(outdir,'paper_figure2_80kph_0p85mu_0deg.mat'),'PAPER_STABILITY3D',save_format);
figure('Color','w');
if ~isempty(result.stable_points)
    scatter3(result.stable_points(:,1),result.stable_points(:,2),result.stable_points(:,3),6,[0.2 0.4 0.9],'filled');
end
xlabel('v_c (m/s)');ylabel('r (rad/s)');zlabel('\phi (rad)');
grid on;view(45,20);
title(sprintf('Fig. 2 stated case: %d/%d stable grid states',result.stable_count,result.total_count));
