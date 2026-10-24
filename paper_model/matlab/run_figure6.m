% Paper Figure 6 reproduction attempt with the ORIGINAL 41x31x31 grid.
% Figure 6 published conditions: 40 km/h, front steering 0, 3.5, 7 deg.
% Fig. 6 friction value not explicitly stated: mu=0.85 is ASSUMED
% from the paper's Fig. 2 example, not a certified Fig. 6 value.
clear; clc; close all;
root=fileparts(mfilename('fullpath'));
addpath(root);
save_format='-v7.3';
if exist('OCTAVE_VERSION','builtin'), save_format='-v7'; end
cfg0=paper_config();
angles=[0 3.5 7];
cases=struct([]);
outdir=fullfile(root,'results_figure6');
if ~exist(outdir,'dir')
    mkdir(outdir);
end
for k=1:numel(angles)
    cfg=cfg0;
    cfg.speed_kmh=40;
    cfg.mu=0.85;
    cfg.steer_deg=angles(k);
    fprintf('\n===== Scenario %d/%d: speed=%g km/h mu=%g steer=%g deg =====\n',k,numel(angles),cfg.speed_kmh,cfg.mu,cfg.steer_deg);
    result=paper_simulate_grid(cfg);
    region=paper_build_region(cfg,result.stable_mask);
    scenario=struct('cfg',cfg,'result',result,'region',region);
    if isempty(cases)
        cases=scenario;
    else
        cases(k)=scenario;
    end
    file=sprintf('paper_stable_point_%gkph_%gmu_%gdeg.mat',cfg.speed_kmh,cfg.mu,cfg.steer_deg);
    PAPER_STABILITY3D=scenario;
    save(fullfile(outdir,file),'PAPER_STABILITY3D',save_format);
    fprintf('   stable %d / %d | inner region valid=%d | volume=%.8g | unstable grid points inside=%d\n',result.stable_count,result.total_count,region.valid,region.volume,region.grid_unstable_inside);
end
save(fullfile(outdir,'figure6_comparison.mat'),'cases',save_format);
paper_plot_region(cases,true);
