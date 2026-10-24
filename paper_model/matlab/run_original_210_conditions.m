% OPTIONAL: 210 environmental scenarios with the old speed/mu/steer vectors.
% Dynamics and tire are still the Xiao 2022 reconstruction, NOT old MF.
% VERY EXPENSIVE: 210 x 39,401 initial-state simulations.
clear; clc; close all;
root=fileparts(mfilename('fullpath'));
addpath(root);
save_format='-v7.3';
if exist('OCTAVE_VERSION','builtin'), save_format='-v7'; end
cfg0=paper_config();
speeds=[50 60 70 80 90 100];
mus=[0.9 0.8 0.7 0.6 0.5];
steers=[0 0.5 1 1.5 2 2.5 3];
outdir=fullfile(root,'results_210');
if ~exist(outdir,'dir')
    mkdir(outdir);
end
N=numel(speeds)*numel(mus)*numel(steers);
k=0;
for speed=speeds
    for mu=mus
        for steer=steers
            k=k+1;
            cfg=cfg0;
            cfg.speed_kmh=speed;
            cfg.mu=mu;
            cfg.steer_deg=steer;
            filename=sprintf('paper_stable_point_%gkph_%.1fmu_%gdeg.mat',speed,mu,steer);
            filepath=fullfile(outdir,filename);
            if exist(filepath,'file')
                fprintf('%d/%d SKIP EXISTING %s\n',k,N,filename);
                continue;
            end
            fprintf('\n%d/%d %s\n',k,N,filename);
            result=paper_simulate_grid(cfg);
            region=paper_build_region(cfg,result.stable_mask);
            PAPER_STABILITY3D=struct('cfg',cfg,'result',result,'region',region);
            save(filepath,'PAPER_STABILITY3D',save_format);
            fprintf('SAVE %s | stable %d | inner-volume %.8g\n',filename,result.stable_count,region.volume);
        end
    end
end
