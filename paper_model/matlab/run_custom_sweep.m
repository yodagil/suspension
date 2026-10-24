% Independent Xiao2022 model. Keeps grid identical to the previous archive.
% Edit the numeric scenario vectors below. All angles are FRONT TIRE angles.
clear; clc; close all;
root=fileparts(mfilename('fullpath'));
addpath(root);
save_format='-v7.3';
if exist('OCTAVE_VERSION','builtin'), save_format='-v7'; end
cfg0=paper_config();
speeds=[40 60 80];
mus=[0.85];
steers=[0 3.5 7];
outdir=fullfile(root,'results_custom');
if ~exist(outdir,'dir')
    mkdir(outdir);
end
manifest=struct('file',{},'speed_kmh',{},'mu',{},'steer_deg',{},'stable_count',{},'inner_volume',{},'valid',{});
k=0;
for speed=speeds
    for mu=mus
        for steer=steers
            k=k+1;
            cfg=cfg0;
            cfg.speed_kmh=speed;
            cfg.mu=mu;
            cfg.steer_deg=steer;
            fprintf('\nCUSTOM %d | %g km/h mu %g delta %g deg\n',k,speed,mu,steer);
            result=paper_simulate_grid(cfg);
            region=paper_build_region(cfg,result.stable_mask);
            PAPER_STABILITY3D=struct('cfg',cfg,'result',result,'region',region);
            filename=sprintf('paper_stable_point_%gkph_%gmu_%gdeg.mat',speed,mu,steer);
            save(fullfile(outdir,filename),'PAPER_STABILITY3D',save_format);
            manifest(k).file=filename;
            manifest(k).speed_kmh=speed;
            manifest(k).mu=mu;
            manifest(k).steer_deg=steer;
            manifest(k).stable_count=result.stable_count;
            manifest(k).inner_volume=region.volume;
            manifest(k).valid=region.valid;
        end
    end
end
save(fullfile(outdir,'manifest.mat'),'manifest',save_format);
for k=1:numel(manifest)
    fprintf('%s | stable %d | volume %.9g | valid %d\n',manifest(k).file,manifest(k).stable_count,manifest(k).inner_volume,manifest(k).valid);
end
