% 210-condition full-resolution MATLAB simulation.
% Put all files from the archive in the same directory, then run this file.
% This entry point uses pure MATLAB M-code; the cloud uses the C++ accelerator.
% Output: results_mcode_reference09_direct_start/stable_point_80kph_0.9mu_0deg.mat, etc.

code_dir = fileparts(mfilename('fullpath'));
addpath(code_dir);
options = struct();
options.engine = 'mcode';
options.speeds = [50 60 70 80 90 100];          % km/h
options.mus = [0.9 0.8 0.7 0.6 0.5];
options.steers = [0 0.5 1 1.5 2 2.5 3];       % front wheel angle, degrees
options.output_dir = fullfile(code_dir,'results_mcode_reference09_direct_start');
manifest = run_stability_sweep(options);
