function R=run_shift_study_2d(mode)
%RUN_SHIFT_STUDY_2D Named study configurations. No file picker is opened.
%   run_shift_study_2d('selftest')   scalar/vector RHS and initialization checks
%   run_shift_study_2d('handling')   2-D local handling diagnostic only (no 3-D sweep)
%   run_shift_study_2d('pilot')      80 km/h, mu=.9, -2/0/+2 deg, healthy tires
%   run_shift_study_2d('wear')       pilot conditions, all FIVE wear cases
%   run_shift_study_2d('full')       full 210 conditions x five wear cases x 1,271 initial vc/r points
%   run_shift_study_2d('ablation')   compare three initialization methods
%   run_shift_study_2d('robust')     test nonzero hidden roll-rate states
%   run_shift_study_2d('refined')    finer spatial mesh on pilot conditions
%   run_shift_study_2d('halfstep')   original pilot mesh with half integration step
%
% Full mode is intentionally NOT the default. It is computationally large.
% A MATLAB selftest pass and pilot inspection should precede a full sweep.
if nargin<1, mode='pilot'; end
if strcmpi(mode,'selftest'), R=stability_shift_cdc_2d('selftest'); return; end
o=stability_shift_cdc_2d('defaults');
root=fileparts(mfilename('fullpath'));
o.output_dir=fullfile(root,['results_2d_' lower(mode)]);
o.horizons=[5 10]; % stores two independent horizon judgments in one integration
allwear=[0 0 0 0;25 25 0 0;50 50 25 25;0 0 25 25;25 25 50 50];
switch lower(mode)
 case 'handling'
    o.action='handling'; o.wear_cases=allwear;
    R=stability_shift_cdc_2d(o);
    save(fullfile(o.output_dir,'handling_results.mat'),'R','-v7');
    plot_shift_results_2d(R,1,[],'handling');
    return;
 case 'pilot'
    % Defaults: healthy tire, all steering angles use one ABSOLUTE coordinate grid.
 case 'wear'
    o.wear_cases=allwear;
 case 'full'
    o.speeds=[50 60 70 80 90 100]; o.mus=[0.9 0.8 0.7 0.6 0.5];
    o.steers=[0 0.5 1 1.5 2 2.5 3]; o.wear_cases=allwear;
    % Preserve the original supplied grid for comparisons; check truncation flags.
    o.grid.vc=(-20:20)*0.5;
    o.grid.r=(-15:15)*(0.8/30);
    o.fixed_phi0=0;
 case 'ablation'
    R=cell(3,1); modes={'legacy','kinematic_zero','trim'};
    for k=1:3
        o.initialization=modes{k}; o.output_dir=fullfile(root,['results_2d_ablation_' modes{k}]);
        R{k}=stability_shift_cdc_2d(o);
    end
    return;
 case 'robust'
    o.hidden_variants=[0 0 0 0;0.1 0 0 0;-0.1 0 0 0];
 case 'refined'
    o.grid.vc=-10:0.25:10; o.grid.r=(-30:30)*(0.8/60);
 case 'halfstep'
    o.Nsub=40;
 otherwise
    error('Unknown mode: %s',mode);
end
R=stability_shift_cdc_2d(o);
if ~strcmpi(mode,'full'), plot_shift_results_2d(R,1); end
end
