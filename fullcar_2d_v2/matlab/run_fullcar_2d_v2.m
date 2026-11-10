function R=run_fullcar_2d_v2(mode)
% Preserve the original CDC dynamics; change the hidden-state slice explicitly.
% This is NOT the no-roll planar model. Roll effects remain in all judgments.
if nargin<1,mode='pilot';end
if strcmpi(mode,'selftest'),R=stability_fullcar_2d_v2('selftest');return;end
o=stability_fullcar_2d_v2('defaults');
o.output_dir=fullfile(fileparts(mfilename('fullpath')),['results_fullcar_v2_' mode]);
o.horizons=[5 10];o.verbose=false;o.resume=true;
switch lower(mode)
 case 'pilot'
 case 'full'
    o.speeds=[50 60 70 80 90 100];o.mus=[.9 .8 .7 .6 .5];o.steers=0:.5:3;
    o.wear_cases=[0 0 0 0;25 25 0 0;50 50 25 25;0 0 25 25;25 25 50 50];
 case 'wear'
    o.wear_cases=[0 0 0 0;25 25 0 0;50 50 25 25;0 0 25 25;25 25 50 50];
 case 'flat'
    o.slice_mode='flat';
 case 'legacy'
    o.slice_mode='legacy_phi0';o.use_yaw_policy_for_region=true;
    o.grid.vc=(-20:20)*.5;o.grid.r=(-15:15)*(.8/30);
 case 'halfstep'
    o.Nsub=40;
 otherwise,error('Unknown fullcar mode: %s',mode);
end
R=stability_fullcar_2d_v2(o);
if ~strcmpi(mode,'full'),audit_existing_2d(o.output_dir,false);end
end
