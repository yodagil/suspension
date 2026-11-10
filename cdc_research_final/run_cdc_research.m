function R=run_cdc_research(mode,overrides)
%RUN_CDC_RESEARCH Single entry point for final research-region generation.
% run_cdc_research('selftest')                  small MATLAB test
% run_cdc_research('pilot')                     50/.9/0,3/C5
% run_cdc_research('study',struct('engine','native')) 20 condition-wear cases
% run_cdc_research('full',struct('engine','native'))  210 x 5 cases (not default)
% run_cdc_research('saved')                     plot the bundled executed results
% cdc_build_native                            optional g++ build
if nargin<1,mode='pilot';end
if nargin<2,overrides=struct();end
o=cdc_research_options();
switch lower(mode)
 case 'saved'
  root=fileparts(mfilename('fullpath'));
  files=dir(fullfile(root,'results','refined_mu09_steer*_C5.mat'));caseIndex=1;
  if isempty(files),files=dir(fullfile(root,'results','research_50kph_mu0.9_delta+*_trim_trim.mat'));caseIndex=5;end
  assert(~isempty(files),'Bundled executed results not found');
  R=cell(numel(files),1);for k=1:numel(files),R{k}=fullfile(files(k).folder,files(k).name);end
  plot_cdc_research(R,caseIndex,'final');return;
 case 'selftest'
  R=cdc_research_selftest(overrides);return;
 case 'pilot'
  o.mus=.9;o.wear_cases=[25 25 50 50];
 case 'study'
  % Four scenarios x five wear cases.
 case 'full'
  o.speeds=[50 60 70 80 90 100];o.mus=[.9 .8 .7 .6 .5];o.steers=0:.5:3;
 case 'refined'
  o.mus=.9;o.wear_cases=[25 25 50 50];o.grid=struct('vc',-3:.05:3,'r',-1.5:.0125:1.5);
 case 'halfstep'
  o.mus=.9;o.wear_cases=[25 25 50 50];o.Nsub=40;
 case 'long'
  o.mus=.9;o.wear_cases=[25 25 50 50];o.horizons=[10 20];
 case 'hidden'
  o.mus=.9;o.wear_cases=[25 25 50 50];o.hidden_variants=[0 0 0 0;.05 0 0 0;-.05 0 0 0];
 case 'current_sweep'
  R=cell(3,1);levels=[500 950 1500];
  for j=1:3
   q=overrides;q.I_mA=repmat(levels(j),1,4);
   q.output_dir=fullfile(fileparts(mfilename('fullpath')),sprintf('results_current_%g',levels(j)));
   R{j}=run_cdc_research('pilot',q);
  end
  return;
 otherwise,error('Unknown mode: %s',mode);
end
o.output_dir=fullfile(fileparts(mfilename('fullpath')),['results_new_' lower(mode)]);
f=fieldnames(overrides);
for j=1:numel(f),assert(isfield(o,f{j}),'Unknown option: %s',f{j});o.(f{j})=overrides.(f{j});end
R=cdc_research_generate(o);
end
