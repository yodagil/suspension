function R=run_hmc_standalone(mode,overrides)
%RUN_HMC_STANDALONE Independent data-driven model; no CarSim/TNO DLL.
% baseline: 50km/h,mu=.5,0deg,healthy. full:210conditions x5wear.
% This is an independently implemented approximation, not a CarSim solver.
if nargin<1,mode='baseline';end
if nargin<2,overrides=struct();end
c=hmc_options();
switch lower(mode)
 case 'baseline'
  c.speeds=50;c.mus=.5;c.steers=0;c.wear_cases=zeros(1,4);
  c.output_dir=fullfile(fileparts(mfilename('fullpath')),'results_baseline');
 case 'pilot'
  c.speeds=50;c.mus=[.5 .9];c.steers=[0 2];
 case 'full'
  % Explicit hmc_options full grid.
 otherwise,error('Unknown mode: %s',mode);
end
keys=fieldnames(overrides);
for j=1:numel(keys)
 key=keys{j};assert(isfield(c,key),'Unknown option: %s',key);c.(key)=overrides.(key);
end
R=hmc_generate(c);
end
