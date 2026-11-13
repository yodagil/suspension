function out=hmc_native(varargin)
%HMC_NATIVE Independent HMC native RK4 rollout / RHS bridge.
% out=hmc_native(X,Q,T,c,wear) returns one row per initial condition.
% dx =hmc_native('rhs',X,Q) evaluates the same vehicle equations.
% Xf =hmc_native('integrate',X,Q,c) integrates without policy shortcuts.
% No CarSim or TNO binary is called. Inputs and output are little-endian doubles.
if ischar(varargin{1}) || isstring(varargin{1})
 mode=char(varargin{1});X=varargin{2};Q=varargin{3};
 if strcmp(mode,'rhs'),T=struct();c=struct();wear=Q.wear;
 elseif ismember(mode,{'integrate','trajectory'}),c=varargin{4};T=struct();wear=Q.wear;
 else,error('Unknown native action: %s',mode);end
else
 mode='rollout';X=varargin{1};Q=varargin{2};T=varargin{3};c=varargin{4};wear=varargin{5};
end
assert(size(X,1)==34&&all(isfinite(X(:))),'Native input must be finite 34xN states');
root=fileparts(mfilename('fullpath'));
exe=fullfile(root,'native','hmc_native');if ispc,exe=[exe '.exe'];end
if isfield(c,'native_executable'),exe=c.native_executable;end
assert(exist(exe,'file')==2,'Build the independent accelerator with hmc_build_native first.');
work=tempname;mkdir(work);cleanup=onCleanup(@()rmdir(work,'s')); %#ok<NASGU>
config=struct('Q',Q,'T',T,'c',c,'wear',wear(:)');
fp=fopen(fullfile(work,'config.json'),'w');assert(fp>=0);fwrite(fp,jsonencode(config),'char');fclose(fp);
fp=fopen(fullfile(work,'states.bin'),'w','ieee-le');assert(fp>=0);fwrite(fp,X,'double');fclose(fp);
threads=4;if isfield(c,'native_threads'),threads=c.native_threads;end
assert(threads>=1&&mod(threads,1)==0,'Invalid native thread count');
cmd=sprintf('"%s" %s "%s" "%s" "%s" %d',exe,mode,fullfile(work,'config.json'),fullfile(work,'states.bin'),fullfile(work,'result.bin'),threads);
[status,msg]=system(cmd);assert(status==0,'Independent native failed: %s',msg);
fp=fopen(fullfile(work,'result.bin'),'r','ieee-le');assert(fp>=0);v=fread(fp,Inf,'double=>double');fclose(fp);
n=size(X,2);
if strcmp(mode,'trajectory')
 assert(n==1&&mod(numel(v),40)==0,'Trajectory output requires one initial state');
 a=reshape(v,40,[]);out=struct('time_s',a(1,:),'state',a(2:35,:), ...
  'ay',a(36,:),'Fz_raw',a(37:40,:));return;
end
if strcmp(mode,'rhs')||strcmp(mode,'integrate')
 assert(numel(v)==34*n,'Wrong native state output size');out=reshape(v,34,n);return;
end
names={'passed','recoverable','model_safe','guarded_final', ...
 'first_H_exit_s','first_physical_failure_s','min_fz_N', ...
 'max_LTR','max_LTR_front','max_LTR_rear','peak_rollrate_degps', ...
 'max_abs_roll_deg','initial_rollrate_degps','last_rollrate_degps', ...
 'first_ay_violation_s','first_yaw_violation_s','peak_abs_r_radps', ...
 'peak_abs_ay_mps2','observed_until','terminal_error', ...
 'stayed_in_handling','yaw_ok','ay_ok'};
h=[5 10];if isfield(c,'horizons'),h=c.horizons;elseif isfield(c,'horizons_s'),h=c.horizons_s;end
nf=numel(names);nh=numel(h);nr=nf*nh+34;
assert(numel(v)==nr*n,'Wrong native diagnostic output size');v=reshape(v,nr,n);
out=struct();for k=1:nf,out.(names{k})=v(k:nf:nf*nh,:)';end
out.final_state=v(nf*nh+1:end,:)';out.horizons_s=h(:)';
out.engine='independent_native';out.trace=[];
end
