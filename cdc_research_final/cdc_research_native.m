function out=cdc_research_native(X,Q,c,T,operation)
% Bridge to the delivered fixed-current engine. All 4 current maps exported.
if nargin<5,operation='simulate';end
assert(isempty(c.controller),'Native backend is fixed-current only. Feedback requires mcode.');
assert(exist(c.native_executable,'file')==2,'Run cdc_build_native first, or select engine=mcode.');
assert(all(abs(Q.kSpring-30790)<1e-10)&&isequal(Q.MR_vec,[.8613;.8613;.797;.797]),'Native model parameters differ from supplied constants.');
assert(c.derivative_time_scale==1,'Native backend currently uses derivative_time_scale=1 only.');
P=struct('m',2218.042,'ms',1967.686,'Iz',4793,'Ixx',924,'Iyy',2500,'lf',1.472,'lr',1.531,'tf',1.633,'tr',1.631,'g',9.81,'h_roll',.20,'h_cg',.541,'tau_load',.03,'Rw',.36,'Iw',1.80,'kt',200000*ones(4,1),'Fz0_vec',5425*ones(4,1),'sigma_kappa',.35*ones(4,1),'sigma_alpha',.75*ones(4,1),'Karb_f',0,'Karb_r',0,'Carb_f',0,'Carb_r',0);
for field={'m','ms','Iz','Ixx','Iyy','lf','lr','tf','tr','g','h_roll','h_cg','tau_load','Rw','Iw','kt','Fz0_vec','sigma_kappa','sigma_alpha','Karb_f','Karb_r','Carb_f','Carb_r'}
 key=field{1};assert(isequaln(Q.(key),P.(key)),'Native model parameter mismatch: %s. Use the mcode backend.',key);
end
expectedMF=[1.5662 1.0278 -.17125 -1.1357 -1.0544 -25.85 2.1377 2 8.675e-4 7.046e-5 -.007239 -7.442e-5;1.5457 1.0286 -.16688 -1.439 -1.6063 -26 1.9164 2 1.82e-3 -7.193e-4 .001802 -.011497;1.5057 1.0067 -.2019 -2 -2 -27.27 1.5963 2 .0023735 .000238 .01487 -.011025];
wi=1+(Q.wear(:)>=12.5)+(Q.wear(:)>=37.5);
assert(isequal(Q.MFy,expectedMF(wi,:)),'Native tire coefficients changed. Use mcode and re-audit.');
assert(Q.mu_scale_x==c.mu/.9&&Q.mu_scale_y==c.mu/.9,'Native friction-scaling mismatch');
assert(all(isfinite(X(:))),'Initial states must be finite.');
tmp=tempname;mkdir(tmp);cleanup=onCleanup(@()rmdir(tmp,'s'));
config=fullfile(tmp,'config.txt');states=fullfile(tmp,'states.txt');output=fullfile(tmp,'output.bin');
f=fopen(config,'w');assert(f>=0,'Cannot create config');
fprintf(f,'CDC_RESEARCH_FIXED_V1\n');
fprintf(f,'%.17g ',c.speed_kmh,c.mu,c.steer_deg);fprintf(f,'\n');
fprintf(f,'%.17g ',Q.wear);fprintf(f,'\n');
fprintf(f,'%.17g ',c.Ts/c.Nsub,c.terminal_window,numel(c.horizons));fprintf(f,'\n');
fprintf(f,'%.17g ',c.horizons);fprintf(f,'\n');
fprintf(f,'%.17g ',T.usable,c.yaw_factor,c.ay_factor,c.LTR_limit,c.reject_force_clipping,c.native_threads,c.trace_stride);fprintf(f,'\n');
fprintf(f,'%.17g ',[c.guard.vc,c.guard.r,c.guard.phi,c.guard.kappa,c.guard.alpha,c.guard.omega]);fprintf(f,'\n');
fprintf(f,'%.17g ',c.state_tolerance);fprintf(f,'\n');
% Failed trims are not used as terminal targets by the native engine.
xt=T.x;if any(~isfinite(xt)),xt=zeros(32,1);end
fprintf(f,'%.17g ',xt);fprintf(f,'\n');
fprintf(f,'%.17g ',Q.damper_map');fprintf(f,'\n');fclose(f);
f=fopen(states,'w');assert(f>=0,'Cannot create initial state file');fprintf(f,'%d\n',size(X,2));fprintf(f,[repmat('%.17g ',1,32) '\n'],X);fclose(f);
cmd=sprintf('"%s" %s "%s" "%s" "%s"',c.native_executable,operation,config,states,output);
[status,msg]=system(cmd);assert(status==0,'Native computation failed: %s',msg);
if ~strcmp(operation,'simulate'),out=dlmread(output);return;end
f=fopen(output,'rb','ieee-le');assert(f>=0,'Missing native output');guard=onCleanup(@()fclose(f));
header=fread(f,4,'uint32');names=cdc_research_fields();n=size(X,2);nh=numel(c.horizons);nf=numel(names);
assert(isequal(header(:)',[hex2dec('43524331') n nh nf]),'Wrong native output schema');
a=fread(f,Inf,'double');assert(numel(a)==n*(nh*nf+32),'Incomplete native output');
clear guard;
a=reshape(a,nh*nf+32,n);out=struct();
for j=1:nf,out.(names{j})=a(j:nf:nh*nf,:)';end
out.final_state=a(nh*nf+1:end,:)';out.trace=[];
tracefile=[output '.trace.csv'];if exist(tracefile,'file')==2,out.trace=dlmread(tracefile,',',1,0);end
end
