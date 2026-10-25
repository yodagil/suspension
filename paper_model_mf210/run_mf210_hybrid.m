function files = run_mf210_hybrid(output_dir,make_plots)
%RUN_MF210_HYBRID Independent healthy-tire sweeps at 0, 3.5, and 7 degrees.
% This generated file is self-contained. All appended dynamics and original
% 210-region helpers are copied verbatim; configuration/provenance are embedded.
% MATLAB: run_mf210_hybrid;  % 138,105 initial states per angle, 5 seconds
% Results: PAPER_STABILITY3D classification and original-region REGION210.
% Model: paper body + original MF210 transient hybrid, not a literal paper
% reproduction or the complete original full-car body/suspension model.
  if nargin<1 || isempty(output_dir)
    output_dir=fullfile(fileparts(mfilename('fullpath')),'results_matlab');
  end
  if nargin<2,make_plots=true;end
  if ~exist(output_dir,'dir'),mkdir(output_dir);end
  base_cfg=delivery_configuration();
  provenance=delivery_provenance();
  region_cfg=delivery_region_configuration();
  angles=[0 3.5 7];
  files=cell(numel(angles),3);
  for ii=1:numel(angles)
    cfg=base_cfg;
    cfg.steer_deg=angles(ii);
    tag=sprintf('%g',cfg.steer_deg);
    fprintf('\nMF210 transient hybrid: 40 km/h fixed, mu=0.85, delta=%s deg\n',tag);
    result=paper_simulate_grid(cfg);
    result.unstable_points=result.initial_points(~result.stable_mask(:),:);
    result.definition=['Independent initial-body-state sweep with original MF210 ' ...
      'tire relaxation, free wheels, filtered Eq11 load transfer and legacy hidden-state reset'];
    meta=struct('version','paper_body_mf210_transient_v1', ...
      'engine','Standalone MATLAB M-code', ...
      'model','Paper body + original MF210 force, relaxation, wheel and load-transfer dynamics', ...
      'paper_exact_reproduction',false, ...
      'original_fullcar_body_model_used',false, ...
      'tire_force_coefficients_changed',false, ...
      'tire_dynamic_relaxation_included',true, ...
      'body_model','Paper Eqs2/3/4 and Eq11 with original MF210 ay_load filter; fixed speed', ...
      'state_order','u,vc,r,phi,p,kappa(4),alpha(4),omega(4),ay_load', ...
      'initialization',cfg.initialization, ...
      'mu_scaling','mu/.9; original MF peak is not forcibly normalized to road mu', ...
      'source_interpretation','User-requested hybrid; not literal Burckhardt paper reproduction', ...
      'force_offset_note','All original MF shifts and longitudinal-priority friction ellipse retained', ...
      'nesting_constraints_used',false, ...
      'source_provenance',provenance);
    PAPER_STABILITY3D=struct('meta',meta,'cfg',cfg,'result',result);
    files{ii,1}=fullfile(output_dir,['classification_' tag 'deg.mat']);
    % Save classification before geometry so it survives an extraction error.
    save(files{ii,1},'PAPER_STABILITY3D','-v7');
    region=build_paper_region(cfg.grid,result.stable_mask, ...
      result.stable_points,result.unstable_points,region_cfg);
    REGION210=struct('region',region,'region_cfg',region_cfg, ...
      'source_cfg',cfg,'source_meta',meta, ...
      'classification_path',files{ii,1},'provenance',provenance, ...
      'selection_independent_of_other_angles',true, ...
      'nesting_constraints_applied',false);
    files{ii,2}=fullfile(output_dir,['region210_' tag 'deg.mat']);
    save(files{ii,2},'REGION210','-v7');
    files{ii,3}=fullfile(output_dir,['stable_point_40kph_0.85mu_' tag 'deg.mat']);
    save(files{ii,3},'PAPER_STABILITY3D','REGION210','-v7');
    fprintf('Saved: stable=%d/%d; valid inner=%d; volume=%.12g; sampled unstable inside=%g\n', ...
      result.stable_count,result.total_count,region.is_valid_inner, ...
      region.inner_volume,region.unstable_inside_inner);
  end
  if make_plots
    plot_mf210_hybrid(output_dir,fullfile(output_dir,'mf210_hybrid_regions'));
  end
end

function cfg=delivery_configuration()
cfg=struct();
cfg.paper_doi='10.1016/j.aei.2022.101764';
cfg.m=1300;
cfg.Izz=2500;
cfg.Ixx=700;
cfg.L=2.3999999999999999;
cfg.lf=1.2;
cfg.lr=1.2;
cfg.h=0.54000000000000004;
cfg.df=0.69999999999999996;
cfg.dr=0.69999999999999996;
cfg.Kphi=43100;
cfg.Cphi=1400;
cfg.g=9.8100000000000005;
cfg.ms=1300;
cfg.hphi=0.54000000000000004;
cfg.deltaFz=[0 0 0 0];
cfg.mu=0.84999999999999998;
cfg.speed_kmh=40;
cfg.steer_deg=0;
cfg.speed_mode='fixed';
cfg.horizon_s=5;
cfg.dt=0.01;
cfg.method='rk4';
cfg.batch_size=5000;
cfg.limit_factor=0.84999999999999998;
cfg.min_u=0.5;
cfg.filename_digits=3;
cfg.grid=struct('vc',[-10 -9.75 -9.5 -9.25 -9 -8.75 -8.5 -8.25 -8 -7.75 -7.5 -7.25 -7 -6.75 -6.5 -6.25 -6 -5.75 -5.5 -5.25 -5 -4.75 -4.5 -4.25 -4 -3.75 -3.5 -3.25 -3 -2.75 -2.5 -2.25 -2 -1.75 -1.5 -1.25 -1 -0.75 -0.5 -0.25 0 0.25 0.5 0.75 1 1.25 1.5 1.75 2 2.25 2.5 2.75 3 3.25 3.5 3.75 4 4.25 4.5 4.75 5 5.25 5.5 5.75 6 6.25 6.5 6.75 7 7.25 7.5 7.75 8 8.25 8.5 8.75 9 9.25 9.5 9.75 10],'r',[-0.72000000000000008 -0.69333333333333336 -0.66666666666666674 -0.64000000000000001 -0.6133333333333334 -0.58666666666666667 -0.56000000000000005 -0.53333333333333333 -0.50666666666666671 -0.48000000000000004 -0.45333333333333337 -0.42666666666666669 -0.40000000000000002 -0.37333333333333335 -0.34666666666666668 -0.32000000000000001 -0.29333333333333333 -0.26666666666666666 -0.24000000000000002 -0.21333333333333335 -0.18666666666666668 -0.16 -0.13333333333333333 -0.10666666666666667 -0.080000000000000002 -0.053333333333333337 -0.026666666666666668 0 0.026666666666666668 0.053333333333333337 0.080000000000000002 0.10666666666666667 0.13333333333333333 0.16 0.18666666666666668 0.21333333333333335 0.24000000000000002 0.26666666666666666 0.29333333333333333 0.32000000000000001 0.34666666666666668 0.37333333333333335 0.40000000000000002 0.42666666666666669 0.45333333333333337 0.48000000000000004 0.50666666666666671 0.53333333333333333 0.56000000000000005 0.58666666666666667 0.6133333333333334 0.64000000000000001 0.66666666666666674 0.69333333333333336 0.72000000000000008],'phi',[-0.40000000000000002 -0.37333333333333335 -0.34666666666666668 -0.32000000000000001 -0.29333333333333333 -0.26666666666666666 -0.24000000000000002 -0.21333333333333335 -0.18666666666666668 -0.16 -0.13333333333333333 -0.10666666666666667 -0.080000000000000002 -0.053333333333333337 -0.026666666666666668 0 0.026666666666666668 0.053333333333333337 0.080000000000000002 0.10666666666666667 0.13333333333333333 0.16 0.18666666666666668 0.21333333333333335 0.24000000000000002 0.26666666666666666 0.29333333333333333 0.32000000000000001 0.34666666666666668 0.37333333333333335 0.40000000000000002]);
cfg.tire_model='MF210_healthy';
cfg.mf_Fz0=5425;
cfg.mf_force_min=100;
cfg.mf_force_max=16000;
cfg.mf_mu_reference=0.90000000000000002;
cfg.mf_mu_scaling_rule='mu_road / 0.9';
cfg.mf_wear=[0 0 0 0];
cfg.slip_angle_mode='relaxed toward exact wheel-frame velocity; legacy initial alpha=-steer';
cfg.tire_dynamics='MF210_relaxation_and_free_wheels';
cfg.model_dimension=18;
cfg.mf_Rw=0.35999999999999999;
cfg.mf_Iw=1.8;
cfg.mf_sigma_kappa=0.34999999999999998;
cfg.mf_sigma_alpha=0.75;
cfg.mf_tau_load=0.029999999999999999;
cfg.Nsub=20;
cfg.integration_dt=0.00050000000000000001;
cfg.load_solver='explicit original210 lateral-load-transfer state';
cfg.initialization='reference09 legacy: alpha=-steer, kappa=0, omega=ux*cos(steer)/Rw, ay_load=0';
cfg.reconstruction_version='XiaoBody-MF210Transient-v1';
end

function cfg=delivery_region_configuration()
cfg=struct();
cfg.hull_inside_tol=1e-10;
cfg.paper_vc_split_tol=9.9999999999999998e-13;
cfg.require_inner_no_unstable=true;
cfg.inner_depth_start=2;
cfg.allow_stable_mask_fallback=true;
cfg.min_limit_points=8;
cfg.max_support_depth=12;
end

function provenance=delivery_provenance()
provenance=struct();
provenance.model='Paper body + original MF210 transient hybrid';
provenance.original_MF_source_sha256='29c4062948d71dd794d042233025718097ad9654526f64fd7699f63321cd0e7c';
provenance.original_MF_verbatim_block_sha256='a26aaa7cc85deb673dbb7c78d0ae0fb307a4aa472d64aec546804a3f90348783';
provenance.original_region210_source_sha256='29c4062948d71dd794d042233025718097ad9654526f64fd7699f63321cd0e7c';
provenance.source_files={'matlab/paper_simulate_grid.m','matlab/paper_initial.m','matlab/paper_step.m','matlab/paper_rhs.m','matlab/mf210_forces.m','matlab/mf210_force_scalar.m','region210/build_paper_region.m','region210/empty_region.m','native_mf.cpp'};
provenance.source_sha256={'afd504d724fe6f22c826d26b47a697c4cd0d398c44d5137cfe02a25bf67f6094','a863a998966f41f5abc968c600253a6e126e556043336ac5e363bab13d7c2a47','4cae3ebc73116508d1b439374ac7f78ecba223fc23db40820455118a3de0b298','2b1a6d7ba74a1d010a79e2fb0899bbd8933ef2c1fc9beeab55da740db9af2970','b61291c65052ef153d81f91bb2dcb272d0e496602245a763f2eab417ee221b65','6357f53b09a143f185d6f4b90f457bf833dc26d09a023dd95400729b46b5678f','2c9b395dd14e12988c637a9353f8c92bb10b698c291e64b16ef4866b524f7751','df191ab4f4e741be23ecd7d919440f653a633c88416a1d826b7b06d4bbd8f0cd','c97e917b136bb373fb5d7e0ea6c7e0af7dc0bfe9c79420700b9a4fdf9e275170'};
provenance.native_reference_report_passed=true;
provenance.native_reference_source_sha256='c97e917b136bb373fb5d7e0ea6c7e0af7dc0bfe9c79420700b9a4fdf9e275170';
provenance.native_reference_executable_sha256='96924934473dacc1ca8515f2fcb925ad709cfe63bcfece29aac1395bdc71aad3';
provenance.selection_independent_of_other_angles=true;
provenance.nesting_constraints_applied=false;
provenance.configuration_source='vc_refined_config_0deg.mat';
provenance.configuration_source_sha256='c1e6efdc8a859cbe9be87c0d3e56504442223c6446dd001ed9a9bdab7823b36b';
end

% BEGIN VERBATIM: matlab/paper_simulate_grid.m; SHA256 afd504d724fe6f22c826d26b47a697c4cd0d398c44d5137cfe02a25bf67f6094
function result=paper_simulate_grid(cfg)
% Simulate all initial [v_c,r,phi] states on EXACT inherited grid.
% Stable means no prescribed constraint violation during finite horizon.
% Bit flags: 1 yaw limit, 2 lateral acceleration, 4 wheel lift, 8 numerical.
[VC,RR,PHI]=ndgrid(cfg.grid.vc,cfg.grid.r,cfg.grid.phi);
P=[VC(:) RR(:) PHI(:)];
N=size(P,1);
mask=false(N,1);
reason=zeros(N,1,'uint8');
maxYaw=zeros(N,1);
maxAy=zeros(N,1);
minFz=inf(N,1);
nonfiniteFailure=false(N,1);
lowSpeedGuard=false(N,1);
maxLoadResidual=0;
nSteps=ceil(cfg.horizon_s/cfg.dt);
h=cfg.horizon_s/nSteps;
started=tic;
for st=1:cfg.batch_size:N
    en=min(N,st+cfg.batch_size-1);
    ix=st:en;
    n=numel(ix);
    x=paper_initial(P(ix,:),cfg);
    bad=zeros(n,1,'uint8');
    maxYawL=zeros(n,1);
    maxAyL=zeros(n,1);
    minFzL=inf(n,1);
    nonfiniteL=false(n,1);
    lowSpeedL=false(n,1);
    for k=0:nSteps
        active=find(bad==0);
        if isempty(active)
            break;
        end
        xa=x(active,:);
        [~,ob]=paper_rhs(xa,cfg);
        rlim=cfg.limit_factor*cfg.mu*cfg.g./max(xa(:,1),cfg.min_u);
        failYaw=abs(xa(:,3))>rlim+1e-10;
        failAy=abs(ob.ay)>cfg.mu*cfg.g+1e-10;
        failLift=any(ob.Fz_raw<=0,2);
        failNonfinite=any(~isfinite(xa),2)|~isfinite(ob.ay)|any(~isfinite(ob.Fz_raw),2);
        failLowSpeed=xa(:,1)<cfg.min_u;
        failNumeric=failNonfinite|failLowSpeed;
        nonfiniteL(active)=nonfiniteL(active)|failNonfinite;
        lowSpeedL(active)=lowSpeedL(active)|failLowSpeed;
        if isfield(ob,'load_solver_max_load_residual_N')
            residuals=ob.load_solver_max_load_residual_N;
            residuals=residuals(isfinite(residuals));
            if ~isempty(residuals),maxLoadResidual=max(maxLoadResidual,max(residuals));end
        end
        newbad=uint8(failYaw)+uint8(2*failAy)+uint8(4*failLift)+uint8(8*failNumeric);
        bad(active)=bitor(bad(active),newbad);
        maxYawL(active)=max(maxYawL(active),abs(xa(:,3)));
        maxAyL(active)=max(maxAyL(active),abs(ob.ay));
        minFzL(active)=min(minFzL(active),min(ob.Fz_raw,[],2));
        if k==nSteps
            break;
        end
        active=active(newbad==0);
        if isempty(active)
            continue;
        end
        xx=x(active,:);
        x(active,:)=paper_step(xx,cfg);
    end
    mask(ix)=bad==0;
    reason(ix)=bad;
    maxYaw(ix)=maxYawL;
    maxAy(ix)=maxAyL;
    minFz(ix)=minFzL;
    nonfiniteFailure(ix)=nonfiniteL;
    lowSpeedGuard(ix)=lowSpeedL;
    fprintf('   states %6d / %6d, stable so far %6d, elapsed %.1f s\n',en,N,nnz(mask(1:en)),toc(started));
    if isfield(cfg,'progress_path') && ~isempty(cfg.progress_path)
        status=struct('speed_kmh',cfg.speed_kmh,'mu',cfg.mu,'steer_deg',cfg.steer_deg, ...
            'processed_initial_states',en,'total_initial_states',N, ...
            'stable_so_far',nnz(mask(1:en)),'elapsed_s',toc(started));
        temp=[cfg.progress_path '.tmp'];fid=fopen(temp,'w');
        if fid>=0
            fprintf(fid,'%s',jsonencode(status));fclose(fid);movefile(temp,cfg.progress_path,'f');
        end
    end
end
result.grid=cfg.grid;
result.stable_mask=reshape(mask,size(VC));
result.reason_mask=reshape(reason,size(VC));
result.initial_points=P;
result.stable_points=P(mask,:);
result.stable_count=nnz(mask);
result.total_count=N;
result.stable_fraction=nnz(mask)/N;
result.max_abs_yaw=reshape(maxYaw,size(VC));
result.max_abs_ay=reshape(maxAy,size(VC));
result.min_wheel_load=reshape(minFz,size(VC));
result.reason_counts=[nnz(bitand(reason,1)) nnz(bitand(reason,2)) nnz(bitand(reason,4)) nnz(bitand(reason,8))];
result.runtime_s=toc(started);
result.reason_semantics='Causes recorded through the first violating observation; stable trajectories cover the full horizon';
result.reason_names={'yaw_limit','lateral_acceleration_limit','wheel_lift','nonfinite_or_low_speed_model_domain_guard'};
result.nonfinite_failure_count=nnz(nonfiniteFailure);
result.low_speed_domain_guard_count=nnz(lowSpeedGuard);
result.maximum_load_equilibrium_residual_N=maxLoadResidual;
result.definition='Finite-horizon stability test using published oversteer/sideslip/wheel-lift criteria; numerical choices assumed';
end


% BEGIN VERBATIM: matlab/paper_initial.m; SHA256 a863a998966f41f5abc968c600253a6e126e556043336ac5e363bab13d7c2a47
function x=paper_initial(points,cfg)
% Reference09 original tire initialization, deliberately not kinematic reset.
n=size(points,1);d=cfg.steer_deg*pi/180;delta=[d d 0 0];
omega=max(0,(cfg.speed_kmh/3.6)*cos(delta))/cfg.mf_Rw;
x=[repmat(cfg.speed_kmh/3.6,n,1),points,zeros(n,1), ...
    zeros(n,4),repmat(-delta,n,1),repmat(omega,n,1),zeros(n,1)];
assert(size(x,2)==18);
end


% BEGIN VERBATIM: matlab/paper_step.m; SHA256 4cae3ebc73116508d1b439374ac7f78ecba223fc23db40820455118a3de0b298
function next=paper_step(x,cfg)
% Same .01s observation, 20 RK4 substeps and tire-state clamps as MF210.
h=cfg.dt/cfg.Nsub;next=x;
for j=1:cfg.Nsub
    k1=paper_rhs(next,cfg);k2=paper_rhs(next+.5*h*k1,cfg);
    k3=paper_rhs(next+.5*h*k2,cfg);k4=paper_rhs(next+h*k3,cfg);
    next=next+(h/6)*(k1+2*k2+2*k3+k4);
end
finite=all(isfinite(next),2);
next(finite,6:9)=min(.5,max(-.5,next(finite,6:9)));
next(finite,10:13)=min(1.2,max(-1.2,next(finite,10:13)));
next(finite,14:17)=min(300,max(0,next(finite,14:17)));
end


% BEGIN VERBATIM: matlab/paper_rhs.m; SHA256 2b1a6d7ba74a1d010a79e2fb0899bbd8933ef2c1fc9beeab55da740db9af2970
function [dx,obs]=paper_rhs(x,cfg)
% Paper body plus the actual MF210 tire relaxation/free-wheel subsystem.
% [u vc r phi p kappa(4) alpha(4) omega(4) ay_load], 18 states.
u=x(:,1);v=x(:,2);r=x(:,3);phi=x(:,4);p=x(:,5);ayLoad=x(:,18);
delta=cfg.steer_deg*pi/180;cs=cos(delta);sn=sin(delta);
f0=cfg.m*cfg.g*cfg.lr/(2*cfg.L);r0=cfg.m*cfg.g*cfg.lf/(2*cfg.L);
fa=cfg.m*ayLoad*cfg.h*cfg.lr/(2*cfg.df*cfg.L);ra=cfg.m*ayLoad*cfg.h*cfg.lf/(2*cfg.dr*cfg.L);
fr=cfg.ms*cfg.g*sin(phi)*cfg.lr*cfg.hphi/(2*cfg.df*cfg.L);
rr=cfg.ms*cfg.g*sin(phi)*cfg.lf*cfg.hphi/(2*cfg.dr*cfg.L);
Fz=bsxfun(@plus,[f0-fa-fr f0+fa+fr r0-ra-rr r0+ra+rr],cfg.deltaFz);
[FxT,FyT]=mf210_forces(x(:,10:13),x(:,6:9),Fz,cfg);
Fx=cs*(FxT(:,1)+FxT(:,2))+FxT(:,3)+FxT(:,4)-sn*(FyT(:,1)+FyT(:,2));
Fy=sn*(FxT(:,1)+FxT(:,2))+cs*(FyT(:,1)+FyT(:,2))+FyT(:,3)+FyT(:,4);ay=Fy/cfg.m;
Mz=cs*cfg.df*(FxT(:,2)-FxT(:,1))+cfg.dr*(FxT(:,4)-FxT(:,3)) ...
    +sn*cfg.df*(FyT(:,1)-FyT(:,2))+sn*cfg.lf*(FxT(:,1)+FxT(:,2)) ...
    +cs*cfg.lf*(FyT(:,1)+FyT(:,2))-cfg.lr*(FyT(:,3)+FyT(:,4));
uB=[u-cfg.df*r,u+cfg.df*r,u-cfg.dr*r,u+cfg.dr*r];
vB=[v+cfg.lf*r,v+cfg.lf*r,v-cfg.lr*r,v-cfg.lr*r];
uW=[cs*uB(:,1:2)+sn*vB(:,1:2),uB(:,3:4)];
vW=[-sn*uB(:,1:2)+cs*vB(:,1:2),vB(:,3:4)];
safe=uW;safe(safe>=0&safe<.1)=.1;safe(safe<0&safe>-.1)=-.1;
alphaTarget=atan2(vW,safe);
kappaTarget=min(.5,max(-.5,(cfg.mf_Rw*x(:,14:17)-uW)./max(.5,abs(uW))));
vroll=max(.5,abs(uW));rk=min(500,vroll/cfg.mf_sigma_kappa);raRate=min(500,vroll/cfg.mf_sigma_alpha);
udot=Fx/cfg.m+v.*r;if strcmp(cfg.speed_mode,'fixed'),udot=zeros(size(v));end
deltaMx=(cfg.deltaFz(2)-cfg.deltaFz(1))*cfg.df+(cfg.deltaFz(4)-cfg.deltaFz(3))*cfg.dr;
pdot=(cfg.ms*cfg.hphi*ay-cfg.Cphi*p+(cfg.ms*cfg.g*cfg.hphi-cfg.Kphi)*phi-deltaMx)/cfg.Ixx;
dx=[udot,ay-u.*r,Mz/cfg.Izz,p,pdot, ...
    rk.*(kappaTarget-x(:,6:9)),raRate.*(alphaTarget-x(:,10:13)), ...
    -cfg.mf_Rw*FxT/cfg.mf_Iw,(ay-ayLoad)/cfg.mf_tau_load];
if nargout>1
    obs=struct('ay',ay,'Fz_raw',Fz,'Fx_tires',FxT,'Fy_tires',FyT,'Fx',Fx,'Fy',Fy,'Mz',Mz, ...
        'alpha',x(:,10:13),'kappa',x(:,6:9),'alpha_target',alphaTarget,'kappa_target',kappaTarget, ...
        'wheel_longitudinal_velocity',uW,'ay_load',ayLoad, ...
        'load_transfer_model','PaperEq11 with original210 0.03s lateral-load-transfer state', ...
        'tire_model','Original210 MF force, slip relaxation, free-wheel dynamics and initialization');
end
end


% BEGIN VERBATIM: matlab/mf210_forces.m; SHA256 b61291c65052ef153d81f91bb2dcb272d0e496602245a763f2eab417ee221b65
function [Fx,Fy,info]=mf210_forces(alpha,kappa,Fz_raw,cfg)
% Vectorized HEALTHY force law of the validated 210-condition full-car code.
% Same MF shifts, load sensitivity, mu/.9 scaling and friction ellipse.
% The original vertical-force clipping and >100N activation are preserved.
assert(all(cfg.mf_wear(:)==0),'This vectorized entry is for healthy tires only');
z=min(max(Fz_raw,cfg.mf_force_min),cfg.mf_force_max);
Fz0=cfg.mf_Fz0;
dfz=(z-Fz0)/Fz0;
sx=cfg.mu/cfg.mf_mu_reference;sy=sx;
% Longitudinal coefficients: copied exactly, without zero-slip simplification.
Cx=1.647;Dx=(1.145-0.2106*dfz).*z*sx;
Kx=z.*(24.125+15.0*dfz).*exp(-0.4832*dfz);
Bx=zeros(size(z));ok=abs(Cx*Dx)>=1e-9;Bx(ok)=Kx(ok)./(Cx*Dx(ok));
Ex=min(1.0,-0.02192+0.23613*dfz);
kx=kappa+(-0.001635+0.0025297*dfz);
px=Bx.*kx;
Fx=Dx.*sin(Cx*atan(px-Ex.*(px-atan(px))));
% Healthy lateral coefficients, including horizontal and vertical shifts.
Cy=1.5662;Dy=(1.0278-0.17125*dfz).*z*sy;
Ky=-25.85*Fz0*sin(2.0*atan(z/(2.1377*Fz0)));
By=zeros(size(z));ok=abs(Cy*Dy)>=1e-9;By(ok)=Ky(ok)./(Cy*Dy(ok));
Ey=min(1.0,-1.1357-1.0544*dfz);
ay=alpha+(8.675e-4+7.046e-5*dfz);
SVy=z.*(-0.007239-7.442e-5*dfz);
py=By.*ay;
Fy0=Dy.*sin(Cy*atan(py-Ey.*(py-atan(py))))+SVy;
% Original longitudinal-priority combined-slip approximation.
mux=max(.05,abs(1.145-0.2106*dfz)*sx);
cap=max(1.0,mux.*z);rho=min(1.0,abs(Fx)./cap);
Gy=sqrt(max(0.0,1.0-rho.*rho));Fy=Fy0.*Gy;
inactive=z<=cfg.mf_force_min;Fx(inactive)=0;Fy(inactive)=0;
invalid=~isfinite(Fz_raw)|~isfinite(alpha)|~isfinite(kappa);
Fx(invalid)=NaN;Fy(invalid)=NaN;
if nargout>2
    info=struct('Fz_force',z,'combined_lateral_factor',Gy,'Fx_at_zero_kappa_not_forced_zero',true, ...
        'lateral_shift_not_removed',true,'Fz0_reference_N',Fz0,'mu_scale_x',sx,'mu_scale_y',sy);
end
end


% BEGIN VERBATIM: matlab/mf210_force_scalar.m; SHA256 6357f53b09a143f185d6f4b90f457bf833dc26d09a023dd95400729b46b5678f
function [Fx,Fy]=mf210_force_scalar(kappa,alpha,Fz,Fz0,wear,mu_scale_x,mu_scale_y)
% Exact scalar MF kernel extracted from the validated 210-condition source.
[Fx,Fy]=calc_combined_tire_force_scalar(kappa,alpha,Fz,Fz0,wear,mu_scale_x,mu_scale_y);
end

function [Fx,Fy] = calc_combined_tire_force_scalar(kappa,alpha,Fz,Fz0,wear,mu_scale_x,mu_scale_y)

%

% The available coefficient set contains pure-slip MF coefficients but not

% the complete MF6.2 combined-slip R-coefficients. Therefore this model uses

% a longitudinal-priority friction ellipse:

%

%   Fx = Fx0

%   Fy = Fy0*sqrt(max(0,1-(Fx0/(mu_x*Fz))^2))

%

% This is specifically chosen because the current comparison shows Fx is

% already close while the pure-slip model overpredicts Fy. Exact CarSim

% agreement requires the TIR RBX/RCX/REX/RBY/RCY/REY/RVY coefficients.

Fx0 = calc_mf_longitudinal_scalar(kappa,Fz,Fz0,wear,mu_scale_x);

Fy0 = calc_mf_lateral_scalar(alpha,Fz,Fz0,wear,mu_scale_y);

mux_peak = calc_longitudinal_peak_mu(Fz,Fz0,wear,mu_scale_x);

Fx_cap = mux_peak*Fz;

if Fx_cap < 1.0; Fx_cap=1.0; end

rho_x = abs(Fx0)/Fx_cap;

if rho_x>1.0; rho_x=1.0; end

Gy = sqrt(max(0.0,1.0-rho_x*rho_x));

Fx = Fx0;

Fy = Fy0*Gy;

end

%% ========================================================================

% Longitudinal peak-friction estimate used by friction ellipse

%% ========================================================================

function mux = calc_longitudinal_peak_mu(Fz,Fz0,wear,mu_scale_x)

PDX1 = 1.145;

PDX2 = -0.2106;

wear_case = normalize_wear_case(wear);

if wear_case == 25

    PDX1 = PDX1*0.95;

elseif wear_case == 50

    PDX1 = PDX1*0.90;

end

dfz = (Fz-Fz0)/Fz0;

mux = abs(PDX1+PDX2*dfz)*mu_scale_x;

if mux<0.05; mux=0.05; end

end

%% ========================================================================

% Pure longitudinal MF-Swift 6.2 force for one tire

%% ========================================================================

function Fx = calc_mf_longitudinal_scalar(kappa, Fz, Fz0, wear, mu_scale_x)

PCX1 = 1.647;

PDX1 = 1.145;

PDX2 = -0.2106;

PEX1 = -0.02192;

PEX2 = 0.23613;

PEX3 = 0.0;

PEX4 = 0.0;

PKX1 = 24.125;

PKX2 = 15.0;

PKX3 = -0.4832;

PHX1 = -0.001635;

PHX2 =  0.0025297;

PVX1 = 0.0;

PVX2 = 0.0;

wear_case = normalize_wear_case(wear);

if wear_case == 25

    PDX1 = PDX1*0.95;

    PKX1 = PKX1*0.95;

    PKX2 = PKX2*0.95;

elseif wear_case == 50

    PDX1 = PDX1*0.90;

    PKX1 = PKX1*0.90;

    PKX2 = PKX2*0.90;

end

dfz = (Fz-Fz0)/Fz0;

mux = PDX1 + PDX2*dfz;

Dx  = mux*Fz*mu_scale_x;

Cx  = PCX1;

Kx = Fz*(PKX1 + PKX2*dfz)*exp(PKX3*dfz);

if abs(Cx*Dx) < 1.0e-9

    Bx = 0.0;

else

    Bx = Kx/(Cx*Dx);

end

Ex_base = PEX1 + PEX2*dfz + PEX3*dfz^2;

SHx = PHX1 + PHX2*dfz;

SVx = Fz*(PVX1 + PVX2*dfz);

kappa_x = kappa + SHx;

E = Ex_base*(1.0 - PEX4*sign(kappa_x));

if E > 1.0

    E = 1.0;

end

phi_x = Bx*kappa_x;

Fx = Dx*sin(Cx*atan(phi_x - E*(phi_x-atan(phi_x)))) + SVx;

end

%% ========================================================================

% Pure lateral MF-Swift 6.2 force for one tire

%% ========================================================================

function Fy = calc_mf_lateral_scalar(alpha, Fz, Fz0, wear, mu_scale_y)

wear_case = normalize_wear_case(wear);

if wear_case == 0

    PCY1 = 1.5662;

    PDY1 = 1.0278;

    PDY2 = -0.17125;

    PEY1 = -1.1357;

    PEY2 = -1.0544;

    PKY1 = -25.85;

    PKY2 = 2.1377;

    PKY4 = 2.0;

    PHY1 = 8.675e-4;

    PHY2 = 7.046e-5;

    PVY1 = -0.007239;

    PVY2 = -7.442e-5;

elseif wear_case == 25

    PCY1 = 1.5457;

    PDY1 = 1.0286;

    PDY2 = -0.16688;

    PEY1 = -1.439;

    PEY2 = -1.6063;

    PKY1 = -26.0;

    PKY2 = 1.9164;

    PKY4 = 2.0;

    PHY1 = 1.82e-3;

    PHY2 = -7.193e-4;

    PVY1 = 1.802e-3;

    PVY2 = -1.1497e-2;

else

    PCY1 = 1.5057;

    PDY1 = 1.0067;

    PDY2 = -0.2019;

    PEY1 = -2.0;

    PEY2 = -2.0;

    PKY1 = -27.27;

    PKY2 = 1.5963;

    PKY4 = 2.0;

    PHY1 = 2.3735e-3;

    PHY2 = 2.38e-4;

    PVY1 = 0.01487;

    PVY2 = -0.011025;

end

dfz = (Fz-Fz0)/Fz0;

muy = PDY1 + PDY2*dfz;

Dy  = muy*Fz*mu_scale_y;

Cy  = PCY1;

Ky = PKY1*Fz0*sin(PKY4*atan(Fz/(PKY2*Fz0)));

if abs(Cy*Dy) < 1.0e-9

    By = 0.0;

else

    By = Ky/(Cy*Dy);

end

Ey_base = PEY1 + PEY2*dfz;

SHy = PHY1 + PHY2*dfz;

SVy = Fz*(PVY1 + PVY2*dfz);

alpha_y = alpha + SHy;

E = Ey_base;

if E > 1.0

    E = 1.0;

end

phi_y = By*alpha_y;

Fy = Dy*sin(Cy*atan(phi_y - E*(phi_y-atan(phi_y)))) + SVy;

end

%% ========================================================================

% Normalize wear input to 0 / 25 / 50

%% ========================================================================

function wear_case = normalize_wear_case(wear)

if wear < 12.5

    wear_case = 0;

elseif wear < 37.5

    wear_case = 25;

else

    wear_case = 50;

end

end

%% ========================================================================

% Calculate model-based damper rates

%% ========================================================================



% BEGIN VERBATIM: region210/build_paper_region.m; SHA256 2c9b395dd14e12988c637a9353f8c92bb10b698c291e64b16ef4866b524f7751
function region = build_paper_region(G,stable_mask,stable_points,unstable_points,cfg)

    % Robust reconstruction of the paper sequence:
    % Stable -> limit-roll -> lateral-speed layers -> eight vertices
    % -> inner/outer hexahedra.
    %
    % Difference from the previous implementation:
    % The old code selected only one "inner" layer.  In the last run that
    % hexahedron contained 7 sampled unstable points, so the script stopped.
    % Here the inner layer is moved inward discretely (depth=2,3,4,and onward) and
    % the largest candidate containing ZERO sampled unstable points is kept.

    region = empty_region();

    if isempty(stable_points) || size(stable_points,1)<8
        return;
    end

    if rank(stable_points-mean(stable_points,1))<3
        return;
    end

    % ---------------------------------------------------------------------
    % STEP 1: LIMIT-ROLL STABLE POINTS
    % For each fixed (vc,r), take the minimum/maximum stable phi.
    % ---------------------------------------------------------------------
    limit_roll = zeros(0,3);

    for iv=1:numel(G.vc)
        for ir=1:numel(G.r)

            idxPhi = find(squeeze(stable_mask(iv,ir,:)));

            if isempty(idxPhi)
                continue;
            end

            ipLo = idxPhi(1);
            ipHi = idxPhi(end);

            limit_roll(end+1,:) = [G.vc(iv),G.r(ir),G.phi(ipLo)]; %#ok<AGROW>

            if ipHi~=ipLo
                limit_roll(end+1,:) = [G.vc(iv),G.r(ir),G.phi(ipHi)]; %#ok<AGROW>
            end
        end
    end

    limit_roll = unique(limit_roll,'rows','stable');
    region.limit_roll_points = limit_roll;

    if size(limit_roll,1)<cfg.min_limit_points
        return;
    end

    % Keep this diagnostic center for compatibility with older MAT files.
    region.vc_split_center = median(stable_points(:,1));

    % ---------------------------------------------------------------------
    % STEP 2A: OUTER LATERAL-SPEED LIMIT
    % depth=1 means the outermost stable vc point on both sides.
    % ---------------------------------------------------------------------
    outer_limit = lateral_layer_from_limit_roll(limit_roll,G,1);
    usedFallback = false;

    if size(outer_limit,1)<cfg.min_limit_points && cfg.allow_stable_mask_fallback

        outer_limit = lateral_layer_from_stable_mask(G,stable_mask,1);
        usedFallback = true;
    end

    outer_limit = unique(outer_limit,'rows','stable');
    region.outer_limit_points = outer_limit;
    region.used_stable_mask_fallback = usedFallback;

    if size(outer_limit,1)<cfg.min_limit_points
        return;
    end

    Vouter = select_ordered_eight_vertices(outer_limit,G,stable_points);

    if size(Vouter,1)~=8
        return;
    end

    [outerVol,okOuter] = convex_volume(Vouter);

    if ~okOuter
        return;
    end

    insideOuter = points_inside_convex_hull( unstable_points,Vouter,cfg.hull_inside_tol);

    region.outer_vertices = Vouter;
    region.outer_volume = outerVol;
    region.unstable_inside_outer = nnz(insideOuter);

    % ---------------------------------------------------------------------
    % STEP 2B / STEP 3: INNER LATERAL-SPEED LAYER SEARCH
    %
    % depth=1 : outermost layer
    % depth=2 : one stable layer inward
    % depth=3 : two stable layers inward
    % and onward
    %
    % Among all candidate hexahedra containing zero sampled unstable points,
    % keep the LARGEST volume.  Thus we do not shrink more than necessary.
    % ---------------------------------------------------------------------
    maxDepth = max(1,ceil(numel(G.vc)/2));

    bestVol = -Inf;
    best = struct( 'limit',zeros(0,3), 'V',zeros(0,3), 'bad',inf, 'depth',NaN, 'fallback',false);

    for depth = cfg.inner_depth_start:maxDepth

        inner_limit = lateral_layer_from_limit_roll(limit_roll,G,depth);
        thisFallback = false;

        if size(inner_limit,1)<cfg.min_limit_points && cfg.allow_stable_mask_fallback

            inner_limit = lateral_layer_from_stable_mask( G,stable_mask,depth);
            thisFallback = true;
        end

        inner_limit = unique(inner_limit,'rows','stable');

        if size(inner_limit,1)<cfg.min_limit_points
            continue;
        end

        Vin = select_ordered_eight_vertices( inner_limit,G,stable_points);

        if size(Vin,1)~=8
            continue;
        end

        [v,ok] = convex_volume(Vin);

        if ~ok || ~(isfinite(v) && v>0)
            continue;
        end

        bad = nnz(points_inside_convex_hull( unstable_points,Vin,cfg.hull_inside_tol));

        fprintf('  Inner search depth=%2d | points=%4d | volume=%10.6g | bad=%d\n', depth,size(inner_limit,1),v,bad);

        if bad==0 && v>bestVol
            bestVol = v;
            best.limit = inner_limit;
            best.V = Vin;
            best.bad = bad;
            best.depth = depth;
            best.fallback = thisFallback;
        end
    end

    % If the outer layer itself is already safe, allow it as a valid region.
    if isempty(best.V) && region.unstable_inside_outer==0
        bestVol = outerVol;
        best.limit = outer_limit;
        best.V = Vouter;
        best.bad = 0;
        best.depth = 1;
        best.fallback = usedFallback;
    end

    % ---------------------------------------------------------------------
    % LAST-RESORT GEOMETRIC FALLBACK
    % Search directly among saved stable points with progressively inward
    % support points.  No vehicle simulation is rerun.
    % ---------------------------------------------------------------------
    if isempty(best.V)

        [Vs,bads,vols,depths] = support_depth_search( stable_points,unstable_points,G,cfg);

        if ~isempty(Vs)
            [bestVol,jj] = max(vols);

            best.V = Vs{jj};
            best.bad = bads(jj);
            best.depth = depths(jj);
            best.limit = stable_points;
            best.fallback = true;

            region.used_support_fallback = true;
        end
    end

    if isempty(best.V)

        % Important: do not throw an error here.
        % The expensive sweep data must still be returned and saved.
        warning(['No zero-unstable inner hexahedron was found from the ' 'current sampled grid. Raw Stable/Unstable data will ' 'still be saved for post-processing.']);
        return;
    end

    region.inner_limit_points = best.limit;
    region.inner_vertices = best.V;
    region.inner_volume = bestVol;
    region.unstable_inside_inner = best.bad;
    region.inner_depth = best.depth;
    region.is_valid_inner = (best.bad==0);
    region.used_stable_mask_fallback = region.used_stable_mask_fallback || best.fallback;

end

function Pts = lateral_layer_from_limit_roll(limit_roll,G,depth)

    % At equal (r,phi), sort limit-roll points by vc.
    % Select the depth-th point from the negative/left and positive/right
    % lateral-speed sides.

    Pts = zeros(0,3);
    tol = 1e-12;

    for ir=1:numel(G.r)

        rr = G.r(ir);

        for ip=1:numel(G.phi)

            ph = G.phi(ip);

            idx = abs(limit_roll(:,2)-rr)<=tol & abs(limit_roll(:,3)-ph)<=tol;

            Q = sortrows(limit_roll(idx,:),1);
            n = size(Q,1);

            if n < depth
                continue;
            end

            Pts(end+1,:) = Q(depth,:); %#ok<AGROW>

            j = n-depth+1;

            if j~=depth && j>=1 && j<=n
                Pts(end+1,:) = Q(j,:); %#ok<AGROW>
            end
        end
    end

    Pts = unique(Pts,'rows','stable');

end

function Pts = lateral_layer_from_stable_mask(G,stable_mask,depth)

    % Fallback for a discrete grid:
    % At each fixed (r,phi), select the depth-th stable vc point from both
    % sides using the already-computed stable_mask.

    Pts = zeros(0,3);

    for ir=1:numel(G.r)
        for ip=1:numel(G.phi)

            idxVc = find(squeeze(stable_mask(:,ir,ip)));
            n = numel(idxVc);

            if n < depth
                continue;
            end

            iL = idxVc(depth);
            iR = idxVc(n-depth+1);

            Pts(end+1,:) = [G.vc(iL),G.r(ir),G.phi(ip)]; %#ok<AGROW>

            if iR~=iL
                Pts(end+1,:) = [G.vc(iR),G.r(ir),G.phi(ip)]; %#ok<AGROW>
            end
        end
    end

    Pts = unique(Pts,'rows','stable');

end

function V = select_ordered_eight_vertices(Pts,G,stable_points)

    % Eight normalized support directions in [vc,r,phi].
    % Returned order:
    % 1 [- - -], 2 [- - +], 3 [- + -], 4 [- + +]
    % 5 [+ - -], 6 [+ - +], 7 [+ + -], 8 [+ + +]

    V = zeros(0,3);

    if size(Pts,1)<8
        return;
    end

    ranges = [ range_nonzero(G.vc), range_nonzero(G.r), range_nonzero(G.phi)];

    c = median(stable_points,1);
    Q = (Pts-c)./ranges;

    signs = [ -1 -1 -1; -1 -1 +1; -1 +1 -1; -1 +1 +1; +1 -1 -1; +1 -1 +1; +1 +1 -1; +1 +1 +1];

    chosen = false(size(Pts,1),1);
    V = nan(8,3);

    for k=1:8

        s = signs(k,:);

        % Prefer a point in the corresponding octant.
        oct = true(size(Pts,1),1);

        for j=1:3
            if s(j)<0
                oct = oct & (Q(:,j)<=1e-12);
            else
                oct = oct & (Q(:,j)>=-1e-12);
            end
        end

        score = Q*s(:);
        score(chosen) = -Inf;
        score(~oct) = -Inf;

        [best,idx] = max(score);

        % If the exact octant has no unused point, use the best unused point
        % along the same normalized support direction.
        if ~isfinite(best)
            score = Q*s(:);
            score(chosen) = -Inf;
            [best,idx] = max(score);
        end

        if ~isfinite(best)
            V = zeros(0,3);
            return;
        end

        V(k,:) = Pts(idx,:);
        chosen(idx) = true;
    end

    if size(unique(V,'rows'),1)~=8
        V = zeros(0,3);
    end

end

function [Vcells,badList,volList,depthList] = support_depth_search( stable_points,unstable_points,G,cfg)

    Vcells = {};
    badList = [];
    volList = [];
    depthList = [];

    if size(stable_points,1)<8
        return;
    end

    ranges = [ range_nonzero(G.vc), range_nonzero(G.r), range_nonzero(G.phi)];

    c = median(stable_points,1);
    Q = (stable_points-c)./ranges;

    signs = [ -1 -1 -1; -1 -1 +1; -1 +1 -1; -1 +1 +1; +1 -1 -1; +1 -1 +1; +1 +1 -1; +1 +1 +1];

    order = cell(8,1);

    for k=1:8
        score = Q*signs(k,:).';
        [~,order{k}] = sort(score,'descend');
    end

    maxDepth = min( cfg.max_support_depth, max(1,floor(size(stable_points,1)/8)));

    for depth=1:maxDepth

        used = false(size(stable_points,1),1);
        V = nan(8,3);
        ok = true;

        for k=1:8

            ord = order{k};
            pick = [];

            for jj=depth:numel(ord)

                ii = ord(jj);

                if ~used(ii)
                    pick = ii;
                    break;
                end
            end

            if isempty(pick)
                ok = false;
                break;
            end

            V(k,:) = stable_points(pick,:);
            used(pick) = true;
        end

        if ~ok || size(unique(V,'rows'),1)~=8
            continue;
        end

        [vol,okVol] = convex_volume(V);

        if ~okVol
            continue;
        end

        bad = nnz(points_inside_convex_hull( unstable_points,V,cfg.hull_inside_tol));

        fprintf('  Support fallback depth=%2d | volume=%10.6g | bad=%d\n', depth,vol,bad);

        if bad==0
            Vcells{end+1,1} = V; %#ok<AGROW>
            badList(end+1,1) = bad; %#ok<AGROW>
            volList(end+1,1) = vol; %#ok<AGROW>
            depthList(end+1,1) = depth; %#ok<AGROW>
        end
    end

end

function y = range_nonzero(x)

    y = max(x)-min(x);

    if ~(isfinite(y) && y>0)
        y = 1;
    end

end

function V = select_eight_hexahedron_vertices(Pts,G)

    V = zeros(0,3);

    if size(Pts,1)<8

        return;

    end

    scale = [max(max(G.vc)-min(G.vc),eps), max(max(G.r)-min(G.r),eps), max(max(G.phi)-min(G.phi),eps)];

    vcMid = median(Pts(:,1));

    leftIdx = find(Pts(:,1)<=vcMid);

    rightIdx = find(Pts(:,1)>=vcMid);

    if numel(leftIdx)<4 || numel(rightIdx)<4

        % Fallback: split by sorted lateral speed into two overlapping halves.

        [~,ord] = sort(Pts(:,1));

        n = numel(ord);

        leftIdx = ord(1:max(4,ceil(n/2)));

        rightIdx = ord(min(n-3,floor(n/2)+1):n);

    end

    VL = select_four_rphi_extremes(Pts(leftIdx,:),scale);

    VR = select_four_rphi_extremes(Pts(rightIdx,:),scale);

    V = unique([VL;VR],'rows','stable');

    if size(V,1)<8

        % Fill deterministically using points farthest from the current center.

        c = median(Pts,1);

        d2 = sum(((Pts-c)./scale).^2,2);

        [~,ord] = sort(d2,'descend');

        for ii=ord(:).'

            p = Pts(ii,:);

            if ~ismember(p,V,'rows')

                V(end+1,:) = p; %#ok<AGROW>

            end

            if size(V,1)>=8

                break;

            end

        end

    end

    if size(V,1)>8

        V = reduce_to_max_volume_eight(V);

    end

end

function V4 = select_four_rphi_extremes(Pts,scale)

    V4 = zeros(0,3);

    if isempty(Pts)

        return;

    end

    c = median(Pts,1);

    Q = (Pts-c)./scale;

    dirs = [-1 -1; -1 +1; +1 -1; +1 +1];

    chosen = zeros(0,1);

    for k=1:4

        score = Q(:,2)*dirs(k,1) + Q(:,3)*dirs(k,2);

        [~,ord] = sort(score,'descend');

        pick = [];

        for jj=ord(:).'

            if ~ismember(jj,chosen)

                pick=jj;

                break;

            end

        end

        if isempty(pick)

            pick=ord(1);

        end

        chosen(end+1,1)=pick; %#ok<AGROW>

    end

    V4 = unique(Pts(chosen,:),'rows','stable');

    if size(V4,1)<4

        d2 = Q(:,2).^2 + Q(:,3).^2;

        [~,ord] = sort(d2,'descend');

        for jj=ord(:).'

            p=Pts(jj,:);

            if ~ismember(p,V4,'rows')

                V4(end+1,:)=p; %#ok<AGROW>

            end

            if size(V4,1)>=4

                break;

            end

        end

    end

end

function V8 = reduce_to_max_volume_eight(V)

    % Normally V already has exactly 8 points.  This deterministic fallback

    % removes points one-by-one while retaining the largest convex volume.

    V8=V;

    while size(V8,1)>8

        bestVol=-Inf;

        bestIdx=1;

        for i=1:size(V8,1)

            T=V8;

            T(i,:)=[];

            [vol,ok]=convex_volume(T);

            if ok && vol>bestVol

                bestVol=vol;

                bestIdx=i;

            end

        end

        V8(bestIdx,:)=[];

    end

end

function inside = points_inside_convex_hull(P,V,tol)

    inside=false(size(P,1),1);

    if isempty(P) || size(V,1)<4

        return;

    end

    try

        K=convhulln(V);

    catch

        return;

    end

    c=mean(V,1);

    for i=1:size(P,1)

        p=P(i,:);

        isIn=true;

        for f=1:size(K,1)

            tri=V(K(f,:),:);

            n=cross(tri(2,:)-tri(1,:),tri(3,:)-tri(1,:));

            nn=norm(n);

            if nn<eps

                continue;

            end

            n=n/nn;

            if dot(n,c-tri(1,:))>0

                n=-n;

            end

            if dot(n,p-tri(1,:))>tol

                isIn=false;

                break;

            end

        end

        inside(i)=isIn;

    end

end

function [vol,ok] = convex_volume(V)

    vol=NaN;

    ok=false;

    if size(V,1)<4 || rank(V-mean(V,1))<3

        return;

    end

    try

        [~,vol]=convhulln(V);

        ok=isfinite(vol) && vol>0;

    catch

        vol=NaN;

        ok=false;

    end

end



% BEGIN VERBATIM: region210/empty_region.m; SHA256 df191ab4f4e741be23ecd7d919440f653a633c88416a1d826b7b06d4bbd8f0cd
function R = empty_region()

    R = struct( 'limit_roll_points',zeros(0,3), 'outer_limit_points',zeros(0,3), 'inner_limit_points',zeros(0,3), 'outer_vertices',zeros(0,3), 'inner_vertices',zeros(0,3), 'outer_volume',NaN, 'inner_volume',NaN, 'unstable_inside_outer',NaN, 'unstable_inside_inner',NaN, 'inner_depth',NaN, 'is_valid_inner',false, 'vc_split_center',NaN, 'used_stable_mask_fallback',false, 'used_support_fallback',false);

end



function fig = plot_mf210_hybrid(result_dir,output_base)
%PLOT_MF210_HYBRID Plot unchanged original-210 inner vertices, common camera.
% Display coordinate order is [r,vc,phi]; data remain in [vc,r,phi].
% This function only loads saved MAT results and never changes their geometry.
  if nargin<1 || isempty(result_dir)
    result_dir=fullfile(fileparts(mfilename('fullpath')),'results');
  end
  if nargin<2 || isempty(output_base)
    output_base=fullfile(result_dir,'mf210_hybrid_regions');
  end
  tags={'0','3.5','7'};
  colors=[.64 .92 .66;.95 .93 .20;.09 .24 .90];
  alpha=[.16 .24 .65];
  fig=figure('Color','w','Position',[100 100 780 760]);
  ax=axes('Parent',fig);hold(ax,'on');
  handles=zeros(1,0);labels={};
  for ii=1:numel(tags)
    region_file=fullfile(result_dir,['region210_' tags{ii} 'deg.mat']);
    if ~exist(region_file,'file')
      region_file=fullfile(result_dir,['stable_point_40kph_0.85mu_' tags{ii} 'deg.mat']);
    end
    loaded=load(region_file,'REGION210');
    selected=loaded.REGION210.region;
    if ~selected.is_valid_inner || size(selected.inner_vertices,1)<4
      warning('No valid inner region at delta=%s degrees; raw classifications remain saved.',tags{ii});
      continue;
    end
    V=selected.inner_vertices(:,[2 1 3]);
    faces=convhulln(V);
    p=patch('Parent',ax,'Vertices',V,'Faces',faces, ...
      'FaceColor',colors(ii,:),'FaceAlpha',alpha(ii),'EdgeColor','none');
    if isempty(handles),handles=p;else,handles(end+1)=p;end %#ok<AGROW>
    labels{end+1}=sprintf('delta = %s deg',tags{ii}); %#ok<AGROW>
    edges=unique(sort([faces(:,[1 2]);faces(:,[2 3]);faces(:,[3 1])],2),'rows');
    % Remove triangulation diagonals shared by coplanar hull triangles.
    for jj=1:size(edges,1)
      e=edges(jj,:);
      adjacent=find(sum(ismember(faces,e),2)==2);
      diagonal=false;
      if numel(adjacent)==2
        T1=V(faces(adjacent(1),:),:);T2=V(faces(adjacent(2),:),:);
        n1=cross(T1(2,:)-T1(1,:),T1(3,:)-T1(1,:));
        n2=cross(T2(2,:)-T2(1,:),T2(3,:)-T2(1,:));
        diagonal=norm(cross(n1,n2))<=1e-10*norm(n1)*norm(n2);
      end
      if ~diagonal
        line('Parent',ax,'XData',V(e,1),'YData',V(e,2),'ZData',V(e,3), ...
          'Color',colors(ii,:)*.65,'LineWidth',1.0,'HandleVisibility','off');
      end
    end
  end
  % Identical fixed axes and orthographic camera for all steering cases.
  set(ax,'XLim',[-.72 .72],'XDir','reverse','YLim',[-5 5], ...
    'ZLim',[-.45 .45],'XTick',-.6:.2:.6,'YTick',[-5 0 5], ...
    'ZTick',-.4:.2:.4,'Projection','orthographic','Box','off');
  view(ax,-72,11);pbaspect(ax,[1.3 2.28 1.65*.9/.84]);grid(ax,'off');
  xlabel(ax,'r (rad/s)');ylabel(ax,'v_c (m/s)');zlabel(ax,'phi (rad)');
  title(ax,'40 km/h fixed, mu = 0.85 | MF210 transient hybrid');
  if ~isempty(handles),legend(ax,handles,labels,'Location','northwest');end
  set(fig,'PaperPositionMode','auto');
  print(fig,[output_base '.png'],'-dpng','-r180');
  if exist('OCTAVE_VERSION','builtin')
    hgsave(fig,[output_base '.ofig']);
  else
    savefig(fig,[output_base '.fig']);
  end
end

