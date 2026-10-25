function files=run_mf210_paper_roll(output_dir,make_plots,smoke_test)
%RUN_MF210_PAPER_ROLL Standalone five-state quasisteady MF reconstruction.
% MATLAB R2016b or newer; base MATLAB only; no external helper is required.
%   run_mf210_paper_roll
%   run_mf210_paper_roll('my_results',false)
%   run_mf210_paper_roll('smoke_results',false,true) % tiny runtime check only
%
% Production: 81x55x31 states per angle, 40 km/h fixed, mu=0.85, 5 s,
% 0.01 s classification observations and 20 RK4 substeps per observation.
% Original210 healthy MF force coefficients/shifts and mu/.9 are retained.
% Tires are explicitly QUASISTEADY: instantaneous wheel alpha and the original
% longitudinal shifted free-rolling root kappa=-Shx. Original210 hidden tire
% relaxation and wheel dynamics are replaced, not claimed to be retained.
% Paper body equations remain; dynamic normal reaction is a physical extension
% of printed Eq11, not a literal reproduction of the Burckhardt paper model.
% Each INNER_REGION is a global maximum-volume grid-aligned all-stable cuboid
% containing its OWN continued stable equilibrium. No centering, aspect ratio,
% cross-angle containment, roll cap, or nesting condition is imposed.
  if nargin<1 || isempty(output_dir)
    output_dir=fullfile(fileparts(mfilename('fullpath')),'results_matlab');
  end
  if nargin<2 || isempty(make_plots), make_plots=true; end
  if nargin<3 || isempty(smoke_test), smoke_test=false; end
  assert(isscalar(make_plots) && isscalar(smoke_test),'Flags must be scalar');
  if ~exist(output_dir,'dir'), mkdir(output_dir); end
  base_cfg=delivery_configuration();
  provenance=delivery_provenance();
  angles=[0 3.5 7];
  [equilibria,continuation]=compute_mf210_body_equilibria(base_cfg,angles);
  save(fullfile(output_dir,'equilibrium_continuation.mat'), ...
    'equilibria','continuation','base_cfg','provenance','-v7');
  if smoke_test
    fprintf('VALIDATION SMOKE TEST: each own equilibrium +/- small fixed spacings; 3x3x3 grid, 0.02 s.\n');
  end
  files=cell(numel(angles),2);
  for ii=1:numel(angles)
    cfg=base_cfg; cfg.steer_deg=angles(ii);
    EQUILIBRIUM=equilibria{ii};
    anchor=EQUILIBRIUM.anchor_vc_r_phi;
    if smoke_test
      cfg.grid.vc=anchor(1)+[-0.01 0 0.01];
      cfg.grid.r=anchor(2)+[-0.005 0 0.005];
      cfg.grid.phi=anchor(3)+[-0.002 0 0.002];
      cfg.horizon_s=0.02; cfg.batch_size=27;
    end
    tag=sprintf('%g',cfg.steer_deg);
    fprintf('\nQuasisteady MF, dynamic roll reaction: 40 km/h, mu=0.85, delta=%s deg\n',tag);
    fprintf('Own equilibrium [vc r phi]=[%.12g %.12g %.12g], max real eigenvalue=%.9g\n', ...
      anchor,EQUILIBRIUM.maximum_dynamic_eigenvalue_real_part);
    result=paper_simulate_grid(cfg);
    result.unstable_points=result.initial_points(~result.stable_mask(:),:);
    result.definition=['Finite-horizon classification of five-state paper body, quasisteady original210 MF ' ...
      'force kernel and dynamic normal reaction; classified samples only'];
    result.raw_stable_bounds_axis_names={'vc','r','phi'};
    if isempty(result.stable_points), result.raw_stable_bounds=nan(2,3);
    else, result.raw_stable_bounds=[min(result.stable_points,[],1);max(result.stable_points,[],1)]; end
    meta=struct('version','paper_body_quasisteady_mf210_rollreaction_local_cuboid_v1', ...
      'engine','Standalone MATLAB M-code', ...
      'model','Five-state paper body; quasisteady original210 MF forces; dynamic roll-reaction extension', ...
      'paper_doi',cfg.paper_doi,'paper_exact_reproduction',false, ...
      'original_fullcar_body_model_used',false,'original_full_tire_transient_model',false, ...
      'tire_force_coefficients_changed',false,'all_original_force_shifts_retained',true, ...
      'tire_dynamic_relaxation_included',false,'free_wheel_dynamic_states_included',false, ...
      'body_model','Unchanged paper Eqs2/3/4; explicit physical extension of Eq11; fixed speed', ...
      'normal_load_reaction','D=Cphi*p+Kphi*phi+ms*g*hphi*(sin(phi)-phi)', ...
      'normal_load_baseline','m=ms=1300 kg, h=hphi=0.54 m; ay coefficient exactly zero', ...
      'state_order','u,vc,r,phi,p','initialization',cfg.initialization, ...
      'tire_algebraic_constraint','Exact instantaneous wheel alpha; kappa=-Shx solves original shifted Fx=0', ...
      'mu_scaling','mu/.9; original MF peak is not normalized to the road coefficient', ...
      'geometry','Independent maximum-volume all-stable grid cuboid containing its own continued stable equilibrium', ...
      'geometry_is_author_method',false,'geometry_is_raw_basin',false, ...
      'equilibrium_containment_required',true,'equilibrium_centering_required',false, ...
      'aspect_ratio_constraints_used',false,'nesting_constraints_used',false, ...
      'angle_dependent_geometry_constraints_used',false,'roll_scaling_or_caps_applied',false, ...
      'selection_independent_of_other_angles',true,'natural_nesting_claimed',false, ...
      'geometry_guarantee','All classified grid nodes in the cuboid are stable; unsampled states unverified', ...
      'validation_smoke_test',logical(smoke_test),'source_provenance',provenance);
    PAPER_STABILITY3D=struct('meta',meta,'cfg',cfg,'result',result);
    files{ii,1}=fullfile(output_dir,['classification_' tag 'deg.mat']);
    % Preserve the classification and its equilibrium before geometry extraction.
    save(files{ii,1},'PAPER_STABILITY3D','EQUILIBRIUM','-v7');
    assert(EQUILIBRIUM.locally_asymptotically_stable_at_fixed_u && ...
      EQUILIBRIUM.admissible_under_instantaneous_constraints,'Equilibrium is ineligible');
    INNER_REGION=build_local_equilibrium_cuboid(result.stable_mask,cfg.grid,anchor);
    INNER_REGION.method='Exact maximum-volume all-stable grid cuboid containing its own physical stable equilibrium';
    INNER_REGION.axis_order={'vc','r','phi'};
    INNER_REGION.angle_deg=cfg.steer_deg;
    INNER_REGION.selection_independent_of_other_angles=true;
    INNER_REGION.equilibrium_containment_required=true;
    INNER_REGION.equilibrium_centering_required=false;
    INNER_REGION.aspect_ratio_constraints_used=false;
    INNER_REGION.nesting_constraints_applied=false;
    INNER_REGION.angle_dependent_constraints_applied=false;
    INNER_REGION.geometry_is_author_method=false;
    INNER_REGION.geometry_is_raw_basin=false;
    INNER_REGION.continuous_states_between_grid_nodes_certified=false;
    INNER_REGION.natural_nesting_claimed=false;
    INNER_REGION.classification_filename=['classification_' tag 'deg.mat'];
    files{ii,2}=fullfile(output_dir,['stable_point_40kph_0.85mu_' tag 'deg.mat']);
    save(files{ii,2},'PAPER_STABILITY3D','INNER_REGION','EQUILIBRIUM','-v7');
    assert(INNER_REGION.valid,'No positive-volume all-stable cuboid contains this equilibrium');
    fprintf('Saved: stable=%d/%d; local cuboid volume=%.12g; ties=%d\n', ...
      result.stable_count,result.total_count,INNER_REGION.volume,INNER_REGION.maximum_volume_tie_count);
  end
  if make_plots
    plot_mf210_paper_roll(output_dir,fullfile(output_dir,'mf210_paper_roll_regions'));
  end
end

function cfg=delivery_configuration()
cfg=struct();
cfg.paper_doi='10.1016/j.aei.2022.101764';
cfg.m=1300;
cfg.ms=1300;
cfg.Izz=2500;
cfg.Ixx=700;
cfg.L=2.3999999999999999;
cfg.lf=1.2;
cfg.lr=1.2;
cfg.df=0.69999999999999996;
cfg.dr=0.69999999999999996;
cfg.h=0.54000000000000004;
cfg.hphi=0.54000000000000004;
cfg.g=9.8100000000000005;
cfg.Kphi=43100;
cfg.Cphi=1400;
cfg.deltaFz=[0 0 0 0];
cfg.mu=0.84999999999999998;
cfg.speed_kmh=40;
cfg.steer_deg=0;
cfg.speed_mode='fixed';
cfg.horizon_s=5;
cfg.dt=0.01;
cfg.method='rk4';
cfg.Nsub=20;
cfg.integration_dt=0.00050000000000000001;
cfg.batch_size=3500;
cfg.limit_factor=0.84999999999999998;
cfg.min_u=0.5;
cfg.filename_digits=3;
cfg.grid=struct('vc',[-10 -9.75 -9.5 -9.25 -9 -8.75 -8.5 -8.25 -8 -7.75 -7.5 -7.25 -7 -6.75 -6.5 -6.25 -6 -5.75 -5.5 -5.25 -5 -4.75 -4.5 -4.25 -4 -3.75 -3.5 -3.25 -3 -2.75 -2.5 -2.25 -2 -1.75 -1.5 -1.25 -1 -0.75 -0.5 -0.25 0 0.25 0.5 0.75 1 1.25 1.5 1.75 2 2.25 2.5 2.75 3 3.25 3.5 3.75 4 4.25 4.5 4.75 5 5.25 5.5 5.75 6 6.25 6.5 6.75 7 7.25 7.5 7.75 8 8.25 8.5 8.75 9 9.25 9.5 9.75 10],'r',[-0.72000000000000008 -0.69333333333333336 -0.66666666666666674 -0.64000000000000001 -0.6133333333333334 -0.58666666666666667 -0.56000000000000005 -0.53333333333333333 -0.50666666666666671 -0.48000000000000004 -0.45333333333333337 -0.42666666666666669 -0.40000000000000002 -0.37333333333333335 -0.34666666666666668 -0.32000000000000001 -0.29333333333333333 -0.26666666666666666 -0.24000000000000002 -0.21333333333333335 -0.18666666666666668 -0.16 -0.13333333333333333 -0.10666666666666667 -0.080000000000000002 -0.053333333333333337 -0.026666666666666668 0 0.026666666666666668 0.053333333333333337 0.080000000000000002 0.10666666666666667 0.13333333333333333 0.16 0.18666666666666668 0.21333333333333335 0.24000000000000002 0.26666666666666666 0.29333333333333333 0.32000000000000001 0.34666666666666668 0.37333333333333335 0.40000000000000002 0.42666666666666669 0.45333333333333337 0.48000000000000004 0.50666666666666671 0.53333333333333333 0.56000000000000005 0.58666666666666667 0.6133333333333334 0.64000000000000001 0.66666666666666674 0.69333333333333336 0.72000000000000008],'phi',[-0.40000000000000002 -0.37333333333333335 -0.34666666666666668 -0.32000000000000001 -0.29333333333333333 -0.26666666666666666 -0.24000000000000002 -0.21333333333333335 -0.18666666666666668 -0.16 -0.13333333333333333 -0.10666666666666667 -0.080000000000000002 -0.053333333333333337 -0.026666666666666668 0 0.026666666666666668 0.053333333333333337 0.080000000000000002 0.10666666666666667 0.13333333333333333 0.16 0.18666666666666668 0.21333333333333335 0.24000000000000002 0.26666666666666666 0.29333333333333333 0.32000000000000001 0.34666666666666668 0.37333333333333335 0.40000000000000002]);
cfg.tire_model='Original210 healthy MF kernel with algebraic free-rolling slip';
cfg.mf_Fz0=5425;
cfg.mf_force_min=100;
cfg.mf_force_max=16000;
cfg.mf_mu_reference=0.90000000000000002;
cfg.mf_mu_scaling_rule='mu_road / 0.9';
cfg.mf_wear=[0 0 0 0];
cfg.slip_angle_mode='Instantaneous exact original210 wheel-frame velocity';
cfg.tire_dynamics='Quasisteady free rolling: solve kappa=-Shx, Fx=0';
cfg.model_dimension=5;
cfg.free_rolling_constraint='kappa=0.001635-0.0025297*((clippedFz-5425)/5425), original Vx offset=0';
cfg.load_solver='explicit dynamic roll-reaction extension; zero baseline ay coefficient';
cfg.load_transfer_model='Dynamic roll-reaction extension of printed paper Eq11';
cfg.roll_reaction_equation='D=(m*h-ms*hphi)*ay+Cphi*p+Kphi*phi+ms*g*hphi*(sin(phi)-phi)';
cfg.roll_reaction_ay_coefficient=0;
cfg.paper_exact_reproduction=0;
cfg.initialization='u=speed, vc/r/phi=grid, p=0; instantaneous algebraic tire forces';
cfg.reconstruction_version='PRIVATE-XiaoBody-MF210-QuasisteadyFreeRolling-DynamicRollReaction-v1';
end

function provenance=delivery_provenance()
provenance=struct();
provenance.model='Five-state paper body, original210 quasisteady MF force kernel, dynamic normal-reaction extension';
provenance.original_MF_source_sha256='29c4062948d71dd794d042233025718097ad9654526f64fd7699f63321cd0e7c';
provenance.original_MF_verbatim_block_sha256='a26aaa7cc85deb673dbb7c78d0ae0fb307a4aa472d64aec546804a3f90348783';
provenance.configuration_source='vc_refined_5s/classification_0deg.mat';
provenance.configuration_source_sha256='a6665a2ada0d2180d3a234533b55d7e13569f18c02465c71839a6d4ab642faa9';
provenance.frozen_dynamic_source_files={'matlab/paper_simulate_grid.m','matlab/paper_initial.m','matlab/paper_step.m','matlab/paper_rhs.m','matlab/mf210_forces.m'};
provenance.frozen_dynamic_source_sha256={'afd504d724fe6f22c826d26b47a697c4cd0d398c44d5137cfe02a25bf67f6094','4af0ce55d09284549e411e6e2dc8c9b5731ee362545a909748955894a8cdb2bd','aa9422ea10f514b1ee6a9652bb1ffe0944ae3f5a394bca8c33072e6cb83049fe','0cdca91157f4a62164d657770aa7b5abb38f2d9bc088ab9cc84f3628ea5bf692','b61291c65052ef153d81f91bb2dcb272d0e496602245a763f2eab417ee221b65'};
provenance.native_cpp_source_sha256='16af6956d881dc9b726bc34b06412d25e4f5fd00fff2ae02dbdab535b53941f9';
provenance.equilibrium_newton_source_sha256='26d3fe364d0c94efc293be4638395b83143b5574b246e2b0edf6d5505d259a2d';
provenance.geometry_helper_source_sha256='20fb5ced7290c3e23e022908692630aee60cc7621abf27b83067884e0c05b6b8';
provenance.paper_exact_reproduction=false;
provenance.original_full_tire_transient_model=false;
provenance.tire_force_coefficients_changed=false;
provenance.selection_independent_of_other_angles=true;
provenance.equilibrium_centering_required=false;
provenance.nesting_constraints_applied=false;
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


% BEGIN VERBATIM: matlab/paper_initial.m; SHA256 4af0ce55d09284549e411e6e2dc8c9b5731ee362545a909748955894a8cdb2bd
function x=paper_initial(points,cfg)
% Five body states only; algebraic tire variables have no initial conditions.
x=[repmat(cfg.speed_kmh/3.6,size(points,1),1),points,zeros(size(points,1),1)];
assert(size(x,2)==5);
end


% BEGIN VERBATIM: matlab/paper_step.m; SHA256 aa9422ea10f514b1ee6a9652bb1ffe0944ae3f5a394bca8c33072e6cb83049fe
function next=paper_step(x,cfg)
% Five-state RK4, 20 substeps per .01s observation; no hidden tire clamps.
h=cfg.dt/cfg.Nsub;next=x;
for j=1:cfg.Nsub
    k1=paper_rhs(next,cfg);k2=paper_rhs(next+.5*h*k1,cfg);
    k3=paper_rhs(next+.5*h*k2,cfg);k4=paper_rhs(next+h*k3,cfg);
    next=next+(h/6)*(k1+2*k2+2*k3+k4);
end
end


% BEGIN VERBATIM: matlab/paper_rhs.m; SHA256 0cdca91157f4a62164d657770aa7b5abb38f2d9bc088ab9cc84f3628ea5bf692
function [dx,obs]=paper_rhs(x,cfg)
% PRIVATE algebraic free rolling MF comparison; x=[u vc r phi p].
% Same paper body ODE and same original210 tire force coefficients and shifts.
u=x(:,1);v=x(:,2);r=x(:,3);phi=x(:,4);p=x(:,5);
delta=cfg.steer_deg*pi/180;cs=cos(delta);sn=sin(delta);
assert(cfg.m*cfg.h-cfg.ms*cfg.hphi==0,'Baseline-only experiment');
D=cfg.Cphi*p+cfg.Kphi*phi+cfg.ms*cfg.g*cfg.hphi*(sin(phi)-phi);
front=D*cfg.lr/(2*cfg.df*cfg.L);rear=D*cfg.lf/(2*cfg.dr*cfg.L);
f0=cfg.m*cfg.g*cfg.lr/(2*cfg.L);r0=cfg.m*cfg.g*cfg.lf/(2*cfg.L);
Fz=bsxfun(@plus,[f0-front f0+front r0-rear r0+rear],cfg.deltaFz);
uB=[u-cfg.df*r,u+cfg.df*r,u-cfg.dr*r,u+cfg.dr*r];
vB=[v+cfg.lf*r,v+cfg.lf*r,v-cfg.lr*r,v-cfg.lr*r];
uW=[cs*uB(:,1:2)+sn*vB(:,1:2),uB(:,3:4)];
vW=[-sn*uB(:,1:2)+cs*vB(:,1:2),vB(:,3:4)];
safe=uW;safe(safe>=0&safe<.1)=.1;safe(safe<0&safe>-.1)=-.1;
alpha=atan2(vW,safe);
z=min(cfg.mf_force_max,max(cfg.mf_force_min,Fz));
kappa=.001635-.0025297*((z-cfg.mf_Fz0)/cfg.mf_Fz0);
[FxT,FyT]=mf210_forces(alpha,kappa,Fz,cfg);
Fx=cs*(FxT(:,1)+FxT(:,2))+FxT(:,3)+FxT(:,4)-sn*(FyT(:,1)+FyT(:,2));
Fy=sn*(FxT(:,1)+FxT(:,2))+cs*(FyT(:,1)+FyT(:,2))+FyT(:,3)+FyT(:,4);ay=Fy/cfg.m;
Mz=cs*cfg.df*(FxT(:,2)-FxT(:,1))+cfg.dr*(FxT(:,4)-FxT(:,3)) ...
    +sn*cfg.df*(FyT(:,1)-FyT(:,2))+sn*cfg.lf*(FxT(:,1)+FxT(:,2)) ...
    +cs*cfg.lf*(FyT(:,1)+FyT(:,2))-cfg.lr*(FyT(:,3)+FyT(:,4));
udot=Fx/cfg.m+v.*r;if strcmp(cfg.speed_mode,'fixed'),udot=zeros(size(v));end
deltaMx=(cfg.deltaFz(2)-cfg.deltaFz(1))*cfg.df+(cfg.deltaFz(4)-cfg.deltaFz(3))*cfg.dr;
pdot=(cfg.ms*cfg.hphi*ay-cfg.Cphi*p+(cfg.ms*cfg.g*cfg.hphi-cfg.Kphi)*phi-deltaMx)/cfg.Ixx;
dx=[udot,ay-u.*r,Mz/cfg.Izz,p,pdot];
invalid=any(~isfinite(x),2);dx(invalid,:)=NaN;
if nargout>1
    ay(invalid)=NaN;Fz(invalid,:)=NaN;FxT(invalid,:)=NaN;FyT(invalid,:)=NaN;
    obs=struct('ay',ay,'Fz_raw',Fz,'Fx_tires',FxT,'Fy_tires',FyT,'Fx',Fx,'Fy',Fy,'Mz',Mz, ...
        'alpha',alpha,'kappa',kappa,'wheel_longitudinal_velocity',uW,'roll_reaction_moment',D, ...
        'load_transfer_model',cfg.load_transfer_model,'tire_model',cfg.tire_model);
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


function [selected,continuation] = compute_mf210_body_equilibria(cfg,angles)
%COMPUTE_MF210_BODY_EQUILIBRIA Toolbox-free physical steering continuation.
% Solve [vdot,rdot,pdot]=0 for [vc,r,phi], with p=0 and fixed u.
% Damped Newton uses a central finite-difference 3x3 Jacobian. Follow the
% small-slip branch from zero steering with increments no larger than 0.1 deg.
% The full 4x4 [vc,r,phi,p] Jacobian must be locally asymptotically stable;
% wheel-contact, yaw, lateral-acceleration and model-domain checks must pass.
% This computes equilibria from the RHS; no saved equilibrium is prescribed.
  if nargin<2 || isempty(angles), angles=[0 3.5 7]; end
  assert(strcmp(cfg.speed_mode,'fixed'),'Equilibrium solver requires fixed speed');
  assert(isvector(angles) && all(isfinite(angles)) && all(angles>=0), ...
    'Continuation angles must be finite and nonnegative');
  angles=angles(:)';
  step_deg=0.1;
  maximum_angle=max(angles);
  trace_angles=unique([0:step_deg:maximum_angle,maximum_angle,angles]);
  selected=cell(1,numel(angles));
  continuation=[];
  q=zeros(1,3);
  for ii=1:numel(trace_angles)
    angle=trace_angles(ii);
    local_cfg=cfg; local_cfg.steer_deg=angle;
    [q,iterations,damping_halvings]=body_newton(q,local_cfg);
    item=describe_body_equilibrium(q,local_cfg);
    item.newton_iterations=iterations;
    item.newton_damping_halvings=damping_halvings;
    item.continuation_step_deg=step_deg;
    assert(item.accepted_equilibrium,'Equilibrium residual exceeds tolerance');
    assert(item.locally_asymptotically_stable_at_fixed_u, ...
      'Continued equilibrium is not locally asymptotically stable');
    assert(item.admissible_under_instantaneous_constraints, ...
      'Continued equilibrium violates a physical stability criterion');
    if isempty(continuation), continuation=item; else, continuation(end+1)=item; end %#ok<AGROW>
    matches=find(abs(angles-angle)<=1e-12);
    for jj=matches
      selected{jj}=item;
    end
  end
  assert(all(~cellfun(@isempty,selected)),'A requested steering equilibrium is missing');
end

function [q,iterations,total_halvings]=body_newton(q,cfg)
  tolerance=1e-12; maximum_iterations=60; total_halvings=0;
  for iterations=0:maximum_iterations
    f=body_equilibrium_residual(q,cfg);
    residual=norm(f,inf);
    assert(all(isfinite(f)),'Nonfinite Newton residual');
    if residual<=tolerance, return; end
    assert(iterations<maximum_iterations,'Newton equilibrium iteration limit reached');
    jac=zeros(3,3);
    for kk=1:3
      h=1e-5*max(1,abs(q(kk)));
      plus=q; minus=q; plus(kk)=plus(kk)+h; minus(kk)=minus(kk)-h;
      jac(:,kk)=(body_equilibrium_residual(plus,cfg)-body_equilibrium_residual(minus,cfg))/(2*h);
    end
    assert(all(isfinite(jac(:))) && rcond(jac)>1e-14,'Singular Newton equilibrium Jacobian');
    direction=-(jac\f)';
    damping=1; accepted=false;
    for halvings=0:30
      trial=q+damping*direction;
      candidate=body_equilibrium_residual(trial,cfg);
      if all(isfinite(candidate)) && ...
          (norm(candidate,inf)<=tolerance || norm(candidate,inf)<residual*(1-1e-4*damping))
        q=trial; accepted=true; total_halvings=total_halvings+halvings; break;
      end
      damping=damping/2;
    end
    assert(accepted,'Damped Newton equilibrium step could not decrease the residual');
  end
end

function f=body_equilibrium_residual(q,cfg)
  dx=paper_rhs([cfg.speed_kmh/3.6,q,0],cfg);
  f=dx([2 3 5])';
end

function jac=body_dynamic_jacobian(z,cfg,scale)
  jac=zeros(4,4);
  for kk=1:4
    h=scale*max(1,abs(z(kk)));
    plus=z; minus=z; plus(kk)=plus(kk)+h; minus(kk)=minus(kk)-h;
    dp=paper_rhs([cfg.speed_kmh/3.6,plus],cfg);
    dm=paper_rhs([cfg.speed_kmh/3.6,minus],cfg);
    jac(:,kk)=(dp(2:5)-dm(2:5))'/(2*h);
  end
end

function item=describe_body_equilibrium(q,cfg)
  state=[cfg.speed_kmh/3.6,q,0];
  [dx,obs]=paper_rhs(state,cfg);
  jac=body_dynamic_jacobian(state(2:5),cfg,1e-5);
  refined=body_dynamic_jacobian(state(2:5),cfg,5e-6);
  eigenvalues=eig(jac);
  maximum_real=max(real(eigenvalues));
  residual=max(abs(dx(2:5)));
  yaw_limit=cfg.limit_factor*cfg.mu*cfg.g/max(state(1),cfg.min_u);
  constraints=struct('positive_raw_normal_loads',all(obs.Fz_raw>0), ...
    'yaw_limit_satisfied',abs(state(3))<=yaw_limit+1e-10, ...
    'lateral_acceleration_limit_satisfied',abs(obs.ay)<=cfg.mu*cfg.g+1e-10, ...
    'finite_state_and_rhs',all(isfinite([state dx obs.Fz_raw obs.ay])), ...
    'fixed_speed_model_domain',state(1)>=cfg.min_u);
  item=struct();
  item.angle_deg=cfg.steer_deg;
  item.state_order={'u','vc','r','phi','p'};
  item.state=state;
  item.dynamic_state_order={'vc','r','phi','p'};
  item.dynamic_state=state(2:5);
  item.anchor_vc_r_phi=q;
  item.maximum_abs_body_RHS_residual=residual;
  item.accepted_equilibrium=residual<1e-8;
  item.ay_m_s2=obs.ay;
  item.raw_normal_loads_N=obs.Fz_raw;
  item.instantaneous_constraints=constraints;
  item.admissible_under_instantaneous_constraints=all(cell2mat(struct2cell(constraints)));
  item.satisfies_stability_criteria=item.admissible_under_instantaneous_constraints;
  item.dynamic_jacobian=jac;
  item.jacobian_step_halving_max_abs_change=max(abs(jac(:)-refined(:)));
  item.dynamic_eigenvalues=eigenvalues;
  item.maximum_dynamic_eigenvalue_real_part=maximum_real;
  item.locally_asymptotically_stable_at_fixed_u=item.accepted_equilibrium && maximum_real< -1e-7;
  item.locally_asymptotically_stable=item.locally_asymptotically_stable_at_fixed_u;
  item.local_stability_scope='Four body coordinates at fixed u; the full five-state Jacobian also has a fixed-u zero eigenvalue';
  item.method='Damped Newton on [vc r phi] with p=0; central finite differences; continuation from zero steer';
  item.root_branch='Stable small-slip branch continuously followed from straight running';
  item.global_uniqueness_claimed=false;
end

function result = build_local_equilibrium_cuboid(stable_mask,G,equilibrium_point)
%BUILD_LOCAL_EQUILIBRIUM_CUBOID Exact largest all-stable box containing equilibrium.
% equilibrium_point = [vc,r,phi] is an anchor to contain, not a prescribed
% center. The same objective and tolerances apply independently at every
% steering angle. No aspect ratios, nesting, angle factors, or roll caps.
%
% Enumerate every distinct phi-bound pair containing the anchor, intersect
% the stable masks through that interval, and enumerate maximal all-one
% vc-r rectangles with a monotone histogram stack. Every positive-volume
% all-one rectangle containing the anchor can expand to a maximal rectangle
% that still contains it. Thus the search gives the global maximum among
% grid-aligned all-stable cuboids containing the specified equilibrium.
% Physical coordinate spans determine volume; nonuniform grids are allowed.
%
% Containment tolerance: 1e-12. Volume ties use
% 1e-13*max(1,abs(best),abs(candidate)); select the lexicographically first
% [vc_lo,vc_hi,r_lo,r_hi,phi_lo,phi_hi] grid-index tuple and save every tie.
% No statement is made about unsampled states inside the cuboid. The caller
% must separately establish equilibrium local stability and vehicle limits.
  stable_mask = logical(stable_mask);
  coordinates = {G.vc(:),G.r(:),G.phi(:)};
  expected_shape = cellfun(@numel,coordinates);
  assert(isequal(size(stable_mask),expected_shape), 'Mask/grid size mismatch');
  for k=1:3
    assert(all(isfinite(coordinates{k})) && all(diff(coordinates{k})>0), ...
           'Grid coordinates must be finite and strictly increasing');
  end
  equilibrium_point=double(equilibrium_point(:).');
  assert(numel(equilibrium_point)==3 && all(isfinite(equilibrium_point)), ...
         'Equilibrium point must contain three finite [vc,r,phi] values');
  containment_tolerance=1e-12;
  nv=expected_shape(1); nr=expected_shape(2); np=expected_shape(3);
  best_volume=-1; best_bounds=[]; ties=zeros(0,6);
  slabs=0; rectangles=0;
  for p0=1:np-1
    if coordinates{3}(p0)>equilibrium_point(3)+containment_tolerance, continue; end
    intersection=true(nv,nr);
    for p1=p0:np
      intersection = intersection & stable_mask(:,:,p1);
      if p1==p0 || coordinates{3}(p1)<equilibrium_point(3)-containment_tolerance, continue; end
      slabs=slabs+1;
      heights=zeros(1,nr);
      for v1=1:nv
        good=intersection(v1,:);
        heights(~good)=0; heights(good)=heights(good)+1;
        stack_start=zeros(1,nr); stack_height=zeros(1,nr); top=0;
        for r_end=1:nr+1
          if r_end<=nr, height=heights(r_end); else, height=0; end
          start=r_end;
          while top>0 && stack_height(top)>height
            r0=stack_start(top); h=stack_height(top); top=top-1;
            start=r0; r1=r_end-1;
            if h<2 || r1<=r0, continue; end
            v0=v1-h+1;
            if ~(coordinates{1}(v0)<=equilibrium_point(1)+containment_tolerance && ...
                 coordinates{1}(v1)>=equilibrium_point(1)-containment_tolerance && ...
                 coordinates{2}(r0)<=equilibrium_point(2)+containment_tolerance && ...
                 coordinates{2}(r1)>=equilibrium_point(2)-containment_tolerance)
              continue;
            end
            volume=(coordinates{1}(v1)-coordinates{1}(v0)) * ...
                   (coordinates{2}(r1)-coordinates{2}(r0)) * ...
                   (coordinates{3}(p1)-coordinates{3}(p0));
            rectangles=rectangles+1;
            bounds=[v0,v1,r0,r1,p0,p1]-1;
            tolerance=1e-13*max([1,abs(best_volume),abs(volume)]);
            if volume>best_volume+tolerance
              best_volume=volume; best_bounds=bounds; ties=bounds;
            elseif abs(volume-best_volume)<=tolerance
              ties(end+1,:)=bounds; %#ok<AGROW>
              if lexicographic_less(bounds,best_bounds), best_bounds=bounds; end
            end
          end
          if height>0 && (top==0 || stack_height(top)<height)
            top=top+1; stack_start(top)=start; stack_height(top)=height;
          end
        end
      end
    end
  end
  if isempty(best_bounds)
    result=struct('valid',false,'volume',NaN,'vertices',zeros(0,3),'faces',zeros(0,4), ...
      'bounds',NaN(3,2),'grid_indices_1based',zeros(0,6),'all_ties_grid_indices_1based',zeros(0,6), ...
      'grid_bound_indices_zero_based',zeros(0,6),'maximum_volume_tie_count',0, ...
      'maximum_volume_ties_grid_bound_indices_zero_based',zeros(0,6), ...
      'equilibrium_anchor_vc_r_phi',equilibrium_point,'equilibrium_containment_required',true, ...
      'containment_tolerance',containment_tolerance, ...
      'positive_equilibrium_containing_roll_intervals_tested',slabs, ...
      'positive_equilibrium_containing_rectangles_evaluated',rectangles, ...
      'unavailable_reason','No positive-volume all-stable grid cuboid contains equilibrium');
    return;
  end
  ties=sortrows(unique(ties,'rows'),1:6);
  lower=zeros(1,3); upper=zeros(1,3);
  for k=1:3
    lower(k)=coordinates{k}(best_bounds(2*k-1)+1);
    upper(k)=coordinates{k}(best_bounds(2*k)+1);
  end
  vertices=zeros(8,3); index=0;
  for iv=1:2
    for ir=1:2
      for ip=1:2
        index=index+1;
        bounds=[lower;upper];
        vertices(index,:)=[bounds(iv,1),bounds(ir,2),bounds(ip,3)];
      end
    end
  end
  tie_roll_bounds=[coordinates{3}(ties(:,5)+1),coordinates{3}(ties(:,6)+1)];
  tie_roll_spans=tie_roll_bounds(:,2)-tie_roll_bounds(:,1);
  b=best_bounds+1;
  subset=stable_mask(b(1):b(2),b(3):b(4),b(5):b(6));
  result=struct('valid',true,'grid_bound_indices_zero_based',best_bounds, ...
       'grid_indices_1based',best_bounds+1,'all_ties_grid_indices_1based',ties+1, ...
       'bounds',[lower(:),upper(:)],'faces',[1 2 4 3;5 7 8 6;1 5 6 2;3 4 8 7;1 3 7 5;2 6 8 4], ...
       'lower_bounds',lower,'upper_bounds',upper,'coordinate_spans',upper-lower, ...
       'roll_bounds',[lower(3),upper(3)],'roll_span',upper(3)-lower(3), ...
       'coordinate_centers',(upper+lower)/2,'roll_center',(upper(3)+lower(3))/2, ...
       'vertices',vertices,'inner_vertices',vertices,'volume',best_volume,'inner_volume',best_volume, ...
       'is_valid_inner',true,'equilibrium_anchor_vc_r_phi',equilibrium_point, ...
       'equilibrium_containment_required',true,'equilibrium_centering_required',false, ...
       'containment_tolerance',containment_tolerance, ...
       'contains_equilibrium',all(equilibrium_point>=lower-containment_tolerance) && ...
                              all(equilibrium_point<=upper+containment_tolerance), ...
       'all_grid_nodes_in_cuboid_stable',all(subset(:)), ...
       'grid_nodes_in_cuboid',numel(subset), ...
       'maximum_volume_tie_count',size(ties,1), ...
       'maximum_volume_ties_grid_bound_indices_zero_based',ties, ...
       'maximum_volume_ties_roll_bounds',tie_roll_bounds, ...
       'maximum_volume_ties_roll_span_range',[min(tie_roll_spans),max(tie_roll_spans)], ...
       'positive_equilibrium_containing_roll_intervals_tested',slabs, ...
       'positive_equilibrium_containing_rectangles_evaluated',rectangles, ...
       'global_maximum_among_equilibrium_containing_grid_cuboids',true);
end

function answer=lexicographic_less(a,b)
  answer=false;
  for k=1:numel(a)
    if a(k)<b(k), answer=true; return; end
    if a(k)>b(k), return; end
  end
end


function fig = plot_mf210_paper_roll(results_dir,output_stem,visible)
%PLOT_MF210_PAPER_ROLL Overlay actual cuboids in a common paper-like camera.
%   plot_mf210_paper_roll('results')
%   plot_mf210_paper_roll('results','regions','off')
% Coordinates are displayed as x=r, y=vc, z=phi without scale or translation.
% The mathematical region retains its original [vc,r,phi] coordinate order.
  if nargin<1 || isempty(results_dir)
    results_dir=fullfile(fileparts(mfilename('fullpath')),'results');
  end
  if nargin<2 || isempty(output_stem)
    output_stem=fullfile(results_dir,'mf210_paper_roll_regions');
  end
  if nargin<3 || isempty(visible), visible='on'; end
  angles=[0 3.5 7];
  colors=[0.23 0.70 0.32;0.97 0.78 0.18;0.18 0.43 0.87];
  edges=[1 2;1 3;1 5;2 4;2 6;3 4;3 7;4 8;5 6;5 7;6 8;7 8];
  fig=figure('Color','w','Position',[80 80 1200 800],'Visible',visible);
  ax=axes('Parent',fig,'Position',[0.13 0.22 0.72 0.69]);
  hold(ax,'on');
  handles=[];
  for ii=1:numel(angles)
    tag=sprintf('%g',angles(ii));
    saved=load(fullfile(results_dir,['stable_point_40kph_0.85mu_' tag 'deg.mat']), ...
      'INNER_REGION','PAPER_STABILITY3D');
    region=saved.INNER_REGION;
    assert(region.valid && size(region.vertices,1)==8 && size(region.faces,1)==6, ...
      'Expected a valid eight-vertex, six-face cuboid');
    vertices=region.vertices(:,[2 1 3]);
    render_faces=region.faces;
    if exist('OCTAVE_VERSION','builtin')~=0
      % Octave's headless gnuplot renderer only fills 3-D triangular patches.
      % Invisible internal diagonals only tessellate the six faces;
      % region.faces and the twelve boundary edges remain unchanged.
      render_faces=[region.faces(:,[1 2 3]);region.faces(:,[1 3 4])];
    end
    h=patch('Parent',ax,'Vertices',vertices,'Faces',render_faces, ...
      'FaceColor',colors(ii,:),'FaceAlpha',0.28,'EdgeColor','none');
    if ii==1
      handles=repmat(h,1,3);
    else
      handles(ii)=h;
    end
    for jj=1:size(edges,1)
      p=vertices(edges(jj,:),:);
      line('Parent',ax,'XData',p(:,1),'YData',p(:,2),'ZData',p(:,3), ...
        'Color',[0.06 0.12 0.17],'LineWidth',1.0,'HandleVisibility','off');
    end
  end
  grid(ax,'on'); box(ax,'on');
  xlabel(ax,'r (rad/s)'); ylabel(ax,'v_c (m/s)'); zlabel(ax,'\phi (rad)');
  title(ax,'40 km/h, \mu = 0.85');
  set(ax,'XDir','reverse','XLim',[-0.72 0.72],'YLim',[-2 2],'ZLim',[-0.3 0.3], ...
    'XTick',[-0.6 -0.3 0 0.3 0.6],'YTick',[-2 -1 0 1 2], ...
    'ZTick',[-0.3 -0.15 0 0.15 0.3],'FontSize',11,'Projection','orthographic');
  pbaspect(ax,[1.3 2.28 1.65*0.6/0.84]);
  view(ax,-72,11);
  legend(ax,handles,{'0 deg','3.5 deg','7 deg'},'Location','northeastoutside');
  annotation(fig,'textbox',[0.10 0.075 0.80 0.04], ...
    'String','Quasisteady MF; dynamic roll-reaction extension of Eq. (11)', ...
    'HorizontalAlignment','center','EdgeColor','none','FontSize',10);
  annotation(fig,'textbox',[0.10 0.035 0.80 0.04], ...
    'String','Independent equilibrium-containing maximum-volume cuboids; sampled states only', ...
    'HorizontalAlignment','center','EdgeColor','none','FontSize',10);
  print(fig,[output_stem '.png'],'-dpng','-r220');
  savefig(fig,[output_stem '.fig']);
end

