function files=run_mf210_final(output_dir,make_plots,smoke_test)
%RUN_MF210_FINAL Standalone five-state quasisteady MF reconstruction.
% MATLAB R2016b or newer; base MATLAB only; no external helper is required.
%   run_mf210_final
%   run_mf210_final('my_results',false)
%   run_mf210_final('smoke_results',false,true) % tiny runtime check only
%
% Production: 81x55x31 states per angle, 40 km/h fixed, mu=0.85, 5 s,
% 0.01 s classification observations and 20 RK4 substeps per observation.
% Original210 healthy MF force coefficients/shifts and mu/.9 are retained.
% Tires are explicitly QUASISTEADY: instantaneous wheel alpha and the original
% longitudinal shifted free-rolling root kappa=-Shx. Original210 hidden tire
% relaxation and wheel dynamics are replaced, not claimed to be retained.
% Paper body equations remain; dynamic normal reaction is a physical extension
% of printed Eq11, not a literal reproduction of the Burckhardt paper model.
% Each INNER_REGION maximizes volume in a data-derived, conservatively
% filtered sheared-grid-box family containing its OWN stable equilibrium.
% Only cells with eight stable corners support a region. No centering, aspect ratio,
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
    meta=struct('version','paper_body_quasisteady_mf210_stable_cell_sheared_region_v1', ...
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
      'geometry','Independent maximum-volume own-equilibrium-containing sheared box within a data-derived conservative stable-cell family', ...
      'geometry_is_author_method',false,'geometry_is_raw_basin',false, ...
      'equilibrium_containment_required',true,'equilibrium_centering_required',false, ...
      'aspect_ratio_constraints_used',false,'nesting_constraints_used',false, ...
      'angle_dependent_geometry_constraints_used',false,'roll_scaling_or_caps_applied',false, ...
      'selection_independent_of_other_angles',true,'natural_nesting_claimed',false, ...
      'geometry_guarantee','Every included raw node is stable; region lies within cells with eight stable corners; continuous stability not certified', ...
      'validation_smoke_test',logical(smoke_test),'source_provenance',provenance);
    PAPER_STABILITY3D=struct('meta',meta,'cfg',cfg,'result',result);
    files{ii,1}=fullfile(output_dir,['classification_' tag 'deg.mat']);
    % Preserve the classification and its equilibrium before geometry extraction.
    save(files{ii,1},'PAPER_STABILITY3D','EQUILIBRIUM','-v7');
    assert(EQUILIBRIUM.locally_asymptotically_stable_at_fixed_u && ...
      EQUILIBRIUM.admissible_under_instantaneous_constraints,'Equilibrium is ineligible');
    INNER_REGION=build_stable_cell_sheared_region(result.stable_mask,cfg.grid,anchor);
    INNER_REGION.method='Maximum volume within conservative stable-cell sheared-box family, containing own stable equilibrium';
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
    assert(INNER_REGION.valid,'No positive-volume conservative candidate contains this equilibrium');
    fprintf('Saved: stable=%d/%d; local inner volume=%.12g; ties=%d\n', ...
      result.stable_count,result.total_count,INNER_REGION.volume,INNER_REGION.maximum_volume_tie_count);
  end
  if make_plots
    plot_mf210_final(output_dir,fullfile(output_dir,'mf210_paper_roll_regions'));
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
provenance.geometry_helper_source_sha256='c83ae19723124e77323cc00661813680237685c6cfb32e9152ef67e6a3d27616';
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

function result=build_stable_cell_sheared_region(stable_mask,G,equilibrium_point)
%BUILD_STABLE_CELL_SHEARED_REGION Independent conservative eight-vertex region.
% Uniform ascending grids G.vc,G.r,G.phi, with mask order [vc,r,phi].
% A six-connected stable component containing the nearest equilibrium node
% provides vc(r) and phi(r) slopes by equal-yaw-row least squares. For every
% yaw interval containing the actual equilibrium, endpoint grid shifts are
% independently selected from {0,floor(slope*interval),ceil(slope*interval)}.
% Maximize volume, breaking ties by the zero-based tuple
% [vc_lo,vc_hi,r_lo,r_hi,phi_lo,phi_hi,vc_shift,phi_shift].
%
% Every accepted shape avoids the conservative inverse-sheared bounding-box
% projections of physical unit cells that do not have eight stable corners.
% It also excludes all raw bad nodes, including boundaries. This is a
% sampled finite-horizon construction, not a continuous-stability proof.
% No other steering angle, target image, nesting condition, roll cap,
% coordinate correction, angle factor or prescribed aspect ratio is used.
  mask=logical(stable_mask); coordinates={G.vc(:),G.r(:),G.phi(:)};
  sz=cellfun(@numel,coordinates);nv=sz(1);nr=sz(2);np=sz(3);
  assert(all(sz>=2)&&isequal(size(mask),sz),'Mask/grid size mismatch');
  origin=zeros(1,3);step=zeros(1,3);
  for k=1:3
    g=coordinates{k};origin(k)=g(1);step(k)=g(2)-g(1);
    assert(all(isfinite(g))&&step(k)>0&&all(abs(diff(g)-step(k))<1e-12), ...
      'Uniform finite ascending grids are required');
  end
  anchor=double(equilibrium_point(:).');
  assert(numel(anchor)==3&&all(isfinite(anchor)),'Finite [vc,r,phi] anchor required');
  ai=(anchor-origin)./step;
  result=empty_result(anchor);
  if any(ai< -1e-10)||any(ai>sz-1+1e-10)
    result.unavailable_reason='Equilibrium is outside the grid';return;
  end
  nearest=zeros(1,3);
  for k=1:3,[~,nearest(k)]=min(abs(coordinates{k}-anchor(k)));end
  if ~mask(nearest(1),nearest(2),nearest(3))
    result.unavailable_reason='Nearest equilibrium grid node is unstable';return;
  end
  % Base-MATLAB six-connected flood fill; no Image Processing Toolbox.
  component=false(sz);queue=zeros(nnz(mask),1);head=1;tail=1;
  queue(1)=sub2ind(sz,nearest(1),nearest(2),nearest(3));component(queue(1))=true;
  while head<=tail
    idx=queue(head);head=head+1;[v,r,p]=ind2sub(sz,idx);
    neighbors=[v-1 r p;v+1 r p;v r-1 p;v r+1 p;v r p-1;v r p+1];
    inside=all(neighbors>=1,2)&neighbors(:,1)<=nv&neighbors(:,2)<=nr&neighbors(:,3)<=np;
    neighbors=neighbors(inside,:);
    ids=sub2ind(sz,neighbors(:,1),neighbors(:,2),neighbors(:,3));
    ids=ids(mask(ids)&~component(ids));
    for ii=1:numel(ids)
      if ~component(ids(ii)),tail=tail+1;queue(tail)=ids(ii);component(ids(ii))=true;end
    end
  end
  rows=zeros(nr,3);nrows=0;
  for r=1:nr
    [v,p]=find(reshape(component(:,r,:),nv,np));
    if ~isempty(v),nrows=nrows+1;rows(nrows,:)=[r-1,mean(v-1),mean(p-1)];end
  end
  rows=rows(1:nrows,:);slopes=[0 0];
  if nrows>=2
    centered=rows(:,1)-mean(rows(:,1));den=sum(centered.^2);
    slopes=[sum(centered.*(rows(:,2)-mean(rows(:,2)))), ...
            sum(centered.*(rows(:,3)-mean(rows(:,3))))]/den;
  end
  % Prefix sums of unsafe PHYSICAL unit cells, once per physical yaw slab.
  safe=mask(1:end-1,1:end-1,1:end-1)&mask(2:end,1:end-1,1:end-1)& ...
       mask(1:end-1,2:end,1:end-1)&mask(2:end,2:end,1:end-1)& ...
       mask(1:end-1,1:end-1,2:end)&mask(2:end,1:end-1,2:end)& ...
       mask(1:end-1,2:end,2:end)&mask(2:end,2:end,2:end);
  prefix=zeros(nv,np,nr-1);
  for r=1:nr-1
    plane=double(~reshape(safe(:,r,:),nv-1,np-1));
    prefix(2:end,2:end,r)=cumsum(cumsum(plane,1),2);
  end
  best=-1;best_bounds=[];ties=0;minroll=Inf;maxroll=-Inf;configs=0;rectangles=0;
  for r0=0:nr-2
    if r0>ai(2)+1e-10,continue;end
    for r1=r0+1:nr-1
      if r1<ai(2)-1e-10,continue;end
      L=r1-r0;dvs=unique([0 floor(slopes(1)*L) ceil(slopes(1)*L)]);
      dps=unique([0 floor(slopes(2)*L) ceil(slopes(2)*L)]);
      for sv=dvs
        for sp=dps
          vl=max(0,-sv);vh=min(nv-1,nv-1-sv);pl=max(0,-sp);ph=min(np-1,np-1-sp);
          V=vh-vl;P=ph-pl;if V<=0||P<=0,continue;end
          qav=ai(1)-sv*(ai(2)-r0)/L;qap=ai(3)-sp*(ai(2)-r0)/L;
          if qav<vl-1e-10||qav>vh+1e-10||qap<pl-1e-10||qap>ph+1e-10,continue;end
          configs=configs+1;good=true(V,P);vi=(vl:vh-1).';pi=(pl:ph-1);
          for r=r0:r1-1
            % Dual of C++ inverse-bbox rasterization: test each q cell's
            % physical vc/phi bbox against unsafe physical unit cells.
            shiftsv=sv*[r-r0,r+1-r0];shiftsp=sp*[r-r0,r+1-r0];
            vlo=floor((vi*L+min(shiftsv))/L);vhi=ceil(((vi+1)*L+max(shiftsv))/L)-1;
            plo=floor((pi*L+min(shiftsp))/L);phi=ceil(((pi+1)*L+max(shiftsp))/L)-1;
            S=prefix(:,:,r+1);
            i11=bsxfun(@plus,vhi+2,nv*(phi+1));i01=bsxfun(@plus,vlo+1,nv*(phi+1));
            i10=bsxfun(@plus,vhi+2,nv*plo);i00=bsxfun(@plus,vlo+1,nv*plo);
            good=good&(S(i11)-S(i01)-S(i10)+S(i00)==0);
          end
          % Unsafe-cell avoidance implies bad-node exclusion: every raw
          % boundary node belongs to the closure of some intersected cell,
          % whose eight corners must all be stable. Test anchor-adjacent
          % q cells before scanning maximal rectangles.
          avc=incident_cells(qav,vl,vh);apc=incident_cells(qap,pl,ph);
          if isempty(avc)||isempty(apc)||~any(any(good(avc-vl+1,apc-pl+1))),continue;end
          activev=find(any(good,2));activep=find(any(good,1));
          if isempty(activev)||isempty(activep),continue;end
          va=activev(1);vb=activev(end);pa=activep(1);pb=activep(end);
          cropped=good(va:vb,pa:pb);W=size(cropped,2);heights=zeros(1,W);
          for row=1:size(cropped,1)
            heights(~cropped(row,:))=0;heights(cropped(row,:))=heights(cropped(row,:))+1;
            starts=zeros(1,W);hs=zeros(1,W);top=0;
            for rend=1:W+1
              if rend<=W,height=heights(rend);else,height=0;end
              start=rend;
              while top>0&&hs(top)>height
                pstart=starts(top);h=hs(top);top=top-1;start=pstart;
                v0=vl+va+row-h-1;v1=vl+va+row-1;
                p0=pl+pa+pstart-2;p1=pl+pa+rend-2;
                if ~(v0<=qav+1e-10&&v1>=qav-1e-10&&p0<=qap+1e-10&&p1>=qap-1e-10),continue;end
                rectangles=rectangles+1;volume=(v1-v0)*(p1-p0)*L;
                b=[v0 v1 r0 r1 p0 p1 sv sp];roll=p1-p0+abs(sp);
                if volume>best
                  best=volume;best_bounds=b;ties=1;minroll=roll;maxroll=roll;
                elseif volume==best
                  ties=ties+1;minroll=min(minroll,roll);maxroll=max(maxroll,roll);
                  if lex_less(b,best_bounds),best_bounds=b;end
                end
              end
              if height>0&&(top==0||hs(top)<height)
                top=top+1;starts(top)=start;hs(top)=height;
              end
            end
          end
        end
      end
    end
  end
  result.fitted_grid_slopes=slopes;result.six_connected_component_nodes=nnz(component);
  result.configurations=configs;result.rectangles=rectangles;
  if isempty(best_bounds),result.unavailable_reason='No positive-volume candidate contains equilibrium';return;end
  b=best_bounds;iv=zeros(8,3);index=0;
  for v=b(1:2)
    for t=0:1
      r=b(3+t);
      for p=b(5:6)
        index=index+1;iv(index,:)=[v+b(7)*t,r,p+b(8)*t];
      end
    end
  end
  vertices=zeros(8,3);
  for k=1:3,vertices(:,k)=coordinates{k}(iv(:,k)+1);end
  lower=min(vertices,[],1);upper=max(vertices,[],1);
  result.valid=true;result.is_valid_inner=true;result.vertices=vertices;result.inner_vertices=vertices;
  result.faces=[1 2 4 3;5 7 8 6;1 5 6 2;3 4 8 7;1 3 7 5;2 6 8 4];
  result.volume=best*prod(step);result.inner_volume=result.volume;result.volume_grid_units=best;
  result.grid_bound_indices_zero_based=b;result.optimizer_bounds_zero_based=b;
  result.bounds=[lower(:),upper(:)];result.lower_bounds=lower;result.upper_bounds=upper;
  result.coordinate_spans=upper-lower;result.coordinate_centers=(upper+lower)/2;
  result.roll_bounds=[lower(3),upper(3)];result.roll_span=upper(3)-lower(3);
  result.roll_center=(lower(3)+upper(3))/2;result.maximum_volume_tie_count=ties;
  result.tie_roll_span_grid_min=minroll;result.tie_roll_span_grid_max=maxroll;
  result.maximum_volume_ties_roll_span_range=[minroll maxroll]*step(3);
  result.contains_equilibrium=true;result.unstable_nodes_inside_inclusive=0;
  result.all_physical_cells_intersected_have_eight_stable_corners=true;
  result.global_maximum_within_specified_conservative_candidate_family=true;
end

function c=incident_cells(x,lo,hi)
  if abs(x-round(x))<=1e-10,c=[round(x)-1 round(x)];else,c=floor(x);end
  c=c(c>=lo&c<hi);
end
function yes=lex_less(a,b)
  i=find(a~=b,1);yes=~isempty(i)&&a(i)<b(i);
end
function result=empty_result(anchor)
  result=struct('valid',false,'is_valid_inner',false,'volume',NaN,'inner_volume',NaN, ...
    'vertices',zeros(0,3),'inner_vertices',zeros(0,3),'faces',zeros(0,4), ...
    'bounds',NaN(3,2),'equilibrium_anchor_vc_r_phi',anchor, ...
    'equilibrium_containment_required',true,'equilibrium_centering_required',false, ...
    'nesting_constraints_applied',false,'roll_scaling_or_caps_applied',false, ...
    'coordinate_correction_applied',false,'continuous_stability_certificate',false, ...
    'method','Maximum-volume conservative stable-cell sheared grid box', ...
    'orientation_rule','Six-connected component row means; OLS slopes; endpoint shifts zero/floor/ceil', ...
    'tie_rule','Lexicographic [vc0,vc1,r0,r1,phi0,phi1,vc_shift,phi_shift]');
end

function fig = plot_mf210_final(results_dir,output_stem,visible)
%PLOT_MF210_FINAL Overlay actual hexahedra in a common paper-like camera.
%   plot_mf210_final('results')
%   plot_mf210_final('results','regions','off')
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
  colors=[161 231 172;240 238 49;24 63 229]/255;
  alphas=[0.26 0.32 0.58];
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
      'Expected a valid eight-vertex, six-face hexahedron');
    vertices=region.vertices(:,[2 1 3]);
    render_faces=region.faces;
    if exist('OCTAVE_VERSION','builtin')~=0
      % Octave's headless gnuplot renderer only fills 3-D triangular patches.
      % Invisible internal diagonals only tessellate the six faces;
      % region.faces and the twelve boundary edges remain unchanged.
      render_faces=[region.faces(:,[1 2 3]);region.faces(:,[1 3 4])];
    end
    h=patch('Parent',ax,'Vertices',vertices,'Faces',render_faces, ...
      'FaceColor',colors(ii,:),'FaceAlpha',alphas(ii),'EdgeColor','none');
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
    'String','Independent conservative stable-cell sheared regions; sampled states only', ...
    'HorizontalAlignment','center','EdgeColor','none','FontSize',10);
  print(fig,[output_stem '.png'],'-dpng','-r220');
  savefig(fig,[output_stem '.fig']);
end
