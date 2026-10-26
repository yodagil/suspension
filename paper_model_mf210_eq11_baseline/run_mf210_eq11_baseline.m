function files=run_mf210_eq11_baseline(output_dir,make_plots,smoke_test)
%RUN_MF210_EQ11_BASELINE Standalone MF + printed paper Eq11 baseline.
% Base MATLAB R2016b or newer; no external helper files/toolboxes required.
%   run_mf210_eq11_baseline('results',true)
%   run_mf210_eq11_baseline('smoke_results',false,true) % tiny validation only
% Fixed 40 km/h, mu=.85, healthy tires, steer 0/3.5/7 deg; 5 seconds.
% Original MF coefficients, shifts and mu/.9 scaling are retained. The normal
% load moment is the printed Eq11 D=m*h*ay+ms*g*hphi*sin(phi), solved at each
% RHS stage against the same MF forces. No -Ixx*pdot extension or load lag.
% Sec3.1 extraction follows roll extrema -> lateral branch extrema -> eight
% cap extrema -> checked inner candidate. Unpublished branch/corner rules are
% declared reconstruction choices; invalid candidates are saved as invalid.
% No max-volume objective, equilibrium anchor, forced nesting or roll cap.
% ms=m,hphi=h,p(0)=0,fixed-speed maintenance and 5s classification are assumed;
% MF replaces the paper's Burckhardt tires. This is not exact reproduction.
  if nargin<1||isempty(output_dir)
    output_dir=fullfile(fileparts(mfilename('fullpath')),'results_eq11_matlab');
  end
  if nargin<2||isempty(make_plots),make_plots=true;end
  if nargin<3||isempty(smoke_test),smoke_test=false;end
  assert(isscalar(make_plots)&&isscalar(smoke_test),'Flags must be scalar');
  if ~exist(output_dir,'dir'),mkdir(output_dir);end
  base_cfg=paper_config();angles=[0 3.5 7];files=cell(3,2);
  if smoke_test
    fprintf('SMOKE VALIDATION ONLY: 27 initial states per angle, 0.02 seconds.\n');
  end
  for ii=1:3
    cfg=base_cfg;cfg.steer_deg=angles(ii);
    if smoke_test
      cfg.grid.vc=[-.01 0 .01];cfg.grid.r=[-.005 0 .005];cfg.grid.phi=[-.002 0 .002];
      cfg.horizon_s=.02;cfg.batch_size=27;
    end
    fprintf('\n%d/3: printed Eq11, 40 km/h, mu=.85, delta=%g deg\n',ii,cfg.steer_deg);
    result=paper_simulate_grid(cfg);
    reason=result.reason_mask(:);unknown=bitand(reason,uint8(8))~=0;
    physical=bitand(reason,uint8(7))~=0 & ~unknown;
    result.numerical_unresolved_mask=reshape(unknown,size(result.stable_mask));
    result.physical_unstable_mask=reshape(physical,size(result.stable_mask));
    result.classified_mask=reshape(~unknown,size(result.stable_mask));
    result.unstable_points=result.initial_points(physical,:);
    result.unresolved_points=result.initial_points(unknown,:);
    result.unresolved_count=nnz(unknown);result.physical_unstable_count=nnz(physical);
    result.definition=['Five-second sampled constraints under printed Eq11 and original MF forces; ' ...
      'yaw threshold inferred from paper ideal-yaw limit, not a published executable classifier'];
    result.reason_names={'yaw_limit','lateral_acceleration_limit','wheel_lift','numerical_unresolved'};
    meta=struct('version','MF210-PrintedEq11-Sec31-Baseline-v1', ...
      'engine','Standalone MATLAB M-code','paper_doi',cfg.paper_doi, ...
      'paper_exact_reproduction',false,'tire_force_coefficients_changed',false, ...
      'load_transfer_equation','D=m*h*ay+ms*g*hphi*sin(phi)', ...
      'dynamic_roll_reaction_extension_used',false,'hidden_load_lag_used',false, ...
      'same_stage_implicit_load_force_closure',true,'max_volume_geometry_used',false, ...
      'equilibrium_anchor_used',false,'nesting_constraints_used',false, ...
      'roll_scaling_or_caps_applied',false,'author_exact_corner_algorithm_claimed',false, ...
      'numerical_unresolved_separate_from_physical_unstable',true, ...
      'assumptions','Fixed speed; ms=m; hphi=h; p(0)=0; five-second sampled classification', ...
      'validation_smoke_test',logical(smoke_test));
    PAPER_STABILITY3D=struct('meta',meta,'cfg',cfg,'result',result);
    tag=sprintf('%g',cfg.steer_deg);
    files{ii,1}=fullfile(output_dir,['classification_' tag 'deg.mat']);
    save(files{ii,1},'PAPER_STABILITY3D','-v7');
    SECTION31=paper_sec31_region(result.stable_mask,cfg.grid);
    INNER_REGION=SECTION31.inner_candidate;
    INNER_REGION.candidate_volume=INNER_REGION.volume;
    INNER_REGION.candidate_vertices=INNER_REGION.vertices;
    if INNER_REGION.valid
      INNER_REGION.inner_volume=INNER_REGION.volume;INNER_REGION.inner_vertices=INNER_REGION.vertices;
    else
      INNER_REGION.volume=NaN;INNER_REGION.inner_volume=NaN;INNER_REGION.inner_vertices=zeros(0,3);
    end
    INNER_REGION.method=SECTION31.method;
    INNER_REGION.angle_deg=cfg.steer_deg;INNER_REGION.axis_order={'vc','r','phi'};
    INNER_REGION.author_exact_algorithm_claimed=false;
    INNER_REGION.max_volume_objective_used=false;INNER_REGION.equilibrium_anchor_used=false;
    INNER_REGION.nesting_constraints_used=false;INNER_REGION.roll_scaling_or_caps_applied=false;
    INNER_REGION.physical_roll_boundary_resolved=SECTION31.physical_roll_boundary_resolved;
    files{ii,2}=fullfile(output_dir,['stable_point_40kph_0.85mu_' tag 'deg.mat']);
    save(files{ii,2},'PAPER_STABILITY3D','SECTION31','INNER_REGION','-v7');
    fprintf('Saved %d/3: stable=%d/%d, physical unstable=%d, unresolved=%d\n', ...
      ii,result.stable_count,result.total_count,nnz(physical),nnz(unknown));
    if INNER_REGION.valid
      fprintf('Accepted sampled inner candidate: volume=%g\n',INNER_REGION.volume);
    else
      fprintf('No accepted inner candidate: %s\n',INNER_REGION.unavailable_reason);
    end
    if SECTION31.roll_domain_censored_slices>0
      fprintf('Roll boundary unresolved at grid ends in %d vc-yaw slices.\n',SECTION31.roll_domain_censored_slices);
    end
  end
  if make_plots
    plot_mf210_eq11_baseline(output_dir,fullfile(output_dir,'eq11_baseline_comparison'),'on');
  end
end



function cfg=paper_config()
% Baseline comparison: quasisteady MF free rolling with literal paper Eq11.
% This replaces the hidden tire transients, not the original MF force kernel.
% It is neither original210 full tire dynamics nor a paper-exact reproduction.
cfg.paper_doi='10.1016/j.aei.2022.101764';
cfg.m=1300;cfg.ms=1300;cfg.Izz=2500;cfg.Ixx=700;
cfg.L=2.4;cfg.lf=1.2;cfg.lr=1.2;cfg.df=.7;cfg.dr=.7;
cfg.h=.54;cfg.hphi=.54;cfg.g=9.81;cfg.Kphi=43100;cfg.Cphi=1400;
cfg.deltaFz=[0 0 0 0];cfg.mu=.85;cfg.speed_kmh=40;cfg.steer_deg=0;
cfg.speed_mode='fixed';cfg.horizon_s=5;cfg.dt=.01;cfg.method='rk4';
cfg.Nsub=20;cfg.integration_dt=cfg.dt/cfg.Nsub;cfg.batch_size=3500;
cfg.limit_factor=.85;cfg.min_u=.5;cfg.filename_digits=3;
cfg.grid.vc=(-40:40)*.25;cfg.grid.r=(-27:27)*(.8/30);cfg.grid.phi=(-15:15)*(.8/30);
cfg.tire_model='Original210 healthy MF kernel with algebraic free-rolling slip';
cfg.mf_Fz0=5425;cfg.mf_force_min=100;cfg.mf_force_max=16000;
cfg.mf_mu_reference=.9;cfg.mf_mu_scaling_rule='mu_road / 0.9';cfg.mf_wear=[0 0 0 0];
cfg.slip_angle_mode='Instantaneous exact original210 wheel-frame velocity';
cfg.tire_dynamics='Quasisteady free rolling: solve kappa=-Shx, Fx=0';cfg.model_dimension=5;
cfg.free_rolling_constraint='kappa=0.001635-0.0025297*((clippedFz-5425)/5425), original Vx offset=0';
cfg.load_solver='Residual-certified secant; breakpoint-split root fallback';
cfg.load_transfer_model='Literal printed paper Eq11, implicitly coupled to preserved MF forces';
cfg.roll_reaction_equation='D=m*h*ay+ms*g*hphi*sin(phi)';
cfg.roll_reaction_ay_coefficient=cfg.m*cfg.h;
cfg.load_solver_residual_tolerance=2e-10;
cfg.load_solver_fast_iterations=10;
cfg.load_solver_hidden_lag=false;
cfg.load_solver_fallback_policy='Unique all-contact root; otherwise unique global root; explicit failure on ambiguity';
cfg.paper_exact_reproduction=false;
cfg.initialization='u=speed, vc/r/phi=grid, p=0; instantaneous algebraic tire forces';
cfg.reconstruction_version='XiaoBody-MF210-QuasisteadyFreeRolling-LiteralEq11-v1';
end


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


function x=paper_initial(points,cfg)
% Five body states only; algebraic tire variables have no initial conditions.
x=[repmat(cfg.speed_kmh/3.6,size(points,1),1),points,zeros(size(points,1),1)];
assert(size(x,2)==5);
end


function next=paper_step(x,cfg)
% Five-state RK4, 20 substeps per .01s observation; no hidden tire clamps.
h=cfg.dt/cfg.Nsub;next=x;
for j=1:cfg.Nsub
    k1=paper_rhs(next,cfg);k2=paper_rhs(next+.5*h*k1,cfg);
    k3=paper_rhs(next+.5*h*k2,cfg);k4=paper_rhs(next+h*k3,cfg);
    next=next+(h/6)*(k1+2*k2+2*k3+k4);
end
end


function [dx,obs]=paper_rhs(x,cfg)
% Literal Xiao Eq (11) with the preserved original210 healthy MF force law.
% x=[u vc r phi p]. No dynamic roll reaction subtraction or load lag.
% The load/force algebraic loop is solved to an explicit residual tolerance.
u=x(:,1);v=x(:,2);r=x(:,3);phi=x(:,4);p=x(:,5);
delta=cfg.steer_deg*pi/180;cs=cos(delta);sn=sin(delta);
uB=[u-cfg.df*r,u+cfg.df*r,u-cfg.dr*r,u+cfg.dr*r];
vB=[v+cfg.lf*r,v+cfg.lf*r,v-cfg.lr*r,v-cfg.lr*r];
uW=[cs*uB(:,1:2)+sn*vB(:,1:2),uB(:,3:4)];
vW=[-sn*uB(:,1:2)+cs*vB(:,1:2),vB(:,3:4)];
safe=uW;safe(safe>=0&safe<.1)=.1;safe(safe<0&safe>-.1)=-.1;
alpha=atan2(vW,safe);
roll=cfg.ms*cfg.g*cfg.hphi*sin(phi);
frontRoll=roll*cfg.lr/(2*cfg.df*cfg.L);rearRoll=roll*cfg.lf/(2*cfg.dr*cfg.L);
f0=cfg.m*cfg.g*cfg.lr/(2*cfg.L);r0=cfg.m*cfg.g*cfg.lf/(2*cfg.L);
A=bsxfun(@plus,[f0-frontRoll f0+frontRoll r0-rearRoll r0+rearRoll],cfg.deltaFz);
B=[-cfg.m*cfg.h*cfg.lr/(2*cfg.df*cfg.L),cfg.m*cfg.h*cfg.lr/(2*cfg.df*cfg.L), ...
   -cfg.m*cfg.h*cfg.lf/(2*cfg.dr*cfg.L),cfg.m*cfg.h*cfg.lf/(2*cfg.dr*cfg.L)];
[ayRoot,valid,evals,fallback,candidates,ambiguous]=eq11_solve_roots(alpha,A,B,cfg);
[ay,Fz,FxT,FyT,kappa]=eq11_forces_at_ay(ayRoot,alpha,A,B,cfg);
evals(valid)=evals(valid)+1;
residual=abs(ayRoot-ay);
% Raw Eq11 loads at the same actual force acceleration used in the ODE.
expected=bsxfun(@plus,A,bsxfun(@times,ay,B));
loadResidual=max(abs(Fz-expected),[],2);
valid=valid&all(isfinite(x),2)&isfinite(residual)&residual<=cfg.load_solver_residual_tolerance;
Fx=cs*(FxT(:,1)+FxT(:,2))+FxT(:,3)+FxT(:,4)-sn*(FyT(:,1)+FyT(:,2));
Fy=sn*(FxT(:,1)+FxT(:,2))+cs*(FyT(:,1)+FyT(:,2))+FyT(:,3)+FyT(:,4);
Mz=cs*cfg.df*(FxT(:,2)-FxT(:,1))+cfg.dr*(FxT(:,4)-FxT(:,3)) ...
    +sn*cfg.df*(FyT(:,1)-FyT(:,2))+sn*cfg.lf*(FxT(:,1)+FxT(:,2)) ...
    +cs*cfg.lf*(FyT(:,1)+FyT(:,2))-cfg.lr*(FyT(:,3)+FyT(:,4));
udot=Fx/cfg.m+v.*r;if strcmp(cfg.speed_mode,'fixed'),udot=zeros(size(v));end
deltaMx=(cfg.deltaFz(2)-cfg.deltaFz(1))*cfg.df+(cfg.deltaFz(4)-cfg.deltaFz(3))*cfg.dr;
pdot=(cfg.ms*cfg.hphi*ay-cfg.Cphi*p+(cfg.ms*cfg.g*cfg.hphi-cfg.Kphi)*phi-deltaMx)/cfg.Ixx;
dx=[udot,ay-u.*r,Mz/cfg.Izz,p,pdot];
invalid=any(~isfinite(x),2)|~valid;dx(invalid,:)=NaN;
if nargout>1
    ay(invalid)=NaN;Fz(invalid,:)=NaN;FxT(invalid,:)=NaN;FyT(invalid,:)=NaN;
    obs=struct('ay',ay,'Fz_raw',Fz,'Fx_tires',FxT,'Fy_tires',FyT,'Fx',Fx,'Fy',Fy,'Mz',Mz, ...
        'alpha',alpha,'kappa',kappa,'wheel_longitudinal_velocity',uW, ...
        'roll_reaction_moment',cfg.m*cfg.h*ay+roll, ...
        'load_transfer_model',cfg.load_transfer_model,'tire_model',cfg.tire_model, ...
        'load_solver_valid',valid,'load_solver_residual_m_s2',residual, ...
        'load_solver_max_load_residual_N',loadResidual,'load_solver_evaluations',evals, ...
        'load_solver_fallback',fallback,'load_solver_candidate_count',candidates, ...
        'load_solver_ambiguous',ambiguous,'load_solver_algebraic_ay',ayRoot);
end
end

function [g,Fz,Fx,Fy,kappa]=eq11_forces_at_ay(a,alpha,A,B,cfg)
Fz=bsxfun(@plus,A,bsxfun(@times,a,B));
z=min(cfg.mf_force_max,max(cfg.mf_force_min,Fz));
kappa=.001635-.0025297*((z-cfg.mf_Fz0)/cfg.mf_Fz0);
[Fx,Fy]=mf210_forces(alpha,kappa,Fz,cfg);
cs=cos(cfg.steer_deg*pi/180);sn=sin(cfg.steer_deg*pi/180);
g=(sn*(Fx(:,1)+Fx(:,2))+cs*(Fy(:,1)+Fy(:,2))+Fy(:,3)+Fy(:,4))/cfg.m;
end

function [root,valid,evals,fallback,candidates,ambiguous]=eq11_solve_roots(alpha,A,B,cfg)
n=size(A,1);root=NaN(n,1);valid=false(n,1);evals=zeros(n,1);
fallback=false(n,1);candidates=zeros(n,1);ambiguous=false(n,1);
tol=cfg.load_solver_residual_tolerance;bound=eq11_force_bound(cfg);
finite=all(isfinite(A),2)&all(isfinite(alpha),2);ids=find(finite);
if isempty(ids),return;end
prev=zeros(n,1);fp=NaN(n,1);current=NaN(n,1);
g0=eq11_forces_at_ay(zeros(numel(ids),1),alpha(ids,:),A(ids,:),B,cfg);
evals(ids)=evals(ids)+1;fp(ids)=-g0;
done=abs(g0)<=tol;solved=ids(done);root(solved)=0;valid(solved)=true;candidates(solved)=1;
current(ids)=max(-bound,min(bound,g0));
active=ids(~done);
for iter=1:cfg.load_solver_fast_iterations
    if isempty(active),break;end
    g=eq11_forces_at_ay(current(active),alpha(active,:),A(active,:),B,cfg);
    evals(active)=evals(active)+1;f=current(active)-g;
    done=isfinite(f)&abs(f)<=tol;solved=active(done);
    root(solved)=current(solved);valid(solved)=true;candidates(solved)=1;
    rows=active(~done);f=f(~done);if isempty(rows),break;end
    den=f-fp(rows);next=current(rows)-f.*(current(rows)-prev(rows))./den;
    usable=isfinite(next)&isfinite(den)&abs(den)>=1e-16&abs(next)<=bound&abs(next-current(rows))>=1e-14;
    prev(rows)=current(rows);fp(rows)=f;
    current(rows)=next;
    active=rows(usable);
end
for i=find(finite&~valid).'
    fallback(i)=true;
    [rr,ok,extra,count,amb]=eq11_fallback(alpha(i,:),A(i,:),B,cfg,bound);
    evals(i)=evals(i)+extra;candidates(i)=count;ambiguous(i)=amb;
    if ok,root(i)=rr;valid(i)=true;end
end
end

function bound=eq11_force_bound(cfg)
dfmax=max(abs(([cfg.mf_force_min cfg.mf_force_max]-cfg.mf_Fz0)/cfg.mf_Fz0));
scale=abs(cfg.mu/cfg.mf_mu_reference);sn=sin(cfg.steer_deg*pi/180);cs=cos(cfg.steer_deg*pi/180);
fxmax=(1.145+.2106*dfmax)*cfg.mf_force_max*scale;
fymax=(1.0278+.17125*dfmax)*cfg.mf_force_max*scale+cfg.mf_force_max*(.007239+7.442e-5*dfmax);
bound=(2*abs(sn)*fxmax+(2*abs(cs)+2)*fymax)/cfg.m+1;
end

function [chosen,valid,evals,count,ambiguous]=eq11_fallback(alpha,A,B,cfg,bound)
tol=cfg.load_solver_residual_tolerance;points=[-bound bound];roots=[];evals=0;
for j=1:4
    if B(j)~=0
        for load=[cfg.mf_force_min cfg.mf_force_max]
            p=(load-A(j))/B(j);if p>-bound&&p<bound,points(end+1)=p;end %#ok<AGROW>
        end
    end
end
points=unique(sort(points));
% Boundary evaluation uses actual preserved force activation, never a sign
% discontinuity as a substitute for a zero of the physical residual.
for p=points
    g=eq11_forces_at_ay(p,alpha,A,B,cfg);evals=evals+1;
    if isfinite(g)&&abs(p-g)<=tol,roots=eq11_add_root(roots,p);end
end
for k=1:numel(points)-1
    lo=points(k);hi=points(k+1);offset=32*eps*max([1 abs(lo) abs(hi)]);
    if hi-lo<=2*offset,continue;end
    nodes=linspace(lo+offset,hi-offset,17).';
    gg=eq11_forces_at_ay(nodes,repmat(alpha,17,1),repmat(A,17,1),B,cfg);
    evals=evals+17;res=nodes-gg;
    for j=1:17
        if isfinite(res(j))&&abs(res(j))<=tol,roots=eq11_add_root(roots,nodes(j));end
    end
    for j=1:16
        if ~isfinite(res(j))||~isfinite(res(j+1))||res(j)*res(j+1)>=0,continue;end
        left=nodes(j);right=nodes(j+1);fl=res(j);fr=res(j+1);
        for iter=1:80
            candidate=(left*fr-right*fl)/(fr-fl);
            if ~isfinite(candidate)||candidate<=left+.05*(right-left)||candidate>=right-.05*(right-left)
                candidate=(left+right)/2;
            end
            g=eq11_forces_at_ay(candidate,alpha,A,B,cfg);evals=evals+1;f=candidate-g;
            if ~isfinite(f),break;end
            if abs(f)<=tol,roots=eq11_add_root(roots,candidate);break;end
            if fl*f<0,right=candidate;fr=f;else,left=candidate;fl=f;end
            if right-left<1e-14,break;end
        end
    end
end
count=numel(roots);chosen=NaN;valid=false;ambiguous=false;
if count==0,return;end
physical=false(size(roots));
for k=1:count,physical(k)=all(A+B*roots(k)>0);end
if nnz(physical)==1
    chosen=roots(physical);valid=true;
elseif nnz(physical)>1
    ambiguous=true;
elseif count==1
    chosen=roots(1);valid=true;
else
    ambiguous=true;
end
end

function roots=eq11_add_root(roots,value)
if isempty(roots)||all(abs(roots-value)>=1e-7)
    roots=sort([roots value]);
end
end


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


function region=paper_sec31_region(stable_mask,G)
%PAPER_SEC31_REGION Explicit discrete completion of paper Sec3.1 Steps1-4.
% No volume objective, equilibrium anchor, rank erosion, cross-angle nesting,
% coordinate correction, roll cap, or fitted aspect ratio is used.
% The publication omits executable branch/eight-corner tie rules. These
% numerical choices are documented in README.md and retained as diagnostics.
% Invalid extreme-point candidates are saved, never silently replaced.
  m=logical(stable_mask);names={'vc','r','phi'};sz=zeros(1,3);
  for k=1:3
    G.(names{k})=double(G.(names{k})(:));g=G.(names{k});sz(k)=numel(g);
    assert(all(isfinite(g))&&all(diff(g)>0),'Finite ascending grid required');
  end
  assert(isequal(size(m),sz),'Grid/mask size mismatch');
  lower=zeros(sz(1)*sz(2),3);upper=lower;n=0;censored=0;disconnected=0;
  for v=1:sz(1)
    for r=1:sz(2)
      ps=find(reshape(m(v,r,:),[],1));if isempty(ps),continue;end
      n=n+1;lower(n,:)=[v r ps(1)]-1;upper(n,:)=[v r ps(end)]-1;
      censored=censored+(ps(1)==1||ps(end)==sz(3));
      disconnected=disconnected+any(diff(ps)>1);
    end
  end
  lower=lower(1:n,:);upper=upper(1:n,:);limit=unique([lower;upper],'rows');
  outer=zeros(0,3);inner=zeros(0,3);rows=cell(0,1);
  counts=struct('one_band_coincident',0,'two_bands_central_pair',0, ...
    'multiple_bands_ambiguous_excluded',0,'central_interval_contains_unstable_excluded',0);
  rps=unique(limit(:,2:3),'rows');
  for ii=1:size(rps,1)
    r=rps(ii,1);p=rps(ii,2);vs=sort(limit(limit(:,2)==r&limit(:,3)==p,1));
    cuts=[0;find(diff(vs)>1);numel(vs)];nb=numel(cuts)-1;
    outer=[outer;vs(1) r p;vs(end) r p]; %#ok<AGROW>
    row=struct('r_index',r,'phi_index',p,'band_count',nb,'status','','inner_vc_indices',zeros(1,0));
    if nb==1
      pair=[vs(1) vs(end)];counts.one_band_coincident=counts.one_band_coincident+1;
      row.status='one_band_inner_outer_coincident_at_grid_resolution';
    elseif nb==2
      pair=[vs(cuts(2)) vs(cuts(2)+1)];
      if ~all(m(pair(1)+1:pair(2)+1,r+1,p+1))
        pair=[];counts.central_interval_contains_unstable_excluded=counts.central_interval_contains_unstable_excluded+1;
        row.status='central_gap_not_all_stable_excluded';
      else
        counts.two_bands_central_pair=counts.two_bands_central_pair+1;
        row.status='facing_endpoints_of_two_branches';
      end
    else
      pair=[];counts.multiple_bands_ambiguous_excluded=counts.multiple_bands_ambiguous_excluded+1;
      row.status='multiple_boundary_branches_ambiguous_excluded';
    end
    row.inner_vc_indices=pair;rows{end+1,1}=row; %#ok<AGROW>
    for v=pair,inner(end+1,:)=[v r p];end %#ok<AGROW>
  end
  outer=unique(outer,'rows');inner=unique(inner,'rows');
  [v,r,p]=ind2sub(sz,find(~m));bad=coords_idx([v r p]-1,G);
  a=eight_from_sections(inner,G,bad);b=eight_from_sections(outer,G,bad);
  region=struct('method','Paper Sec3.1 extreme-point sequence with declared discrete branch/corner completion', ...
    'author_exact_algorithm_claimed',false,'max_volume_objective_used',false,'equilibrium_anchor_used',false, ...
    'excluded_node_definition','Every node not classified stable; may include physically unstable and numerically unresolved nodes', ...
    'nesting_constraints_used',false,'roll_scaling_or_caps_applied',false, ...
    'limit_roll_points',coords_idx(limit,G),'lower_roll_points',coords_idx(lower,G), ...
    'upper_roll_points',coords_idx(upper,G),'outer_limit_points',coords_idx(outer,G), ...
    'inner_limit_points',coords_idx(inner,G),'lower_roll_indices_zero_based',lower, ...
    'upper_roll_indices_zero_based',upper,'limit_roll_indices_zero_based',limit, ...
    'outer_limit_indices_zero_based',outer,'inner_limit_indices_zero_based',inner, ...
    'slice_diagnostics',{rows},'branch_counts',counts,'stable_vc_yaw_slices',n, ...
    'roll_domain_censored_slices',censored,'disconnected_roll_slices',disconnected, ...
    'physical_roll_boundary_resolved',n>0&&censored==0,'inner_candidate',a,'outer_candidate',b, ...
    'is_valid_inner',a.valid,'continuous_stability_certificate',false);
end

function p=coords_idx(idx,G)
  p=zeros(size(idx,1),3);names={'vc','r','phi'};
  for k=1:3,p(:,k)=G.(names{k})(idx(:,k)+1);end
end

function out=eight_from_sections(idx,G,bad)
  idx=unique(idx,'rows');sections=cell(0,1);
  for r=unique(idx(:,2)).'
    a=idx(idx(:,2)==r,:);p=[G.vc(a(:,1)+1),G.phi(a(:,3)+1)];
    [four,rule]=section_four(p);
    if ~isempty(four),sections{end+1,1}=struct('r',r,'four',four,'rule',rule);end %#ok<AGROW>
  end
  out=struct('valid',false,'vertices',zeros(0,3),'faces',zeros(0,4),'volume',NaN, ...
    'unavailable_reason','Fewer than two yaw sections admit four distinct convex extrema');
  if numel(sections)<2,return;end
  lo=sections{1};hi=sections{end};v=[lo.four(:,1),repmat(G.r(lo.r+1),4,1),lo.four(:,2); ...
                                    hi.four(:,1),repmat(G.r(hi.r+1),4,1),hi.four(:,2)];
  out=hull_audit(v,bad);out.yaw_section_indices_zero_based=[lo.r hi.r];
  out.yaw_section_selection='First and last yaw sections admitting four nondegenerate extrema; no inward search';
  out.cap_selection_rules={lo.rule,hi.rule};
  if ~out.valid,out.unavailable_reason='Extreme-point candidate fails eight-vertex/six-quad/zero-unstable audit';end
end

function [four,rule]=section_four(p)
  p=unique(p,'rows');four=zeros(0,2);rule='fewer_than_four_nondegenerate_points';
  if size(p,1)<4||rank(bsxfun(@minus,p,mean(p,1)),1e-11)<2,return;end
  [h,ok]=canonical_polygon(p);if ~ok,rule='degenerate_section';return;end
  if size(h,1)==4,four=h;rule='exact_four_vertex_section_hull';return;end
  selected=zeros(4,2);axes=[1 2 1 2];signs=[-1 1 1 -1];
  for ii=1:4
    a=axes(ii);s=signs(ii);value=max(p(:,a)*s);hits=p(abs(p(:,a)*s-value)<=1e-12,:);
    other=3-a;mid=(min(p(:,other))+max(p(:,other)))/2;
    order=sortrows([abs(hits(:,other)-mid),hits],[1 2 3]);selected(ii,:)=order(1,2:3);
  end
  selected=unique(selected,'rows');
  if size(selected,1)~=4,rule='cardinal_extrema_not_four_distinct_points';return;end
  [h,ok]=canonical_polygon(selected);
  if ~ok,rule='cardinal_extrema_degenerate';return;end
  if size(h,1)~=4,rule='cardinal_extrema_not_convex_four_vertices';return;end
  four=h;rule='four_cardinal_vc_phi_extrema';
end

function [q,ok]=canonical_polygon(p)
  q=zeros(0,2);ok=false;
  try,k=convhull(p(:,1),p(:,2));catch,return;end
  q=p(k(1:end-1),:);j=[2:size(q,1),1];area=sum(q(:,1).*q(j,2)-q(:,2).*q(j,1));
  if area<0,q=flipud(q);end
  [~,order]=sortrows(q,[1 2]);start=order(1);q=q([start:end,1:start-1],:);ok=true;
end

function out=hull_audit(v,bad)
  out=struct('valid',false,'vertices',v,'faces',zeros(0,4),'volume',NaN, ...
    'extreme_vertices',0,'planar_face_count',0,'unstable_nodes_inside_inclusive',-1, ...
    'excluded_not_stable_nodes_inside_inclusive',-1, ...
    'excluded_node_definition','Every node not classified stable; may include physically unstable and numerically unresolved nodes');
  if size(v,1)~=8||size(unique(v,'rows'),1)~=8,return;end
  try,[tri,vol]=convhulln(v);catch,return;end
  center=mean(v,1);planes=zeros(0,4);
  for ii=1:size(tri,1)
    p=v(tri(ii,:),:);n=cross(p(2,:)-p(1,:),p(3,:)-p(1,:));normn=norm(n);if normn<1e-14,continue;end
    n=n/normn;if dot(n,center-p(1,:))>0,n=-n;end
    eq=[n -dot(n,p(1,:))];
    if isempty(planes)||~any(max(abs(bsxfun(@minus,planes,eq)),[],2)<1e-9),planes(end+1,:)=eq;end %#ok<AGROW>
  end
  count=0;
  for start=1:20000:size(bad,1)
    pts=bad(start:min(start+19999,size(bad,1)),:);
    count=count+nnz(all(bsxfun(@plus,pts*planes(:,1:3).',planes(:,4).')<=1e-10,2));
  end
  faces=zeros(0,4);
  for ii=1:size(planes,1)
    eq=planes(ii,:);ids=find(abs(v*eq(1:3).'+eq(4))<1e-9);
    if numel(ids)~=4,continue;end
    pts=v(ids,:);c=mean(pts,1);u=pts(1,:)-c;u=u/norm(u);w=cross(eq(1:3),u);
    z=bsxfun(@minus,pts,c);ang=atan2(z*w.',z*u.');[~,ord]=sort(ang);cycle=ids(ord).';
    [~,first]=min(cycle);cycle=cycle([first:end,1:first-1]);
    if cycle(2)>cycle(end),cycle=[cycle(1),cycle(end:-1:2)];end
    faces(end+1,:)=cycle; %#ok<AGROW>
  end
  faces=sortrows(faces);extreme=numel(unique(tri(:)));
  out.volume=vol;out.extreme_vertices=extreme;out.planar_face_count=size(planes,1);
  out.hull_triangles_1based=tri;out.unstable_nodes_inside_inclusive=count;
  out.excluded_not_stable_nodes_inside_inclusive=count;
  if size(faces,1)==6,out.faces=faces;end
  out.valid=extreme==8&&size(planes,1)==6&&size(faces,1)==6&&count==0&&vol>1e-14;
end


function fig=plot_mf210_eq11_baseline(results_dir,output_stem,visible)
%PLOT_MF210_EQ11_BASELINE Actual sampled sets and accepted Sec3.1 regions.
% No invalid extreme-point hull is filled as an accepted stability region.
  if nargin<1||isempty(results_dir),results_dir=fullfile(fileparts(mfilename('fullpath')),'results');end
  if nargin<2||isempty(output_stem),output_stem=fullfile(results_dir,'eq11_baseline_comparison');end
  if nargin<3||isempty(visible),visible='on';end
  fig=figure('Color','w','Position',[50 100 1500 560],'Visible',visible);
  angles=[0 3.5 7];colors=[.2 .65 .32;.78 .64 .05;.15 .30 .84];
  for ii=1:3
    s=load(fullfile(results_dir,sprintf('stable_point_40kph_0.85mu_%gdeg.mat',angles(ii))));
    p=s.PAPER_STABILITY3D.result.stable_points;region=s.INNER_REGION;
    ax=subplot(1,3,ii,'Parent',fig);hold(ax,'on');
    if ~isempty(p)
      ids=unique(round(linspace(1,size(p,1),min(size(p,1),1800))));
      scatter3(ax,p(ids,2),p(ids,1),p(ids,3),5,colors(ii,:),'filled');
    end
    if region.valid
      faces=region.faces;
      if exist('OCTAVE_VERSION','builtin')~=0,faces=[faces(:,[1 2 3]);faces(:,[1 3 4])];end
      patch('Parent',ax,'Vertices',region.vertices(:,[2 1 3]),'Faces',faces, ...
        'FaceColor',colors(ii,:),'FaceAlpha',.25,'EdgeColor',[.2 .2 .2]);
      region_status=sprintf('Accepted inner V=%.4g',region.volume);
    else
      region_status='No accepted inner hexahedron';
    end
    plot3(ax,0,0,0,'kp','MarkerFaceColor','w','MarkerSize',9);
    set(ax,'XDir','reverse','XLim',[-.72 .72],'YLim',[-10 10],'ZLim',[-.4 .4], ...
      'Projection','orthographic');view(ax,-72,11);pbaspect(ax,[1.3 2.28 1.65]);
    xlabel(ax,'r (rad/s)');ylabel(ax,'v_c (m/s)');zlabel(ax,'\phi (rad)');grid(ax,'on');
    title(ax,{sprintf('%g deg | %d stable',angles(ii),s.PAPER_STABILITY3D.result.stable_count),region_status},'FontSize',10);
  end
  annotation(fig,'textbox',[.05 .02 .90 .06], ...
    'String','40 km/h fixed, mu=0.85 | Printed Eq11 + unchanged MF | Points: five-second sampled stable states | Star: initial origin', ...
    'HorizontalAlignment','center','EdgeColor','none','FontSize',10);
  print(fig,[output_stem '.png'],'-dpng','-r180');savefig(fig,[output_stem '.fig']);
end

