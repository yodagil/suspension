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
