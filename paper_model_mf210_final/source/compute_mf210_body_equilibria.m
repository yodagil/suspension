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
