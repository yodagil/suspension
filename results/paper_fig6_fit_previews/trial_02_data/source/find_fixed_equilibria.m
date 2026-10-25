function find_fixed_equilibria(source_dir,outfile)
% Read-only equilibrium and local-attraction check for the comparison model.
addpath('/workspace/suspension/paper_model_dynamic_roll/matlab');
angles=[0 3.5 7]; equilibrium_cases=struct([]);
for k=1:3
    angle=angles(k);s=load(fullfile(source_dir,sprintf('screen_%gdeg.mat',angle)));
    cfg=s.PAPER_SCREEN.cfg;u=cfg.speed_kmh/3.6;
    rg=u*(angle*pi/180)/cfg.L;
    pg=cfg.ms*cfg.hphi*u*rg/(cfg.Kphi-cfg.ms*cfg.g*cfg.hphi);
    guess=[-cfg.lr*rg;rg;pg];
    opt=optimset('TolX',1e-11,'TolFun',1e-11,'MaxIter',500,'Display','off');
    [y,fval,info]=fsolve(@(y)residual(y,u,cfg),guess,opt);
    x=[u y(1) y(2) y(3) 0];[dx,obs]=paper_rhs(x,cfg);
    J=zeros(4);columns=2:5;
    for j=1:4
        step=1e-6;xp=x;xm=x;xp(columns(j))=xp(columns(j))+step;xm(columns(j))=xm(columns(j))-step;
        fp=paper_rhs(xp,cfg);fm=paper_rhs(xm,cfg);
        J(:,j)=(fp(columns)-fm(columns)).'/(2*step);
    end
    eigvals=eig(J);limit=cfg.limit_factor*cfg.mu*cfg.g/u;
    e=struct('angle_deg',angle,'state_u_v_r_phi_p',x,'residual',dx, ...
        'fsolve_info',info,'max_abs_residual',max(abs(dx)), ...
        'eigenvalues_real',real(eigvals),'eigenvalues_imag',imag(eigvals), ...
        'locally_asymptotically_stable',all(real(eigvals)<0), ...
        'yaw_limit',limit,'ay',obs.ay,'minimum_wheel_load_N',min(obs.Fz_raw), ...
        'physical_constraints_valid',abs(x(3))<=limit&&abs(obs.ay)<=cfg.mu*cfg.g&&all(obs.Fz_raw>0));
    if isempty(equilibrium_cases),equilibrium_cases=e;else,equilibrium_cases(k)=e;end
    fprintf('EQUILIBRIUM %gdeg: vc=%.9g r=%.9g phi=%.9g residual=%.3g maxeig=%.5g valid=%d\n', ...
        angle,y(1),y(2),y(3),e.max_abs_residual,max(real(eigvals)),e.physical_constraints_valid);
end
save(outfile,'equilibrium_cases','-v7');
end

function f=residual(y,u,cfg)
x=[u y(1) y(2) y(3) 0];dx=paper_rhs(x,cfg);f=dx([2 3 5]).';
end
