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
