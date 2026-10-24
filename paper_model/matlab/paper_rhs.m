function [dx, obs] = paper_rhs(x,cfg)
% RHS implements Xiao et al. 2022 Eqs (1)-(11), with stated assumptions.
% x: N-by-5 [u_c, v_c, r, phi, p]. Each row is one candidate state.
% Fx/Fy below represent ROAD forces in the paper's force resolution (Eq 2).
% WARNING: force_sign_y=-1 is an explicitly documented sign convention
% correction because literal Eq (5)+Eq (10) otherwise yields anti-restoring
% lateral forces about straight driving in these coordinate conventions.
u=x(:,1); v=x(:,2); r=x(:,3); phi=x(:,4); p=x(:,5);
n=size(x,1);
uc=sign(u).*max(abs(u),cfg.min_u);
uc(uc==0)=cfg.min_u;
delta=cfg.steer_deg*pi/180;
cs=cos(delta); sn=sin(delta);
% Eq (5): front / rear tire slip angles (printed small-angle ratio, not atan).
aFL=(v+cfg.lf*r)./signed_guard(uc-cfg.df*r)-delta;
aFR=(v+cfg.lf*r)./signed_guard(uc+cfg.df*r)-delta;
aRL=(v-cfg.lr*r)./signed_guard(uc-cfg.dr*r);
aRR=(v-cfg.lr*r)./signed_guard(uc+cfg.dr*r);
A=[aFL aFR aRL aRR];
% Eqs (6)-(10): longitudinal slips assumed known, zero by default.
LL=repmat(cfg.lambda_long,n,1);
LS=tan(A);
braking=(LL<=0);
LS(braking)=(1+LL(braking)).*LS(braking);
RES=sqrt(LL.^2+LS.^2);
shape=cfg.burckhardt_shape;
C1=shape(1); C2=shape(2); C3=shape(3);
if cfg.scale_burckhardt_to_mu
    peakS=log(C1*C2/C3)/C2;
    peakMu=C1*(1-exp(-C2*peakS))-C3*peakS;
    mult=cfg.mu/peakMu;
else
    mult=1;
end
muRes=mult*(C1*(1-exp(-C2*RES))-C3*RES);
muRes=max(0,muRes);
% Eq (11) is affine in ay. The tire coefficients below depend on the
% instantaneous slips, not on Fz, so its coupling with ay=Fy/m is solved
% algebraically. No fixed number of unconverged load iterations is used.
denom=max(RES,1e-12);
cx=muRes.*(LL./denom);
cy=cfg.force_sign_y*cfg.Ks*muRes.*(LS./denom);
qy=[sn*cx(:,1:2)+cs*cy(:,1:2),cy(:,3:4)];
baseLoads=paper_loads(phi,zeros(n,1),cfg);
frontB=cfg.m*cfg.h*cfg.lr/(2*cfg.df*cfg.L);
rearB=cfg.m*cfg.h*cfg.lf/(2*cfg.dr*cfg.L);
loadSlope=[-frontB frontB -rearB rearB];
[ay,loadValid,equilibriumCount]=solve_load_equilibrium(baseLoads,loadSlope,qy,cfg.m);
FzRaw=baseLoads+bsxfun(@times,ay,loadSlope);
Fz=max(0,FzRaw);
Fz(~loadValid,:)=NaN;
FxT=Fz.*cx;
FyT=Fz.*cy;
Fy=sn*(FxT(:,1)+FxT(:,2))+cs*(FyT(:,1)+FyT(:,2))+FyT(:,3)+FyT(:,4);
% Reconcile floating-point roundoff once after the analytic solve. This is
% not an iterative contact solver; the active-set equilibrium is above.
ayForce=Fy/cfg.m;
FzRaw=paper_loads(phi,ayForce,cfg);
Fz=max(0,FzRaw);
Fz(~loadValid,:)=NaN;
FxT=Fz.*cx;
FyT=Fz.*cy;
Fx=cs*(FxT(:,1)+FxT(:,2))+FxT(:,3)+FxT(:,4)-sn*(FyT(:,1)+FyT(:,2));
Fy=sn*(FxT(:,1)+FxT(:,2))+cs*(FyT(:,1)+FyT(:,2))+FyT(:,3)+FyT(:,4);
Mz=cs*cfg.df*(FxT(:,2)-FxT(:,1))+cfg.dr*(FxT(:,4)-FxT(:,3))+sn*cfg.df*(FyT(:,1)-FyT(:,2))+sn*cfg.lf*(FxT(:,1)+FxT(:,2))+cs*cfg.lf*(FyT(:,1)+FyT(:,2))-cfg.lr*(FyT(:,3)+FyT(:,4));
deltaMx=(cfg.deltaFz(2)-cfg.deltaFz(1))*cfg.df+(cfg.deltaFz(4)-cfg.deltaFz(3))*cfg.dr;
% Eqs (1)-(4). Equation (4) contains lateral acceleration (v_dot+u*r).
udot=Fx/cfg.m+v.*r;
if strcmp(cfg.speed_mode,'fixed')
    udot=zeros(n,1);
end
vdot=Fy/cfg.m-u.*r;
rdot=Mz/cfg.Izz;
phidot=p;
pdot=(cfg.ms*cfg.hphi*(Fy/cfg.m)-cfg.Cphi*p+(cfg.ms*cfg.g*cfg.hphi-cfg.Kphi)*phi-deltaMx)/cfg.Ixx;
dx=[udot vdot rdot phidot pdot];
dx(~loadValid,:)=NaN;
% Report Eq (11) loads at the SAME final Fy/m as the lateral equation.
obs.ay=Fy/cfg.m;
obs.Fz_raw=paper_loads(phi,obs.ay,cfg);
obs.Fx_tires=FxT;
obs.Fy_tires=FyT;
obs.alpha=A;
obs.lambda_res=RES;
obs.mu_res=muRes;
obs.Fx=Fx;
obs.Fy=Fy;
obs.Mz=Mz;
obs.load_solver_valid=loadValid;
obs.load_solver_equilibrium_count=equilibriumCount;
obs.load_solver_contact_count=sum(obs.Fz_raw>0,2);
obs.load_solver_residual_m_s2=abs(obs.ay-ay);
obs.load_solver_max_load_residual_N=max(abs(obs.Fz_raw-FzRaw),[],2);
obs.load_solver_method='analytic piecewise-affine contact equilibrium';
end

function z=signed_guard(z)
small=abs(z)<0.5;
z(small & z>=0)=0.5;
z(small & z<0)=-0.5;
end

function Fz=paper_loads(phi,ay,cfg)
% Eq (11): explicit front/rear loads, lateral transfer, roll coupling.
front0=cfg.m*cfg.g*cfg.lr/(2*cfg.L);
rear0=cfg.m*cfg.g*cfg.lf/(2*cfg.L);
frontAy=cfg.m*ay*cfg.h*cfg.lr/(2*cfg.df*cfg.L);
rearAy=cfg.m*ay*cfg.h*cfg.lf/(2*cfg.dr*cfg.L);
frontRoll=cfg.ms*cfg.g*sin(phi)*cfg.lr*cfg.hphi/(2*cfg.df*cfg.L);
rearRoll=cfg.ms*cfg.g*sin(phi)*cfg.lf*cfg.hphi/(2*cfg.dr*cfg.L);
Fz=[front0-frontAy-frontRoll+cfg.deltaFz(1), front0+frontAy+frontRoll+cfg.deltaFz(2), rear0-rearAy-rearRoll+cfg.deltaFz(3), rear0+rearAy+rearRoll+cfg.deltaFz(4)];
end

function [ay,valid,equilibriumCount]=solve_load_equilibrium(a,b,q,m)
% Solve m*ay=sum_i q_i*max(0,a_i+b_i*ay), including contact changes.
% A strict contraction bound proves uniqueness for the usual all-loaded
% branch. Only other rows require enumeration of the sixteen contact sets.
n=size(a,1);
ay=nan(n,1); valid=false(n,1); equilibriumCount=zeros(n,1);
qb=bsxfun(@times,q,b);
den=m-sum(qb,2);
candidate=sum(q.*a,2)./den;
raw=a+bsxfun(@times,candidate,b);
finiteInput=all(isfinite(a),2)&all(isfinite(q),2)&isfinite(m)&m>0;
contractive=sum(abs(qb),2)<m;
denTol=64*eps(max(m,sum(abs(qb),2)));
fast=finiteInput&contractive&isfinite(candidate)&abs(den)>denTol&all(raw>=0,2);
ay(fast)=candidate(fast);valid(fast)=true;equilibriumCount(fast)=1;
rows=find(finiteInput&~fast);
if isempty(rows), return; end
aa=a(rows,:);qq=q(rows,:);qbb=qb(rows,:);
roots=nan(numel(rows),16);
continuum=false(numel(rows),1);
for code=0:15
    contact=double(bitget(uint8(code),1:4));
    dd=m-sum(bsxfun(@times,qbb,contact),2);
    numerator=sum(bsxfun(@times,qq.*aa,contact),2);
    singular=abs(dd)<=64*eps(max(m,sum(abs(qbb),2)));
    zeroNumerator=abs(numerator)<=128*eps(max(1,sum(abs(qq.*aa),2)));
    if any(singular&zeroNumerator)
        % A zero denominator and numerator can describe an interval of
        % equilibria. Do not mistake a boundary root from another contact
        % label for a unique solution in this case.
        lower=-inf(numel(rows),1);upper=inf(numel(rows),1);
        possible=true(numel(rows),1);
        for wheel=1:4
            direction=2*contact(wheel)-1;
            slope=direction*b(wheel);
            offset=direction*aa(:,wheel);
            if slope>0
                lower=max(lower,-offset/slope);
            elseif slope<0
                upper=min(upper,-offset/slope);
            else
                possible=possible&offset>=0;
            end
        end
        continuum=continuum|(singular&zeroNumerator&possible&upper>lower);
    end
    y=numerator./dd;
    rr=aa+bsxfun(@times,y,b);
    loadTol=128*eps(max(1,max(abs(rr),[],2)));
    consistent=true(numel(rows),1);
    for wheel=1:4
        if contact(wheel)
            consistent=consistent&rr(:,wheel)>=-loadTol;
        else
            consistent=consistent&rr(:,wheel)<=loadTol;
        end
    end
    residual=abs(m*y-sum(qq.*max(0,rr),2));
    residualTol=256*eps(max(1,abs(m*y)+sum(abs(qq.*max(0,rr)),2)));
    ok=isfinite(y)&abs(dd)>64*eps(max(m,sum(abs(qbb),2)))&consistent&residual<=residualTol;
    % Different contact labels can represent the same root at Fz=0.
    rootTol=128*eps(max(1,abs(y)));
    duplicate=any(bsxfun(@le,abs(bsxfun(@minus,roots,y)),rootTol),2);
    roots(ok&~duplicate,code+1)=y(ok&~duplicate);
end
counts=sum(isfinite(roots),2);
counts(continuum)=Inf;
equilibriumCount(rows)=counts;
uniqueRows=find(counts==1);
if ~isempty(uniqueRows)
    chosen=roots(uniqueRows,:);chosen(~isfinite(chosen))=0;
    ay(rows(uniqueRows))=sum(chosen,2);
    valid(rows(uniqueRows))=true;
end
% No equilibrium or several distinct equilibria are explicit numerical
% failures. Leaving NaN propagates to the existing reason-bit-8 check.
end
