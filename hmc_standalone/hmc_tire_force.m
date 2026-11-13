function [Fx,Fy,Mz,diag] = hmc_tire_force(kappa,alpha,Fz,gamma,Vx,pressure,T,road_mu)
%HMC_TIRE_FORCE Independent MF6.2 steady-state forces from supplied .tir data.
% [Fx,Fy,Mz,D] = hmc_tire_force(kappa,alpha,Fz,gamma,Vx,pressure,T,road_mu)
% Units: alpha/gamma rad; Fz/Fx/Fy N; Vx m/s; pressure Pa; moments N*m.
% Inputs may be scalars or equally sized arrays. T is one scalar coefficient
% struct loaded from D.tires. Positive alpha=atan2(wheel Vy,wheel Vx) gives
% negative cornering force with the supplied negative PKY1 coefficients.
% Optional T.SIDE=+1 (left/default) or -1 (right) mirrors alpha/gamma and Fy/Mz.
% road_mu directly multiplies LMUX/LMUY. There is no hidden mu/0.9 scaling.
% Implements pure + combined slip, camber/pressure, pneumatic trail/residual
% aligning moment and rolling resistance. The effective radius diagnostic
% uses Vx/R0 as an omega proxy; use the actual wheel omega in vehicle dynamics.
% Relaxation is NOT integrated here. D.sigma_x/y=abs(Kx/Ky)/structural stiffness
% are data-based first-order approximations for the caller. No proprietary
% MF-Swift belt/road-enveloping/turn-slip or full TNO transient solver is claimed.
% The caller must record TIR validity-range violations; inputs are not clipped.

if nargin<8,road_mu=1;end
if isempty(pressure),pressure=coef(T,'INFLPRES',coef(T,'NOMPRES',0));end
assert(isstruct(T)&&isscalar(T),'T must be one tire coefficient struct');
inputs={kappa,alpha,Fz,gamma,Vx,pressure,road_mu};shape=[1 1];
for j=1:numel(inputs)
 assert(isnumeric(inputs{j}),'Tire inputs must be numeric');
 if ~isscalar(inputs{j})
  if isequal(shape,[1 1]),shape=size(inputs{j});else,assert(isequal(shape,size(inputs{j})),'Non-scalar tire inputs must have equal size');end
 end
end
z=zeros(shape);k=kappa+z;a=alpha+z;fz=Fz+z;g=gamma+z;vx=Vx+z;pressure=pressure+z;mu=road_mu+z;
p=@(name)coef(T,name,default_value(name));
r0=p('UNLOADED_RADIUS');f0=p('FNOMIN')*p('LFZO');pn=p('NOMPRES');
assert(r0>0&&f0>0&&pn>0&&p('VERTICAL_STIFFNESS')>0&&p('LONGITUDINAL_STIFFNESS')>0&&p('LATERAL_STIFFNESS')>0,'Missing positive tire physical parameter');
side=coef(T,'SIDE',1);assert(side==1||side==-1,'T.SIDE must be +1 or -1');
input_valid=isfinite(k)&isfinite(a)&isfinite(fz)&isfinite(g)&isfinite(vx)&isfinite(pressure)&isfinite(mu)&pressure>0&mu>=0;
% Use harmless values only while evaluating invalid samples; diagnostics flag
% them invalid and zero outputs rather than letting NaNs spread to neighbors.
k(~input_valid)=0;a(~input_valid)=0;fz(~input_valid)=0;g(~input_valid)=0;vx(~input_valid)=0;pressure(~input_valid)=pn;mu(~input_valid)=0;
f=max(fz,0)/f0;
Re=r0*(p('Q_RE0')+p('Q_V1')*(vx/p('LONGVL')).^2)-f0/p('VERTICAL_STIFFNESS')*(p('DREFF')*atan(p('BREFF')*f)+p('FREFF')*f);
direction=ones(shape);direction(vx<0)=-1;
ast=tan(side*a).*direction;gst=sin(side*g);df=(fz-f0)/f0;dp=(pressure-pn)/pn;
mux=p('LMUX')*mu;muy=p('LMUY')*mu;
cx=p('PCX1')*p('LCX');
mu_x=(p('PDX1')+p('PDX2')*df).*(1+p('PPX3')*dp+p('PPX4')*dp.^2).*(1-p('PDX3')*gst.^2).*mux;
Dx=mu_x.*fz;
Kx=fz.*(p('PKX1')+p('PKX2')*df).*exp(p('PKX3')*df).*(1+p('PPX1')*dp+p('PPX2')*dp.^2)*p('LKX');
shx=(p('PHX1')+p('PHX2')*df)*p('LHX');
svx=fz.*(p('PVX1')+p('PVX2')*df)*p('LVX').*mux;
kxp=k+shx;
ex=min((p('PEX1')+p('PEX2')*df+p('PEX3')*df.^2).*(1-p('PEX4')*sign(kxp))*p('LEX'),1);
bx=Kx./safe_den(cx*Dx);
fx0=Dx.*sin(cx*atan(mf_wave(bx.*kxp,ex)))+svx;
bxa=(p('RBX1')+p('RBX3')*gst.^2).*cos(atan(p('RBX2')*k))*p('LXAL');
exa=min(p('REX1')+p('REX2')*df,1);
gxa=cos(p('RCX1')*atan(mf_wave(bxa.*(ast+p('RHX1')),exa)))./safe_den(cos(p('RCX1')*atan(mf_wave(bxa*p('RHX1'),exa))));
Fx=fx0.*gxa;
cy=p('PCY1')*p('LCY');
mu_y=(p('PDY1')+p('PDY2')*df).*(1+p('PPY3')*dp+p('PPY4')*dp.^2).*(1-p('PDY3')*gst.^2).*muy;
Dy=mu_y.*fz;
Ky=p('PKY1')*f0*(1+p('PPY1')*dp).*(1-p('PKY3')*abs(gst)).* ...
 sin(p('PKY4')*atan(fz./safe_den((p('PKY2')+p('PKY5')*gst.^2)*f0.*(1+p('PPY2')*dp))))*p('LKY');
kyg=fz.*(p('PKY6')+p('PKY7')*df).*(1+p('PPY5')*dp)*p('LKYC');
svyg=fz.*(p('PVY3')+p('PVY4')*df).*gst*p('LKYC').*muy;
shy=(p('PHY1')+p('PHY2')*df)*p('LHY')+(kyg.*gst-svyg)./safe_den(Ky);
svy=fz.*(p('PVY1')+p('PVY2')*df)*p('LVY').*muy+svyg;
ay=ast+shy;
ey=min((p('PEY1')+p('PEY2')*df).*(1+p('PEY5')*gst.^2-(p('PEY3')+p('PEY4')*gst).*sign(ay))*p('LEY'),1);
by=Ky./safe_den(cy*Dy);
fy0=Dy.*sin(cy*atan(mf_wave(by.*ay,ey)))+svy;
byk=(p('RBY1')+p('RBY4')*gst.^2).*cos(atan(p('RBY2')*(ast-p('RBY3'))))*p('LYKA');
eyk=min(p('REY1')+p('REY2')*df,1);shyk=p('RHY1')+p('RHY2')*df;
gyk=cos(p('RCY1')*atan(mf_wave(byk.*(k+shyk),eyk)))./safe_den(cos(p('RCY1')*atan(mf_wave(byk.*shyk,eyk))));
dvyk=mu_y.*fz.*(p('RVY1')+p('RVY2')*df+p('RVY3')*gst).*cos(atan(p('RVY4')*ast));
svyk=dvyk.*sin(p('RVY5')*atan(p('RVY6')*k))*p('LVYKA');
fylocal=fy0.*gyk+svyk;
% MF6.2 aligning moment, excluding turn-slip/spin effects.
sht=p('QHZ1')+p('QHZ2')*df+(p('QHZ3')+p('QHZ4')*df).*gst;
at=ast+sht;ar=ast+shy+svy./safe_den(Ky);
bt=(p('QBZ1')+p('QBZ2')*df+p('QBZ3')*df.^2).*(1+p('QBZ4')*gst+p('QBZ5')*abs(gst))*p('LKY')./safe_den(muy);
ct=p('QCZ1');
dt=(p('QDZ1')+p('QDZ2')*df).*(1-p('PPZ1')*dp).*fz*r0/f0*p('LTR').* ...
 (1+p('QDZ3')*abs(gst)+p('QDZ4')*gst.^2).*direction;
et=min((p('QEZ1')+p('QEZ2')*df+p('QEZ3')*df.^2).* ...
 (1+(p('QEZ4')+p('QEZ5')*gst)*(2/pi).*atan(bt*ct.*at)),1);
kr=Kx./safe_den(Ky);
ateq=atan(hypot(tan(at),kr.*k)).*sign(at);areq=atan(hypot(tan(ar),kr.*k)).*sign(ar);
trail=dt.*cos(ct*atan(mf_wave(bt.*ateq,et))).*cos(a);
br=p('QBZ9')*p('LKY')./safe_den(muy)+p('QBZ10')*by*cy;
dr=fz*r0.*((p('QDZ6')+p('QDZ7')*df)*p('LRES')+ ...
 (p('QDZ8')+p('QDZ9')*df).*(1+p('PPZ2')*dp).*gst*p('LKZC')+ ...
 (p('QDZ10')+p('QDZ11')*df).*gst.*abs(gst)).*muy.*direction.*cos(a);
mzr=dr.*cos(atan(br.*areq));
lever=r0*(p('SSZ1')+p('SSZ2')*fylocal/f0+(p('SSZ3')+p('SSZ4')*df).*gst)*p('LS');
Mz=side*(-trail.*(fylocal-svyk)+mzr+lever.*Fx);Fy=side*fylocal;
vnorm=abs(vx/p('LONGVL'));
My=-r0*f0*(p('QSY1')+p('QSY2')*Fx/f0+p('QSY3')*vnorm+p('QSY4')*vnorm.^4+ ...
 (p('QSY5')+p('QSY6')*fz/f0).*gst.^2).*(fz/f0).^p('QSY7').* ...
 (pressure/pn).^p('QSY8')*p('LMY').*sign(vx);
sigma_x=abs(Kx)/p('LONGITUDINAL_STIFFNESS');sigma_y=abs(Ky)/p('LATERAL_STIFFNESS');kappa_zero=-shx;
valid=input_valid&isfinite(Fx)&isfinite(Fy)&isfinite(Mz)&isfinite(My)&Re>0;
inactive=fz<=0|mu==0;
Fx(inactive|~input_valid)=0;Fy(inactive|~input_valid)=0;Mz(inactive|~input_valid)=0;My(inactive|~input_valid)=0;
Kx(inactive|~input_valid)=0;Ky(inactive|~input_valid)=0;Dx(inactive|~input_valid)=0;Dy(inactive|~input_valid)=0;kappa_zero(inactive|~input_valid)=0;
sigma_x(inactive|~input_valid)=0;sigma_y(inactive|~input_valid)=0;Re(~input_valid)=0;
valid(inactive&input_valid)=true;
diag=struct('My',My,'Re',Re,'Kx',Kx,'Ky',Ky,'Dx',Dx,'Dy',Dy,'kappa_zero',kappa_zero, ...
 'sigma_x',sigma_x,'sigma_y',sigma_y,'valid',valid,'Fy_pure',side*fy0,'Fx_pure',fx0, ...
 'combined_reduction_x',gxa,'combined_reduction_y',gyk);
end

function y=coef(T,name,fallback)
if isfield(T,name),y=T.(name);else,y=fallback;end
assert(isnumeric(y)&&isscalar(y)&&isfinite(y),'Invalid scalar tire coefficient %s',name);
end
function y=safe_den(x)
y=x;small=abs(y)<1e-12;y(small&y<0)=-1e-12;y(small&y>=0)=1e-12;
end
function y=mf_wave(x,e)
y=x-e.*(x-atan(x));
end
function y=default_value(name)
% Non-supplied MF scaling factors use the MF neutral value; other absent
% coefficients are zero. All five supplied TIRs have every non-SIDE key used.
if startsWith(name,'L')&&~any(strcmp(name,{'LONGVL','LONGITUDINAL_STIFFNESS','LATERAL_STIFFNESS'})),y=1;else,y=0;end
end
