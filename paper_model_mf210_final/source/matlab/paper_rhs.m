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
