function [dx,obs]=paper_rhs(x,cfg)
% Paper body plus the actual MF210 tire relaxation/free-wheel subsystem.
% [u vc r phi p kappa(4) alpha(4) omega(4) ay_load], 18 states.
u=x(:,1);v=x(:,2);r=x(:,3);phi=x(:,4);p=x(:,5);ayLoad=x(:,18);
delta=cfg.steer_deg*pi/180;cs=cos(delta);sn=sin(delta);
f0=cfg.m*cfg.g*cfg.lr/(2*cfg.L);r0=cfg.m*cfg.g*cfg.lf/(2*cfg.L);
fa=cfg.m*ayLoad*cfg.h*cfg.lr/(2*cfg.df*cfg.L);ra=cfg.m*ayLoad*cfg.h*cfg.lf/(2*cfg.dr*cfg.L);
fr=cfg.ms*cfg.g*sin(phi)*cfg.lr*cfg.hphi/(2*cfg.df*cfg.L);
rr=cfg.ms*cfg.g*sin(phi)*cfg.lf*cfg.hphi/(2*cfg.dr*cfg.L);
Fz=bsxfun(@plus,[f0-fa-fr f0+fa+fr r0-ra-rr r0+ra+rr],cfg.deltaFz);
[FxT,FyT]=mf210_forces(x(:,10:13),x(:,6:9),Fz,cfg);
Fx=cs*(FxT(:,1)+FxT(:,2))+FxT(:,3)+FxT(:,4)-sn*(FyT(:,1)+FyT(:,2));
Fy=sn*(FxT(:,1)+FxT(:,2))+cs*(FyT(:,1)+FyT(:,2))+FyT(:,3)+FyT(:,4);ay=Fy/cfg.m;
Mz=cs*cfg.df*(FxT(:,2)-FxT(:,1))+cfg.dr*(FxT(:,4)-FxT(:,3)) ...
    +sn*cfg.df*(FyT(:,1)-FyT(:,2))+sn*cfg.lf*(FxT(:,1)+FxT(:,2)) ...
    +cs*cfg.lf*(FyT(:,1)+FyT(:,2))-cfg.lr*(FyT(:,3)+FyT(:,4));
uB=[u-cfg.df*r,u+cfg.df*r,u-cfg.dr*r,u+cfg.dr*r];
vB=[v+cfg.lf*r,v+cfg.lf*r,v-cfg.lr*r,v-cfg.lr*r];
uW=[cs*uB(:,1:2)+sn*vB(:,1:2),uB(:,3:4)];
vW=[-sn*uB(:,1:2)+cs*vB(:,1:2),vB(:,3:4)];
safe=uW;safe(safe>=0&safe<.1)=.1;safe(safe<0&safe>-.1)=-.1;
alphaTarget=atan2(vW,safe);
kappaTarget=min(.5,max(-.5,(cfg.mf_Rw*x(:,14:17)-uW)./max(.5,abs(uW))));
vroll=max(.5,abs(uW));rk=min(500,vroll/cfg.mf_sigma_kappa);raRate=min(500,vroll/cfg.mf_sigma_alpha);
udot=Fx/cfg.m+v.*r;if strcmp(cfg.speed_mode,'fixed'),udot=zeros(size(v));end
deltaMx=(cfg.deltaFz(2)-cfg.deltaFz(1))*cfg.df+(cfg.deltaFz(4)-cfg.deltaFz(3))*cfg.dr;
pdot=(cfg.ms*cfg.hphi*ay-cfg.Cphi*p+(cfg.ms*cfg.g*cfg.hphi-cfg.Kphi)*phi-deltaMx)/cfg.Ixx;
dx=[udot,ay-u.*r,Mz/cfg.Izz,p,pdot, ...
    rk.*(kappaTarget-x(:,6:9)),raRate.*(alphaTarget-x(:,10:13)), ...
    -cfg.mf_Rw*FxT/cfg.mf_Iw,(ay-ayLoad)/cfg.mf_tau_load];
if nargout>1
    obs=struct('ay',ay,'Fz_raw',Fz,'Fx_tires',FxT,'Fy_tires',FyT,'Fx',Fx,'Fy',Fy,'Mz',Mz, ...
        'alpha',x(:,10:13),'kappa',x(:,6:9),'alpha_target',alphaTarget,'kappa_target',kappaTarget, ...
        'wheel_longitudinal_velocity',uW,'ay_load',ayLoad, ...
        'load_transfer_model','PaperEq11 with original210 0.03s lateral-load-transfer state', ...
        'tire_model','Original210 MF force, slip relaxation, free-wheel dynamics and initialization');
end
end
