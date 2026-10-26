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
