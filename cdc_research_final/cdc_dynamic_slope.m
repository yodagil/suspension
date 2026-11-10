function C=cdc_dynamic_slope(X,Q)
% -partial Fy/partial alpha at actual effective kappa and actual dynamic Fz.
Fz=cdc_fullcar_model('loads',X,Q);m=Q.MFy;df=(Fz-Q.Fz0_vec)./Q.Fz0_vec;
D=(m(:,2)+m(:,3).*df).*Fz*Q.mu_scale_y;
K=m(:,6).*Q.Fz0_vec.*sin(m(:,8).*atan(Fz./(m(:,7).*Q.Fz0_vec)));
B=K./(m(:,1).*D);B(abs(m(:,1).*D)<1e-9)=0;E=min(m(:,4)+m(:,5).*df,1);
z=B.*(X(21:24,:)+m(:,9)+m(:,10).*df);u=z-E.*(z-atan(z));
d=D.*cos(m(:,1).*atan(u)).*m(:,1)./(1+u.^2).*B.*(1-E+E./(1+z.^2));
[Fx,~]=cdc_fullcar_model('tires',X(17:20,:),X(21:24,:),Fz,Q);
cap=max(max(abs(1.145*Q.wear_x-0.2106*df)*Q.mu_scale_x,0.05).*Fz,1);
C=-d.*sqrt(max(0,1-min(abs(Fx)./cap,1).^2));C(Fz<=Q.Fz_force_min)=0;
end
