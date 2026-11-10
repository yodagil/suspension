function H=cdc_handling_reference(points,Q)
% Evaluate original static-load handling inequalities at ACTUAL coordinates.
% Exact derivatives of the original MF replace finite differences only.
% No contour interpolation, fitted polygon, yaw-policy clip, or geometric shift.
assert(size(points,2)==2,'points must be N x 2: [vy,r].');
v=points(:,1)';r=points(:,2)';
ub=Q.ux-Q.yw.*r;vb=v+Q.xw.*r;
uw=Q.cdelta.*ub+Q.sdelta.*vb;vw=-Q.sdelta.*ub+Q.cdelta.*vb;
a=atan2(vw,uw);fz=Q.Fz_static;
m=Q.MFy;df=(fz-Q.Fz0_vec)./Q.Fz0_vec;
D=(m(:,2)+m(:,3).*df).*fz*Q.mu_scale_y;
K=m(:,6).*Q.Fz0_vec.*sin(m(:,8).*atan(fz./(m(:,7).*Q.Fz0_vec)));
B=K./(m(:,1).*D);B(abs(m(:,1).*D)<1e-9)=0;
E=min(m(:,4)+m(:,5).*df,1);
z=B.*(a+m(:,9)+m(:,10).*df);u=z-E.*(z-atan(z));ang=m(:,1).*atan(u);
Fy=D.*sin(ang)+fz.*(m(:,11)+m(:,12).*df);
dFy=D.*cos(ang).*m(:,1)./(1+u.^2).*B.*(1-E+E./(1+z.^2));
den=ub.^2+vb.^2;av=ub./den;ar=(Q.xw.*ub+Q.yw.*vb)./den;
b=Q.cdelta/Q.m;zcoef=(Q.xw.*Q.cdelta+Q.yw.*Q.sdelta)/Q.Iz;
a11=sum(b.*dFy.*av,1);a12=sum(b.*dFy.*ar,1)-Q.ux;
a21=sum(zcoef.*dFy.*av,1);a22=sum(zcoef.*dFy.*ar,1);
tr=a11+a22;detA=a11.*a22-a12.*a21;C=-dFy;
valid=all(uw>0.1,1)&all(isfinite(C),1)&isfinite(tr)&isfinite(detA);
inside=valid & tr<0 & detA>0 & all(C>0,1);
% Sign-preserving diagnostic quantity, NOT a Euclidean distance. Trace/det
% are normalized by 1 s^-1 and 1 s^-2, slopes by static |K| in N/rad.
margin=min([-tr;detA;min(C./max(abs(K),1),[],1)],[],1);margin(~valid)=-Inf;
H=struct('inside',inside','margin',margin','trace',tr','determinant',detA', ...
 'min_tire_restoring_slope',min(C,[],1)', ...
 'flow',[sum(Q.cdelta.*Fy,1)/Q.m-Q.ux*r;sum(zcoef.*Fy,1)]', ...
 'definition','Static-load MF reference: tr(J)<0, det(J)>0, all -dFy/dalpha>0; not a stability certificate');
end
