function H = hmc_handling_reference(points,c,wear)
%HMC_HANDLING_REFERENCE CarSim runner's original static-load MF H policy.
% H = hmc_handling_reference([vy_mps r_radps],c,wear)
% points is N-by-2. The fixed reference uses nominal c.speed_kmh (or c.ux),
% c.mu, and one mean front-wheel angle in degrees. c.reference_steer_deg,
% then c.steer_actual_deg, then c.steer_deg select that angle.
% This reproduces cs_reference/cs_mf_handling in CarSim_1050_Run. It is NOT
% the full HMC vehicle Jacobian and does not replace the five measured TIR
% datasets used by the independent vehicle's dynamic force calculation.
if nargin<3
    if isfield(c,'wear'),wear=c.wear;else,wear=zeros(1,4);end
end
assert(isnumeric(points)&&ismatrix(points)&&size(points,2)==2,'points must be N-by-2 [vy r].');
assert(isnumeric(wear)&&numel(wear)==4&&all(isfinite(wear(:))),'wear must contain four finite values.');
Q=struct();
Q.m=2218.042;Q.Iz=4793;Q.lf=1.472;Q.lr=1.531;
Q.tf=1.633;Q.tr=1.631;Q.g=9.81;
if isfield(c,'speed_kmh'),Q.ux=c.speed_kmh/3.6;
elseif isfield(c,'ux'),Q.ux=c.ux;
else,error('Nominal c.speed_kmh or c.ux is required.');end
assert(isscalar(Q.ux)&&isfinite(Q.ux)&&Q.ux>0,'Nominal speed must be positive.');
assert(isfield(c,'mu')&&isscalar(c.mu)&&isfinite(c.mu)&&c.mu>0,'Positive c.mu is required.');
if isfield(c,'reference_steer_deg'),steer=c.reference_steer_deg;
elseif isfield(c,'steer_actual_deg'),steer=c.steer_actual_deg;
elseif isfield(c,'steer_deg'),steer=c.steer_deg;
else,error('Front-wheel c.steer_deg is required.');end
assert(isscalar(steer)&&isfinite(steer),'Reference steer must be one finite mean front-wheel angle.');
Q.mu_scale_y=c.mu/0.9;
Q.delta=deg2rad([steer;steer;0;0]);
Q.cdelta=cos(Q.delta);Q.sdelta=sin(Q.delta);
Q.xw=[Q.lf;Q.lf;-Q.lr;-Q.lr];
Q.yw=[Q.tf/2;-Q.tf/2;Q.tr/2;-Q.tr/2];
Q.Fz0_vec=5425*ones(4,1);
Q.Fz_static=[Q.m*Q.g*Q.lr/(2*(Q.lf+Q.lr))*ones(2,1); ...
             Q.m*Q.g*Q.lf/(2*(Q.lf+Q.lr))*ones(2,1)];
M=[1.5662 1.0278 -0.17125 -1.1357 -1.0544 -25.85 2.1377 2 8.675e-4 7.046e-5 -0.007239 -7.442e-5; ...
   1.5457 1.0286 -0.16688 -1.439 -1.6063 -26 1.9164 2 1.82e-3 -7.193e-4 1.802e-3 -1.1497e-2; ...
   1.5057 1.0067 -0.2019 -2 -2 -27.27 1.5963 2 2.3735e-3 2.38e-4 0.01487 -0.011025];
wi=1+(wear(:)>=12.5)+(wear(:)>=37.5);
Q.MFy=M(wi,:);
v=points(:,1)';r=points(:,2)';
ub=Q.ux-Q.yw.*r;vb=v+Q.xw.*r;
uw=Q.cdelta.*ub;uw=uw+Q.sdelta.*vb;
vw=-Q.sdelta.*ub+Q.cdelta.*vb;
a=atan2(vw,uw);
fz=Q.Fz_static;m=Q.MFy;df=(fz-Q.Fz0_vec)./Q.Fz0_vec;
D=(m(:,2)+m(:,3).*df).*fz*Q.mu_scale_y;
K=m(:,6).*Q.Fz0_vec.*sin(m(:,8).*atan(fz./(m(:,7).*Q.Fz0_vec)));
B=K./(m(:,1).*D);B(abs(m(:,1).*D)<1e-9)=0;
E=min(m(:,4)+m(:,5).*df,1);
z=B.*(a+m(:,9)+m(:,10).*df);
u=z-E.*(z-atan(z));ang=m(:,1).*atan(u);
dFy=D.*cos(ang).*m(:,1)./(1+u.^2).*B.*(1-E+E./(1+z.^2));
den=ub.^2+vb.^2;av=ub./den;
ar=(Q.xw.*ub+Q.yw.*vb)./den;
b=Q.cdelta/Q.m;
zcoef=(Q.xw.*Q.cdelta+Q.yw.*Q.sdelta)/Q.Iz;
a11=sum(b.*dFy.*av,1);
a12=sum(b.*dFy.*ar,1)-Q.ux;
a21=sum(zcoef.*dFy.*av,1);
a22=sum(zcoef.*dFy.*ar,1);
tr=a11+a22;detA=a11.*a22-a12.*a21;C=-dFy;
valid=all(uw>0.1,1)&all(isfinite(C),1)&isfinite(tr)&isfinite(detA);
inside=valid & tr<0 & detA>0 & all(C>0,1);
margin=min([-tr;detA;min(C./max(abs(K),1),[],1)],[],1);
margin(~valid)=-Inf;
H=struct('inside',inside','margin',margin','trace',tr','determinant',detA', ...
    'definition','Original static-load 2x2 4-wheel MF trace<0 determinant>0 and restoring slopes positive', ...
    'source','CarSim_1050_Run original MF reference policy; not CarSim/HMC full-state Jacobian', ...
    'reference_steer_deg',steer,'nominal_speed_mps',Q.ux);
end
