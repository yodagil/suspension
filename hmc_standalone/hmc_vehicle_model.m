function varargout=hmc_vehicle_model(action,varargin)
%HMC_VEHICLE_MODEL Independent HMC-data vehicle equations (not a CarSim solver).
% 34 states, columns independent:
% [vy r z phi theta zu(4) vz p q vu(4) kappa(4) alpha(4) omega(4)
%  ay_diagnostic psi X Y ux speed_integral]. Angles are rad, SI otherwise.
% z/zu are perturbations from the analytically balanced static configuration.
% phi is positive left-side-up; theta is positive nose-down. Body corner
% height = z + y*phi - x*theta. No algebraic lateral-load-transfer correction.
% Public: [Q,T]=...('prepare',c,wear,D), [dx,O]=...('rhs',X,Q),
% X=...('lift',points,Q,c,T), plus loads/slips/geometry/trimstate actions.
% Explicit engineering approximations are retained in Q.assumptions. No
% stability-region calibration, trajectory translation or shape constraints.
switch lower(action)
 case 'prepare'
  Q=prepare_model(varargin{1},varargin{2},varargin{3});
  T=find_trim(Q,varargin{1});varargout={Q,T};
 case 'configuration'
  varargout{1}=prepare_model(varargin{:});
 case 'trim'
  varargout{1}=find_trim(varargin{:});
 case 'rhs'
  [varargout{1:nargout}]=rhs(varargin{:});
 case 'lift'
  varargout{1}=lift_points(varargin{:});
 case 'loads'
  [varargout{1:nargout}]=loads(varargin{:});
 case 'slips'
  [varargout{1:nargout}]=slips(varargin{:});
 case 'geometry'
  [varargout{1:nargout}]=geometry(varargin{:});
 case 'trimstate'
  varargout{1}=trim_state(varargin{:});
 otherwise,error('Unknown vehicle action: %s',action);
end
end

function Q=prepare_model(c,wear,D)
Q=D.vehicle;Q.model_version='HMC_INDEPENDENT_34_V1';
Q.ux_target=field_or(c,'speed_kmh',50)/3.6;
Q.mu=field_or(c,'mu',.5);Q.steer_deg=field_or(c,'steer_deg',0);
Q.Icmd=reshape(field_or(c,'I_mA',[950 950 950 950]),4,1);
Q.spring_modes=reshape(field_or(c,'spring_modes',[2 2 2 2]),4,1);
Q.wear=reshape(wear,4,1);Q.side=[1;-1;1;-1];Q.corner_names={'FL','FR','RL','RR'};
Q.xw=[Q.lf;Q.lf;-Q.lr;-Q.lr];Q.yw=[Q.tf/2;-Q.tf/2;Q.tr/2;-Q.tr/2];
Q.B=[ones(4,1),Q.yw,-Q.xw];
Q.mu_u=Q.mu_u(:);Q.Iw=Q.Iw(:);Q.camber_static_rad=Q.camber_static_rad(:);
Q.toe_static_rad=Q.toe_static_rad(:);Q.Fs0=Q.ms*Q.g*[Q.lr;Q.lr;Q.lf;Q.lf]/(2*Q.L);
Q.Fz0=Q.Fs0+Q.mu_u*Q.g;
Q.Iz_total=Q.Iz+sum(Q.mu_u.*(Q.xw.^2+Q.yw.^2));
Q.spring_compression0=zeros(4,1);Q.jounce0=zeros(4,1);
Q.spring_mode_rate=zeros(4,1);front=[22161 30790 36086];rear=[23632 30790 35007];
Q.tires=cell(4,1);Q.kt=zeros(4,1);Q.ct=zeros(4,1);Q.pressure=zeros(4,1);
Q.Re0=zeros(4,1);Q.Fzmax=zeros(4,1);Q.vaxis=D.damper.v_mm_s(:)';
Q.damper_map=zeros(4,numel(Q.vaxis));
for i=1:4
 if wear(i)==0,name='new';elseif wear(i)==25&&i<=2,name='front25';elseif wear(i)==50&&i<=2,name='front50';elseif wear(i)==25,name='rear25';elseif wear(i)==50,name='rear50';else,error('Unsupported measured tire wear');end
 % All supplied files declare TYRESIDE='Left'. Retain that convention on
 % all corners; physical wheel-side signs are applied in geometry below.
 Q.tires{i}=D.tires.(name);Q.tires{i}.SIDE=1;T=Q.tires{i};
 Q.kt(i)=T.VERTICAL_STIFFNESS;Q.ct(i)=T.VERTICAL_DAMPING;
 Q.pressure(i)=T.INFLPRES;Q.Fzmax(i)=field_or(T,'FZMAX',Inf);
 tab=Q.tables.spring_comp.(Q.corner_names{i});
 Q.spring_compression0(i)=interp_linear(tab.y_N,tab.x_m,Q.Fs0(i),false);
 if i<=2,Q.spring_mode_rate(i)=front(Q.spring_modes(i));map=D.damper.front_N;else,Q.spring_mode_rate(i)=rear(Q.spring_modes(i));map=D.damper.rear_N;end
 for j=1:numel(Q.vaxis),Q.damper_map(i,j)=interp_linear(D.damper.current_mA,map(:,j),Q.Icmd(i),true);end
 Q.Re0(i)=effective_radius(Q.Fz0(i),Q.ux_target/T.UNLOADED_RADIUS,T);
end
% OPT_JNC_DESIGN=0 derives design jounce from the spring equilibrium.
% CMP_SPR_SEAT=1 with zero offsets makes absolute jounce equal compression.
% SV_JNC is subsequent movement from that design datum, not the table input.
Q.jounce0=Q.spring_compression0;
Q.drive_split=[(1-Q.rear_drive_fraction)/2;(1-Q.rear_drive_fraction)/2;Q.rear_drive_fraction/2;Q.rear_drive_fraction/2];
Q.include_compliance=field_or(c,'include_compliance',true);
Q.assumptions={...
 'Independent body/unsprung vehicle topology, not proprietary CarSim/TNO equations.';...
 'Rigid sprung body uses small-angle heave/roll/pitch and four vertical unsprung coordinates.';...
 'Design jounce is independently inverted from static spring load using OPT_JNC_DESIGN=0 and unity spring-seat mapping; no fitted trim offsets.';...
 'Body lateral/longitudinal force coupling uses the supplied CG height above flat ground; exact suspension jacking is not implemented.';...
 'Suspension SPLINE data use piecewise-linear interpolation, retaining knots without a fitted spline.';...
 'Tire vertical perturbations use supplied vertical stiffness/damping at a static load reference.';...
 'Camber/toe and wheel x/y geometry follow measured jounce tables; compliance uses two fixed-point force updates.';...
 'PI speed error uses m/s from documented gain units; demand is limited power fraction with conditional-integration antiwindup.';...
 'Combined MF6.2 forces and first-order relaxation K/stiffness are independent approximations; no MF-Swift belt-state equivalence claim.';...
 'Yaw inertia includes supplied sprung inertia plus four unsprung point-mass parallel-axis terms.'};
end

function [dx,O]=rhs(X,Q)
assert(size(X,1)==34,'Expected34states');n=size(X,2);
[Fz,raw]=loads(X,Q);[delta,gamma,xw,yw,jounce]=geometry(X,Q,Fz);
[kt,at,uw,vw,Re]=slips(X,Q,delta,xw,yw);
Fx=zeros(4,n);Fy=Fx;Mz=Fx;My=Fx;sx=Fx;sy=Fx;
for i=1:4
 [Fx(i,:),Fy(i,:),Mz(i,:),d]=hmc_tire_force(X(16+i,:),X(20+i,:),Fz(i,:),gamma(i,:),uw(i,:),Q.pressure(i),Q.tires{i},Q.mu);
 My(i,:)=d.My;sx(i,:)=max(d.sigma_x,.001);sy(i,:)=max(d.sigma_y,.001);
end
cd=cos(delta);sd=sin(delta);Fxb=Fx.*cd-Fy.*sd;Fyb=Fx.*sd+Fy.*cd;
ay=sum(Fyb,1)/Q.m;
drag=.5*Q.rho_air*Q.aero_area_m2*Q.aero_Cx*X(33,:).*abs(X(33,:));
ax=(sum(Fxb,1)-drag)/Q.m;
[Fs,fd]=suspension_forces(X,Q,jounce);
dFs=Fs-Q.Fs0;
dx=zeros(34,n);dx(1,:)=ay-X(33,:).*X(2,:);
dx(2,:)=sum(xw.*Fyb-yw.*Fxb+Mz,1)/Q.Iz_total;
dx(3:9,:)=X(10:16,:);
dx(10,:)=sum(dFs,1)/Q.ms;
dx(11,:)=(sum(Q.yw.*dFs,1)+Q.ms*Q.h_cg_sprung*ay)/Q.Ixx;
dx(12,:)=(-sum(Q.xw.*dFs,1)-Q.ms*Q.h_cg_sprung*ax)/Q.Iyy;
dx(13:16,:)=(raw-Q.Fz0-dFs)./Q.mu_u;
speed=max(abs(uw),.5);
dx(17:20,:)=speed./sx.*(kt-X(17:20,:));
dx(21:24,:)=speed./sy.*(at-X(21:24,:));
error=Q.ux_target-X(33,:);
demand=Q.speed_KP*error+Q.speed_KP3*error.^3+Q.speed_KI*X(34,:);
throttle=min(max(demand,0),1);
force_drive=throttle*Q.max_power_W./max(abs(X(33,:)),1);
torque=Q.drive_split.*force_drive.*Re;
dx(25:28,:)=(torque-Re.*Fx+My)./Q.Iw;
dx(29,:)=ay-X(29,:); % diagnostic low-pass only; never drives load transfer
psi=X(30,:);dx(30,:)=X(2,:);
dx(31,:)=X(33,:).*cos(psi)-X(1,:).*sin(psi);
dx(32,:)=X(33,:).*sin(psi)+X(1,:).*cos(psi);
dx(33,:)=ax+X(2,:).*X(1,:);
dx(34,:)=error;
windup=(demand<=0&error<0)|(demand>=1&error>0);dx(34,windup)=0;
if nargout>1
 O=struct('ay',ay,'ax',ax,'Fz_raw',raw,'Fz',Fz,'Fx',Fx,'Fy',Fy,'Mz',Mz,'My',My, ...
 'LTR',(raw(1,:)+raw(3,:)-raw(2,:)-raw(4,:))./sum(raw,1), ...
 'LTRf',(raw(1,:)-raw(2,:))./sum(raw(1:2,:),1), ...
 'LTRr',(raw(3,:)-raw(4,:))./sum(raw(3:4,:),1), ...
 'alpha_target',at,'kappa_target',kt,'uw',uw,'vw',vw,'Re',Re, ...
 'delta',delta,'gamma',gamma,'spring_force',Fs-fd,'damper_force',fd,'jounce',jounce, ...
 'throttle',throttle,'speed_error',error,'tire_load_in_range',all(raw<=Q.Fzmax&raw>0,1));
end
end

function [Fz,raw]=loads(X,Q)
% Dynamic reactions only: there is no superimposed algebraic ay load transfer.
raw=Q.Fz0-Q.kt.*X(6:9,:)-Q.ct.*X(13:16,:);
Fz=max(raw,0);
end

function [delta,gamma,xw,yw,jounce]=geometry(X,Q,Fz)
if nargin<3,Fz=loads(X,Q);end
n=size(X,2);jounce=Q.jounce0+X(6:9,:)-Q.B*X([3 4 5],:);
delta=zeros(4,n);gamma=delta;xw=Q.xw+zeros(4,n);yw=Q.yw+zeros(4,n);
gear=deg2rad(Q.steer_deg*Q.steering_ratio)*Q.gear_rot_ratio;
for i=1:4
 corner=Q.corner_names{i};j=jounce(i,:);side=Q.side(i);
 tc=Q.tables.camber.(corner);tt=Q.tables.toe.(corner);
 tx=Q.tables.susp_x.(corner);ty=Q.tables.susp_lat.(corner);
 base_toe=Q.toe_static_rad(i)+interp_linear(tt.x_m,tt.y_rad,j,false);
 base_camber=Q.camber_static_rad(i)+interp_linear(tc.x_m,tc.y_rad,j,false);
 if i<=2,ts=Q.tables.steering.(corner);command=interp_linear(ts.x_rad,ts.y_rad,gear,true);else,command=0;end
 delta(i,:)=command-side*base_toe;
 % MF inclination is positive for the left-wheel inward camber direction.
 gamma(i,:)=-side*base_camber+X(4,:);
 xw(i,:)=Q.xw(i)+interp_linear(tx.x_m,tx.y_m,j,false);
 yw(i,:)=Q.yw(i)-side*interp_linear(ty.x_m,ty.y_m,j,false);
end
if Q.include_compliance
 base_delta=delta;base_gamma=gamma;
 for it=1:2
  for i=1:4
   ub=X(33,:)-yw(i,:).*X(2,:);vb=X(1,:)+xw(i,:).*X(2,:);
   uw=cos(delta(i,:)).*ub+sin(delta(i,:)).*vb;
   [Fx,Fy,Mz]=hmc_tire_force(X(16+i,:),X(20+i,:),Fz(i,:),gamma(i,:),uw,Q.pressure(i),Q.tires{i},Q.mu);
   delta(i,:)=base_delta(i,:)-Q.side(i)*Q.compliance.CT_FX.SI(i)*Fx+Q.compliance.CS_FY.SI(i)*Fy+Q.compliance.CS_MZ.SI(i)*Mz;
   gamma(i,:)=base_gamma(i,:)+Q.side(i)*Q.compliance.CC_FX.SI(i)*Fx+Q.compliance.CI_FY.SI(i)*Fy+Q.compliance.CI_MZ.SI(i)*Mz;
  end
 end
end
end

function [k,a,uw,vw,Re]=slips(X,Q,delta,xw,yw)
if nargin<3,[delta,~,xw,yw]=geometry(X,Q);end
ub=X(33,:)-yw.*X(2,:);vb=X(1,:)+xw.*X(2,:);
uw=cos(delta).*ub+sin(delta).*vb;vw=-sin(delta).*ub+cos(delta).*vb;
a=atan2(vw,uw);Fz=loads(X,Q);Re=zeros(size(uw));
for i=1:4,Re(i,:)=effective_radius(Fz(i,:),X(24+i,:),Q.tires{i});end
k=(Re.*X(25:28,:)-uw)./max(abs(uw),.5);
end

function [Fs,fd]=suspension_forces(X,Q,jounce)
if nargin<3,jounce=Q.jounce0+X(6:9,:)-Q.B*X([3 4 5],:);end
comp_rate=X(13:16,:)-Q.B*X([10 11 12],:);Fs=zeros(size(jounce));fd=Fs;
for i=1:4
 corner=Q.corner_names{i};c=Q.spring_compression0(i)+jounce(i,:)-Q.jounce0(i);
 tc=Q.tables.spring_comp.(corner);te=Q.tables.spring_ext.(corner);
 fs=interp_linear(tc.x_m,tc.y_N,c,false);ext=comp_rate(i,:)<0;
 fs(ext)=interp_linear(te.x_m,te.y_N,c(ext),false);
 fs=fs+(Q.spring_mode_rate(i)-30790)*c;
 tj=Q.tables.jounce_stop.(corner);tr=Q.tables.rebound_stop.(corner);
 fs=fs+interp_linear(tj.x_m,tj.y_N,jounce(i,:),false)+interp_linear(tr.x_m,tr.y_N,jounce(i,:),false);
 fd(i,:)=interp_linear(Q.vaxis,Q.damper_map(i,:),1000*comp_rate(i,:),true);
 Fs(i,:)=fs+fd(i,:);
end
front_rel=X(4,:)-(X(6,:)-X(7,:))/Q.tf;
rear_rel=X(4,:)-(X(8,:)-X(9,:))/Q.tr;
front_rate=X(11,:)-(X(13,:)-X(14,:))/Q.tf;
rear_rate=X(11,:)-(X(15,:)-X(16,:))/Q.tr;
Mf=-Q.Karb_f*front_rel-Q.Carb_f*front_rate;
Mr=-Q.Karb_r*rear_rel-Q.Carb_r*rear_rate;
Fs=Fs+[Mf/Q.tf;-Mf/Q.tf;Mr/Q.tr;-Mr/Q.tr];
end

function X=lift_points(points,Q,c,T)
assert(isfield(T,'x')&&all(isfinite(T.x)),'A valid independent trim is required');
X=repmat(T.x(:),1,size(points,1));X(1,:)=points(:,1)';X(2,:)=points(:,2)';
% CarSim restart changes only vy/yaw-rate: tire and hidden states are retained.
% This deliberately does NOT reinitialize slip states from each new grid point.
if isfield(c,'initialization')&&~strcmp(c.initialization,'trim')
 error('HMC reference study only supports retained trim-state initialization');
end
end

function T=find_trim(Q,c)
T=struct('x',nan(34,1),'root_found',false,'usable',false,'residual_norm',Inf, ...
 'max_real_eigenvalue',NaN,'locally_stable_test',false,'reference_snapshot_used',false, ...
 'note','Independent numerical equilibrium; no CarSim snapshot or stability-region fit');
q=Q;q.steer_deg=0;y=zeros(18,1);y(18)=.01;
y(10:13)=.001;
angles=linspace(0,Q.steer_deg,max(2,ceil(abs(Q.steer_deg)/.5)+1));
for angle=angles
 q.steer_deg=angle;
 if angle~=0&&y(2)==0,y(2)=q.ux_target*deg2rad(angle)/q.L;end
 [y,ok,res]=newton_trim(y,q,c);
 if ~ok,T.residual_norm=res;return;end
end
X=trim_state(y,Q);[d,O]=rhs(X,Q);indices=[1:29 33:34];res=max(abs(d(indices)));
T.x=X;T.root_found=isfinite(res)&&res<field_or(c,'trim_residual_tol',1e-5);
T.residual_norm=res;T.vy_mps=X(1);T.r_radps=X(2);T.vx_kmh=X(33)*3.6;
T.roll_deg=rad2deg(X(4));T.pitch_deg=rad2deg(X(5));
T.rollrate_degps=rad2deg(X(11));T.pitchrate_degps=rad2deg(X(12));
T.usable=T.root_found&&all(O.Fz_raw>0)&&all(isfinite(X))&&abs(X(4))<deg2rad(40)&&X(33)>0;
% State stability excludes advancing pose and a diagnostic-only filter.
ids=[1:28 33:34];J=zeros(numel(ids));
if T.root_found
 h=1e-6*max(1,abs(X(ids)));xx=repmat(X,1,2*numel(ids));
 for j=1:numel(ids),xx(ids(j),j)=xx(ids(j),j)+h(j);xx(ids(j),j+numel(ids))=xx(ids(j),j+numel(ids))-h(j);end
 dd=rhs(xx,Q);J=bsxfun(@rdivide,dd(ids,1:numel(ids))-dd(ids,numel(ids)+1:end),2*h');
 ev=eig(J);T.max_real_eigenvalue=max(real(ev));T.locally_stable_test=T.max_real_eigenvalue<1e-5;
 % Usability is a numerical/domain check. Local stability is saved separately.
end
end

function X=trim_state(y,Q)
n=size(y,2);X=zeros(34,n);X(1:9,:)=y(1:9,:);X(17:20,:)=y(10:13,:);X(21:24,:)=y(14:17,:);
X(33,:)=Q.ux_target;X(34,:)=y(18,:);
Fz=loads(X,Q);[delta,~,xw,yw]=geometry(X,Q,Fz);
ub=X(33,:)-yw.*X(2,:);vb=X(1,:)+xw.*X(2,:);uw=cos(delta).*ub+sin(delta).*vb;
X(25:28,:)=uw.*(1+X(17:20,:))./Q.Re0;
for j=1:7
 for i=1:4
  re=effective_radius(Fz(i,:),X(24+i,:),Q.tires{i});X(24+i,:)=uw(i,:).*(1+X(16+i,:))./re;
 end
end
X(29,:)=Q.ux_target*X(2,:);
end

function [y,ok,res]=newton_trim(y,Q,c)
sc=[1;.1;.01;.03;.03;.01*ones(4,1);.01*ones(4,1);.03*ones(4,1);1];
rs=[2;1;2*ones(7,1);10*ones(4,1);100*ones(4,1);2];
z=y./sc;ok=false;res=Inf;tol=field_or(c,'trim_residual_tol',1e-5);
for it=1:55
 f=trim_equations(z.*sc,Q)./rs;res=max(abs(f.*rs));
 if all(isfinite(f))&&res<tol,ok=true;y=z.*sc;return;end
 if any(~isfinite(f)),break;end
 h=1e-5*max(1,abs(z));zz=repmat(z,1,36);
 for j=1:18,zz(j,j)=zz(j,j)+h(j);zz(j,j+18)=zz(j,j+18)-h(j);end
 ff=trim_equations(bsxfun(@times,zz,sc),Q);
 J=bsxfun(@rdivide,bsxfun(@rdivide,ff(:,1:18)-ff(:,19:36),2*h'),rs);
 if rcond(J)<1e-14,dz=-pinv(J)*f;else,dz=-J\f;end
 if any(~isfinite(dz)),break;end
 merit=norm(f);accepted=false;
 for a=2.^-(0:14)
  trial=z+a*dz;physical=abs(trial(4)*sc(4))<.6&&abs(trial(5)*sc(5))<.4&&all(abs(trial(10:13).*sc(10:13))<.5)&&all(abs(trial(14:17).*sc(14:17))<1.2);
  if ~physical,continue;end
  ft=trim_equations(trial.*sc,Q)./rs;
  if all(isfinite(ft))&&norm(ft)<merit,z=trial;accepted=true;break;end
 end
 if ~accepted,break;end
end
y=z.*sc;
end

function f=trim_equations(y,Q)
X=trim_state(y,Q);d=rhs(X,Q);f=d([1 2 10:16 21:24 25:28 33],:);
end

function r=effective_radius(Fz,omega,T)
R0=T.UNLOADED_RADIUS;F0=T.FNOMIN;V0=T.LONGVL;
r=R0*(field_or(T,'Q_RE0',1)+field_or(T,'Q_V1',0)*(omega*R0/V0).^2)- ...
 (F0/T.VERTICAL_STIFFNESS)*(field_or(T,'DREFF',0)*atan(field_or(T,'BREFF',0)*Fz/F0)+field_or(T,'FREFF',0)*Fz/F0);
r=max(r,.1*R0);
end

function y=interp_linear(x,v,q,clamp)
x=x(:)';v=v(:)';shape=size(q);q=q(:)';
if isempty(q),y=reshape(q,shape);return;end
if clamp,q=min(max(q,x(1)),x(end));end
j=1+sum(q>=x(2:end-1)',1);a=(q-x(j))./(x(j+1)-x(j));y=reshape(v(j)+a.*(v(j+1)-v(j)),shape);
end
function v=field_or(s,key,default)
if isfield(s,key),v=s.(key);else,v=default;end
end
