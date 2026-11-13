function s = hmc_classify(t,X,O,T,c,wear)
%HMC_CLASSIFY Apply the uploaded CarSim runner's finite-time output policy.
% t is simulation time; c.t_start is its origin (default zero). X is 34-by-Nt
% with vy1,r2,roll4,pitch5,rollrate11,pitchrate12,ux33, all in SI.
% O.ay is m/s^2. O.Fz_raw (preferred) or O.Fz is 4-by-Nt [FL FR RL RR] N.
% T can contain x (34 states) or the CarSim trim fields in m/s,rad/s,deg,km/h.
% All observed samples count, including the initial roll-rate transient.
% Path accelerations and yaw limits affect guarded_final, NOT main passed.
% There is no derivative-to-equilibrium criterion in this CarSim policy.
if nargin<6
    if isfield(c,'wear'),wear=c.wear;else,wear=zeros(1,4);end
end
c=policy_defaults(c);
t=double(t(:));
assert(~isempty(t)&&isnumeric(X)&&size(X,1)>=34&&size(X,2)==numel(t),'Expected 34-by-Nt X and Nt-by-1 t.');
assert(all(isfinite(t))&&all(diff(t)>0),'Time must be finite and strictly increasing.');
assert(isstruct(O)&&isfield(O,'ay')&&numel(O.ay)==numel(t),'O.ay must contain Nt samples.');
if isfield(O,'Fz_raw'),fz=double(O.Fz_raw);
elseif isfield(O,'Fz'),fz=double(O.Fz);
else,error('O.Fz_raw or O.Fz is required.');end
assert(isequal(size(fz),[4,numel(t)]),'Raw normal loads must be 4-by-Nt [FL FR RL RR].');
trim=trim_values(T);
dt=t-c.t_start;
vy=X(1,:)';r=X(2,:)';
roll=rad2deg(X(4,:))';pitch=rad2deg(X(5,:))';
rollrate=rad2deg(X(11,:))';pitchrate=rad2deg(X(12,:))';
vx=3.6*X(33,:)';ay=O.ay(:);fz=fz';
fl=fz(:,1);fr=fz(:,2);rl=fz(:,3);rr=fz(:,4);
ltr=(fl+rl-fr-rr)./max(fl+fr+rl+rr,eps);
ltrf=(fl-fr)./max(fl+fr,eps);ltrr=(rl-rr)./max(rl+rr,eps);
physical=all(isfinite([vy,r,vx,roll,pitch,rollrate,pitchrate,ay,fz]),2) & ...
    all(fz>0,2)&abs(ltr)<c.LTRLimit&abs(ltrf)<c.LTRLimit& ...
    abs(ltrr)<c.LTRLimit&abs(roll)<c.rollAbsoluteLimit_deg&vx>0;
yawok=abs(r)<=c.yawPolicyFactor*c.mu*9.81/(c.speed_kmh/3.6);
ayok=abs(ay)<=c.ayPolicyFactor*c.mu*9.81;
h=hmc_handling_reference([vy,r],c,wear);inH=logical(h.inside(:));
ht=hmc_handling_reference([trim.vy_mps,trim.r_radps],c,wear);
trim_admissible=logical(ht.inside)||logical(c.allowTrimOutsideH);
s=struct('horizons_s',c.horizons_s,'passed',false(1,numel(c.horizons_s)), ...
    'recoverable',false(1,numel(c.horizons_s)), ...
    'model_safe',false(1,numel(c.horizons_s)), ...
    'guarded_final',false(1,numel(c.horizons_s)), ...
    'min_fz_N',min(fz(:)),'max_LTR',max(abs(ltr)), ...
    'max_LTR_front',max(abs(ltrf)),'max_LTR_rear',max(abs(ltrr)), ...
    'peak_rollrate_degps',max(abs(rollrate)), ...
    'max_abs_roll_deg',max(abs(roll)),'first_H_exit_s',NaN, ...
    'first_physical_failure_s',NaN,'initial_rollrate_degps',rollrate(1), ...
    'last_rollrate_degps',rollrate(end), ...
    'initial_in_reference',logical(inH(1)), ...
    'trim_in_reference',logical(ht.inside),'trim_policy_admissible',trim_admissible, ...
    'time_origin_s',c.t_start, ...
    'definition','CarSim runner output-level finite-time recovery and H-preservation policy');
j=find(~inH,1,'first');if ~isempty(j),s.first_H_exit_s=dt(j);end
j=find(~physical,1,'first');if ~isempty(j),s.first_physical_failure_s=dt(j);end
for k=1:numel(c.horizons_s)
    hz=c.horizons_s(k);
    ids=dt>=-1e-8&dt<=hz+1e-8;
    last=dt>=hz-c.lastWindow_s-1e-8&dt<=hz+1e-8;
    % The source runner checks start/end coverage outside cs_classify. Make
    % the required start explicit here, rather than accepting a partial path.
    if abs(dt(1))>0.03||~any(ids)||max(dt(ids))<hz-0.03||nnz(last)<3
        continue;
    end
    tail=abs(vy(last)-trim.vy_mps)<=c.terminalTol.vy_mps & ...
        abs(r(last)-trim.r_radps)<=c.terminalTol.r_radps & ...
        abs(roll(last)-trim.roll_deg)<=c.terminalTol.roll_deg & ...
        abs(rollrate(last)-trim.rollrate_degps)<=c.terminalTol.rollrate_degps & ...
        abs(pitch(last)-trim.pitch_deg)<=c.terminalTol.pitch_deg & ...
        abs(pitchrate(last)-trim.pitchrate_degps)<=c.terminalTol.pitchrate_degps & ...
        abs(vx(last)-trim.vx_kmh)<=c.terminalTol.vx_kmh;
    recovered=all(tail);
    s.model_safe(k)=all(physical(ids));
    s.recoverable(k)=recovered&&s.model_safe(k);
    s.passed(k)=s.recoverable(k)&&all(inH(ids))&&trim_admissible;
    s.guarded_final(k)=s.passed(k)&&all(yawok(ids))&&all(ayok(ids));
end
end

function c=policy_defaults(c)
if ~isfield(c,'speed_kmh')
    assert(isfield(c,'ux'),'c.speed_kmh or c.ux is required.');c.speed_kmh=3.6*c.ux;
end
assert(isscalar(c.speed_kmh)&&isfinite(c.speed_kmh)&&c.speed_kmh>0,'Positive nominal speed required.');
assert(isfield(c,'mu')&&isscalar(c.mu)&&isfinite(c.mu)&&c.mu>0,'Positive c.mu required.');
defs=struct('horizons_s',[5,10],'lastWindow_s',1,'LTRLimit',1, ...
    'rollAbsoluteLimit_deg',40,'yawPolicyFactor',0.85,'ayPolicyFactor',1, ...
    'allowTrimOutsideH',false,'t_start',0);
if ~isfield(c,'horizons_s')&&isfield(c,'horizons'),c.horizons_s=c.horizons;end
if ~isfield(c,'lastWindow_s')&&isfield(c,'terminal_window'),c.lastWindow_s=c.terminal_window;end
fields=fieldnames(defs);
for i=1:numel(fields),key=fields{i};if ~isfield(c,key),c.(key)=defs.(key);end,end
tol=struct('vy_mps',0.10,'r_radps',0.005,'roll_deg',0.2, ...
    'rollrate_degps',0.5,'pitch_deg',0.2,'pitchrate_degps',0.5,'vx_kmh',0.5);
if ~isfield(c,'terminalTol'),c.terminalTol=tol;
else
    fields=fieldnames(tol);
    for i=1:numel(fields),key=fields{i};if ~isfield(c.terminalTol,key),c.terminalTol.(key)=tol.(key);end,end
end
assert(all(isfinite(c.horizons_s))&&all(c.horizons_s>0)&&all(diff(c.horizons_s)>0),'Horizons must increase.');
assert(isfinite(c.lastWindow_s)&&c.lastWindow_s>0&&c.lastWindow_s<=min(c.horizons_s),'Invalid lastWindow_s.');
end

function trim=trim_values(T)
names={'vy_mps','r_radps','roll_deg','pitch_deg','rollrate_degps','pitchrate_degps','vx_kmh'};
if all(isfield(T,names))
    trim=T;
elseif isfield(T,'x')&&numel(T.x)>=34
    x=T.x(:);trim=struct('vy_mps',x(1),'r_radps',x(2), ...
        'roll_deg',rad2deg(x(4)),'pitch_deg',rad2deg(x(5)), ...
        'rollrate_degps',rad2deg(x(11)),'pitchrate_degps',rad2deg(x(12)), ...
        'vx_kmh',3.6*x(33));
else,error('T must contain 34-state x or all CarSim trim output fields.');end
assert(all(cellfun(@(k)isscalar(trim.(k))&&isfinite(trim.(k)),names)),'Trim outputs must be finite scalars.');
end
