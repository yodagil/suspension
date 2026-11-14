function varargout=HMC_FullCar_Compare(mode,varargin)
%HMC_FULLCAR_COMPARE Independent full-car response model with sampled CDC damping.
%   HMC_FullCar_Compare('import',cfg) imports the user's local PAR/TIR/SLX inputs.
%   R=HMC_FullCar_Compare('run',opts) runs one condition (default: 90/.6/2.5).
%   R=HMC_FullCar_Compare('three',opts) runs the measured 50/70/90 km/h conditions.
%   HMC_FullCar_Compare('selftest',opts) checks the root and 5 ms integration.
% Vehicle/tire/damper source data are imported locally; no source-data
% archive is embedded. Explicit mode rates and control settings are code.
% Body rates p/q are states 11/12, not Euler-angle derivatives.
% Model revision: HMC_TRANSIENT_RESPONSE_V3
if nargin<1 || isempty(mode),mode='run';end
if isstruct(mode),varargin=[{mode},varargin];mode='run';end
switch lower(char(mode))
 case 'import'
  assert(~isempty(varargin),'Supply cfg.echo_par, cfg.tire_zip and cfg.slx.');
  D=hmc_import_local_data(varargin{1});
  if nargout>0,varargout{1}=D;end
 case 'rhs'
  [varargout{1:nargout}]=hmc_vehicle_model('rhs',varargin{:});
 case 'trajectory'
  varargout{1}=delivery_trajectory(varargin{:});
 case {'run','three','selftest'}
  supplied=struct();if ~isempty(varargin),supplied=varargin{1};end
  c=delivery_options(supplied); data=load(c.data_file,'D');
  assert(isfield(data,'D'),'Local data MAT must contain D; run import first.');
  if strcmpi(mode,'three'),conditions=[50 .5 0;70 .6 2.5;90 .6 2.5];
  else,conditions=[c.speed_kmh c.mu c.steer_deg];end
  if strcmpi(mode,'selftest'),c.duration=.005;c.save_results=false;c.make_plot=false;end
  results=cell(size(conditions,1),1);previous=[];
  for ic=1:size(conditions,1)
   cc=c;cc.speed_kmh=conditions(ic,1);cc.mu=conditions(ic,2);cc.steer_deg=conditions(ic,3);
   if isfield(cc,'trim_seed_y'),cc=rmfield(cc,'trim_seed_y');end
   if ~isempty(previous),cc.trim_seed_y=previous;end
   fprintf('%d/%d: %g km/h, mu=%g, steer command=%g deg\n',ic,size(conditions,1),cc.speed_kmh,cc.mu,cc.steer_deg);
   [Q,T]=hmc_vehicle_model('prepare',cc,cc.wear,data.D);
   if (~T.root_found || ~T.usable) && isfield(cc,'trim_seed_y')
    cc=rmfield(cc,'trim_seed_y');[Q,T]=hmc_vehicle_model('prepare',cc,cc.wear,data.D);
   end
   assert(T.root_found&&T.usable,'Independent equilibrium failed; residual %.8g.',T.residual_norm);
   if isfield(T,'trim_unknowns'),previous=T.trim_unknowns;else,previous=[];end
   X=hmc_vehicle_model('lift',[cc.vc0_mps cc.r0_radps],Q,cc,T);
   tr=delivery_trajectory(X,Q,cc,T);R=delivery_result(tr,Q,T,cc);
   reference='';
   if isfield(cc,'reference_files') && numel(cc.reference_files)>=ic,reference=cc.reference_files{ic};
   elseif isfield(cc,'reference_file'),reference=cc.reference_file;end
   if ~isempty(reference)
    R.reference=delivery_reference(reference,Q.g);R.metrics=delivery_metrics(R);
   end
   if cc.save_results || cc.make_plot
    if ~exist(cc.output_dir,'dir'),mkdir(cc.output_dir);end
    base=fullfile(cc.output_dir,sprintf('response_%gkph_%gmu_%gdeg',cc.speed_kmh,cc.mu,cc.steer_deg));
    if cc.save_results,Result=R;save([base '.mat'],'Result','-v7');end
    if cc.make_plot,delivery_plot(R,[base '.png'],cc.figure_visible);end
    fprintf('Saved outputs: %s\n',base);
   end
   fprintf('Completed %d/%d; independent-root residual %.6g\n',ic,size(conditions,1),T.residual_norm);
   results{ic}=R;
  end
  if nargout>0
   if numel(results)==1,varargout{1}=results{1};else,varargout{1}=results;end
  end
 otherwise,error('Unknown mode: %s',char(mode));
end
end

function c=delivery_options(opts)
assert(isstruct(opts)&&isscalar(opts),'opts must be one struct.');
c=hmc_options();
c.angular_model='euler_gyro';
c.geometric_jacking=true;
c.rolling_pitch_moment=true;
c.damper_delay_s=0.002;
c.damper_sample_dt=0.001;
c.vertical_model='mf62';
c.tire_restart='zero';
c.roll_geometry_balance=true;
c.spatial_wheel_velocity=true;
c.compliance_body_frame=true;
c.steering_axis_camber=true;
c.heading_frame_translation=true;
c.source_drive_model=true;
c.full_wheel_orientation=false;
c.coupled_sprung_reference=true;
c.tire_deformation_states=true;
c.tire_longitudinal_deformation=false;
folder=fileparts(mfilename('fullpath'));
c.data_file=fullfile(folder,'hmc_data.mat');c.output_dir=fullfile(folder,'results_response');
c.engine='mcode';c.speed_kmh=90;c.mu=.6;c.steer_deg=2.5;
c.duration=10;c.dt=.001;c.horizons=10;c.vc0_mps=.1;c.r0_radps=.35;
c.trim_step_deg=.125; % Numerical continuation step, not a fitted physical parameter.
c.wear=[0 0 0 0];c.I_mA=[950 950 950 950];c.spring_modes=[2 2 2 2];
c.make_plot=true;c.save_results=true;c.figure_visible='on';c.reference_file='';c.progress_stride=1000;
names=fieldnames(opts);for k=1:numel(names),c.(names{k})=opts.(names{k});end
if isfield(opts,'wheelSteer_deg'),c.steer_deg=opts.wheelSteer_deg;end
if isfield(opts,'runTime_s'),c.duration=opts.runTime_s;end
if isfield(opts,'targetStep_s'),c.dt=opts.targetStep_s;end
assert(isscalar(c.dt)&&isfinite(c.dt)&&c.dt>0,'dt must be positive.');
assert(isscalar(c.duration)&&isfinite(c.duration)&&c.duration>0,'duration must be positive.');
assert(numel(c.wear)==4&&numel(c.I_mA)==4&&numel(c.spring_modes)==4,'Four corner settings are required.');
end

function tr=delivery_trajectory(X,Q,c,T)
% Independent fixed-step RK4. Commands sample at 1 ms, apply after 2 ms,
% and remain held through every RK4 substage. The source trim seeds history.
X=X(:);assert(numel(X)==34&&all(isfinite(X)),'Expected 34 finite states.');
if nargin<4,T=struct();end
duration=10;if isfield(c,'duration'),duration=c.duration;elseif isfield(c,'horizons'),duration=max(c.horizons);end
steps=round(duration/c.dt);assert(abs(steps*c.dt-duration)<1e-9,'Duration must lie on the integration lattice.');
delayed=isfield(Q,'damper_delay_s')&&Q.damper_delay_s>0;
if delayed
 sample_dt=.001;if isfield(Q,'damper_sample_dt'),sample_dt=Q.damper_sample_dt;end
 sample_steps=round(sample_dt/c.dt);lag=round(Q.damper_delay_s/sample_dt);
 assert(sample_steps>=1&&abs(sample_steps*c.dt-sample_dt)<1e-10,'CDC sample time must lie on the integration lattice.');
 assert(lag>=1&&abs(lag*sample_dt-Q.damper_delay_s)<1e-10,'CDC delay must lie on the command lattice.');
 past=X;if isfield(T,'x'),past=T.x(:);end
 [~,seed]=hmc_vehicle_model('rhs',past,Q);fd_hold=seed.damper_force;
 history=repmat(fd_hold,1,lag);pos=1;
end
tr=struct('time_s',(0:steps)*c.dt,'state',zeros(34,steps+1), ...
 'ay_mps2',zeros(1,steps+1),'Fz_raw_N',zeros(4,steps+1), ...
 'spring_force_N',zeros(4,steps+1),'damper_force_N',zeros(4,steps+1), ...
 'jacking_force_N',zeros(4,steps+1));
stride=1000;if isfield(c,'progress_stride'),stride=c.progress_stride;end
for k=0:steps
 if delayed
  if mod(k,sample_steps)==0
   [~,command]=hmc_vehicle_model('rhs',X,Q);
   fd_hold=history(:,pos);history(:,pos)=command.damper_force;pos=mod(pos,lag)+1;
  end
  [a,O]=hmc_vehicle_model('rhs',X,Q,fd_hold);
 else,[a,O]=hmc_vehicle_model('rhs',X,Q);end
 assert(all(isfinite(X))&&all(isfinite(a)),'Nonfinite state/RHS at t=%g s.',k*c.dt);
 tr.state(:,k+1)=X;tr.ay_mps2(k+1)=O.ay;tr.Fz_raw_N(:,k+1)=O.Fz_raw;
 tr.spring_force_N(:,k+1)=O.spring_force;tr.damper_force_N(:,k+1)=O.damper_force;
 if isfield(O,'jacking_force'),tr.jacking_force_N(:,k+1)=O.jacking_force;end
 if k==steps,break;end
 if delayed
  b=hmc_vehicle_model('rhs',X+.5*c.dt*a,Q,fd_hold);
  d=hmc_vehicle_model('rhs',X+.5*c.dt*b,Q,fd_hold);
  e=hmc_vehicle_model('rhs',X+c.dt*d,Q,fd_hold);
 else
  b=hmc_vehicle_model('rhs',X+.5*c.dt*a,Q);
  d=hmc_vehicle_model('rhs',X+.5*c.dt*b,Q);
  e=hmc_vehicle_model('rhs',X+c.dt*d,Q);
 end
 X=X+c.dt/6*(a+2*b+2*d+e);
 if stride>0&&mod(k+1,stride)==0,fprintf('  t = %.3f / %.3f s\n',(k+1)*c.dt,duration);end
end
end

function R=delivery_result(tr,Q,T,c)
R=struct();R.model_revision='HMC_TRANSIENT_RESPONSE_V3';R.time_s=tr.time_s(:);
R.conditions=struct('speed_kmh',c.speed_kmh,'mu',c.mu,'steer_command_deg',c.steer_deg, ...
 'wear_FL_FR_RL_RR',reshape(c.wear,1,4),'initial_vy_mps',c.vc0_mps,'initial_r_radps',c.r0_radps, ...
 'CDC_current_mA',reshape(c.I_mA,1,4),'spring_modes',reshape(c.spring_modes,1,4), ...
 'dt_s',c.dt,'duration_s',tr.time_s(end));
R.model_options=delivery_selected_options(c);
R.normal_state_method='Own numerical equilibrium, not a 12 s trim integration or a CarSim reference snapshot';
R.normal_state_check=struct('root_found',T.root_found,'usable',T.usable, ...
 'residual_norm',T.residual_norm,'locally_stable_test',T.locally_stable_test);
X=tr.state;
R.response=struct('vy_mps',X(1,:)','r_radps',X(2,:)','roll_deg',rad2deg(X(4,:))', ...
 'pitch_deg',rad2deg(X(5,:))','rollrate_degps',rad2deg(X(11,:))', ...
 'pitchrate_degps',rad2deg(X(12,:))','vx_kmh',3.6*X(33,:)','ay_mps2',tr.ay_mps2(:));
R.forces=struct('corner_order',{{'FL','FR','RL','RR'}},'Fz_N',tr.Fz_raw_N', ...
 'spring_N',tr.spring_force_N','damper_N',tr.damper_force_N');
R.body_rate_note='p=state11, q=state12; body angular rates, not differentiated Euler angles';
R.engine='Pure MATLAB fixed-step RK4, sampled CDC commands with held force across RK4 stages';
% Q/T/c and raw input coefficient tables are deliberately not saved.
end

function s=delivery_selected_options(c)
keys={'angular_model','geometric_jacking','rolling_pitch_moment','damper_delay_s','damper_sample_dt','vertical_model','tire_restart','roll_geometry_balance','spatial_wheel_velocity','compliance_body_frame','steering_axis_camber','heading_frame_translation','source_drive_model','full_wheel_orientation','coupled_sprung_reference','tire_deformation_states','tire_longitudinal_deformation'};s=struct();
for k=1:numel(keys),if isfield(c,keys{k}),s.(keys{k})=c.(keys{k});end;end
end

function ref=delivery_reference(file,g)
S=load(file,'CarSimOut_trim');
assert(isfield(S,'CarSimOut_trim'),'Reference MAT must contain CarSimOut_trim.');
v=S.CarSimOut_trim;
if isa(v,'timeseries'),Y=v.Data;t=v.Time;
elseif isstruct(v)&&isfield(v,'Data')&&isfield(v,'Time'),Y=v.Data;t=v.Time;
else,error('CarSimOut_trim must be a timeseries, or a struct containing Data and Time.');end
Y=squeeze(double(Y));t=double(t(:));
if size(Y,1)==72&&size(Y,2)==numel(t),Y=Y';end
assert(size(Y,1)==numel(t)&&size(Y,2)==72,'Reference must contain N by 72 channels.');
assert(all(isfinite(t))&&all(diff(t)>0)&&all(isfinite(Y(:))),'Reference samples must be finite and time must increase.');
ref=struct();ref.time_s=t-t(1);
ref.response=struct('vy_mps',Y(:,4)/3.6,'r_radps',deg2rad(Y(:,14)), ...
 'roll_deg',Y(:,9),'pitch_deg',Y(:,10),'rollrate_degps',Y(:,12), ...
 'pitchrate_degps',Y(:,13),'vx_kmh',Y(:,3),'ay_mps2',Y(:,7)*g);
ref.note='Original reference samples retained; elapsed time starts at first timestamp, no fitted time shift';
end

function metrics=delivery_metrics(R)
t=R.reference.time_s;
assert(t(1)>=R.time_s(1)-1e-8&&t(end)<=R.time_s(end)+1e-8, ...
 'Run duration must cover every reference timestamp; no transient rows are removed.');
names=fieldnames(R.response);metrics=struct();
for k=1:numel(names)
 key=names{k};pred=interp1(R.time_s,R.response.(key),t,'linear');truth=R.reference.response.(key);
 e=pred-truth;[mx,imx]=max(abs(e));
 metrics.(key)=struct('rmse',sqrt(mean(e.^2)),'max_abs_error',mx, ...
  'time_of_max_error_s',t(imx),'reference_final',truth(end),'model_final',pred(end));
end
metrics.sample_count=numel(t);metrics.initial_transient_retained=true;
end

function delivery_plot(R,file,visible)
names={'vy_mps','r_radps','roll_deg','pitch_deg','rollrate_degps','pitchrate_degps','vx_kmh','ay_mps2'};
labels={'Lateral velocity [m/s]','Yaw rate [rad/s]','Roll angle [deg]','Pitch angle [deg]', ...
 'Body roll rate p [deg/s]','Body pitch rate q [deg/s]','Longitudinal speed [km/h]','Lateral acceleration [m/s^2]'};
f=figure('Name','Independent full-car response','Visible',visible,'Color','w','Position',[50 50 1100 1200]);
for k=1:8
 subplot(4,2,k);hold on;
 if isfield(R,'reference'),plot(R.reference.time_s,R.reference.response.(names{k}),'b-','LineWidth',1.1);end
 plot(R.time_s,R.response.(names{k}),'r--','LineWidth',1.1);grid on;
 ylabel(labels{k});xlim([R.time_s(1),R.time_s(end)]);
 if k==1
  title(sprintf('%g km/h, mu=%g, wheel-steer command=%g deg',R.conditions.speed_kmh,R.conditions.mu,R.conditions.steer_command_deg));
  if isfield(R,'reference'),legend('CarSim reference','Independent model');else,legend('Independent model');end
 end
 if k>=7,xlabel('Time [s]');end
end
print(f,file,'-dpng','-r160');
end

function varargout=hmc_vehicle_model(action,varargin)
%HMC_VEHICLE_MODEL Independent HMC-data vehicle equations (not a CarSim solver).
% 34 states, columns independent:
% [vy r z phi theta zu(4) vz p q vu(4) kappa(4) alpha(4) omega(4)
%  ay_diagnostic psi X Y ux speed_integral]. Angles are rad, SI otherwise.
% If tire_deformation_states is enabled, states 21:24 are lateral
% relaxation deflection coordinates [m], rather than effective alpha.
% States 17:20 are longitudinal deflections only when the additional
% tire_longitudinal_deformation flag is true; otherwise they remain kappa.
% Normal-state unknowns remain effective kappa/alpha in either mode.
% z/zu are relative to the supplied design H_WC/spring equilibrium datum.
% With the MF6.2 vertical law, loaded radius and ride height are solved at trim.
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
Q=D.vehicle;Q.model_version='HMC_ROLL_LATERAL_REVISION_20261113';Q.angular_model=field_or(c,'angular_model','euler_gyro');Q.geometric_jacking=field_or(c,'geometric_jacking',false);Q.rolling_pitch_moment=field_or(c,'rolling_pitch_moment',false);Q.damper_delay_s=field_or(c,'damper_delay_s',0);Q.damper_sample_dt=.001;
Q.tables=prepare_splines(Q.tables);
Q.tire_deformation_states=field_or(c,'tire_deformation_states',false);
Q.tire_longitudinal_deformation=field_or(c,'tire_longitudinal_deformation',true);
Q.source_drive_model=field_or(c,'source_drive_model',false);
Q.roll_geometry_balance=field_or(c,'roll_geometry_balance',false);
Q.spatial_wheel_velocity=field_or(c,'spatial_wheel_velocity',false);
Q.compliance_body_frame=field_or(c,'compliance_body_frame',false);
Q.steering_axis_camber=field_or(c,'steering_axis_camber',false);
Q.full_wheel_orientation=field_or(c,'full_wheel_orientation',false);
Q.heading_frame_translation=field_or(c,'heading_frame_translation',false);
Q.coupled_sprung_reference=field_or(c,'coupled_sprung_reference',false);
Q.vertical_model=field_or(c,'vertical_model','linear');
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
 'Optional suspension jacking is a body/unsprung action-reaction pair derived from measured wheel-center and camber path slopes; it is an independent virtual-work approximation.';...
 'Optional roll balance includes unsprung lateral moment, rotated support-force levers and contact-patch/tire overturning moments; the body topology remains an independent approximation.';...
 'Optional spatial wheel velocity includes body roll/pitch rotation and measured suspension-path velocity.';...
 'Optional force-compliance uses body-frame contact loads; steering camber follows the supplied static caster/KPI axis, without a proprietary steering-carrier model.';...
 'Source SPLINE curves use standard natural cubic interpolation with tangent extrapolation; proprietary endpoint convention is not supplied.';...
 'Optional MF6.2 vertical law uses source speed growth, nonlinear deflection and inclination-corrected loaded radius; camber compliance and free-radius growth are omitted from the vertical damping derivative.';...
 'Camber/toe and wheel x/y geometry follow measured jounce tables; compliance uses two fixed-point force updates.';...
 'PI speed error uses m/s; source_drive_model interprets demand as acceleration in g, capped by source maximum power, with conditional antiwindup. The legacy branch treats demand as power fraction.';...
 'MF6.2 forces use load/pressure-dependent structural relaxation lengths. Optional lateral deformation v=sigma_y*tan(alpha_eff) is a first-order carcass approximation, not an MF-Swift belt-state equivalent.';...
 'Yaw inertia includes supplied sprung inertia plus four unsprung point-mass parallel-axis terms.';...
 'Optional coupled_sprung_reference solves lateral, roll and yaw accelerations about sprung CG with unsprung point-mass cross terms. Other equations retain the prior topology; this is not complete multibody angular momentum.'};
end

function [dx,O]=rhs(X,Q,fd_override)
if nargin<3,fd_override=[];end
assert(size(X,1)==34,'Expected34states');n=size(X,2);
[Fz,raw]=loads(X,Q);[delta,gamma,xw,yw,jounce]=geometry(X,Q,Fz);
[kt,at,uw,vw,Re]=slips(X,Q,delta,xw,yw);
Fx=zeros(4,n);Fy=Fx;Mz=Fx;My=Fx;Mx=Fx;sx=Fx;sy=Fx;
for i=1:4
 [Fx(i,:),Fy(i,:),Mz(i,:),d]=state_tire_force(X(16+i,:),X(20+i,:),Fz(i,:),gamma(i,:),uw(i,:),Q,i);
 My(i,:)=d.My;Mx(i,:)=d.Mx;sx(i,:)=max(d.sigma_x,.001);sy(i,:)=max(d.sigma_y,.001);
end
cd=cos(delta);sd=sin(delta);Fxb=Fx.*cd-Fy.*sd;Fyb=Fx.*sd+Fy.*cd;
ay=sum(Fyb,1)/Q.m;
drag=.5*Q.rho_air*Q.aero_area_m2*Q.aero_Cx*X(33,:).*abs(X(33,:));
ax=(sum(Fxb,1)-drag)/Q.m;
[Fs,fd,Fs_pure]=suspension_forces(X,Q,jounce,fd_override);
J=zeros(4,n);
if Q.geometric_jacking
 for i=1:4
  name=Q.corner_names{i};tx=Q.tables.susp_x.(name);ty=Q.tables.susp_lat.(name);tg=Q.tables.camber.(name);
  dxj=lookup_slope(tx,jounce(i,:));dyj=-Q.side(i)*lookup_slope(ty,jounce(i,:));
  dgj=-Q.side(i)*lookup_slope(tg,jounce(i,:));zw=Q.wheel_center_height_m(i)+X(5+i,:);
  % Virtual-work force pair from measured wheel-center path and camber IC.
  J(i,:)=-Fxb(i,:).*dxj-Fyb(i,:).*(dyj+zw.*dgj);
  if Q.roll_geometry_balance
   residual=(Mx(i,:)+zw.*tan(gamma(i,:)).*Fz(i,:)).*cos(delta(i,:))-My(i,:).*sin(delta(i,:));
   J(i,:)=J(i,:)-residual.*dgj;
  end
 end
end
dFs=Fs+J-Q.Fs0;
[phidot,thetadot,psidot]=angle_rates(X,Q);frame_yaw=X(2,:);if Q.heading_frame_translation,frame_yaw=psidot;end
dx=zeros(34,n);dx(1,:)=ay-X(33,:).*frame_yaw;
dx(2,:)=sum(xw.*Fyb-yw.*Fxb+Mz,1)/Q.Iz_total;
dx(3:9,:)=X(10:16,:);
dx(4,:)=phidot;dx(5,:)=thetadot;
dx(10,:)=sum(dFs,1)/Q.ms;
dx(11,:)=(sum(Q.yw.*dFs,1)+Q.ms*Q.h_cg_sprung*ay)/Q.Ixx;
if Q.roll_geometry_balance
 zrel=Q.wheel_center_height_m(:)-Q.h_cg_sprung+jounce-Q.jounce0;
 lever=yw.*cos(X(4,:))-zrel.*sin(X(4,:));
 tire_moment=sum((Mx+(Q.wheel_center_height_m(:)+X(6:9,:)).*tan(gamma).*Fz).*cos(delta)-My.*sin(delta),1);
 lateral_lever=Q.ms*(Q.h_cg_sprung+X(3,:))+sum(Q.mu_u.*(Q.wheel_center_height_m(:)+X(6:9,:)),1);
 dx(11,:)=(sum(lever.*(Fs+J)-Q.yw.*Q.Fs0,1)+lateral_lever.*ay+tire_moment)/Q.Ixx;
end
dx(12,:)=(-sum(Q.xw.*dFs,1)-Q.ms*Q.h_cg_sprung*ax)/Q.Iyy;
if Q.rolling_pitch_moment,dx(12,:)=dx(12,:)+sum(My.*cos(delta),1)/Q.Iyy;end
if strcmp(Q.angular_model,'euler_gyro')
 dx(11,:)=dx(11,:)+(Q.Iyy-Q.Iz)/Q.Ixx.*X(12,:).*X(2,:);
 dx(12,:)=dx(12,:)+(Q.Iz-Q.Ixx)/Q.Iyy.*X(2,:).*X(11,:);
 dx(2,:)=dx(2,:)+(Q.Ixx-Q.Iyy)/Q.Iz_total.*X(11,:).*X(12,:);
end
if Q.coupled_sprung_reference
 % Point-mass unsprung inertia about the sprung-CG reference. The measured
 % CarSim velocity is at that reference, rather than the total vehicle CG.
 zrel=Q.wheel_center_height_m(:)-Q.h_cg_sprung+jounce-Q.jounce0;
 Az=sum(Q.mu_u.*zrel,1);Ax=sum(Q.mu_u.*xw,1);Ay=sum(Q.mu_u.*yw,1);
 Z2=sum(Q.mu_u.*zrel.^2,1);Zx=sum(Q.mu_u.*zrel.*xw,1);Zy=sum(Q.mu_u.*zrel.*yw,1);
 Fy=sum(Fyb,1);fy_rhs=Fy+Ay.*frame_yaw.^2;
 lever=yw.*cos(X(4,:))-zrel.*sin(X(4,:));
 Mr=sum(lever.*(Fs+J)-Q.yw.*Q.Fs0,1)+(Q.h_cg_sprung+X(3,:)).*Fy;
 Mr=Mr+sum((Mx+(Q.wheel_center_height_m(:)+X(6:9,:)).*tan(gamma).*Fz).*cos(delta)-My.*sin(delta),1)-Zy.*frame_yaw.^2;
 Mz_total=sum(xw.*Fyb-yw.*Fxb+Mz-(Q.wheel_center_height_m(:)+X(6:9,:)).*tan(gamma).*Fx,1);
 if strcmp(Q.angular_model,'euler_gyro')
  Mr=Mr+(Q.Iyy-Q.Iz)*X(12,:).*X(2,:);Mz_total=Mz_total+(Q.Ixx-Q.Iyy)*X(11,:).*X(12,:);
 end
 ar=Q.Ixx+Z2-Az.^2/Q.m;coupling=-Zx+Az.*Ax/Q.m;
 iz=Q.Iz+sum(Q.mu_u.*(xw.^2+yw.^2),1)-Ax.^2/Q.m;
 br=Mr+Az.*fy_rhs/Q.m;bz=Mz_total-Ax.*fy_rhs/Q.m;detm=ar.*iz-coupling.^2;
 pd=(br.*iz-coupling.*bz)./detm;rd=(ar.*bz-coupling.*br)./detm;
 ay=(fy_rhs+Az.*pd-Ax.*rd)/Q.m;
 dx(1,:)=ay-X(33,:).*frame_yaw;dx(2,:)=rd;dx(11,:)=pd;
end
dx(13:16,:)=(raw-Q.Fz0-dFs)./Q.mu_u;
speed=max(abs(uw),.5);
if field_or(Q,'tire_deformation_states',false)
 if field_or(Q,'tire_longitudinal_deformation',true)
  dx(17:20,:)=speed.*(kt-X(17:20,:)./sx);
 else
  dx(17:20,:)=speed./sx.*(kt-X(17:20,:));
 end
 dx(21:24,:)=speed.*(tan(at)-X(21:24,:)./sy);
else
 dx(17:20,:)=speed./sx.*(kt-X(17:20,:));dx(21:24,:)=speed./sy.*(at-X(21:24,:));
end
error=Q.ux_target-X(33,:);
demand=Q.speed_KP*error+Q.speed_KP3*error.^3+Q.speed_KI*X(34,:);
if Q.source_drive_model
 % Source speed-controller PI requests acceleration in g. Maximum engine
 % power limits the requested force; it is not a proportional drive gain.
 raw_force=Q.m*Q.g*demand;force_limit=Q.max_power_W./max(abs(X(33,:)),1);
 force_drive=min(max(raw_force,0),force_limit);throttle=force_drive./force_limit;
 Rl=zeros(4,n);for i=1:4,Rl(i,:)=loaded_radius(Fz(i,:),X(24+i,:),Q.pressure(i),Q.tires{i});end
 torque=zeros(4,n);
 front_torque=(1-Q.rear_drive_fraction)*force_drive./(1./Rl(1,:)+1./Rl(2,:));
 rear_torque=Q.rear_drive_fraction*force_drive./(1./Rl(3,:)+1./Rl(4,:));
 torque(1:2,:)=repmat(front_torque,2,1);torque(3:4,:)=repmat(rear_torque,2,1);
 dx(25:28,:)=(torque-Rl.*Fx+My.*cos(gamma)+Mz.*sin(gamma))./Q.Iw;
else
 throttle=min(max(demand,0),1);
 force_drive=throttle*Q.max_power_W./max(abs(X(33,:)),1);
 torque=Q.drive_split.*force_drive.*Re;
 dx(25:28,:)=(torque-Re.*Fx+My)./Q.Iw;
end
dx(29,:)=ay-X(29,:); % diagnostic low-pass only; never drives load transfer
psi=X(30,:);dx(30,:)=psidot;
dx(31,:)=X(33,:).*cos(psi)-X(1,:).*sin(psi);
dx(32,:)=X(33,:).*sin(psi)+X(1,:).*cos(psi);
dx(33,:)=ax+frame_yaw.*X(1,:);
dx(34,:)=error;
if Q.source_drive_model
 windup=(raw_force<=0&error<0)|(raw_force>=force_limit&error>0);
else
 windup=(demand<=0&error<0)|(demand>=1&error>0);
end
dx(34,windup)=0;
if nargout>1
 O=struct('ay',ay,'ax',ax,'Fz_raw',raw,'Fz',Fz,'Fx',Fx,'Fy',Fy,'Mz',Mz,'My',My, ...
 'LTR',(raw(1,:)+raw(3,:)-raw(2,:)-raw(4,:))./sum(raw,1), ...
 'LTRf',(raw(1,:)-raw(2,:))./sum(raw(1:2,:),1), ...
 'LTRr',(raw(3,:)-raw(4,:))./sum(raw(3:4,:),1), ...
 'alpha_target',at,'kappa_target',kt,'uw',uw,'vw',vw,'Re',Re, ...
 'delta',delta,'gamma',gamma,'spring_force',Fs_pure,'damper_force',fd,'jounce',jounce, ...
 'jacking_force',J,'throttle',throttle,'speed_error',error,'tire_load_in_range',all(raw<=Q.Fzmax&raw>0,1));
end
end

function [Fx,Fy,Mz,d]=state_tire_force(k,a,Fz,gamma,uw,Q,i)
if field_or(Q,'tire_deformation_states',false)
 [~,~,~,sd]=hmc_tire_force(0,0,Fz,gamma,uw,Q.pressure(i),Q.tires{i},Q.mu);
 if field_or(Q,'tire_longitudinal_deformation',true),k=k./max(sd.sigma_x,.001);end
 a=atan(a./max(sd.sigma_y,.001));
end
[Fx,Fy,Mz,d]=hmc_tire_force(k,a,Fz,gamma,uw,Q.pressure(i),Q.tires{i},Q.mu);
end

function [Fz,raw]=loads(X,Q)
% Dynamic reactions only: there is no superimposed algebraic ay load transfer.
if strcmp(Q.vertical_model,'mf62')
 raw=zeros(4,size(X,2));
 for i=1:4
  T=Q.tires{i};R0=T.UNLOADED_RADIUS;fn=T.FNOMIN;
  % World wheel-center height projects onto the inclined tire plane. The
  % camber here uses measured kinematics; force-compliance closure is omitted.
  j=Q.jounce0(i)+X(5+i,:)-Q.B(i,:)*X([3 4 5],:);
  camber=-Q.side(i)*(Q.camber_static_rad(i)+lookup_table(Q.tables.camber.(Q.corner_names{i}),j,false))+X(4,:);
  cs=max(cos(camber),.5);
  q1=sqrt((T.VERTICAL_STIFFNESS*R0/fn)^2-4*T.Q_FZ2);
  speed=abs(X(24+i,:))*R0/T.LONGVL;
  rfree=R0*(T.Q_RE0+T.Q_V1*speed.^2);
  rho=max(rfree-(Q.wheel_center_height_m(i)+X(5+i,:))./cs,0)/R0;
  dp=(Q.pressure(i)-T.NOMPRES)/T.NOMPRES;
  raw(i,:)=fn*(1+T.PFZ1*dp).*(1+T.Q_V2*speed).* ...
   (q1*rho+T.Q_FZ2*rho.^2)-Q.ct(i)*X(12+i,:)./cs;
 end
else
 raw=Q.Fz0-Q.kt.*X(6:9,:)-Q.ct.*X(13:16,:);
end
Fz=max(raw,0);
end

function [delta,gamma,xw,yw,jounce]=geometry(X,Q,Fz)
if nargin<3,Fz=loads(X,Q);end
n=size(X,2);jounce=Q.jounce0+X(6:9,:)-Q.B*X([3 4 5],:);
delta=zeros(4,n);gamma=delta;toe_unsteered=delta;xw=Q.xw+zeros(4,n);yw=Q.yw+zeros(4,n);
gear=deg2rad(Q.steer_deg*Q.steering_ratio)*Q.gear_rot_ratio;
for i=1:4
 corner=Q.corner_names{i};j=jounce(i,:);side=Q.side(i);
 tc=Q.tables.camber.(corner);tt=Q.tables.toe.(corner);
 tx=Q.tables.susp_x.(corner);ty=Q.tables.susp_lat.(corner);
 base_toe=Q.toe_static_rad(i)+lookup_table(tt,j,false);
 base_camber=Q.camber_static_rad(i)+lookup_table(tc,j,false);
 if i<=2,ts=Q.tables.steering.(corner);command=lookup_table(ts,gear,true);else,command=0;end
 delta(i,:)=command-side*base_toe;
 toe_unsteered(i,:)=-side*base_toe;
 % MF inclination is positive for the left-wheel inward camber direction.
 gamma(i,:)=-side*base_camber+X(4,:);
 xw(i,:)=Q.xw(i)+lookup_table(tx,j,false);
 yw(i,:)=Q.yw(i)-side*lookup_table(ty,j,false);
end
gamma_unsteered=gamma;
if Q.steering_axis_camber
 for i=1:2,gamma(i,:)=gamma(i,:)+steering_camber(delta(i,:),toe_unsteered(i,:),gamma_unsteered(i,:)-X(4,:),Q,i);end
end
if Q.include_compliance
 base_delta=delta;base_gamma=gamma_unsteered;
 [uball,vball]=wheel_velocities(X,Q,xw,yw,jounce);
 for it=1:2
  for i=1:4
   ub=uball(i,:);vb=vball(i,:);
   uw=cos(delta(i,:)).*ub+sin(delta(i,:)).*vb;
   [Fx,Fy,Mz,tdiag]=state_tire_force(X(16+i,:),X(20+i,:),Fz(i,:),gamma(i,:),uw,Q,i);
   if Q.compliance_body_frame
    cd=cos(delta(i,:));sd=sin(delta(i,:));cp=cos(X(4,:));sp=sin(X(4,:));ct=cos(X(5,:));st=sin(X(5,:));
    fxr=Fx.*cd-Fy.*sd;fyr=Fx.*sd+Fy.*cd;
    Fx=ct.*fxr-st.*Fz(i,:);Fy=sp.*st.*fxr+cp.*fyr+sp.*ct.*Fz(i,:);
    Mz=-cp.*st.*(tdiag.Mx.*cd-tdiag.My.*sd)-sp.*(tdiag.Mx.*sd+tdiag.My.*cd)+cp.*ct.*Mz;
   end
   delta(i,:)=base_delta(i,:)-Q.side(i)*Q.compliance.CT_FX.SI(i)*Fx+Q.compliance.CS_FY.SI(i)*Fy+Q.compliance.CS_MZ.SI(i)*Mz;
   gamma(i,:)=base_gamma(i,:)+Q.side(i)*Q.compliance.CC_FX.SI(i)*Fx+Q.compliance.CI_FY.SI(i)*Fy+Q.compliance.CI_MZ.SI(i)*Mz;
   if Q.steering_axis_camber&&i<=2,gamma(i,:)=gamma(i,:)+steering_camber(delta(i,:),toe_unsteered(i,:),base_gamma(i,:)-X(4,:),Q,i);end
  end
 end
end
if Q.full_wheel_orientation
 for i=1:4
  dive=lookup_table(Q.tables.dive.(Q.corner_names{i}),jounce(i,:),false);
  local_gamma=gamma(i,:)-X(4,:);d=delta(i,:);
  nx=cos(d).*sin(dive).*sin(local_gamma)-sin(d).*cos(local_gamma);
  ny=sin(d).*sin(dive).*sin(local_gamma)+cos(d).*cos(local_gamma);
  nz=cos(dive).*sin(local_gamma);
  cp=cos(X(4,:));sp=sin(X(4,:));ct=cos(X(5,:));st=sin(X(5,:));
  wx=ct.*nx+st.*sp.*ny+st.*cp.*nz;wy=cp.*ny-sp.*nz;wz=-st.*nx+ct.*sp.*ny+ct.*cp.*nz;
  delta(i,:)=atan2(-wx,wy);gamma(i,:)=asin(min(max(wz,-1),1));
 end
end
end

function dg=steering_camber(target,toe,gamma,Q,i)
% Rotate the wheel axle about the supplied caster/KPI axis, solving its
% rotation so wheel heading equals the commanded/compliant heading.
C=Q.caster_static_rad(i);K=Q.kingpin_inclination_rad(i);
axis=[-sin(C);-Q.side(i)*sin(K)*cos(C);cos(K)*cos(C)];
n0=[-sin(toe).*cos(gamma);cos(toe).*cos(gamma);sin(gamma)];
dotp=sum(axis.*n0,1);cr=[axis(2)*n0(3,:)-axis(3)*n0(2,:);axis(3)*n0(1,:)-axis(1)*n0(3,:);axis(1)*n0(2,:)-axis(2)*n0(1,:)];a=(target-toe)/axis(3);
for it=1:4
 co=cos(a);si=sin(a);v=n0.*co+cr.*si+axis.*dotp.*(1-co);d=-n0.*si+cr.*co+axis.*dotp.*si;
 heading=atan2(-v(1,:),v(2,:));der=(-v(2,:).*d(1,:)+v(1,:).*d(2,:))./(v(1,:).^2+v(2,:).^2);
 a=a-atan2(sin(heading-target),cos(heading-target))./der;
end
v=n0.*cos(a)+cr.*sin(a)+axis.*dotp.*(1-cos(a));dg=asin(min(max(v(3,:),-1),1))-gamma;
end

function [k,a,uw,vw,Re]=slips(X,Q,delta,xw,yw)
if nargin<3,[delta,~,xw,yw]=geometry(X,Q);end
[ub,vb]=wheel_velocities(X,Q,xw,yw,Q.jounce0+X(6:9,:)-Q.B*X([3 4 5],:));
uw=cos(delta).*ub+sin(delta).*vb;vw=-sin(delta).*ub+cos(delta).*vb;
a=atan2(vw,uw);Fz=loads(X,Q);Re=zeros(size(uw));
for i=1:4,Re(i,:)=effective_radius(Fz(i,:),X(24+i,:),Q.tires{i});end
k=(Re.*X(25:28,:)-uw)./max(abs(uw),.5);
end

function [Fs,fd,Fs_pure]=suspension_forces(X,Q,jounce,fd_override)
if nargin<4,fd_override=[];end
if nargin<3,jounce=Q.jounce0+X(6:9,:)-Q.B*X([3 4 5],:);end
[phidot,thetadot]=angle_rates(X,Q);
comp_rate=X(13:16,:)-Q.B*[X(10,:);phidot;thetadot];Fs=zeros(size(jounce));fd=Fs;Fs_pure=Fs;
for i=1:4
 corner=Q.corner_names{i};c=Q.spring_compression0(i)+jounce(i,:)-Q.jounce0(i);
 tc=Q.tables.spring_comp.(corner);te=Q.tables.spring_ext.(corner);
 fs=lookup_table(tc,c,false);ext=comp_rate(i,:)<0;
 fs(ext)=lookup_table(te,c(ext),false);
 fs=fs+(Q.spring_mode_rate(i)-30790)*c;
 tj=Q.tables.jounce_stop.(corner);tr=Q.tables.rebound_stop.(corner);
 fs=fs+lookup_table(tj,jounce(i,:),false)+lookup_table(tr,jounce(i,:),false);
 fd(i,:)=interp_linear(Q.vaxis,Q.damper_map(i,:),1000*comp_rate(i,:),true);
 if ~isempty(fd_override),fd(i,:)=fd_override(i,:);end
 Fs_pure(i,:)=fs;Fs(i,:)=fs+fd(i,:);
end

front_rel=X(4,:)-(X(6,:)-X(7,:))/Q.tf;
rear_rel=X(4,:)-(X(8,:)-X(9,:))/Q.tr;
front_rate=X(11,:)-(X(13,:)-X(14,:))/Q.tf;
rear_rate=X(11,:)-(X(15,:)-X(16,:))/Q.tr;
Mf=-Q.Karb_f*front_rel-Q.Carb_f*front_rate;
Mr=-Q.Karb_r*rear_rel-Q.Carb_r*rear_rate;
Fs=Fs+[Mf/Q.tf;-Mf/Q.tf;Mr/Q.tr;-Mr/Q.tr];
end

function [u,v]=wheel_velocities(X,Q,xw,yw,jounce)
if ~Q.spatial_wheel_velocity
 u=X(33,:)-yw.*X(2,:);v=X(1,:)+xw.*X(2,:);return;
end
[pd,td]=angle_rates(X,Q);jr=X(13:16,:)-Q.B*[X(10,:);pd;td];
cp=cos(X(4,:));sp=sin(X(4,:));ct=cos(X(5,:));st=sin(X(5,:));
u=zeros(size(xw));v=u;
for i=1:4
 name=Q.corner_names{i};z=Q.wheel_center_height_m(i)-Q.h_cg_sprung+jounce(i,:)-Q.jounce0(i);
 sx=lookup_slope(Q.tables.susp_x.(name),jounce(i,:));sy=-Q.side(i)*lookup_slope(Q.tables.susp_lat.(name),jounce(i,:));
 % Body-fixed angular velocity cross wheel-center position, plus measured
 % suspension-path velocity, rotated into the yaw-aligned road frame.
 a=X(12,:).*z-X(2,:).*yw(i,:)+sx.*jr(i,:);
 b=X(2,:).*xw(i,:)-X(11,:).*z+sy.*jr(i,:);
 d=X(11,:).*yw(i,:)-X(12,:).*xw(i,:)+jr(i,:);
 u(i,:)=X(33,:)+ct.*a+st.*sp.*b+st.*cp.*d;
 v(i,:)=X(1,:)+cp.*b-sp.*d;
end
end

function X=lift_points(points,Q,c,T)
assert(isfield(T,'x')&&all(isfinite(T.x)),'A valid independent trim is required');
X=repmat(T.x(:),1,size(points,1));X(1,:)=points(:,1)';X(2,:)=points(:,2)';
if strcmp(field_or(c,'tire_restart','retained'),'zero'),X(17:24,:)=0;end
% Body, wheel-spin and suspension states retain the independent trim.
% Native TNO restart force records support zero internal tire-relaxation
% states; tire_restart='zero' enables that separately from the physical ICs.
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
angles=linspace(0,Q.steer_deg,max(2,ceil(abs(Q.steer_deg)/field_or(c,'trim_step_deg',.25))+1));
if isfield(c,'trim_seed_y'),y=c.trim_seed_y(:);angles=Q.steer_deg;end
for angle=angles
 q.steer_deg=angle;
 if angle~=0&&y(2)==0,y(2)=q.ux_target*deg2rad(angle)/q.L;end
 [y,ok,res]=newton_trim(y,q,c);
 if ~ok,T.residual_norm=res;T.failed_angle_deg=angle;T.last_y=y;return;end
end
X=trim_state(y,Q);[d,O]=rhs(X,Q);indices=[1:29 33:34];res=max(abs(d(indices)));
T.x=X;T.trim_unknowns=y;T.root_found=isfinite(res)&&res<field_or(c,'trim_residual_tol',1e-5);
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
if field_or(Q,'tire_deformation_states',false)
 q=Q;q.tire_deformation_states=false;X=trim_state(y,q);
 Fz=loads(X,q);[de,ga,xw,yw,j]=geometry(X,q,Fz);[ub,vb]=wheel_velocities(X,q,xw,yw,j);uw=cos(de).*ub+sin(de).*vb;
 for i=1:4
  [~,~,~,d]=hmc_tire_force(y(9+i,:),y(13+i,:),Fz(i,:),ga(i,:),uw(i,:),Q.pressure(i),Q.tires{i},Q.mu);
  if field_or(Q,'tire_longitudinal_deformation',true),X(16+i,:)=d.sigma_x.*y(9+i,:);else,X(16+i,:)=y(9+i,:);end
  X(20+i,:)=d.sigma_y.*tan(y(13+i,:));
 end
 for it=1:12
  old=X([17:28],:);Fz=loads(X,Q);[de,ga,xw,yw,j]=geometry(X,Q,Fz);[ub,vb]=wheel_velocities(X,Q,xw,yw,j);uw=cos(de).*ub+sin(de).*vb;
  for i=1:4
   [~,~,~,d]=hmc_tire_force(y(9+i,:),y(13+i,:),Fz(i,:),ga(i,:),uw(i,:),Q.pressure(i),Q.tires{i},Q.mu);
   if field_or(Q,'tire_longitudinal_deformation',true),X(16+i,:)=d.sigma_x.*y(9+i,:);else,X(16+i,:)=y(9+i,:);end
  X(20+i,:)=d.sigma_y.*tan(y(13+i,:));
   re=effective_radius(Fz(i,:),X(24+i,:),Q.tires{i});X(24+i,:)=uw(i,:).*(1+y(9+i,:))./re;
  end
  change=X([17:28],:)-old;if max(abs(change(:)))<1e-11,break;end
 end
 return;
end
n=size(y,2);X=zeros(34,n);X(1:9,:)=y(1:9,:);X(17:20,:)=y(10:13,:);X(21:24,:)=y(14:17,:);
X(33,:)=Q.ux_target;X(34,:)=y(18,:);
if ~strcmp(Q.angular_model,'legacy')
 X(11,:)=-X(2,:).*tan(X(5,:))./cos(X(4,:));
 X(12,:)=X(2,:).*tan(X(4,:));
end
Fz=loads(X,Q);[delta,~,xw,yw]=geometry(X,Q,Fz);
[ub,vb]=wheel_velocities(X,Q,xw,yw,Q.jounce0+X(6:9,:)-Q.B*X([3 4 5],:));uw=cos(delta).*ub+sin(delta).*vb;
X(25:28,:)=uw.*(1+X(17:20,:))./Q.Re0;
for j=1:14
 if strcmp(Q.vertical_model,'mf62')
  Fz=loads(X,Q);[delta,~,xw,yw]=geometry(X,Q,Fz);
  [ub,vb]=wheel_velocities(X,Q,xw,yw,Q.jounce0+X(6:9,:)-Q.B*X([3 4 5],:));uw=cos(delta).*ub+sin(delta).*vb;
 end
 previous=X(25:28,:);
 for i=1:4
  re=effective_radius(Fz(i,:),X(24+i,:),Q.tires{i});X(24+i,:)=uw(i,:).*(1+X(16+i,:))./re;
 end
 change=X(25:28,:)-previous;if max(abs(change(:)))<1e-11,break;end
end
if Q.heading_frame_translation
 [~,~,yd]=angle_rates(X,Q);X(29,:)=Q.ux_target*yd;
else
 X(29,:)=Q.ux_target*X(2,:);
end
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

function r=loaded_radius(Fz,omega,pressure,T)
R0=T.UNLOADED_RADIUS;fn=T.FNOMIN;v=abs(omega)*R0/T.LONGVL;
q1=sqrt((T.VERTICAL_STIFFNESS*R0/fn)^2-4*T.Q_FZ2);
normalizer=fn*(1+T.PFZ1*(pressure-T.NOMPRES)/T.NOMPRES).*(1+T.Q_V2*v);
z=max(Fz,0)./normalizer;
rho=R0*2*z./(q1+sqrt(q1^2+4*T.Q_FZ2*z));
r=max(R0*(T.Q_RE0+T.Q_V1*v.^2)-rho,.1*R0);
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

function [pd,td,yd]=angle_rates(X,Q)
p=X(11,:);q=X(12,:);r=X(2,:);ph=X(4,:);th=X(5,:);
if strcmp(Q.angular_model,'legacy'),pd=p;td=q;yd=r;return;end
pd=p+tan(th).*(q.*sin(ph)+r.*cos(ph));
td=q.*cos(ph)-r.*sin(ph);yd=(q.*sin(ph)+r.*cos(ph))./cos(th);
end

function v=table_slope(x,y,q)
x=x(:)';y=y(:)';j=1+sum(q>=x(2:end-1)',1);v=(y(j+1)-y(j))./(x(j+1)-x(j));
end

function groups=prepare_splines(groups)
g=fieldnames(groups);
for i=1:numel(g)
 group=groups.(g{i});if ~isstruct(group),continue;end
 names=fieldnames(group);
 for k=1:numel(names)
  tab=group.(names{k});
  if ~isstruct(tab)||~isfield(tab,'interpolation')||~strcmp(tab.interpolation,'SPLINE'),continue;end
  fields=fieldnames(tab);xkey=fields{find(strncmp(fields,'x_',2),1)};ykey=fields{find(strncmp(fields,'y_',2),1)};
  x=tab.(xkey)(:)';y=tab.(ykey)(:)';h=diff(x);v=diff(y)./h;n=numel(x);M=zeros(1,n);
  if n>2
   a=h(1:end-1);b=2*(h(1:end-1)+h(2:end));c=h(2:end);d=6*diff(v);
   for j=2:n-2,z=a(j)/b(j-1);b(j)=b(j)-z*c(j-1);d(j)=d(j)-z*d(j-1);end
   M(n-1)=d(end)/b(end);for j=n-3:-1:1,M(j+1)=(d(j)-c(j)*M(j+2))/b(j);end
  end
  tab.spline_x=x;tab.spline_y=y;tab.spline_b=v-h.*(2*M(1:end-1)+M(2:end))/6;
  tab.spline_c=M(1:end-1)/2;tab.spline_d=diff(M)./(6*h);
  group.(names{k})=tab;
 end
 groups.(g{i})=group;
end
end
function y=lookup_table(tab,q,clamp)
fields=fieldnames(tab);xkey=fields{find(strncmp(fields,'x_',2),1)};ykey=fields{find(strncmp(fields,'y_',2),1)};
if ~isfield(tab,'spline_b'),y=interp_linear(tab.(xkey),tab.(ykey),q,clamp);return;end
x=tab.spline_x;v=tab.spline_y;shape=size(q);q=q(:)';if clamp,q=min(max(q,x(1)),x(end));end
j=1+sum(q>=x(2:end-1)',1);z=q-x(j);y=v(j)+z.*(tab.spline_b(j)+z.*(tab.spline_c(j)+z.*tab.spline_d(j)));
left=q<x(1);right=q>x(end);sl=tab.spline_b(1);hh=x(end)-x(end-1);sr=tab.spline_b(end)+2*tab.spline_c(end)*hh+3*tab.spline_d(end)*hh^2;
y(left)=v(1)+sl*(q(left)-x(1));y(right)=v(end)+sr*(q(right)-x(end));y=reshape(y,shape);
end
function y=lookup_slope(tab,q)
if ~isfield(tab,'spline_b')
 fields=fieldnames(tab);xkey=fields{find(strncmp(fields,'x_',2),1)};ykey=fields{find(strncmp(fields,'y_',2),1)};y=table_slope(tab.(xkey),tab.(ykey),q);return;
end
x=tab.spline_x;j=1+sum(q>=x(2:end-1)',1);z=q-x(j);y=tab.spline_b(j)+2*tab.spline_c(j).*z+3*tab.spline_d(j).*z.^2;
y(q<x(1))=tab.spline_b(1);hh=x(end)-x(end-1);y(q>x(end))=tab.spline_b(end)+2*tab.spline_c(end)*hh+3*tab.spline_d(end)*hh^2;
end


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
% The supplied FITTYP62 coefficients use slip/inclination angles in radians
% in their MF polynomials. Native force/stiffness replay verifies this;
% physical projections such as cos(alpha) remain trigonometric below.
ast=side*a.*direction;gst=side*g;df=(fz-f0)/f0;dp=(pressure-pn)/pn;
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
% MF6.2 overturning moment, independently checked against TNO VARINF #4.
gx=side*g;
Mx=side*r0*fz*p('LMX').*(p('QSX1')*p('LVMX')-p('QSX2')*gx.*(1+p('PPMX1')*dp)+ ...
 p('QSX3')*fylocal/f0+p('QSX4')*cos(p('QSX5')*atan((p('QSX6')*fz/f0).^2)).* ...
 sin(p('QSX7')*gx+p('QSX8')*atan(p('QSX9')*fylocal/f0))+ ...
 p('QSX10')*atan(p('QSX11')*fz/f0).*gx)+ ...
 side*r0*fylocal*p('LMX').*(p('QSX13')+p('QSX14')*abs(gx));
vnorm=abs(vx/p('LONGVL'));
My=-r0*f0*(p('QSY1')+p('QSY2')*Fx/f0+p('QSY3')*vnorm+p('QSY4')*vnorm.^4+ ...
 (p('QSY5')+p('QSY6')*fz/f0).*gst.^2).*(fz/f0).^p('QSY7').* ...
 (pressure/pn).^p('QSY8')*p('LMY').*sign(vx);
% MF6.2 carcass stiffness varies with load and inflation pressure. With the
% supplied PCFX/PCFY coefficients this reproduces native VARINF19/20; using
% nominal stiffness alone changes the relaxation length during load transfer.
Cfx=p('LONGITUDINAL_STIFFNESS')*(1+p('PCFX1')*df+p('PCFX2')*df.^2).*(1+p('PCFX3')*dp);
Cfy=p('LATERAL_STIFFNESS')*(1+p('PCFY1')*df+p('PCFY2')*df.^2).*(1+p('PCFY3')*dp);
sigma_x=abs(Kx)./safe_den(Cfx);sigma_y=abs(Ky)./safe_den(Cfy);kappa_zero=-shx;
valid=input_valid&isfinite(Fx)&isfinite(Fy)&isfinite(Mx)&isfinite(Mz)&isfinite(My)&Re>0&Cfx>0&Cfy>0;
inactive=fz<=0|mu==0;
Fx(inactive|~input_valid)=0;Fy(inactive|~input_valid)=0;Mz(inactive|~input_valid)=0;My(inactive|~input_valid)=0;
Mx(inactive|~input_valid)=0;
Kx(inactive|~input_valid)=0;Ky(inactive|~input_valid)=0;Dx(inactive|~input_valid)=0;Dy(inactive|~input_valid)=0;kappa_zero(inactive|~input_valid)=0;
sigma_x(inactive|~input_valid)=0;sigma_y(inactive|~input_valid)=0;Re(~input_valid)=0;
valid(inactive&input_valid)=true;
diag=struct('Mx',Mx,'My',My,'Re',Re,'Kx',Kx,'Ky',Ky,'Dx',Dx,'Dy',Dy,'kappa_zero',kappa_zero, ...
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


function c=hmc_options()
%HMC_OPTIONS Explicit operating conditions for the independent HMC model.
root=fileparts(mfilename('fullpath'));
c=struct('speeds',[50 60 70 80 90 100],'mus',[.9 .8 .7 .6 .5], ...
 'steers',0:.5:3,'wear_cases',[0 0 0 0;25 25 0 0;50 50 25 25;0 0 25 25;25 25 50 50], ...
 'I_mA',[950 950 950 950],'spring_modes',[2 2 2 2], ...
 'horizons',[5 10],'dt',.001,'terminal_window',1,'trim_time_s',12, ...
 'trim_residual_tol',1e-6,'trim_step_deg',.25,'native_threads',4, ...
 'engine','native','batch_size',8,'resume',true,'overwrite',false, ...
 'check_cell_centers',true,'allowTrimOutsideH',false, ...
 'LTRLimit',1,'rollAbsoluteLimit_deg',40,'yawPolicyFactor',.85,'ayPolicyFactor',1, ...
 'trace_stride',0,'verbose',true);
c.grid=struct('vc',-3:.1:3,'r',-1.5:.025:1.5);
c.reference_steer_deg=[]; % Empty: mean front wheel angle at independent trim.
c.terminalTol=struct('vy_mps',.10,'r_radps',.005,'roll_deg',.2, ...
 'rollrate_degps',.5,'pitch_deg',.2,'pitchrate_degps',.5,'vx_kmh',.5);
c.data_file=fullfile(root,'data','hmc_data.mat');
c.native_executable=fullfile(root,'native','hmc_native');
if ispc,c.native_executable=[c.native_executable '.exe'];end
c.output_dir=fullfile(root,'results_new_210');
end


function D = hmc_import_local_data(cfg)
%HMC_IMPORT_LOCAL_DATA Import the user's local runtime inputs without CarSim.
% D = hmc_import_local_data(cfg)
% Required: cfg.echo_par, cfg.tire_zip (ZIP or directory), cfg.slx.
% Optional: cfg.output_mat, cfg.steering_ratio, cfg.current_mA.
% The latter two describe the external steering and CDC commands, not fitted
% vehicle properties. Source tables and tire coefficients are read at runtime.
% No source callbacks, SLX code, or executable tire code are evaluated.
% An end PAR / CarSim reference state is deliberately not required.

imp_require_fields(cfg, {'echo_par','tire_zip','slx'}, 'cfg');
echoPath = char(cfg.echo_par); tirePath = char(cfg.tire_zip); slxPath = char(cfg.slx);
text = imp_read_text(echoPath);
scalars = imp_parse_scalars(text);
tables = imp_parse_tables(text);
[tires, tireMeta] = imp_parse_tires(tirePath);
damper = imp_parse_damper(slxPath);
corners = {'FL','FR','RL','RR'};
indices = [1 1; 1 2; 2 1; 2 2];
s = @(key) imp_get(scalars,key,'scalar');
arr = @(key) imp_corner_array(scalars,key,indices);
ms = s('M_SU');
mus = [s('M_US(1)')/2 s('M_US(1)')/2 s('M_US(2)')/2 s('M_US(2)')/2];
L = (s('LX_AXLE(2)')-s('LX_AXLE(1)'))*1e-3;
lf = (s('LX_CG_SU')-s('LX_AXLE(1)'))*1e-3; lr = L-lf;
g = 9.80665*s('R_GRAVITY');
if ~(ms>0 && all(mus>0) && L>0 && lf>0 && lr>0 && g>0)
    error('HMC:ImportVehicle','Invalid mass, axle geometry, or gravity in echo PAR.');
end
V = struct();
V.ms=ms; V.m=ms+sum(mus); V.g=g;
V.Ixx=s('IXX_SU'); V.Iyy=s('IYY_SU'); V.Iz=s('IZZ_SU');
V.L=L; V.lf=lf; V.lr=lr;
V.tf=s('L_TRACK(1)')*1e-3; V.tr=s('L_TRACK(2)')*1e-3;
V.h_cg_sprung=s('H_CG_SU')*1e-3; V.h_cg=V.h_cg_sprung;
V.mu_u=mus; V.Iw=arr('IW')+arr('IT');
V.wheel_center_height_m=arr('H_WC')*1e-3;
V.camber_static_rad=arr('A_CAMBER')*pi/180;
V.toe_static_rad=arr('A_TOE')*pi/180;
V.caster_static_rad=arr('A_CASTER')*pi/180;
V.kingpin_inclination_rad=arr('A_KPI')*pi/180;
V.kingpin_offset_m=arr('L_KPO')*1e-3;
V.Karb_f=s('MX_AUX_COEFFICIENT(1)')*180/pi;
V.Karb_r=s('MX_AUX_COEFFICIENT(2)')*180/pi;
V.Carb_f=s('DAUX(1)')*180/pi; V.Carb_r=s('DAUX(2)')*180/pi;
V.speed_KP=s('SPEED_KP'); V.speed_KI=s('SPEED_KI'); V.speed_KP3=s('SPEED_KP3');
V.max_power_W=s('PMAX_SC')*1000; V.rear_drive_fraction=s('R_REAR_DRIVE_SC');
V.steering_ratio=imp_option(cfg,'steering_ratio',14.805);
V.gear_rot_ratio=s('GEAR_ROT_COEFFICIENT(1)');
V.speed_control_mode=s('OPT_SC'); V.powertrain_mode=s('OPT_PT');
V.damper_motion_ratio=arr('CMP_DAMP_COEFFICIENT').*arr('CMP_DAMP_GAIN');
V.spring_motion_ratio=arr('CMP_SPR_SEAT_COEFFICIENT').*arr('CMP_SPR_SEAT_GAIN');
V.rho_air=s('D_AIR'); V.aero_area_m2=s('AREA_AERO'); V.aero_Cx=s('FX_AERO_SHAPING_CONSTANT');
V.corner_order=corners; V.normal_spring_mode=2;
current=imp_option(cfg,'current_mA',950);
if isscalar(current), current=repmat(current,1,4); end
if numel(current)~=4 || any(~isfinite(current(:)))
    error('HMC:ImportCurrent','cfg.current_mA must be one finite value or four corner values.');
end
V.CDC_current_mA=reshape(current,1,4);
V.raw_scalars=struct(); keys=scalars.keys;
for k=1:numel(keys)
    V.raw_scalars.(imp_matkey(keys{k}))=scalars(keys{k});
end
V.Fz_static_N=[repmat(ms*g*lr/(2*L),1,2),repmat(ms*g*lf/(2*L),1,2)]+mus*g;
specs={ ...
    'spring_comp','FS_COMP_TABLE',1e-3,1,'x_m','y_N'; ...
    'spring_ext','FS_EXT_TABLE',1e-3,1,'x_m','y_N'; ...
    'jounce_stop','F_JNC_STOP_TABLE',1e-3,1,'x_m','y_N'; ...
    'rebound_stop','F_REB_STOP_TABLE',1e-3,1,'x_m','y_N'; ...
    'camber','CAMBER_TABLE',1e-3,pi/180,'x_m','y_rad'; ...
    'toe','TOE_TABLE',1e-3,pi/180,'x_m','y_rad'; ...
    'dive','SUSP_DIVE_TABLE',1e-3,pi/180,'x_m','y_rad'; ...
    'susp_x','SUSP_X_TABLE',1e-3,1e-3,'x_m','y_m'; ...
    'susp_lat','SUSP_LAT_TABLE',1e-3,1e-3,'x_m','y_m'};
V.tables=struct();
for k=1:size(specs,1)
    for j=1:4
        key=sprintf('%s(%d,%d)',specs{k,2},indices(j,1),indices(j,2));
        q=imp_get(tables,key,'table');
        V.tables.(specs{k,1}).(corners{j})=imp_convert_table(q,specs{k,3},specs{k,4},specs{k,5},specs{k,6});
    end
end
for j=1:2
    key=sprintf('STEER_KIN_TABLE(%d,%d)',indices(j,1),indices(j,2));
    V.tables.steering.(corners{j})=imp_convert_table(imp_get(tables,key,'table'),pi/180,pi/180,'x_rad','y_rad');
end
V.tables.steering_wheel=imp_convert_table(imp_get(tables,'STEER_SW_TABLE','table'),1,pi/180,'x_s','y_rad');
V.tables.steering_wheel.note='Inactive internal time-command table, replaced by external steering-wheel import. Not a gear map.';
V.spring_rate_N_m=zeros(1,4); V.spring_intercept_N=zeros(1,4);
for j=1:4
    q=V.tables.spring_comp.(corners{j}); x=q.x_m; y=q.y_N;
    slope=diff(y)./diff(x);
    V.spring_rate_N_m(j)=imp_interp_clamped((x(1:end-1)+x(2:end))/2,slope,0);
    V.spring_intercept_N(j)=imp_interp_clamped(x,y,0);
end
complianceKeys={'CT_FX','CS_FY','CS_MZ','CC_FX','CI_FY','CI_MZ','C_LONG','C_LAT'};
V.compliance=struct();
for j=1:numel(complianceKeys)
    key=complianceKeys{j}; values=arr([key '_COEFFICIENT']);
    if any(strcmp(key,{'C_LONG','C_LAT'}))
        factor=1e-3; units='m/N';
    elseif any(strcmp(key,{'CS_MZ','CI_MZ'}))
        factor=pi/180; units='rad/(N*m)';
    else
        factor=pi/180; units='rad/N';
    end
    V.compliance.(key)=struct('native',values,'SI',values*factor,'units',units);
end
V.assumptions=struct( ...
    'authority','Runtime echo PAR is authoritative for all imported vehicle parameters.', ...
    'interpolation','Source interpolation labels are preserved; the standalone solver supplies its own interpolation implementation.', ...
    'tire_dynamics','TIR numeric coefficient data are imported; proprietary executable tire state evolution is not supplied.', ...
    'reference_trim','No reference CarSim state is imported or used to initialize the standalone model.');
damper.current_nominal_mA=imp_option(cfg,'current_mA',950);
if ~isscalar(damper.current_nominal_mA), damper.current_nominal_mA=V.CDC_current_mA; end
source=struct();
source.version='HMC_LOCAL_MATLAB_IMPORT_1';
source.authority='User-owned runtime echo PAR, active Simulink CDC maps, and supplied MF-Tyre coefficient files';
source.files=struct('echo',imp_file_meta(echoPath),'tire_zip',imp_file_meta(tirePath),'slx',imp_file_meta(slxPath));
source.tires=tireMeta; source.public_data=false;
source.friction_mode=1114; source.corner_order='FL FR RL RR';
source.units='SI unless field name explicitly states mm_s/mA or native';
source.no_stability_result_fitting=true;
D=struct('vehicle',V,'tires',tires,'damper',damper,'source',source);
if isfield(cfg,'output_mat') && ~isempty(cfg.output_mat)
    outputPath=char(cfg.output_mat); parent=fileparts(outputPath);
    if ~isempty(parent) && exist(parent,'dir')~=7, mkdir(parent); end
    save(outputPath,'D','-v7');
end
end

function value=imp_option(cfg,key,default)
if isfield(cfg,key) && ~isempty(cfg.(key)), value=cfg.(key); else, value=default; end
end

function imp_require_fields(s,keys,label)
if ~isstruct(s), error('HMC:ImportConfig','%s must be a structure.',label); end
for j=1:numel(keys)
    if ~isfield(s,keys{j}) || isempty(s.(keys{j}))
        error('HMC:ImportConfig','Missing %s.%s.',label,keys{j});
    end
end
end

function text=imp_read_text(path)
fid=fopen(path,'rb');
if fid<0, error('HMC:ImportFile','Cannot open input file: %s',path); end
cleanup=onCleanup(@()fclose(fid)); %#ok<NASGU>
% TIR/PAR comments may use Latin-1; decoding keeps Octave regexp input valid.
% Numeric keys and array syntax are ASCII in all supported input formats.
text=native2unicode(fread(fid,Inf,'*uint8').','ISO-8859-1');
text=strrep(strrep(text,sprintf('\r\n'),sprintf('\n')),sprintf('\r'),sprintf('\n'));
end

function p=imp_num_pattern()
p='[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eEdD][-+]?\d+)?';
end

function value=imp_number(text)
value=str2double(strrep(strrep(text,'D','E'),'d','e'));
if ~isscalar(value) || ~isfinite(value), error('HMC:ImportNumber','Invalid finite numeric literal.'); end
end

function s=imp_parse_scalars(text)
s=containers.Map('KeyType','char','ValueType','double');
p=['(?m)^([A-Z][A-Z0-9_]*(?:\([^\r\n]*?\))?)\s+(' imp_num_pattern() ')(?=\s|$)'];
tokens=regexp(text,p,'tokens');
for j=1:numel(tokens), s(tokens{j}{1})=imp_number(tokens{j}{2}); end
if s.Count==0, error('HMC:ImportScalars','No numeric runtime parameters found in echo PAR.'); end
end

function tables=imp_parse_tables(text)
tables=containers.Map('KeyType','char','ValueType','any');
p='(?m)^([A-Z][A-Z0-9_]*(?:\([^\r\n]*?\))?)\s+(SPLINE|LINEAR_FLAT|LINEAR|CUBIC)[^\n]*\n([\s\S]*?)^ENDTABLE\s*$';
tokens=regexp(text,p,'tokens');
rowPattern=['^(' imp_num_pattern() ')\s*,\s*(' imp_num_pattern() ')$'];
for j=1:numel(tokens)
    key=tokens{j}{1}; lines=regexp(tokens{j}{3},'\n','split'); values=zeros(0,2);
    for n=1:numel(lines)
        line=strtrim(regexprep(lines{n},'!.*$',''));
        if isempty(line), continue; end
        q=regexp(line,rowPattern,'tokens','once');
        if isempty(q), error('HMC:ImportTable','Invalid numeric row in table %s.',key); end
        values(end+1,:)=[imp_number(q{1}),imp_number(q{2})]; %#ok<AGROW>
    end
    if size(values,1)<2 || any(diff(values(:,1))<=0)
        error('HMC:ImportTable','Table %s needs at least two strictly increasing axis samples.',key);
    end
    tables(key)=struct('interpolation',tokens{j}{2},'native_values',values);
end
end

function value=imp_get(map,key,kind)
if ~isKey(map,key), error('HMC:ImportMissing','Missing required %s: %s',kind,key); end
value=map(key);
end

function values=imp_corner_array(s,key,indices)
values=zeros(1,4);
for j=1:4
    values(j)=imp_get(s,sprintf('%s(%d,%d)',key,indices(j,1),indices(j,2)),'scalar');
end
end

function key=imp_matkey(key)
key=regexprep(key,'[^A-Za-z0-9_]','_');
if numel(key)>63
    h=imp_hash(uint8(key),'SHA-1');
    key=[key(1:54) '_' h(1:8)];
end
end

function out=imp_convert_table(q,xf,yf,xname,yname)
out=struct(); out.(xname)=q.native_values(:,1)*xf; out.(yname)=q.native_values(:,2)*yf;
out.interpolation=q.interpolation; out.native_values=q.native_values;
end

function value=imp_interp_clamped(x,y,q)
if numel(x)==1, value=y(1); else, value=interp1(x,y,min(max(q,x(1)),x(end)),'linear'); end
end

function [tires,meta]=imp_parse_tires(path)
cleanup=[]; %#ok<NASGU>
if exist(path,'dir')==7
    root=path;
else
    root=imp_unpack_archive(path);
    cleanup=onCleanup(@()imp_remove_tree(root));
end
files=imp_find_files(root,'.tir'); tires=struct(); meta=struct();
p=['(?m)^\s*([A-Za-z][A-Za-z0-9_]*)\s*=\s*(' imp_num_pattern() ')(?=\s|$)'];
for j=1:numel(files)
    [~,base,ext]=fileparts(files{j}); base=[base ext]; low=lower(base);
    if ~isempty(strfind(low,'_new_')) %#ok<STREMP>
        key='new';
    else
        if ~isempty(strfind(low,'_fr_')), axle='front'; else, axle='rear'; end %#ok<STREMP>
        if ~isempty(strfind(low,'_25k_')), wear='25'; else, wear='50'; end %#ok<STREMP>
        key=[axle wear];
    end
    if isfield(tires,key), error('HMC:ImportTire','Duplicate tire category %s.',key); end
    text=imp_read_text(files{j}); tokens=regexp(text,p,'tokens'); coeff=struct();
    for n=1:numel(tokens), coeff.(tokens{n}{1})=imp_number(tokens{n}{2}); end
    imp_require_fields(coeff,{'FITTYP','FNOMIN'},base);
    if coeff.FITTYP~=62 || coeff.FNOMIN<=0
        error('HMC:ImportTire','Expected MF-Tyre 6.2 numeric coefficients and positive nominal load: %s',base);
    end
    tires.(key)=coeff;
    info=imp_file_meta(files{j});
    meta.(key)=struct('filename',base,'sha256',info.sha256,'numeric_coefficients',numel(fieldnames(coeff)), ...
        'model','MF-Tyre/MF-Swift 6.2 coefficient data; not executable TNO dynamics');
end
expected=sort({'new';'front25';'front50';'rear25';'rear50'});
if ~isequal(sort(fieldnames(tires)),expected)
    error('HMC:ImportTire','Tire input must contain new, front25, front50, rear25, and rear50 TIR categories.');
end
end

function files=imp_find_files(root,extension)
listing=dir(root); files={};
for j=1:numel(listing)
    if any(strcmp(listing(j).name,{'.','..'})), continue; end
    path=fullfile(root,listing(j).name);
    if listing(j).isdir
        files=[files,imp_find_files(path,extension)]; %#ok<AGROW>
    else
        [~,~,ext]=fileparts(listing(j).name);
        if strcmpi(ext,extension), files{end+1}=path; end %#ok<AGROW>
    end
end
end

function damper=imp_parse_damper(path)
root=imp_unpack_archive(path); cleanup=onCleanup(@()imp_remove_tree(root)); %#ok<NASGU>
xmlPath=fullfile(root,'simulink','stateflow','chart_28.xml');
xml=imp_read_text(xmlPath);
script=regexp(xml,'<P(?=\s|>)[^>]*\sName\s*=\s*["'']script["''][^>]*>([\s\S]*?)</P>','tokens','once');
if isempty(script), error('HMC:ImportDamper','Active CDC script is missing from chart_28.xml.'); end
script=script{1}; script=regexprep(script,'^\s*<!\[CDATA\[([\s\S]*)\]\]>\s*$','$1');
entities={'&lt;','<';'&gt;','>';'&quot;','"';'&apos;','''';'&#13;',sprintf('\r');'&#10;',sprintf('\n');'&amp;','&'};
for j=1:size(entities,1), script=strrep(script,entities{j,1},entities{j,2}); end
current=imp_literal_array(script,'I_axis'); velocity=imp_literal_array(script,'v_axis');
front=imp_literal_array(script,'F_front'); rear=imp_literal_array(script,'F_rear');
current=reshape(current.',1,[]); velocity=reshape(velocity.',1,[]);
if ~isequal(size(front),[numel(current),numel(velocity)]) || ~isequal(size(rear),size(front)) ...
        || numel(current)<2 || numel(velocity)<2 || any(diff(current)<=0) || any(diff(velocity)<=0)
    error('HMC:ImportDamper','CDC maps and increasing current/velocity axes have incompatible dimensions.');
end
damper=struct('current_mA',current,'v_mm_s',velocity,'front_N',front,'rear_N',rear, ...
    'current_nominal_mA',950,'compression_sign',1, ...
    'note','Active SLX integer maps. Positive velocity/force=compression. Clamp current and velocity; bilinear interpolation. No extra MR multiplication in forward block.');
end

function values=imp_literal_array(script,key)
q=regexp(script,['(?<![A-Za-z0-9_])' key '\s*=\s*\[([\s\S]*?)\]\s*;'],'tokens','once');
if isempty(q), error('HMC:ImportDamper','Missing numeric CDC array %s.',key); end
body=regexprep(q{1},'%[^\n]*',''); body=strrep(body,'...','');
rows=regexp(body,';','split'); values=[]; width=0;
for j=1:numel(rows)
    row=strtrim(rows{j}); if isempty(row), continue; end
    num=imp_num_pattern();
    literalRow=['^' num '(?:(?:\s*,\s*|\s+)' num ')*\s*$'];
    if isempty(regexp(row,literalRow,'once'))
        error('HMC:ImportDamper','CDC array %s contains an expression instead of numeric literals.',key);
    end
    numbers=regexp(row,imp_num_pattern(),'match');
    if isempty(numbers), error('HMC:ImportDamper','Empty numeric row in %s.',key); end
    if width==0, width=numel(numbers); end
    if width~=numel(numbers), error('HMC:ImportDamper','Inconsistent row lengths in %s.',key); end
    rowvalues=zeros(1,width);
    for n=1:width, rowvalues(n)=imp_number(numbers{n}); end
    values(end+1,:)=rowvalues; %#ok<AGROW>
end
end

function root=imp_unpack_archive(path)
% Preflight entry names before extraction; source code is never executed.
if exist(path,'file')~=2, error('HMC:ImportArchive','Cannot find archive: %s',path); end
try
    archive=javaObject('java.util.zip.ZipFile',path);
catch err
    error('HMC:ImportArchive','Cannot inspect ZIP/SLX input (MATLAB JVM required): %s',err.message);
end
cleanup=onCleanup(@()archive.close()); %#ok<NASGU>
entries=archive.entries();
while entries.hasMoreElements()
    entry=entries.nextElement(); name=strrep(char(entry.getName()),'\','/');
    segments=regexp(name,'/','split');
    if ~isempty(regexp(name,'^(?:/|[A-Za-z]:)','once')) || any(strcmp(segments,'..'))
        error('HMC:ImportArchive','Unsafe archive entry path.');
    end
end
root=tempname; mkdir(root);
try
    unzip(path,root);
catch err
    imp_remove_tree(root); rethrow(err);
end
end

function imp_remove_tree(path)
if exist(path,'dir')==7, rmdir(path,'s'); end
end

function meta=imp_file_meta(path)
[~,name,ext]=fileparts(path);
if exist(path,'dir')==7
    checksum='directory; individual tire checksums retained';
else
    fid=fopen(path,'rb');
    if fid<0, error('HMC:ImportFile','Cannot open input for checksum: %s',path); end
    cleanup=onCleanup(@()fclose(fid)); %#ok<NASGU>
    checksum=imp_hash(fread(fid,Inf,'*uint8'),'SHA-256');
end
meta=struct('filename',[name ext],'sha256',checksum);
end

function out=imp_hash(bytes,algorithm)
if exist('OCTAVE_VERSION','builtin')~=0 && exist('hash','builtin')~=0
    out=lower(hash(lower(strrep(algorithm,'-','')),char(bytes(:).')));
else
    md=javaMethod('getInstance','java.security.MessageDigest',algorithm);
    md.update(typecast(uint8(bytes(:)),'int8'));
    raw=md.digest();
    out=lower(reshape(dec2hex(mod(double(raw),256),2).',1,[]));
end
end
