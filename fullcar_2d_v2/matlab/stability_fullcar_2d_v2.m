function OUT = stability_fullcar_2d_v2(options)
% V2: full 32-state CDC dynamics retained. This is NOT a no-roll model.
% recoverable still includes ay/contact/LTR/model path constraints.
% Default selected mask excludes the separate absolute yaw policy.
%STABILITY_SHIFT_CDC_2D Conditional 2-D fixed-roll safety/recovery maps for the supplied CDC car.
% Save this entire file as stability_fullcar_2d_v2.m. No external data/toolbox needed.
%   o = stability_fullcar_2d_v2('defaults');
%   R = stability_fullcar_2d_v2(o);
%   test = stability_fullcar_2d_v2('selftest');
%   o.action='trim'; T=stability_fullcar_2d_v2(o);
%   o.action='handling'; H=stability_fullcar_2d_v2(o);
%
% IMPORTANT DEFINITIONS
% 1. The 2-D frozen-load handling diagnostic is NOT the paper's exact method.
% 2. These maps vary initial vc/r only; hidden states follow slice_mode.
%    Default trim retains ALL hidden trim states including phi_eq.
%    flat resets all vertical states and load-transfer acceleration together.
%    legacy_phi0 reproduces the old internally mixed phi0=0 lift.
%    All 32 states, including roll/roll rate, evolve during the trajectory.
% 3. 'recoverable' means safe and numerically settled near a checked trim over
%    a finite terminal window; it is not a certified region of attraction.
% 4. The paper's shift is a diagnostic prediction, never applied to move data.
% 5. CDC maps, MF coefficients, force/load equations and motion ratios are
%    retained. MRf/MRr are MOTION RATIOS, not magnetorheological dampers.
% 6. The original relative mu/0.9 calibration is retained and explicitly logged.
% Requires MATLAB R2016b or newer (implicit expansion). No Optimization Toolbox.

if nargin==0, options=struct(); end
if ischar(options) || (isstring(options) && isscalar(options))
    action=char(options);
    if strcmpi(action,'defaults'), OUT=default_options(); return; end
    options=struct('action',action);
end
assert(isstruct(options)&&isscalar(options),'options must be a scalar struct.');
o=merge_options(default_options(),options);
validate_options(o);
if strcmpi(o.action,'selftest'), OUT=run_selftest(o); return; end
P=ref_default_fullcar_params();
P.MRf=0.8613; P.MRr=0.7970;
P.MR_vec=[P.MRf;P.MRf;P.MRr;P.MRr];
if ~isempty(o.parameter_overrides)
    names=fieldnames(o.parameter_overrides);
    for j=1:numel(names)
        assert(isfield(P,names{j}),'Unknown parameter override: %s',names{j});
        P.(names{j})=o.parameter_overrides.(names{j});
    end
    P.L=P.lf+P.lr;
    P.MR_vec=[P.MRf;P.MRf;P.MRr;P.MRr];
    P.Fz_static=[P.m*P.g*P.lr/(2*P.L)*ones(2,1); P.m*P.g*P.lf/(2*P.L)*ones(2,1)];
end
if ~exist(o.output_dir,'dir'), mkdir(o.output_dir); end
source_path=[mfilename('fullpath') '.m'];
source=fileread(source_path);
provenance=struct('main_source_sha256',file_sha256(source_path));
if strcmp(o.engine,'native')
    bridge=which('cdc2d_native_simulate'); assert(~isempty(bridge),'Native MATLAB bridge is missing');
    provenance.native_executable_sha256=file_sha256(o.native_executable);
    provenance.native_bridge_sha256=file_sha256(bridge);
    exporter=which('export_cdc_native_case'); assert(~isempty(exporter),'Native configuration exporter is missing');
    provenance.native_exporter_sha256=file_sha256(exporter);
end
OUT=struct('files',{{}},'summaries',{{}},'options',o);
for ispeed=1:numel(o.speeds)
 for imu=1:numel(o.mus)
  for isteer=1:numel(o.steers)
    c=o; c.speed_kmh=o.speeds(ispeed); c.ux=c.speed_kmh/3.6;
    c.mu=o.mus(imu); c.delta=deg2rad([o.steers(isteer);o.steers(isteer);0;0]);
    c.steer_deg=o.steers(isteer);
    tag=sprintf('fullcar2d_v2_%gkph_mu%g_delta%+g_%s_%s',c.speed_kmh,c.mu,c.steer_deg,c.initialization,c.slice_mode);
    path=fullfile(o.output_dir,[tag '.mat']);
    signature=struct('options',c,'P',P,'source_text',source,'provenance',provenance);
    if strcmpi(o.action,'run') && exist(path,'file') && o.resume
        saved=load(path,'STABILITY2D');
        assert(isfield(saved,'STABILITY2D')&&isfield(saved.STABILITY2D,'signature'), ...
            'Unrecognized existing file. Use a different output_dir.');
        assert(isequaln(saved.STABILITY2D.signature,signature), ...
            'Cache mismatch: use a new output_dir or remove this generated file: %s',path);
        assert(numel(saved.STABILITY2D.CASE)==size(o.wear_cases,1),'Incomplete scenario cache');
        for iw=1:size(o.wear_cases,1)
            validate_saved_case(saved.STABILITY2D.CASE(iw),c,o.grid,o.wear_cases(iw,:));
        end
        fprintf('Validated cache: %s\n',path);
        OUT.files{end+1}=path; OUT.summaries{end+1}=saved.STABILITY2D.Summary;
        continue;
    end
    fprintf('\n%g km/h | mu=%g | front wheel steer=%+g deg | %s\n', ...
        c.speed_kmh,c.mu,c.steer_deg,c.initialization);
    CASE=cell(size(o.wear_cases,1),1);
    for iw=1:size(o.wear_cases,1)
        checkpoint_path=fullfile(o.output_dir,sprintf('%s_wear%d_checkpoint.mat',tag,iw));
        if strcmpi(o.action,'run') && o.resume && exist(checkpoint_path,'file')
            saved_case=load(checkpoint_path,'CHECKPOINT');
            assert(isfield(saved_case,'CHECKPOINT'),'Unknown wear checkpoint');
            W=saved_case.CHECKPOINT;
            assert(isequaln(W.signature,signature)&&W.case_index==iw,'Wear checkpoint provenance mismatch');
            validate_saved_case(W.case_result,c,o.grid,o.wear_cases(iw,:));
            CASE{iw}=W.case_result;
            fprintf(' wear %d/%d: validated completed checkpoint, selected=%d/%d\n', ...
                iw,size(o.wear_cases,1),W.case_result.stable_count,W.case_result.total_count);
            continue;
        end
        Q=prepare_model(P,c,o.wear_cases(iw,:));
        T=find_trim(Q,c);
        fprintf(' wear %d: trim root=%d, admissible=%d, maxReEig=%.4g, residual=%.3g\n', ...
            iw,T.root_found,T.admissible,T.max_real_eigenvalue,T.residual_norm);
        if strcmpi(o.action,'trim')
            CASE{iw}=struct('wear',Q.wear,'trim',T); continue;
        end
        if strcmpi(o.action,'prepare') || strcmpi(o.action,'probe')
            casecfg=c;
            if strcmp(c.initialization,'trim') && ~T.root_found
                casecfg.initialization='kinematic_zero';
            end
            probe=struct();
            if strcmpi(o.action,'probe')
                switch lower(c.probe_operation)
                 case 'rhs'
                    assert(size(c.probe_states,1)==32,'probe_states needs 32 rows');
                    [probe.dx,probe.observation]=rhs(c.probe_states,Q);
                 case 'lift'
                    probe.initial_states=lift_points(c.probe_points,Q,casecfg,T,c.hidden_variants(1,:));
                 case 'simulate'
                    probe.raw=simulate_points(c.probe_points,Q,casecfg,T);
                 otherwise
                    error('Unknown probe_operation: %s',c.probe_operation);
                end
            end
            CASE{iw}=struct('wear',Q.wear,'trim',T,'model',Q,'cfg',casecfg, ...
                'initialization_used',casecfg.initialization,'probe',probe);
            continue;
        end
        H=handling_map(Q,c,T);
        if strcmpi(o.action,'handling')
            CASE{iw}=struct('wear',Q.wear,'trim',T,'handling',H); continue;
        end
        G=o.grid;
        [V,R]=ndgrid(G.vc,G.r);
        points=[V(:),R(:)];
        casecfg=c;
        if strcmp(c.initialization,'trim') && ~T.root_found
            warning('Trim continuation did not converge. Safety ONLY uses an explicitly recorded kinematic_zero slice; recovery remains unresolved.');
            casecfg.initialization='kinematic_zero';
        end
        raw=simulate_points(points,Q,casecfg,T);
        C=struct('wear',Q.wear,'grid',G,'trim',T,'handling',H,'raw',raw);
        C.initialization_requested=c.initialization; C.initialization_used=casecfg.initialization;
        C.slice_mode=c.slice_mode;
        C.fixed_initial_roll_rad=initial_phi(casecfg,T);
        C.masks=make_masks(raw,numel(G.vc),numel(G.r));
        C.stable_mask=C.masks.recoverable(:,:,end);
        if c.use_yaw_policy_for_region
            C.stable_mask=C.stable_mask & C.masks.yaw_policy(:,:,end);
        end
        C.reason_mask=reshape(raw.reason(:,end),size(V));
        C.stable_points=points(C.stable_mask(:),:);
        C.nonselected_points=points(~C.stable_mask(:),:); % complement includes unresolved!
        C.complement_note='unsafe OR safe-unresolved; not all are proven unstable';
        C.total_count=numel(V); C.stable_count=nnz(C.stable_mask);
        C.stable_fraction=C.stable_count/C.total_count;
        C.region=sampled_cells(G,C.stable_mask,Q,casecfg,T,iw);
        C.summary=summarize_case(C,points,c);
        CASE{iw}=C;
        fprintf('  safe=%d, guarded-safe=%d, recoverable=%d, selected=%d / %d\n', ...
            nnz(raw.safe(:,end)),nnz(raw.safe(:,end)&raw.yaw_ok(:,end)), ...
            nnz(raw.recovered(:,end)),C.stable_count,C.total_count);
        fprintf('  selected centroid [v r] = [%+.5g %+.5g], fixed initial phi=%+.5g rad\n', ...
            C.summary.selected_centroid,C.fixed_initial_roll_rad);
        if any(C.summary.touches_grid)
            warning('Selected mask touches a grid face: this map can be truncated. Expand the grid.');
        end
        % Save each completed wear case; no expensive case is lost to plotting.
        checkpoint=struct('case_result',C,'cfg',c,'signature',signature,'case_index',iw);
        validate_saved_case(C,c,G,Q.wear);
        atomic_save(checkpoint_path,'CHECKPOINT',checkpoint);
    end
    S=struct();
    S.meta=struct('definition','conditional finite-horizon sampled safety and recovery', ...
        'version','cdc-fullcar-2d-v2','speed_kmh',c.speed_kmh,'mu_road',c.mu, ...
        'wheel_steer_deg',c.delta'*180/pi,'damper_type','CDC', ...
        'motion_ratios',[P.MRf P.MRr],'horizons_s',c.horizons, ...
        'initial_axes',{{'vc_m_s','yaw_rate_rad_s'}},'requested_phi0_parameter_rad',c.fixed_phi0, ...
        'slice_mode',c.slice_mode,'actual_phi0_location','CASE(i).fixed_initial_roll_rad', ...
        'roll_dynamics_retained',true,'is_projection_of_3d',false, ...
        'engine',c.engine,'provenance',provenance, ...
        'dt_s',c.Ts/c.Nsub,'initialization',c.initialization, ...
        'certified_invariant',false,'asymptotic_ROA_proven',false, ...
        'friction_note','Original MF amplitudes scaled by mu/0.9; not absolute peak-mu normalization', ...
        'reason_semantics','Through first physical/model-domain violation; yaw recorded separately', ...
        'hidden_variants',c.hidden_variants, ...
        'created',datestr(now,30));
    S.cfg=c; S.parameters=P; S.signature=signature;
    S.grid_coarse=o.grid; S.wear_cases=o.wear_cases; S.CASE=vertcat(CASE{:});
    if strcmpi(o.action,'run')
        S.Summary=vertcat(S.CASE.summary);
        atomic_save(path,'STABILITY2D',S);
        write_summary_csv(fullfile(o.output_dir,[tag '_summary.csv']),S);
        OUT.files{end+1}=path; OUT.summaries{end+1}=S.Summary;
    else
        OUT.results{ispeed,imu,isteer}=S;
    end
  end
 end
end
OUT.completed=true;
end

function o=default_options()
o=struct();
o.action='run';
o.speeds=80; o.mus=0.9; o.steers=[-2 0 2];
o.wear_cases=[0 0 0 0]; % pilot; full five-case list in run_shift_study.m
o.grid=struct('vc',-6:0.25:6,'r',-1.2:0.03:1.2);
o.fixed_phi0=0; % Used only for flat/legacy_phi0 slices.
o.slice_mode='trim'; % trim, flat, or legacy_phi0 (old mixed initialization)
o.handling_grid=struct('vc',linspace(-6,6,121),'r',linspace(-1.5,1.5,121));
o.Ts=0.01; o.Nsub=20; o.horizons=[5 10];
o.engine='mcode'; o.native_executable=''; o.native_threads=4;
o.probe_points=zeros(0,2); o.probe_states=zeros(32,0); o.probe_operation='rhs';
o.terminal_window=1; o.derivative_time_scale=1;
o.batch_size=256;
o.I_mA=[950 950 950 950]; o.k_spring_N_m=[30790 30790 30790 30790];
o.initialization='trim'; % 'trim', 'kinematic_zero', or 'legacy'
o.use_yaw_policy_for_region=false; % keep absolute yaw policy as separate saved mask
o.yaw_factor=0.85; o.ay_factor=1.0; o.LTR_limit=1.0;
o.reject_force_clipping=true;
o.guard=struct('vc',30,'r',2,'phi',deg2rad(40),'kappa',0.5,'alpha',1.2,'omega',300);
% Full 29-state terminal tolerances, engineering choices, NOT paper parameters.
o.state_tolerance=[0.10;0.005;0.002;deg2rad(0.2);deg2rad(0.2); ...
    0.002*ones(4,1);0.01;0.01;0.01;0.01*ones(4,1); ...
    0.001*ones(4,1);0.002*ones(4,1);0.2*ones(4,1);0.05];
% Each row: delta roll-rate [rad/s], delta alpha_front [rad],
% delta alpha_rear [rad], delta load-transfer acceleration [m/s^2].
% Maps are intersected across these variants; default is one conditional slice.
o.hidden_variants=[0 0 0 0];
o.trim_step_deg=0.25; o.trim_residual_tol=1e-6; o.eigenvalue_tolerance=1e-5;
o.check_cell_centers=true; o.random_points_per_cell=0; o.random_seed=210;
o.output_dir=fullfile(fileparts(mfilename('fullpath')),'results_fullcar2d_v2');
o.resume=true; o.verbose=true; o.parameter_overrides=struct();
end

function o=merge_options(o,u)
f=fieldnames(u);
for i=1:numel(f)
    assert(isfield(o,f{i}),'Unknown option: %s',f{i});
    if isstruct(o.(f{i}))&&isstruct(u.(f{i}))&&~strcmp(f{i},'parameter_overrides')
        o.(f{i})=merge_options(o.(f{i}),u.(f{i}));
    else
        o.(f{i})=u.(f{i});
    end
end
end

function validate_options(o)
assert(any(strcmpi(o.action,{'run','trim','handling','selftest','prepare','probe'})),'Unknown action');
assert(all(isfinite(o.speeds))&&all(o.speeds>0)&&isvector(o.speeds),'Invalid speeds');
assert(all(isfinite(o.mus))&&all(o.mus>0)&&isvector(o.mus),'Invalid mus');
assert(all(isfinite(o.steers))&&isvector(o.steers),'Invalid steering');
assert(size(o.wear_cases,2)==4&&all(ismember(o.wear_cases(:),[0 25 50])),'Wear must be 0/25/50, four columns');
assert(o.Nsub>=1&&o.Nsub==round(o.Nsub)&&o.Ts>0,'Invalid integration step');
assert(any(strcmp(o.engine,{'mcode','native'})),'engine must be mcode or native');
assert(isscalar(o.native_threads)&&o.native_threads>=1&&o.native_threads==round(o.native_threads),'Invalid native_threads');
if strcmp(o.engine,'native'), assert(exist(o.native_executable,'file')==2,'Native executable is missing'); end
assert(all(diff(o.horizons)>0)&&all(o.horizons>=o.terminal_window),'Horizons must increase and cover terminal_window');
assert(all(abs(o.horizons/(o.Ts/o.Nsub)-round(o.horizons/(o.Ts/o.Nsub)))<1e-7), ...
    'Each horizon must be an integer multiple of dt');
assert(numel(o.I_mA)==4&&all(isfinite(o.I_mA))&&all(o.I_mA>=100)&&all(o.I_mA<=1800),'CDC currents must lie within 100..1800 mA');
assert(numel(o.k_spring_N_m)==4&&all(isfinite(o.k_spring_N_m))&&all(o.k_spring_N_m>0),'Invalid spring rates');
assert(o.batch_size>=1&&o.batch_size==round(o.batch_size),'Invalid batch_size');
assert(numel(o.state_tolerance)==29&&all(o.state_tolerance>0),'Need 29 positive terminal tolerances');
assert(o.terminal_window>0&&o.derivative_time_scale>0,'Invalid terminal settings');
assert(size(o.hidden_variants,2)==4&&all(isfinite(o.hidden_variants(:))),'Invalid hidden_variants');
assert(any(strcmp(o.initialization,{'trim','kinematic_zero','legacy'})),'Unknown initialization');
assert(isscalar(o.fixed_phi0)&&isfinite(o.fixed_phi0),'fixed_phi0 must be a finite scalar');
assert(any(strcmp(o.slice_mode,{'trim','flat','legacy_phi0'})),'Unknown slice_mode');
% The optional native bridge uses the validated FULLCAR2D_V2 slice protocol.
% Its distinct configuration and binary schemas reject the older phi0 backend.
for f={'vc','r'}
    v=o.grid.(f{1}); assert(isvector(v)&&numel(v)>=2&&all(isfinite(v))&&all(diff(v)>0),'Grid axes must increase');
end
assert(o.random_points_per_cell>=0&&mod(o.random_points_per_cell,1)==0,'Invalid random cell count');
end

function Q=prepare_model(P,c,wear)
P.mu_scale_x=c.mu/0.9; P.mu_scale_y=c.mu/0.9;
Q=P; Q.ux=c.ux; Q.delta=c.delta(:); Q.wear=wear(:)';
Q.Icmd=c.I_mA(:); Q.kSpring=c.k_spring_N_m(:);
Q.kw=ref_spring_to_wheel_rate(Q.kSpring,P);
Q.xw=[P.lf;P.lf;-P.lr;-P.lr]; Q.yw=[P.tf/2;-P.tf/2;P.tr/2;-P.tr/2];
Q.B=[ones(4,1),Q.yw,Q.xw];
Q.vaxis=[-1048 -524 -393 -262 -131 -79 -52 -26 0 26 52 79 131 262 393 524 1048];
Q.damper_map=zeros(4,numel(Q.vaxis));
for j=1:numel(Q.vaxis)
    Q.damper_map(:,j)=ref_lookup_all_damper_forces(Q.vaxis(j)*ones(4,1),Q.Icmd);
end
Q.cdelta=cos(Q.delta); Q.sdelta=sin(Q.delta);
Kf=(Q.kw(1)+Q.kw(2))*Q.tf^2/4+Q.Karb_f;
Kr=(Q.kw(3)+Q.kw(4))*Q.tr^2/4+Q.Karb_r;
if Kf+Kr<1, lam=0.5; else, lam=Kf/(Kf+Kr); end
hm=max(0,Q.m*Q.h_cg-Q.ms*Q.h_roll);
Q.load_gain=[-lam/Q.tf;lam/Q.tf;-(1-lam)/Q.tr;(1-lam)/Q.tr]*hm;
% Seven static vertical equations, using the original roll-gravity sign.
A=zeros(7); b=zeros(7,1); b(2)=-Q.ms*Q.h_roll;
for j=1:7
    e=zeros(7,1); e(j)=1; A(:,j)=vertical_equations(e,Q);
end
Q.vertical_gain=A\b;
% Lateral MF coefficients by wear (the original fitted sets).
M=[1.5662 1.0278 -0.17125 -1.1357 -1.0544 -25.85 2.1377 2 8.675e-4 7.046e-5 -0.007239 -7.442e-5; ...
   1.5457 1.0286 -0.16688 -1.439 -1.6063 -26 1.9164 2 1.82e-3 -7.193e-4 1.802e-3 -1.1497e-2; ...
   1.5057 1.0067 -0.2019 -2 -2 -27.27 1.5963 2 2.3735e-3 2.38e-4 0.01487 -0.011025];
wi=1+(wear(:)>=12.5)+(wear(:)>=37.5); Q.MFy=M(wi,:);
Q.wear_x=1-0.05*(wi-1);
end

function e=vertical_equations(z,Q)
Fs=-Q.kw.*(Q.B*z(1:3)-z(4:7));
Fs=Fs+[-Q.Karb_f;Q.Karb_f;0;0]*z(2)/Q.tf+ ...
    [0;0;-Q.Karb_r;Q.Karb_r]*z(2)/Q.tr;
e=[sum(Fs);Q.yw'*Fs-Q.ms*Q.g*Q.h_roll*z(2);Q.xw'*Fs;-Fs-Q.kt.*z(4:7)];
end

function [dx,O]=rhs(X,Q)
% Vectorized, column-independent implementation of the supplied 32-state RHS.
n=size(X,2); vc=X(1,:); r=X(2,:);
zbody=Q.B*X([3 4 5],:); vbody=Q.B*X([10 11 12],:);
Fs=-Q.kw.*(zbody-X(6:9,:));
vrel=vbody-X(13:16,:);
Fs=Fs+Q.MR_vec.*damper_force(-1000*Q.MR_vec.*vrel,Q);
Mf=-Q.Karb_f*X(4,:)-Q.Carb_f*X(11,:);
Mr=-Q.Karb_r*X(4,:)-Q.Carb_r*X(11,:);
Fs=Fs+[Mf/Q.tf;-Mf/Q.tf;Mr/Q.tr;-Mr/Q.tr];
[Fz,Fze,Fzr,Fzer]=normal_loads(X,Q);
[kt,at,uw]=slips(X,Q);
[Fxt,Fyt]=tire_force(X(17:20,:),X(21:24,:),Fz,Q);
Fxb=Fxt.*Q.cdelta-Fyt.*Q.sdelta;
Fyb=Fxt.*Q.sdelta+Fyt.*Q.cdelta;
ay=sum(Fyb,1)/Q.m;
Mz=sum(Q.xw.*Fyb-Q.yw.*Fxb,1);
dx=zeros(32,n);
dx(1,:)=ay-Q.ux*r; dx(2,:)=Mz/Q.Iz;
dx(3:9,:)=X(10:16,:);
dx(10,:)=sum(Fs,1)/Q.ms;
dx(11,:)=(sum(Q.yw.*Fs,1)+Q.ms*Q.h_roll*ay-Q.ms*Q.g*Q.h_roll*X(4,:))/Q.Ixx;
dx(12,:)=sum(Q.xw.*Fs,1)/Q.Iyy;
dx(13:16,:)=(-Fs+Fze-Q.Fz_static)./Q.mu_u;
vr=max(abs(uw),0.5);
dx(17:20,:)=min(vr./Q.sigma_kappa,500).*(kt-X(17:20,:));
dx(21:24,:)=min(vr./Q.sigma_alpha,500).*(at-X(21:24,:));
dx(25:28,:)=-Q.Rw*Fxt/Q.Iw;
dx(29,:)=(ay-X(29,:))/Q.tau_load;
dx(30,:)=r;
dx(31,:)=Q.ux*cos(X(30,:))-vc.*sin(X(30,:));
dx(32,:)=Q.ux*sin(X(30,:))+vc.*cos(X(30,:));
if nargout>1
    left=Fzr(1,:)+Fzr(3,:); right=Fzr(2,:)+Fzr(4,:);
    den=left+right;
    ltr=(left-right)./den;
    ltr(abs(den)<1e-9)=Inf;
    ltr(abs(den)<1e-9 & abs(left-right)<1e-9)=0;
    O=struct('ay',ay,'Fz_raw',Fzr,'Fz_elastic_raw',Fzer,'LTR',ltr, ...
        'LTRf',(Fzr(1,:)-Fzr(2,:))./sum(Fzr(1:2,:),1), ...
        'LTRr',(Fzr(3,:)-Fzr(4,:))./sum(Fzr(3:4,:),1), ...
        'alpha_target',at,'kappa_target',kt,'uw',uw);
end
end

function F=damper_force(v,Q)
F=zeros(size(v));
for i=1:4
    q=min(max(v(i,:),Q.vaxis(1)),Q.vaxis(end));
    j=1+sum(q>=Q.vaxis(2:end-1)',1);
    a=(q-Q.vaxis(j))./(Q.vaxis(j+1)-Q.vaxis(j));
    F(i,:)=Q.damper_map(i,j)+a.*(Q.damper_map(i,j+1)-Q.damper_map(i,j));
end
end

function [Fz,Fze,raw,elastic]=normal_loads(X,Q)
elastic=Q.Fz_static-Q.kt.*X(6:9,:);
raw=elastic+Q.load_gain.*X(29,:);
Fz=raw; Fze=elastic;
for i=1:4
    Fz(i,~isfinite(Fz(i,:)))=Q.Fz_static(i);
    Fze(i,~isfinite(Fze(i,:)))=Q.Fz_static(i);
end
Fz=min(max(Fz,Q.Fz_force_min),Q.Fz_force_max);
Fze=min(max(Fze,Q.Fz_force_min),Q.Fz_force_max);
end

function [k,a,uw]=slips(X,Q)
ub=Q.ux-Q.yw.*X(2,:); vb=X(1,:)+Q.xw.*X(2,:);
uw=Q.cdelta.*ub+Q.sdelta.*vb; vw=-Q.sdelta.*ub+Q.cdelta.*vb;
us=uw; us(us>=0&us<0.1)=0.1; us(us<0&us>-0.1)=-0.1;
a=atan2(vw,us);
k=(Q.Rw*X(25:28,:)-uw)./max(abs(uw),0.5);
k=min(max(k,-0.5),0.5);
end

function [Fx,Fy]=tire_force(k,a,Fz,Q)
dfz=(Fz-Q.Fz0_vec)./Q.Fz0_vec;
Dx=(1.145*Q.wear_x-0.2106*dfz).*Fz*Q.mu_scale_x;
Kx=Fz.*(24.125*Q.wear_x+15*Q.wear_x.*dfz).*exp(-0.4832*dfz);
Bx=Kx./(1.647*Dx); Bx(abs(1.647*Dx)<1e-9)=0;
xx=Bx.*(k-0.001635+0.0025297*dfz);
Ex=min(-0.02192+0.23613*dfz,1);
Fx=Dx.*sin(1.647*atan(xx-Ex.*(xx-atan(xx))));
m=Q.MFy;
Dy=(m(:,2)+m(:,3).*dfz).*Fz*Q.mu_scale_y;
Ky=m(:,6).*Q.Fz0_vec.*sin(m(:,8).*atan(Fz./(m(:,7).*Q.Fz0_vec)));
By=Ky./(m(:,1).*Dy); By(abs(m(:,1).*Dy)<1e-9)=0;
xy=By.*(a+m(:,9)+m(:,10).*dfz);
Ey=min(m(:,4)+m(:,5).*dfz,1);
Fy=Dy.*sin(m(:,1).*atan(xy-Ey.*(xy-atan(xy))))+Fz.*(m(:,11)+m(:,12).*dfz);
cap=max(max(abs(1.145*Q.wear_x-0.2106*dfz)*Q.mu_scale_x,0.05).*Fz,1);
Fy=Fy.*sqrt(max(0,1-min(abs(Fx)./cap,1).^2));
Fx(Fz<=Q.Fz_force_min)=0; Fy(Fz<=Q.Fz_force_min)=0;
end

function k=free_rolling_kappa(Fz,Q)
% Exact zero-Fx root of the retained pure-longitudinal MF (SVx=0).
dfz=(Fz-Q.Fz0_vec)./Q.Fz0_vec;
k=0.001635-0.0025297*dfz;
end

function X=kinematic_tires(X,Q,zero_force)
[~,a,uw]=slips(X,Q);
if zero_force
    Fz=normal_loads(X,Q); k=free_rolling_kappa(Fz,Q);
else
    k=zeros(size(a));
end
X(17:20,:)=k; X(21:24,:)=a;
X(25:28,:)=(uw+k.*max(abs(uw),0.5))/Q.Rw;
end

function X=trim_state(y,Q)
X=zeros(32,1); X(1:2)=y;
ay=Q.ux*y(2); z=Q.vertical_gain*ay;
X([3 4 5 6 7 8 9])=z; X(29)=ay;
X=kinematic_tires(X,Q,true);
end

function T=find_trim(Q,c)
T=struct('x',nan(32,1),'root_found',false,'admissible',false, ...
    'yaw_admissible',false,'locally_stable_test',false,'usable',false, ...
    'residual_norm',Inf,'max_real_eigenvalue',NaN,'eigenvalues',[], ...
    'continuation_completed',false,'note','One zero-steer-connected numerical branch; uniqueness not proved');
steps=max(1,ceil(abs(c.steer_deg)/c.trim_step_deg));
angles=linspace(0,c.steer_deg,steps+1); y=[0;0]; allok=true;
for angle=angles
    q=Q; q.delta=deg2rad([angle;angle;0;0]); q.cdelta=cos(q.delta); q.sdelta=sin(q.delta);
    seeds=[y,[q.ux*q.lr*deg2rad(angle)/q.L;q.ux*deg2rad(angle)/q.L],[0;0]];
    ok=false;
    for j=1:size(seeds,2)
        [z,good]=newton_trim(seeds(:,j),q);
        if good, y=z; ok=true; break; end
    end
    if ~ok, allok=false; break; end
end
if ~allok, return; end
X=trim_state(y,Q); f=rhs(X,Q);
T.x=X; T.residual_norm=max(abs(f(1:29))./c.state_tolerance(:));
T.root_found=isfinite(T.residual_norm)&&T.residual_norm<c.trim_residual_tol;
T.continuation_completed=true;
if ~T.root_found, return; end
[~,O]=rhs(X,Q); [bits,~]=constraint_codes(X,O,Q,c);
T.admissible=bitand(bits,uint16(254))==0;
T.yaw_admissible=bitand(bits,uint16(1))==0;
% Global yaw angle and XY positions are excluded from the internal Jacobian.
s=[1;0.1;0.01;0.1;0.1;0.01*ones(4,1);0.1*ones(7,1); ...
    0.1*ones(8,1);10*ones(4,1);1];
h=1e-5*s;
XP=repmat(X,1,29); XM=XP;
for j=1:29, XP(j,j)=XP(j,j)+h(j); XM(j,j)=XM(j,j)-h(j); end
FP=rhs(XP,Q); FM=rhs(XM,Q);
A=(FP(1:29,:)-FM(1:29,:))./(2*h');
T.jacobian=A; T.eigenvalues=eig(A); T.max_real_eigenvalue=max(real(T.eigenvalues));
T.locally_stable_test=T.max_real_eigenvalue < -c.eigenvalue_tolerance;
T.usable=T.root_found&&T.admissible&&T.locally_stable_test;
end

function [y,ok]=newton_trim(y,Q)
ok=false;
for iter=1:60
    f=planar_trim_residual(y,Q);
    if ~all(isfinite(f)), return; end
    if norm(f,Inf)<1e-10, ok=true; return; end
    h=[1e-5;1e-6]; J=zeros(2);
    for j=1:2
        yp=y; ym=y; yp(j)=yp(j)+h(j); ym(j)=ym(j)-h(j);
        J(:,j)=(planar_trim_residual(yp,Q)-planar_trim_residual(ym,Q))/(2*h(j));
    end
    if rcond(J)<1e-12, return; end
    step=-J\f; improved=false;
    for k=0:18
        z=y+step*0.5^k;
        if abs(z(1))>30||abs(z(2))>2, continue; end
        fz=planar_trim_residual(z,Q);
        if all(isfinite(fz))&&norm(fz,2)<norm(f,2)
            y=z; improved=true; break;
        end
    end
    if ~improved, return; end
end
end

function f=planar_trim_residual(y,Q)
f=rhs(trim_state(y,Q),Q); f=f(1:2);
end

function X=lift_points(points,Q,c,T,variant)
n=size(points,1);
if strcmp(c.initialization,'trim') && T.root_found
    X=repmat(T.x,1,n);
else
    X=zeros(32,n);
end
assert(size(points,2)==2,'Two initial-state coordinates [vc,r] are required');
X(1,:)=points(:,1)'; X(2,:)=points(:,2)';
if strcmp(c.slice_mode,'flat')
    X(3:16,:)=0; X(29,:)=0; X(4,:)=c.fixed_phi0;
elseif strcmp(c.slice_mode,'legacy_phi0')
    X(4,:)=c.fixed_phi0;
elseif ~strcmp(c.initialization,'trim') || ~T.root_found
    X(4,:)=c.fixed_phi0;
end
X(30:32,:)=0;
if strcmp(c.initialization,'legacy')
    X(17:20,:)=0;
    X(21:24,:)=repmat(-Q.delta,1,n);
    X(25:28,:)=repmat(max(Q.ux*cos(Q.delta),0)/Q.Rw,1,n);
else
    X=kinematic_tires(X,Q,true);
end
X(11,:)=X(11,:)+variant(1);
X(21:22,:)=X(21:22,:)+variant(2);
X(23:24,:)=X(23:24,:)+variant(3);
X(29,:)=X(29,:)+variant(4);
end

function p=initial_phi(c,T)
p=c.fixed_phi0;
if strcmp(c.slice_mode,'trim')&&strcmp(c.initialization,'trim')&&T.root_found,p=T.x(4);end
end

function [bits,bad]=constraint_codes(X,O,Q,c)
n=size(X,2); bits=zeros(1,n,'uint16');
yaw=abs(X(2,:))>c.yaw_factor*c.mu*Q.g/Q.ux;
ay=abs(O.ay)>c.ay_factor*c.mu*Q.g;
ltr=abs(O.LTR)>=c.LTR_limit;
contact=any(O.Fz_raw<=0,1);
axle=abs(O.LTRf)>=c.LTR_limit|abs(O.LTRr)>=c.LTR_limit;
num=any(~isfinite(X),1)|~isfinite(O.ay)|any(~isfinite(O.Fz_raw),1)| ...
    ~isfinite(O.LTR)|~isfinite(O.LTRf)|~isfinite(O.LTRr);
guard=abs(X(1,:))>c.guard.vc|abs(X(2,:))>c.guard.r|abs(X(4,:))>c.guard.phi| ...
    any(abs(X(17:20,:))>c.guard.kappa+1e-8,1)|any(abs(X(21:24,:))>c.guard.alpha,1)| ...
    any(X(25:28,:)<0|X(25:28,:)>c.guard.omega,1)|any(O.uw<=0,1);
clipped=any(O.Fz_raw<=Q.Fz_force_min|O.Fz_raw>=Q.Fz_force_max| ...
    O.Fz_elastic_raw<=Q.Fz_force_min|O.Fz_elastic_raw>=Q.Fz_force_max,1);
bits(yaw)=bitor(bits(yaw),uint16(1)); bits(ay)=bitor(bits(ay),uint16(2));
bits(ltr)=bitor(bits(ltr),uint16(4)); bits(contact)=bitor(bits(contact),uint16(8));
bits(axle)=bitor(bits(axle),uint16(16)); bits(num)=bitor(bits(num),uint16(32));
bits(guard)=bitor(bits(guard),uint16(64));
if c.reject_force_clipping, bits(clipped)=bitor(bits(clipped),uint16(128)); end
bad=bitand(bits,uint16(254))~=0;
end

function out=simulate_points(points,Q,c,T)
if strcmp(c.engine,'native')
    out=cdc2d_native_simulate(points,Q,c,T);
    return;
end
n=size(points,1); nh=numel(c.horizons);
out=struct('safe',true(n,nh),'yaw_ok',true(n,nh),'recovered',true(n,nh), ...
    'reason',zeros(n,nh,'uint16'),'terminal_error',zeros(n,nh), ...
    'terminal_derivative',zeros(n,nh),'first_physical_violation',inf(n,1), ...
    'first_yaw_violation',inf(n,1),'peak_r',zeros(n,1),'peak_ay',zeros(n,1), ...
    'peak_LTR',zeros(n,1),'min_Fz',inf(n,1));
for ivar=1:size(c.hidden_variants,1)
 for first=1:c.batch_size:n
    ids=first:min(first+c.batch_size-1,n);
    X=lift_points(points(ids,:),Q,c,T,c.hidden_variants(ivar,:));
    b=simulate_batch(X,Q,c,T);
    out.safe(ids,:)=out.safe(ids,:)&b.safe;
    out.yaw_ok(ids,:)=out.yaw_ok(ids,:)&b.yaw_ok;
    out.recovered(ids,:)=out.recovered(ids,:)&b.recovered;
    out.reason(ids,:)=bitor(out.reason(ids,:),b.reason);
    out.terminal_error(ids,:)=max(out.terminal_error(ids,:),b.terminal_error);
    out.terminal_derivative(ids,:)=max(out.terminal_derivative(ids,:),b.terminal_derivative);
    out.first_physical_violation(ids)=min(out.first_physical_violation(ids),b.first_physical_violation);
    out.first_yaw_violation(ids)=min(out.first_yaw_violation(ids),b.first_yaw_violation);
    out.peak_r(ids)=max(out.peak_r(ids),b.peak_r); out.peak_ay(ids)=max(out.peak_ay(ids),b.peak_ay);
    out.peak_LTR(ids)=max(out.peak_LTR(ids),b.peak_LTR); out.min_Fz(ids)=min(out.min_Fz(ids),b.min_Fz);
    if c.verbose, fprintf('  variant %d/%d: %d/%d initial states\n',ivar,size(c.hidden_variants,1),ids(end),n); end
 end
end
out.status=uint8(out.safe); out.status(out.recovered)=uint8(2);
out.status_definition='0 constraint/domain violation; 1 safe but recovery unresolved; 2 safe and recovered';
out.first_yaw_violation_note='Observed until physical/domain failure; not necessarily full-horizon yaw history';
end

function out=simulate_batch(X,Q,c,T)
n=size(X,2); nh=numel(c.horizons); dt=c.Ts/c.Nsub;
H=round(c.horizons/dt); laststep=H(end);
out=struct('safe',false(n,nh),'yaw_ok',false(n,nh),'recovered',false(n,nh), ...
    'reason',zeros(n,nh,'uint16'),'terminal_error',inf(n,nh), ...
    'terminal_derivative',inf(n,nh),'first_physical_violation',inf(n,1), ...
    'first_yaw_violation',inf(n,1),'peak_r',zeros(n,1),'peak_ay',zeros(n,1), ...
    'peak_LTR',zeros(n,1),'min_Fz',inf(n,1));
active=true(1,n); reason=zeros(1,n,'uint16'); last_not_close=-inf(1,n);
err=inf(1,n); derr=inf(1,n); ih=1;
for k=0:laststep
    t=k*dt; ids=find(active); k1=[];
    if ~isempty(ids)
        xa=X(:,ids); [k1,O]=rhs(xa,Q);
        [bits,bad]=constraint_codes(xa,O,Q,c);
        nonfinitedx=any(~isfinite(k1),1);
        bits(nonfinitedx)=bitor(bits(nonfinitedx),uint16(32)); bad=bad|nonfinitedx;
        reason(ids)=bitor(reason(ids),bits);
        yhit=ids(bitand(bits,uint16(1))~=0);
        out.first_yaw_violation(yhit)=min(out.first_yaw_violation(yhit),t);
        out.peak_r(ids)=max(out.peak_r(ids),abs(xa(2,:))');
        out.peak_ay(ids)=max(out.peak_ay(ids),abs(O.ay)');
        out.peak_LTR(ids)=max(out.peak_LTR(ids),max(abs([O.LTR;O.LTRf;O.LTRr]),[],1)');
        out.min_Fz(ids)=min(out.min_Fz(ids),min(O.Fz_raw,[],1)');
        if T.usable
            err(ids)=max(abs(xa(1:29,:)-T.x(1:29))./c.state_tolerance(:),[],1);
            derr(ids)=max(abs(k1(1:29,:))./(c.state_tolerance(:)/c.derivative_time_scale),[],1);
            last_not_close(ids(err(ids)>1|derr(ids)>1))=t;
        end
        hit=ids(bad); active(hit)=false; out.first_physical_violation(hit)=t;
        keep=~bad; ids=ids(keep); xa=xa(:,keep); k1=k1(:,keep);
    end
    if ih<=nh && k==H(ih)
        out.safe(:,ih)=active'; out.yaw_ok(:,ih)=(bitand(reason,uint16(1))==0)';
        close_window=(t-last_not_close>=c.terminal_window-1e-10) & err<=1 & derr<=1;
        out.recovered(:,ih)=(active&T.usable&close_window)';
        out.reason(:,ih)=reason'; out.terminal_error(:,ih)=err'; out.terminal_derivative(:,ih)=derr';
        ih=ih+1;
    end
    if k==laststep, break; end
    if isempty(ids)
        % No recovery can occur after an earlier path-constraint violation.
        for j=ih:nh
            out.reason(:,j)=reason'; out.yaw_ok(:,j)=(bitand(reason,uint16(1))==0)';
        end
        break;
    end
    k2=rhs(xa+0.5*dt*k1,Q); k3=rhs(xa+0.5*dt*k2,Q); k4=rhs(xa+dt*k3,Q);
    X(:,ids)=xa+dt*(k1+2*k2+2*k3+k4)/6;
    % No post-step clipping: excursions are diagnosed, not silently projected.
end
end

function M=make_masks(R,nv,nr)
nh=size(R.safe,2); sz=[nv nr nh];
M=struct('safe',reshape(R.safe,sz),'yaw_policy',reshape(R.yaw_ok,sz), ...
    'guarded_safe',reshape(R.safe&R.yaw_ok,sz),'recoverable',reshape(R.recovered,sz), ...
    'guarded_recoverable',reshape(R.recovered&R.yaw_ok,sz), ...
    'safe_unresolved',reshape(R.safe&~R.recovered,sz));
end

function H=handling_map(Q,c,T)
% MF-specific frozen-load / quasi-steady-slip local handling diagnostic.
% Tire forces are recomputed for EVERY steering angle. No geometric shifting.
[v,r]=ndgrid(c.handling_grid.vc,c.handling_grid.r);
pts=[v(:),r(:)]; Fz=repmat(Q.Fz_static,1,numel(v));
[f,C]=handling_field(pts,Q,Fz);
hv=1e-4; hr=1e-5;
p=pts; p(:,1)=p(:,1)+hv; fp=handling_field(p,Q,Fz);
p=pts; p(:,1)=p(:,1)-hv; fm=handling_field(p,Q,Fz); jv=(fp-fm)/(2*hv);
p=pts; p(:,2)=p(:,2)+hr; fp=handling_field(p,Q,Fz);
p=pts; p(:,2)=p(:,2)-hr; fm=handling_field(p,Q,Fz); jr=(fp-fm)/(2*hr);
tr=jv(1,:)+jr(2,:); detA=jv(1,:).*jr(2,:)-jr(1,:).*jv(2,:);
mask=(tr<0)&(detA>0)&all(C>0,1);
H=struct('definition','Frozen-static-load MF diagnostic: tr(J)<0, det(J)>0, all -dFy/dalpha>0', ...
    'not_exact_paper_reproduction',true,'grid',c.handling_grid, ...
    'mask',reshape(mask,size(v)),'trace',reshape(tr,size(v)), ...
    'determinant',reshape(detA,size(v)),'min_tire_restoring_slope',reshape(min(C,[],1),size(v)), ...
    'flow_v',reshape(f(1,:),size(v)),'flow_r',reshape(f(2,:),size(v)), ...
    'paper_kinematic_shift',[Q.ux*Q.lr*c.delta(1)/Q.L,Q.ux*c.delta(1)/Q.L], ...
    'centroid',centroid_or_nan(pts(mask,:)), ...
    'trim_state',T.x([1 2 4])','load_mode','Fixed original static normal loads, same across steering');
end

function [f,C]=handling_field(points,Q,Fz)
X=zeros(32,size(points,1)); X(1:2,:)=points';
[~,a,~]=slips(X,Q); k=free_rolling_kappa(Fz,Q);
[Fx,Fy]=tire_force(k,a,Fz,Q);
Fxb=Fx.*Q.cdelta-Fy.*Q.sdelta; Fyb=Fx.*Q.sdelta+Fy.*Q.cdelta;
f=[sum(Fyb,1)/Q.m-Q.ux*X(2,:);sum(Q.xw.*Fyb-Q.yw.*Fxb,1)/Q.Iz];
if nargout>1
    da=1e-5; [~,yp]=tire_force(k,a+da,Fz,Q); [~,ym]=tire_force(k,a-da,Fz,Q);
    C=-(yp-ym)/(2*da);
end
end

function region=sampled_cells(G,mask,Q,c,T,case_index)
% A sampled 2-D cell needs all FOUR vertices and the requested interior tests.
assert(isequal(size(mask),[numel(G.vc),numel(G.r)]),'2-D mask size mismatch');
cm=true(numel(G.vc)-1,numel(G.r)-1);
for a=0:1
 for b=0:1
  cm=cm & mask(1+a:end-1+a,1+b:end-1+b);
 end
end
cand=find(cm); tested=0; fail=0;
[iv,ir]=ind2sub(size(cm),cand);
lo=[reshape(G.vc(iv),[],1),reshape(G.r(ir),[],1)];
hi=[reshape(G.vc(iv+1),[],1),reshape(G.r(ir+1),[],1)];
region=struct();
if ~isempty(cand)
    oldrng=rng; cleanup=onCleanup(@()rng(oldrng)); %#ok<NASGU>
    rng(c.random_seed+case_index,'twister'); valid=true(numel(cand),1);
    nround=double(c.check_cell_centers)+c.random_points_per_cell;
    for j=1:nround
        ids=find(valid); if isempty(ids), break; end
        if j==1&&c.check_cell_centers, u=0.5*ones(numel(ids),2); else, u=rand(numel(ids),2); end
        points=lo(ids,:)+u.*(hi(ids,:)-lo(ids,:));
        r=simulate_points(points,Q,c,T); pass=r.recovered(:,end);
        if c.use_yaw_policy_for_region, pass=pass&r.yaw_ok(:,end); end
        valid(ids(~pass))=false; tested=tested+numel(ids);
    end
    cm(cand(~valid))=false; fail=nnz(~valid);
end
region.cell_mask=cm; region.candidate_cells=numel(cand);
region.accepted_cells=nnz(cm); region.interior_points_tested=tested; region.failed_cells=fail;
region.certified_continuous=false;
region.definition='Union of sampled 2-D cells with four recovered vertices and requested interior tests; NOT a certified inner approximation';
region.area_units='(m/s)*(rad/s)';
[dv,dr]=ndgrid(diff(G.vc),diff(G.r)); area=dv.*dr;
region.sampled_area=sum(area(cm));
[cv,cr]=ndgrid((G.vc(1:end-1)+G.vc(2:end))/2, ...
    (G.r(1:end-1)+G.r(2:end))/2);
if region.sampled_area>0
    region.cell_area_centroid=[sum(cv(cm).*area(cm)),sum(cr(cm).*area(cm))]/region.sampled_area;
else
    region.cell_area_centroid=[NaN NaN];
end
end

function s=summarize_case(C,points,c)
r=C.raw;
s=struct('wear',C.wear,'safe_count',nnz(r.safe(:,end)), ...
    'guarded_safe_count',nnz(r.safe(:,end)&r.yaw_ok(:,end)), ...
    'recovered_count',nnz(r.recovered(:,end)),'selected_count',nnz(C.stable_mask), ...
    'safe_unresolved_count',nnz(r.safe(:,end)&~r.recovered(:,end)), ...
    'selected_centroid',centroid_or_nan(points(C.stable_mask(:),:)), ...
    'safe_centroid',centroid_or_nan(points(r.safe(:,end),:)), ...
    'trim',C.trim.x([1 2 4])','paper_kinematic_shift',C.handling.paper_kinematic_shift, ...
    'sampled_cell_area',C.region.sampled_area,'touches_grid',touches_faces(C.stable_mask), ...
    'constraint_counts',zeros(1,8),'use_yaw_policy',c.use_yaw_policy_for_region);
for j=1:8, s.constraint_counts(j)=nnz(bitand(r.reason(:,end),uint16(2^(j-1)))); end
end

function c=centroid_or_nan(p)
if isempty(p), c=nan(1,size(p,2)); else, c=mean(p,1); end
end
function b=touches_faces(m)
b=[any(m(1,:)),any(m(end,:)),any(m(:,1)),any(m(:,end))];
end
function validate_saved_case(C,c,G,wear)
nv=numel(G.vc); nr=numel(G.r); nh=numel(c.horizons); n=nv*nr;
assert(isequal(C.grid,G)&&isequal(C.wear(:)',wear(:)'),'Checkpoint grid/wear mismatch');
assert(C.fixed_initial_roll_rad==initial_phi(c,C.trim),'Checkpoint roll-slice mismatch');
assert(strcmp(C.slice_mode,c.slice_mode),'Checkpoint slice mode mismatch');
assert(strcmp(C.initialization_requested,c.initialization),'Checkpoint initialization mismatch');
assert(C.total_count==n && isequal(size(C.stable_mask),[nv nr]),'Checkpoint dimensions mismatch');
assert(isequal(size(C.raw.safe),[n nh])&&isequal(size(C.raw.reason),[n nh]),'Raw checkpoint dimensions mismatch');
assert(all(C.raw.reason(:)<=255),'Unknown classification reason bits');
expected=reshape(C.raw.recovered(:,end),nv,nr);
if c.use_yaw_policy_for_region, expected=expected&reshape(C.raw.yaw_ok(:,end),nv,nr); end
assert(isequal(C.stable_mask,expected)&&C.stable_count==nnz(expected),'Checkpoint selected mask/count mismatch');
[V,R]=ndgrid(G.vc,G.r); points=[V(:),R(:)];
assert(isequal(C.stable_points,points(expected(:),:)),'Checkpoint selected points mismatch');
assert(isequal(C.nonselected_points,points(~expected(:),:)),'Checkpoint complement mismatch');
assert(isequal(size(C.region.cell_mask),[nv-1 nr-1]),'Checkpoint cell dimensions mismatch');
assert(C.region.accepted_cells==nnz(C.region.cell_mask),'Checkpoint cell count mismatch');
assert(isfinite(C.region.sampled_area)&&C.region.sampled_area>=0,'Checkpoint cell area invalid');
[dv,dr]=ndgrid(diff(G.vc),diff(G.r)); areas=dv.*dr;
assert(abs(C.region.sampled_area-sum(areas(C.region.cell_mask)))<=1e-12,'Checkpoint cell area/count mismatch');
assert(C.summary.selected_count==C.stable_count,'Checkpoint summary mismatch');
end

function digest=file_sha256(path)
fid=fopen(path,'rb'); assert(fid>=0,'Cannot fingerprint %s',path);
cleanup=onCleanup(@()fclose(fid)); %#ok<NASGU>
if exist('OCTAVE_VERSION','builtin')
    bytes=fread(fid,inf,'*uint8'); digest=hash('sha256',char(bytes.'));
else
    assert(usejava('jvm'),'MATLAB Java VM is required for SHA-256 cache provenance');
    md=java.security.MessageDigest.getInstance('SHA-256');
    while true
        bytes=fread(fid,1024*1024,'*uint8'); if isempty(bytes), break; end
        md.update(typecast(bytes,'int8'));
    end
    bytes=typecast(md.digest(),'uint8'); digest=lower(reshape(dec2hex(bytes,2).',1,[]));
end
end

function atomic_save(path,name,value)
s=struct(); s.(name)=value; tmp=[path '.tmp'];
save(tmp,'-struct','s','-v7'); [ok,msg]=movefile(tmp,path,'f'); assert(ok,msg);
end
function write_summary_csv(path,S)
f=fopen(path,'w'); assert(f>=0,'Cannot write summary'); closefile=onCleanup(@()fclose(f)); %#ok<NASGU>
fprintf(f,'wear_FL,wear_FR,wear_RL,wear_RR,safe,guarded_safe,recovered,selected,unresolved,centroid_v,centroid_r,trim_v,trim_r,trim_phi,sampled_cell_area\n');
for i=1:numel(S.CASE)
    s=S.CASE(i).summary;
    fprintf(f,'%g,%g,%g,%g,%d,%d,%d,%d,%d,%.12g,%.12g,%.12g,%.12g,%.12g,%.12g\n', ...
        s.wear,s.safe_count,s.guarded_safe_count,s.recovered_count,s.selected_count, ...
        s.safe_unresolved_count,s.selected_centroid,s.trim,s.sampled_cell_area);
end
end

function report=run_selftest(o)
% Runs in MATLAB. Reference scalar equations are embedded below unchanged.
o.verbose=false; o.speeds=80; o.mus=0.9; o.steers=2;
c=o; c.speed_kmh=80; c.ux=80/3.6; c.mu=0.9; c.steer_deg=2; c.delta=deg2rad([2;2;0;0]);
P=ref_default_fullcar_params(); Q=prepare_model(P,c,[25 25 50 50]);
T=find_trim(Q,c); assert(T.root_found,'Trim solve failed');
oldrng=rng; restore=onCleanup(@()rng(oldrng)); %#ok<NASGU>
rng(211,'twister'); X=repmat(T.x,1,12);
sc=[1;0.1;0.002;0.03;0.002;0.002*ones(4,1);0.02*ones(7,1); ...
    0.01*ones(4,1);0.02*ones(4,1);0.3*ones(4,1);0.1;0.1;1;1];
X=X+sc.*randn(size(X)); B=rhs(X,Q); A=zeros(size(B));
P.mu_scale_x=Q.mu_scale_x; P.mu_scale_y=Q.mu_scale_y;
for j=1:size(X,2)
    A(:,j)=ref_fullcar_rhs_wrapper_stability(X(:,j),Q.delta,Q.ux,Q.wear',Q.Icmd,Q.kw,P);
end
relative=max(abs(A(:)-B(:))./max(1,abs(A(:))));
assert(relative<1e-10,'Vectorized/scalar RHS mismatch');
Xt=kinematic_tires(X,Q,true); [kt,at,~]=slips(Xt,Q);
sliperror=max([abs(kt(:)-reshape(Xt(17:20,:),[],1));abs(at(:)-reshape(Xt(21:24,:),[],1))]);
assert(sliperror<1e-12,'Initialization mismatch');
% dt versus dt/2 at and near the trim, with a real recovery horizon.
c.horizons=1; c.terminal_window=0.2; c.check_cell_centers=false;
p=[T.x([1 2])'; T.x([1 2])'+[0.02 0.001]];
X2=lift_points(p,Q,c,T,[0 0 0 0]);
assert(all(X2(4,:)==initial_phi(c,T)),'Initial roll does not match slice_mode');
assert(max(abs(X2(3:9,1)-T.x(3:9)))<1e-12,'Trim slice changed hidden vertical equilibrium');
r1=simulate_points(p,Q,c,T); c.Nsub=2*c.Nsub; r2=simulate_points(p,Q,c,T);
rt=simulate_batch(T.x,Q,c,T);
assert(rt.recovered(1),'Checked trim does not remain recovered');
Gtest=struct('vc',[-0.5 0 0.5],'r',[-0.1 0 0.1]);
A2=sampled_cells(Gtest,true(3,3),Q,c,T,1);
assert(abs(A2.sampled_area-0.2)<1e-14 && A2.accepted_cells==4,'2-D cell area failed');
report=struct('passed',true,'relative_rhs_error',relative,'initial_slip_error',sliperror, ...
    'trim_residual',T.residual_norm,'trim_max_real_eig',T.max_real_eigenvalue, ...
    'slice_initial_roll_verified',all(X2(4,:)==initial_phi(c,T)), ...
    'full_grid_initial_points',numel(o.grid.vc)*numel(o.grid.r), ...
    'two_dimensional_cell_area',A2.sampled_area, ...
    'dt_classification_equal',isequal(r1.status,r2.status), ...
    'dt_terminal_error_difference',max(abs(r1.terminal_error(:)-r2.terminal_error(:))));
disp(report);
end


%% REFERENCE SCALAR MODEL: copied from supplied code, names only changed.
function P = ref_default_fullcar_params()

    P.m   = 2218.042;

    P.ms  = 1967.686;

    P.Iz  = 4793.0;

    P.Ixx = 924.0;

    P.Iyy = 2500.0;

    P.lf = 1.472;

    P.lr = 1.531;

    P.tf = 1.633;

    P.tr = 1.631;

    P.L  = P.lf+P.lr;

    P.g  = 9.81;

    P.h_roll = 0.20;

    P.h_cg = 0.541;

    P.tau_load = 0.03;

    P.Rw = 0.360;

    P.Iw = 1.80;

    P.MRf = 0.8613;

    P.MRr = 0.7970;

    P.MR_vec = [P.MRf;P.MRf;P.MRr;P.MRr];

    P.sigma_kappa = [0.35;0.35;0.35;0.35];

    P.sigma_alpha = [0.75;0.75;0.75;0.75];

    P.kt = [200000;200000;200000;200000];

    P.Fz0_vec = [5425;5425;5425;5425];

    P.Fz_force_min = 100.0;

    P.Fz_force_max = 16000.0;

    P.Karb_f = 0.0;

    P.Karb_r = 0.0;

    P.Carb_f = 0.0;

    P.Carb_r = 0.0;

    mu_total = P.m-P.ms;

    mu_each = mu_total/4.0;

    P.mu_u = [mu_each;mu_each;mu_each;mu_each];

    P.Fz_static = [ P.m*P.g*P.lr/(2.0*P.L); P.m*P.g*P.lr/(2.0*P.L); P.m*P.g*P.lf/(2.0*P.L); P.m*P.g*P.lf/(2.0*P.L)];

end

function kw = ref_spring_to_wheel_rate(kSpring,P)

    kw = [kSpring(1)*P.MRf^2; kSpring(2)*P.MRf^2; kSpring(3)*P.MRr^2; kSpring(4)*P.MRr^2];

    for i=1:4

        if ~isfinite(kw(i)) || kw(i)<1000.0

            kw(i)=30000.0;

        end

    end

end

function dx = ref_fullcar_rhs_wrapper_stability(x,delta_w,ux,wear_cmd,Icmd,kw,P)

    dx = ref_fullcar_derivative_internal( x,delta_w,ux,wear_cmd,Icmd, P.m,P.ms,P.Iz,P.Ixx,P.Iyy,P.lf,P.lr,P.tf,P.tr, P.h_roll,P.h_cg,P.g,P.Rw,P.Iw,P.tau_load, P.Fz_static,P.Fz0_vec,P.Fz_force_min,P.Fz_force_max, P.sigma_kappa,P.sigma_alpha,kw,P.MR_vec,P.mu_u,P.kt, P.Karb_f,P.Karb_r,P.Carb_f,P.Carb_r,P.mu_scale_x,P.mu_scale_y);

end

%% ========================================================================

% LOCAL FUNCTIONS: PAPER SECT. 3.2 3-D REGION

% ========================================================================

function dx = ref_fullcar_derivative_internal( x,delta_w,ux,wear_cmd,I_cmd, m,ms,Iz,Ixx,Iyy,lf,lr,tf,tr,h_roll,h_cg,g,Rw,Iw,tau_load, Fz_static,Fz0_vec,Fz_force_min,Fz_force_max, sigma_kappa,sigma_alpha,kw,MR_vec,mu_u,kt, Karb_f,Karb_r,Carb_f,Carb_r,mu_scale_x,mu_scale_y)

vc    = x(1);

r     = x(2);

zs    = x(3);

phi   = x(4);

theta = x(5);

dzs = x(10);

p   = x(11);

q   = x(12);

zu  = [x(6);x(7);x(8);x(9)];

vzu = [x(13);x(14);x(15);x(16)];

kappa_eff = x(17:20);

alpha_eff = x(21:24);

omega     = x(25:28);

xw = [lf;lf;-lr;-lr];

yw = [tf/2.0;-tf/2.0;tr/2.0;-tr/2.0];

%% Sprung-body corner kinematics

zB = [zs+lf*theta+(tf/2.0)*phi;

      zs+lf*theta-(tf/2.0)*phi;

      zs-lr*theta+(tr/2.0)*phi;

      zs-lr*theta-(tr/2.0)*phi];

vB = [dzs+lf*q+(tf/2.0)*p;

      dzs+lf*q-(tf/2.0)*p;

      dzs-lr*q+(tr/2.0)*p;

      dzs-lr*q-(tr/2.0)*p];

%% Suspension forces

Fspring = -kw.*(zB-zu);

vrel = vB-vzu;

cmp_rate = -1000.0*MR_vec.*vrel;

Fd_damper = ref_lookup_all_damper_forces(cmp_rate,I_cmd);

Fd_wheel  = MR_vec.*Fd_damper;

Fs = Fspring+Fd_wheel;

Marb_f = -Karb_f*phi-Carb_f*p;

Marb_r = -Karb_r*phi-Carb_r*p;

Fs(1)=Fs(1)+Marb_f/tf;

Fs(2)=Fs(2)-Marb_f/tf;

Fs(3)=Fs(3)+Marb_r/tr;

Fs(4)=Fs(4)-Marb_r/tr;

%% Internal normal loads

[Fz,Fz_elastic] = ref_calculate_internal_fz( x,Fz_static,kt,Fz_force_min,Fz_force_max, m,ms,h_cg,h_roll,kw,tf,tr,Karb_f,Karb_r);

%% Internal wheel kinematics and raw slips

[kappa_target,alpha_target,u_wheel] = ref_calculate_internal_slips( x,delta_w,ux,lf,lr,tf,tr,Rw);

%% Tire forces used by dynamics

[Fx_body,Fy_body,Fx_tire,~] = ref_calculate_tire_forces( delta_w,kappa_eff,Fz,alpha_eff,wear_cmd,Fz0_vec,Fz_force_min,mu_scale_x,mu_scale_y);

Fx_total=sum(Fx_body);

Fy_total=sum(Fy_body);

Mz=0.0;

for i=1:4

    Mz=Mz+xw(i)*Fy_body(i)-yw(i)*Fx_body(i);

end

%% Planar dynamics

vc_dot = Fy_total/m-ux*r;

r_dot  = Mz/Iz;

ay     = Fy_total/m;

%% Sprung vertical, roll and pitch dynamics

zs_ddot=sum(Fs)/ms;

Mroll_susp=(tf/2.0)*(Fs(1)-Fs(2))+(tr/2.0)*(Fs(3)-Fs(4));

Mroll_lat=ms*h_roll*ay;

Mroll_gravity=-ms*g*h_roll*phi;

phi_ddot=(Mroll_susp+Mroll_lat+Mroll_gravity)/Ixx;

Mpitch=lf*(Fs(1)+Fs(2))-lr*(Fs(3)+Fs(4));

theta_ddot=Mpitch/Iyy;

%% Unsprung vertical dynamics about static equilibrium

% Only the elastic tire-deflection load is used in the unsprung equation.

% The additional geometric load-transfer term is an equivalent correction

% for omitted roll-center/unsprung mechanisms and must not be integrated a

% second time through the tire-deflection dynamics.

Ftire_vertical = Fz_elastic-Fz_static;

zu_ddot=(-Fs+Ftire_vertical)./mu_u;

%% Tire relaxation

kappa_dot=zeros(4,1);

alpha_dot=zeros(4,1);

for i=1:4

    vroll=abs(u_wheel(i));

    if vroll<0.5; vroll=0.5; end

    rate_k=vroll/sigma_kappa(i);

    rate_a=vroll/sigma_alpha(i);

    if rate_k>500.0; rate_k=500.0; end

    if rate_a>500.0; rate_a=500.0; end

    kappa_dot(i)=rate_k*(kappa_target(i)-kappa_eff(i));

    alpha_dot(i)=rate_a*(alpha_target(i)-alpha_eff(i));

end

%% Free-rolling wheel rotational dynamics

omega_dot=zeros(4,1);

for i=1:4

    % Positive tire Fx acts forward on the vehicle and produces a negative

    % reaction torque on the freely rolling wheel.

    omega_dot(i)=(-Rw*Fx_tire(i))/Iw;

end

%% Lateral-load-transfer state

ay_load = x(29);

ay_load_dot = (ay-ay_load)/tau_load;

%% Global-coordinate vehicle kinematics

% Internal yaw rate r is already in rad/s.

psi = x(30);

psi_dot = r;

X_dot = ux*cos(psi) - vc*sin(psi);

Y_dot = ux*sin(psi) + vc*cos(psi);

%% Assemble

dx=zeros(32,1);

dx(1)=vc_dot;

dx(2)=r_dot;

dx(3)=dzs;

dx(4)=p;

dx(5)=q;

dx(6)=vzu(1);

dx(7)=vzu(2);

dx(8)=vzu(3);

dx(9)=vzu(4);

dx(10)=zs_ddot;

dx(11)=phi_ddot;

dx(12)=theta_ddot;

dx(13)=zu_ddot(1);

dx(14)=zu_ddot(2);

dx(15)=zu_ddot(3);

dx(16)=zu_ddot(4);

dx(17)=kappa_dot(1);

dx(18)=kappa_dot(2);

dx(19)=kappa_dot(3);

dx(20)=kappa_dot(4);

dx(21)=alpha_dot(1);

dx(22)=alpha_dot(2);

dx(23)=alpha_dot(3);

dx(24)=alpha_dot(4);

dx(25)=omega_dot(1);

dx(26)=omega_dot(2);

dx(27)=omega_dot(3);

dx(28)=omega_dot(4);

dx(29)=ay_load_dot;

dx(30)=psi_dot;

dx(31)=X_dot;

dx(32)=Y_dot;

end

%% ========================================================================

% Internal wheel-coordinate slip calculation

%% ========================================================================

function [kappa_target,alpha_target,u_wheel] = ref_calculate_internal_slips( x,delta_w,ux,lf,lr,tf,tr,Rw)

vc=x(1);

r=x(2);

omega=[x(25);x(26);x(27);x(28)];

xw=[lf;lf;-lr;-lr];

yw=[tf/2.0;-tf/2.0;tr/2.0;-tr/2.0];

kappa_target=zeros(4,1);

alpha_target=zeros(4,1);

u_wheel=zeros(4,1);

for i=1:4

    u_body=ux-r*yw(i);

    v_body=vc+r*xw(i);

    c=cos(delta_w(i));

    s=sin(delta_w(i));

    % Body velocity resolved in the steered wheel frame.

    uw=c*u_body+s*v_body;

    vw=-s*u_body+c*v_body;

    u_wheel(i)=uw;

    uw_safe=uw;

    if uw_safe>=0.0 && uw_safe<0.1; uw_safe=0.1; end

    if uw_safe<0.0 && uw_safe>-0.1; uw_safe=-0.1; end

    alpha_target(i)=atan2(vw,uw_safe);

    denom=abs(uw);

    if denom<0.5; denom=0.5; end

    kappa_target(i)=(Rw*omega(i)-uw)/denom;

    if kappa_target(i)<-0.5; kappa_target(i)=-0.5; end

    if kappa_target(i)> 0.5; kappa_target(i)= 0.5; end

end

end

%% ========================================================================

% Internal vertical tire loads with total lateral-load-transfer correction

%% ========================================================================

function [Fz,Fz_elastic,Fz_raw] = ref_calculate_internal_fz( x,Fz_static,kt,Fz_force_min,Fz_force_max, m,ms,h_cg,h_roll,kw,tf,tr,Karb_f,Karb_r)

%

% z is positive upward and the road is flat:

%   Fz_elastic = Fz_static - kt*zu

%

% The simple roll model generates only the elastic sprung-mass contribution

% associated with ms*h_roll*ay. A real vehicle also has geometric load

% transfer through the roll centers and unsprung-mass contributions. The

% missing roll moment is added here as an equal-and-opposite left/right load

% pair at each axle. For a left turn (ay > 0), FR and RR are the outer tires.

zu=[x(6);x(7);x(8);x(9)];

ay_load=x(29);

% Tire-deflection contribution already represented by vertical dynamics.

Fz_elastic=Fz_static-kt.*zu;

% Front/rear roll-stiffness distribution. For two corner springs per axle,

% Kphi = (k_left+k_right)*track^2/4.

Kphi_f=(kw(1)+kw(2))*tf*tf/4.0 + Karb_f;

Kphi_r=(kw(3)+kw(4))*tr*tr/4.0 + Karb_r;

Kphi_sum=Kphi_f+Kphi_r;

if Kphi_sum<1.0

    lambda_f=0.5;

else

    lambda_f=Kphi_f/Kphi_sum;

end

lambda_r=1.0-lambda_f;

% Total desired load-transfer roll moment minus the elastic moment already

% created by the identified sprung roll arm. Clamp at zero to avoid applying

% a correction in the wrong direction if parameters are changed later.

h_missing=m*h_cg-ms*h_roll;

if h_missing<0.0; h_missing=0.0; end

M_missing=h_missing*ay_load;

% Per-wheel load shift. ay > 0 is a left turn, so the right tires are outer.

dF_front=lambda_f*M_missing/tf;

dF_rear =lambda_r*M_missing/tr;

Fz_raw =Fz_elastic + [-dF_front; dF_front; -dF_rear; dF_rear];

Fz = Fz_raw;

for i=1:4

    if ~isfinite(Fz_elastic(i)); Fz_elastic(i)=Fz_static(i); end

    if ~isfinite(Fz(i)); Fz(i)=Fz_static(i); end

    if Fz_elastic(i)<Fz_force_min; Fz_elastic(i)=Fz_force_min; end

    if Fz_elastic(i)>Fz_force_max; Fz_elastic(i)=Fz_force_max; end

    if Fz(i)<Fz_force_min; Fz(i)=Fz_force_min; end

    if Fz(i)>Fz_force_max; Fz(i)=Fz_force_max; end

end

end

%% ========================================================================

% Tire forces from internal effective slips and normal loads

%% ========================================================================

function [Fx_body,Fy_body,Fx_tire,Fy_tire] = ref_calculate_tire_forces( delta_w,kappa_eff,Fz,alpha_eff,wear_cmd,Fz0_vec,Fz_force_min,mu_scale_x,mu_scale_y)

Fx_tire=zeros(4,1);

Fy_tire=zeros(4,1);

Fx_body=zeros(4,1);

Fy_body=zeros(4,1);

for i=1:4

    if Fz(i)>Fz_force_min

        [Fx_tire(i),Fy_tire(i)] = ref_calc_combined_tire_force_scalar( kappa_eff(i),alpha_eff(i),Fz(i),Fz0_vec(i),wear_cmd(i),mu_scale_x,mu_scale_y);

    end

    c=cos(delta_w(i));

    s=sin(delta_w(i));

    Fx_body(i)=Fx_tire(i)*c-Fy_tire(i)*s;

    Fy_body(i)=Fx_tire(i)*s+Fy_tire(i)*c;

end

end

%% ========================================================================

% Combined-slip correction

%% ========================================================================

function [Fx,Fy] = ref_calc_combined_tire_force_scalar(kappa,alpha,Fz,Fz0,wear,mu_scale_x,mu_scale_y)

%

% The available coefficient set contains pure-slip MF coefficients but not

% the complete MF6.2 combined-slip R-coefficients. Therefore this model uses

% a longitudinal-priority friction ellipse:

%

%   Fx = Fx0

%   Fy = Fy0*sqrt(max(0,1-(Fx0/(mu_x*Fz))^2))

%

% This is specifically chosen because the current comparison shows Fx is

% already close while the pure-slip model overpredicts Fy. Exact CarSim

% agreement requires the TIR RBX/RCX/REX/RBY/RCY/REY/RVY coefficients.

Fx0 = ref_calc_mf_longitudinal_scalar(kappa,Fz,Fz0,wear,mu_scale_x);

Fy0 = ref_calc_mf_lateral_scalar(alpha,Fz,Fz0,wear,mu_scale_y);

mux_peak = ref_calc_longitudinal_peak_mu(Fz,Fz0,wear,mu_scale_x);

Fx_cap = mux_peak*Fz;

if Fx_cap < 1.0; Fx_cap=1.0; end

rho_x = abs(Fx0)/Fx_cap;

if rho_x>1.0; rho_x=1.0; end

Gy = sqrt(max(0.0,1.0-rho_x*rho_x));

Fx = Fx0;

Fy = Fy0*Gy;

end

%% ========================================================================

% Longitudinal peak-friction estimate used by friction ellipse

%% ========================================================================

function mux = ref_calc_longitudinal_peak_mu(Fz,Fz0,wear,mu_scale_x)

PDX1 = 1.145;

PDX2 = -0.2106;

wear_case = ref_normalize_wear_case(wear);

if wear_case == 25

    PDX1 = PDX1*0.95;

elseif wear_case == 50

    PDX1 = PDX1*0.90;

end

dfz = (Fz-Fz0)/Fz0;

mux = abs(PDX1+PDX2*dfz)*mu_scale_x;

if mux<0.05; mux=0.05; end

end

%% ========================================================================

% Pure longitudinal MF-Swift 6.2 force for one tire

%% ========================================================================

function Fx = ref_calc_mf_longitudinal_scalar(kappa, Fz, Fz0, wear, mu_scale_x)

PCX1 = 1.647;

PDX1 = 1.145;

PDX2 = -0.2106;

PEX1 = -0.02192;

PEX2 = 0.23613;

PEX3 = 0.0;

PEX4 = 0.0;

PKX1 = 24.125;

PKX2 = 15.0;

PKX3 = -0.4832;

PHX1 = -0.001635;

PHX2 =  0.0025297;

PVX1 = 0.0;

PVX2 = 0.0;

wear_case = ref_normalize_wear_case(wear);

if wear_case == 25

    PDX1 = PDX1*0.95;

    PKX1 = PKX1*0.95;

    PKX2 = PKX2*0.95;

elseif wear_case == 50

    PDX1 = PDX1*0.90;

    PKX1 = PKX1*0.90;

    PKX2 = PKX2*0.90;

end

dfz = (Fz-Fz0)/Fz0;

mux = PDX1 + PDX2*dfz;

Dx  = mux*Fz*mu_scale_x;

Cx  = PCX1;

Kx = Fz*(PKX1 + PKX2*dfz)*exp(PKX3*dfz);

if abs(Cx*Dx) < 1.0e-9

    Bx = 0.0;

else

    Bx = Kx/(Cx*Dx);

end

Ex_base = PEX1 + PEX2*dfz + PEX3*dfz^2;

SHx = PHX1 + PHX2*dfz;

SVx = Fz*(PVX1 + PVX2*dfz);

kappa_x = kappa + SHx;

E = Ex_base*(1.0 - PEX4*sign(kappa_x));

if E > 1.0

    E = 1.0;

end

phi_x = Bx*kappa_x;

Fx = Dx*sin(Cx*atan(phi_x - E*(phi_x-atan(phi_x)))) + SVx;

end

%% ========================================================================

% Pure lateral MF-Swift 6.2 force for one tire

%% ========================================================================

function Fy = ref_calc_mf_lateral_scalar(alpha, Fz, Fz0, wear, mu_scale_y)

wear_case = ref_normalize_wear_case(wear);

if wear_case == 0

    PCY1 = 1.5662;

    PDY1 = 1.0278;

    PDY2 = -0.17125;

    PEY1 = -1.1357;

    PEY2 = -1.0544;

    PKY1 = -25.85;

    PKY2 = 2.1377;

    PKY4 = 2.0;

    PHY1 = 8.675e-4;

    PHY2 = 7.046e-5;

    PVY1 = -0.007239;

    PVY2 = -7.442e-5;

elseif wear_case == 25

    PCY1 = 1.5457;

    PDY1 = 1.0286;

    PDY2 = -0.16688;

    PEY1 = -1.439;

    PEY2 = -1.6063;

    PKY1 = -26.0;

    PKY2 = 1.9164;

    PKY4 = 2.0;

    PHY1 = 1.82e-3;

    PHY2 = -7.193e-4;

    PVY1 = 1.802e-3;

    PVY2 = -1.1497e-2;

else

    PCY1 = 1.5057;

    PDY1 = 1.0067;

    PDY2 = -0.2019;

    PEY1 = -2.0;

    PEY2 = -2.0;

    PKY1 = -27.27;

    PKY2 = 1.5963;

    PKY4 = 2.0;

    PHY1 = 2.3735e-3;

    PHY2 = 2.38e-4;

    PVY1 = 0.01487;

    PVY2 = -0.011025;

end

dfz = (Fz-Fz0)/Fz0;

muy = PDY1 + PDY2*dfz;

Dy  = muy*Fz*mu_scale_y;

Cy  = PCY1;

Ky = PKY1*Fz0*sin(PKY4*atan(Fz/(PKY2*Fz0)));

if abs(Cy*Dy) < 1.0e-9

    By = 0.0;

else

    By = Ky/(Cy*Dy);

end

Ey_base = PEY1 + PEY2*dfz;

SHy = PHY1 + PHY2*dfz;

SVy = Fz*(PVY1 + PVY2*dfz);

alpha_y = alpha + SHy;

E = Ey_base;

if E > 1.0

    E = 1.0;

end

phi_y = By*alpha_y;

Fy = Dy*sin(Cy*atan(phi_y - E*(phi_y-atan(phi_y)))) + SVy;

end

%% ========================================================================

% Normalize wear input to 0 / 25 / 50

%% ========================================================================

function wear_case = ref_normalize_wear_case(wear)

if wear < 12.5

    wear_case = 0;

elseif wear < 37.5

    wear_case = 25;

else

    wear_case = 50;

end

end

%% ========================================================================

% Calculate model-based damper rates

%% ========================================================================

function Fd = ref_lookup_all_damper_forces(cmp_rate, I_cmd)

%

% Inputs

%   cmp_rate : [FL; FR; RL; RR] [mm/s]

%   I_cmd    : [FL; FR; RL; RR] [mA]

%

% Output

%   Fd       : damper axial force [N]

%              [FL; FR; RL; RR]

%% Current axis [mA]

I_axis = (100:100:1800).';

%% Damper-rate axis [mm/s]

v_axis = [-1048 -524 -393 -262 -131 -79 -52 -26 0 26 52 79 131 262 393 524 1048];

%% Converted front damper-force map [N]

%

% Rows    : I_axis

% Columns : v_axis

F_front = [ -1733,  -980,  -838,  -693,  -371,  -184,  -117,   -69,     0,    71,   125,   191,   378,   539,   660,   803,  1544; -1781,  -996,  -844,  -692,  -371,  -183,  -116,   -69,     0,    71,   127,   196,   378,   544,   671,   812,  1551; -1962, -1048,  -877,  -707,  -371,  -183,  -115,   -67,     0,    71,   127,   196,   389,   558,   698,   855,  1613; -2302, -1228,  -983,  -757,  -379,  -186,  -116,   -67,     0,    72,   127,   198,   399,   601,   782,   966,  1702; -2789, -1544, -1215,  -883,  -406,  -197,  -121,   -69,     0,    72,   133,   209,   431,   694,   925,  1136,  1800; -3171, -1890, -1524, -1088,  -465,  -224,  -136,   -71,     0,    75,   141,   228,   490,   813,  1078,  1324,  1928; -3448, -2303, -1901, -1397,  -614,  -289,  -166,   -79,     0,    80,   167,   274,   577,   968,  1267,  1462,  1994; -3705, -2720, -2268, -1702,  -791,  -399,  -235,   -99,     0,    96,   214,   349,   701,  1150,  1408,  1572,  2085; -4029, -3061, -2682, -2033, -1041,  -599,  -383,  -164,     0,   136,   297,   460,   852,  1318,  1505,  1660,  2171; -4211, -3320, -2993, -2431, -1377,  -903,  -633,  -306,     0,   200,   429,   612,  1013,  1408,  1581,  1728,  2227; -4361, -3531, -3255, -2831, -1767, -1278,  -946,  -382,     0,   231,   571,   782,  1171,  1471,  1634,  1777,  2315; -4546, -3709, -3443, -3094, -2151, -1615, -1063,  -390,     0,   231,   656,   939,  1259,  1512,  1677,  1827,  2364; -4725, -3850, -3578, -3235, -2560, -1783, -1104,  -387,     0,   229,   678,  1024,  1298,  1545,  1715,  1861,  2409; -4871, -3961, -3676, -3377, -2901, -1818, -1108,  -384,     0,   227,   669,  1053,  1300,  1563,  1752,  1909,  2493; -4972, -4040, -3766, -3474, -2958, -1817, -1086,  -374,     0,   230,   668,  1047,  1291,  1567,  1734,  1921,  2545; -5101, -4100, -3814, -3494, -2974, -1820, -1085,  -373,     0,   225,   652,  1030,  1287,  1561,  1730,  1902,  2550; -5195, -4134, -3842, -3496, -2983, -1789, -1060,  -365,     0,   222,   641,  1025,  1279,  1553,  1751,  1916,  2546; -5350, -4259, -3953, -3625, -3182, -2208, -1283,  -446,     0,   285,   776,  1142,  1375,  1619,  1805,  1974,  2634 ];

%% Converted rear damper-force map [N]

F_rear = [ -1888,  -965,  -798,  -644,  -455,  -237,  -151,   -87,     0,    78,   125,   189,   321,   540,   683,   854,  1482; -1905,  -965,  -795,  -643,  -453,  -235,  -149,   -86,     0,    78,   124,   187,   320,   539,   688,   860,  1489; -2124, -1038,  -837,  -660,  -457,  -237,  -151,   -87,     0,    76,   122,   186,   322,   557,   725,   915,  1530; -2591, -1258,  -967,  -726,  -468,  -238,  -151,   -86,     0,    76,   123,   189,   334,   621,   836,  1059,  1624; -3255, -1662, -1271,  -885,  -509,  -253,  -156,   -87,     0,    80,   131,   203,   367,   724,   989,  1210,  1723; -3611, -2078, -1642, -1147,  -604,  -287,  -171,   -92,     0,    85,   146,   234,   446,   878,  1150,  1303,  1800; -3956, -2558, -2078, -1509,  -768,  -375,  -214,  -105,     0,    96,   179,   289,   554,  1053,  1270,  1429,  1910; -4241, -3040, -2503, -1888, -1001,  -513,  -309,  -130,     0,   119,   236,   374,   708,  1171,  1361,  1510,  2018; -4512, -3525, -3010, -2333, -1309,  -743,  -464,  -208,     0,   169,   340,   521,   874,  1260,  1434,  1581,  2105; -4802, -3801, -3406, -2789, -1688, -1106,  -785,  -398,     0,   247,   485,   681,   998,  1319,  1493,  1633,  2140; -5047, -4085, -3780, -3269, -2141, -1549, -1195,  -534,     0,   307,   664,   869,  1072,  1355,  1528,  1679,  2267; -5305, -4331, -4014, -3611, -2587, -2009, -1450,  -558,     0,   317,   808,   963,  1118,  1401,  1559,  1714,  2285; -5535, -4509, -4206, -3829, -3044, -2328, -1489,  -548,     0,   314,   842,   985,  1140,  1424,  1591,  1742,  2386; -5762, -4665, -4358, -3973, -3425, -2430, -1514,  -540,     0,   311,   851,   982,  1139,  1431,  1607,  1764,  2386; -5891, -4770, -4469, -4109, -3600, -2435, -1512,  -537,     0,   307,   855,   979,  1134,  1434,  1605,  1762,  2426; -6039, -4861, -4541, -4156, -3603, -2417, -1488,  -528,     0,   300,   854,   974,  1130,  1428,  1599,  1756,  2442; -6148, -4906, -4554, -4164, -3602, -2415, -1495,  -520,     0,   298,   862,   971,  1126,  1422,  1597,  1756,  2424; -6318, -5054, -4706, -4306, -3798, -3011, -1748,  -617,     0,   356,   928,  1052,  1199,  1475,  1662,  1825,  2488 ];

%% Evaluate all four corners

Fd = zeros(4,1);

Fd(1) = ref_interpDamperForce( I_cmd(1), cmp_rate(1), I_axis, v_axis, F_front);

Fd(2) = ref_interpDamperForce( I_cmd(2), cmp_rate(2), I_axis, v_axis, F_front);

Fd(3) = ref_interpDamperForce( I_cmd(3), cmp_rate(3), I_axis, v_axis, F_rear);

Fd(4) = ref_interpDamperForce( I_cmd(4), cmp_rate(4), I_axis, v_axis, F_rear);

end

%% ========================================================================

% Bilinear CDC-map interpolation

%% ========================================================================

function Fd_N = ref_interpDamperForce( I_mA, cmp_rate, I_axis, v_axis, F_map)

%% Clamp current and rate to map ranges

I = I_mA;

v = cmp_rate;

if I < I_axis(1)

    I = I_axis(1);

elseif I > I_axis(end)

    I = I_axis(end);

end

if v < v_axis(1)

    v = v_axis(1);

elseif v > v_axis(end)

    v = v_axis(end);

end

%% Find current interval

i_low = 1;

for i = 1:(length(I_axis)-1)

    if I >= I_axis(i) && I <= I_axis(i+1)

        i_low = i;

        break;

    end

end

i_high = i_low + 1;

I1 = I_axis(i_low);

I2 = I_axis(i_high);

if I2 == I1

    alpha = 0.0;

else

    alpha = (I-I1)/(I2-I1);

end

%% Find damper-rate interval

j_low = 1;

for j = 1:(length(v_axis)-1)

    if v >= v_axis(j) && v <= v_axis(j+1)

        j_low = j;

        break;

    end

end

j_high = j_low + 1;

v1 = v_axis(j_low);

v2 = v_axis(j_high);

if v2 == v1

    beta = 0.0;

else

    beta = (v-v1)/(v2-v1);

end

%% Bilinear interpolation

F11 = F_map(i_low,  j_low);

F12 = F_map(i_low,  j_high);

F21 = F_map(i_high, j_low);

F22 = F_map(i_high, j_high);

F_low  = F11 + beta*(F12-F11);

F_high = F21 + beta*(F22-F21);

Fd_N = F_low + alpha*(F_high-F_low);

end

