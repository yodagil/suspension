function manifest = stability_sweep_210(options)
%STABILITY_SWEEP_210 한 파일로 실행하는 210조건 안정영역 시뮬레이션.
% 파일명을 stability_sweep_210.m으로 유지하고 MATLAB에서 실행하세요:
%   stability_sweep_210
% 속도 [50 60 70 80 90 100] km/h, mu [0.9 0.8 0.7 0.6 0.5],
% 앞 휠 조향각 [0 0.5 1 1.5 2 2.5 3] deg, 마모 5종을 계산합니다.
% 41x31x31 초기조건, 5초, Ts=0.01초, RK4 하위 적분 20회를 유지합니다.
% 결과는 이 파일 옆 results_single_mfile 폴더에 시나리오별 MAT로 저장합니다.
% 종/횡 타이어 진폭 배율 mu/0.9, 요율 한계 0.85*mu*g/ux,
% 횡가속도 한계 mu*g입니다. mu=0.9에서 원본 타이어 계수 보정을 유지합니다.
% v4 초기화: vc0/r0/조향각으로 계산한 운동학적 alpha를 완화 상태에 넣습니다.
% 초기 휠 속도는 max(u_wheel,0)/Rw, 완화 종슬립 kappa 초기값은 0입니다.
% 초기화가 변경되었으므로 과거 direct-start MAT와 같다고 가정하지 마세요.
% 다른 M 파일, 외부 데이터, C++ 실행 파일은 필요하지 않습니다.
% MATLAB JVM은 이 파일의 SHA-256 재현성 검증에 사용합니다.
% 클라우드 계산은 같은 방정식을 C++로 가속하며 이 M-code는 더 느립니다.
% 일부 조건만 실행하는 예:
%   stability_sweep_210(struct('speeds',80,'mus',0.9,'steers',2))
% 정상 타이어만 80 km/h, mu=0.9, 조향각 2도를 계산하는 예:
%   stability_sweep_210(struct('speeds',80,'mus',0.9,'steers',2, ...
%       'wear_cases',[0 0 0 0]))
% 기존 클라우드 MAT와 이 파일의 계산 캐시는 코드 해시가 달라 혼용하지 마세요.
%RUN_STABILITY_SWEEP Full 210-scenario, 5-wear, 39401-initial-state sweep.
%   stability_sweep_210() uses exactly the original 41x31x31 grid,
%   Ts=0.01 s, Nsub=20, and 5 s observation horizon. Front wheel steer is
%   [angle angle 0 0] degrees, NOT the steering-wheel angle.
%   Files are MATLAB-compatible v7 MAT files containing STABILITY3D.
%   Optional options fields: speeds, mus, steers, output_dir, engine, wear_cases.
%   wear_cases=[0 0 0 0] computes only the healthy tire case.
if nargin < 1, options = struct(); end
root = fileparts(mfilename('fullpath'));
[cfg0,G,default_wear_cases,P0,GF] = model_fullcar_model('defaults');
wear_cases = sweep_get_option(options,'wear_cases',default_wear_cases);
assert(isnumeric(wear_cases) && ismatrix(wear_cases) && ~isempty(wear_cases) && ...
 size(wear_cases,2)==4 && all(isfinite(wear_cases(:))), ...
 'wear_cases must be a finite nonempty N-by-4 numeric array.');
assert(all(ismember(wear_cases,default_wear_cases,'rows')), ...
 'wear_cases must select rows from the original five wear combinations.');
assert(size(unique(wear_cases,'rows'),1)==size(wear_cases,1), ...
 'Duplicate wear cases are not permitted.');
healthy_case_idx=find(all(wear_cases==0,2),1);
if isempty(healthy_case_idx), healthy_case_idx=NaN; end
healthy_only=size(wear_cases,1)==1 && all(wear_cases(1,:)==0);
speeds = sweep_get_option(options,'speeds',[50 60 70 80 90 100]);
mus = sweep_get_option(options,'mus',[0.9 0.8 0.7 0.6 0.5]);
steers = sweep_get_option(options,'steers',[0 0.5 1 1.5 2 2.5 3]);
outdir = sweep_get_option(options,'output_dir',fullfile(root,'results_single_mfile'));
engine = sweep_get_option(options,'engine','mcode');
assert(isnumeric(speeds) && isnumeric(mus) && isnumeric(steers),'Scenario options must be numeric vectors.');
assert(isvector(speeds) && isvector(mus) && isvector(steers),'Scenario options must be nonempty vectors.');
speeds=speeds(:).'; mus=mus(:).'; steers=steers(:).';
assert(all(isfinite(speeds)) && all(speeds>0) && all(isfinite(mus)) && all(mus>0) && all(isfinite(steers)), ...
 'Scenario options must be finite; speed and mu must be positive.');
assert(numel(unique(speeds))==numel(speeds) && numel(unique(mus))==numel(mus) && numel(unique(steers))==numel(steers), ...
 'Duplicate scenarios are not permitted.');
assert(all(abs(mus*10-round(mus*10))<1e-10),'Filename format supports mu values with one decimal.');
assert(strcmp(engine,'mcode'),'This single-file distribution uses engine=mcode only.');
if ~exist(outdir,'dir'), mkdir(outdir); end
cache = fullfile(outdir,'checkpoints');
if ~exist(cache,'dir'), mkdir(cache); end
n = numel(speeds)*numel(mus)*numel(steers);
manifest = repmat(struct('filename','','speed_kmh',0,'mu_road',0,'steer_deg',0, ...
    'completed',false,'stable_counts',zeros(1,size(wear_cases,1)),'elapsed_s',0),n,1);
idx=0;
for speed=speeds
 for mu=mus
  for steer=steers
   idx=idx+1;
   tag=sprintf('stable_point_%gkph_%.1fmu_%gdeg',speed,mu,steer);
   target=fullfile(outdir,[tag '.mat']);
   cfg=cfg0; P=P0;
   cfg.speed_kmh=speed; cfg.ux=speed/3.6;
   cfg.mu_road=mu; cfg.mu_scale_x=mu/cfg.mu_reference; cfg.mu_scale_y=mu/cfg.mu_reference;
   cfg.steer_deg=[steer steer 0 0];
   cfg.r_limit=cfg.limit_factor*mu*cfg.g/cfg.ux;
   cfg.ay_limit=mu*cfg.g;
   cfg.save_dir=outdir; cfg.final_mat=target;
   cfg.checkpoint_mat=fullfile(cache,[tag '_checkpoint.mat']);
   cfg.raw_mat=fullfile(cache,[tag '_raw.mat']);
   P.mu_scale_x=cfg.mu_scale_x; P.mu_scale_y=cfg.mu_scale_y;
   scenario_signature=single_stability_run_signature(cfg,G,wear_cases,['scenario-' engine]);
   manifest(idx).filename=[tag '.mat'];
   manifest(idx).speed_kmh=speed; manifest(idx).mu_road=mu;
   manifest(idx).steer_deg=steer;
   fprintf('\nSCENARIO %d/%d: %s\n',idx,n,tag);
   began=tic;
   if exist(target,'file')
    loaded=load(target,'STABILITY3D');
    if isfield(loaded,'STABILITY3D') && isfield(loaded.STABILITY3D,'meta') && ...
       isfield(loaded.STABILITY3D.meta,'run_version') && ...
       strcmp(loaded.STABILITY3D.meta.run_version,'mu-sweep-v4-kinematic-initialization')
      S=loaded.STABILITY3D;
      sweep_validate_saved_scenario(S,cfg,G,wear_cases,scenario_signature);
      manifest(idx).completed=true;
      manifest(idx).stable_counts=[S.CASE.stable_count];
      fprintf('Already complete; validated metadata.\n');
      sweep_write_manifest(outdir,manifest);
      continue;
    end
    error('Existing file has an unknown format; refusing to overwrite: %s',target);
   end
   CASE = repmat(sweep_empty_case(),size(wear_cases,1),1);
   for iw=1:size(wear_cases,1)
    wear=wear_cases(iw,:);
    if strcmp(engine,'mcode')
      [stable_mask,reason_mask]=sweep_mcode_masks(cfg,G,wear,P);
      if cfg.early_exit_on_unstable
       reason_semantics='through_first_violation';
      else
       reason_semantics='full_horizon_accumulated';
      end
      run_info=struct('engine','mcode','reason_semantics',reason_semantics);
    else
      error('Unknown engine: %s',engine);
    end
    CASE_RAW=struct('cfg',cfg,'grid',G,'wear',wear, ...
      'stable_mask',stable_mask,'reason_mask',reason_mask,'run_info',run_info, ...
      'scenario_signature',scenario_signature);
    case_raw_path=fullfile(cache,sprintf('%s_wear%d_raw.mat',tag,iw));
    sweep_atomic_mat_save(case_raw_path,'CASE_RAW',CASE_RAW);
    C=sweep_analyze_case(cfg,G,wear,stable_mask,reason_mask,run_info);
    CASE(iw)=C;
    CHECKPOINT=struct('cfg',cfg,'grid_coarse',G,'wear_cases',wear_cases, ...
       'last_completed_case',iw,'CASE',CASE);
    sweep_atomic_mat_save(cfg.checkpoint_mat,'CHECKPOINT',CHECKPOINT);
    fprintf('Wear %d/%d: stable %d/%d; inner volume %.8g; valid %d\n', ...
      iw,size(wear_cases,1),C.stable_count,C.total_count,C.region.inner_volume,C.region.is_valid_inner);
   end
   RAW=struct('cfg',cfg,'grid_coarse',G,'wear_cases',wear_cases,'CASE',CASE);
   sweep_atomic_mat_save(cfg.raw_mat,'RAW',RAW);
   volumes=arrayfun(@(c)c.region.inner_volume,CASE);
   bad=arrayfun(@(c)c.region.unstable_inside_inner,CASE);
   valid=isfinite(volumes)&volumes>0&bad==0;
   if any(valid)
    score=volumes; score(~valid)=inf;
    minv=min(score); candidates=find(abs(score-minv)<=max(1e-12,1e-9*abs(minv)));
    [~,j]=min([CASE(candidates).stable_fraction]); worst=candidates(j);
   else
    [~,worst]=min([CASE.stable_fraction]);
   end
   STABILITY3D=struct();
   STABILITY3D.meta=struct('run_version','mu-sweep-v4-kinematic-initialization','speed_kmh',speed, ...
     'speed_m_s',cfg.ux,'wheel_steer_deg',cfg.steer_deg,'mu_road',mu, ...
     'mu_scale_x',cfg.mu_scale_x,'mu_scale_y',cfg.mu_scale_y, ...
     'mu_reference',cfg.mu_reference,'friction_scaling_mode',cfg.friction_scaling_mode, ...
     'grid_generation',cfg.grid_generation,'initialization_mode',cfg.initialization_mode, ...
     'case_count',size(wear_cases,1),'healthy_only',healthy_only, ...
     'r_limit_rad_s',cfg.r_limit,'ay_limit_m_s2',cfg.ay_limit, ...
     'LTR_limit',cfg.LTR_limit,'limit_factor',cfg.limit_factor, ...
     'k_spring_N_m',cfg.k_spring_N_m,'I_mA',cfg.I_mA, ...
     'engine',engine,'mat_format','MATLAB v7', ...
     'classification_horizon_s',cfg.sim_time,'Ts_s',cfg.Ts,'RK4_substeps',cfg.Nsub, ...
     'reason_mask_semantics','5-second accumulated causes; numerical failures stop', ...
     'definition','Finite-horizon constraint satisfaction, not asymptotic stability', ...
     'friction_note','Relative road scaling mu/0.9; mu=0.9 reproduces original first-code MF calibration, not an absolute force-peak normalization', ...
     'initialization','Kinematic initial slip angles from vc0/r0/wheel steer; per-wheel free rolling speeds; zero relaxed longitudinal slips', ...
     'has_valid_inner_region',any(valid),'completed_utc',datestr(now,30));
   STABILITY3D.meta.signature=scenario_signature;
   STABILITY3D.meta.postprocessing_complete=all(arrayfun(@(c) ...
     isempty(c.postprocess.stable_hull_error) && isempty(c.postprocess.region_error),CASE));
   STABILITY3D.cfg=cfg; STABILITY3D.grid_coarse=G; STABILITY3D.grid_fine=GF;
   STABILITY3D.wear_cases=wear_cases; STABILITY3D.healthy_case_idx=healthy_case_idx;
   STABILITY3D.smallest_region_case_idx=worst;
   STABILITY3D.smallest_region_wear=wear_cases(worst,:);
   STABILITY3D.CASE=CASE; STABILITY3D.CASE_FINE=struct();
   % A struct, not MATLAB table: portable between Octave and MATLAB.
   STABILITY3D.CaseSummary=sweep_make_summary(CASE);
   rankv=volumes; rankv(~isfinite(rankv))=inf;
   [~,order]=sortrows([~valid(:),rankv(:),[CASE.stable_fraction].'],[1 2 3]);
   STABILITY3D.CaseRanking=STABILITY3D.CaseSummary(order);
   sweep_atomic_mat_save(target,'STABILITY3D',STABILITY3D);
   check=load(target,'STABILITY3D');
   sweep_validate_saved_scenario(check.STABILITY3D,cfg,G,wear_cases,scenario_signature);
   manifest(idx).completed=true;
   manifest(idx).stable_counts=[CASE.stable_count];
   manifest(idx).elapsed_s=toc(began);
   sweep_write_manifest(outdir,manifest);
   fprintf('SAVED %s | %.1f s\n',target,manifest(idx).elapsed_s);
  end
 end
end
assert(all([manifest.completed]),'Sweep did not complete');
fprintf('\nALL %d SCENARIOS COMPLETE\n',numel(manifest));
end

function C=sweep_empty_case()
C=struct('wear',[],'grid',[],'stable_mask',[],'reason_mask',[], ...
 'stable_points',[],'unstable_points',[],'total_count',0,'stable_count',0, ...
 'unstable_count',0,'stable_fraction',NaN,'stable_hull_volume',NaN, ...
 'region',sweep_empty_region(),'run_info',struct(), ...
 'postprocess',struct('stable_hull_error','','region_error',''));
end

function C=sweep_analyze_case(cfg,G,wear,mask,reasons,info)
[vc,r,phi]=ndgrid(G.vc,G.r,G.phi); points=[vc(:),r(:),phi(:)];
assert(isequal(size(mask),size(vc)) && isequal(size(reasons),size(vc)));
assert(isequal(mask,reasons==0),'Reason/stable masks disagree');
C=sweep_empty_case(); C.wear=wear; C.grid=G; C.stable_mask=mask; C.reason_mask=reasons;
C.stable_points=points(mask(:),:); C.unstable_points=points(~mask(:),:);
C.total_count=numel(mask); C.stable_count=nnz(mask); C.unstable_count=nnz(~mask);
C.stable_fraction=C.stable_count/C.total_count; C.run_info=info;
try
 if C.stable_count>=4 && rank(C.stable_points-mean(C.stable_points,1))>=3
  [~,C.stable_hull_volume]=convhulln(C.stable_points);
 end
catch ME
 C.postprocess.stable_hull_error=ME.message;
 warning('Stable hull construction failed; raw masks are preserved: %s',ME.message);
end
try
 C.region=region_stability_region(G,mask,C.stable_points,C.unstable_points,cfg);
catch ME
 C.postprocess.region_error=ME.message;
 warning('Region construction failed; raw masks are preserved: %s',ME.message);
end
end

function [mask,reasons]=sweep_mcode_masks(cfg,G,wear,P)
mask=false(numel(G.vc),numel(G.r),numel(G.phi)); reasons=zeros(size(mask),'uint8');
for ip=1:numel(G.phi)
 for ir=1:numel(G.r)
  for iv=1:numel(G.vc)
   [mask(iv,ir,ip),reasons(iv,ir,ip)]=model_fullcar_model('classify',cfg,wear,P,G.vc(iv),G.r(ir),G.phi(ip));
  end
 end
end
end

function S=sweep_make_summary(CASE)
template=struct('Case',0,'Wear_FL',0,'Wear_FR',0,'Wear_RL',0,'Wear_RR',0, ...
 'StableCount',0,'UnstableCount',0,'StableFraction',NaN,'StableHullVolume',NaN, ...
 'OuterVolume',NaN,'InnerVolume',NaN,'OuterBadCount',NaN,'InnerBadCount',NaN);
S=repmat(template,numel(CASE),1);
for i=1:numel(CASE)
 C=CASE(i);
 S(i)=struct('Case',i,'Wear_FL',C.wear(1),'Wear_FR',C.wear(2), ...
 'Wear_RL',C.wear(3),'Wear_RR',C.wear(4),'StableCount',C.stable_count, ...
 'UnstableCount',C.unstable_count,'StableFraction',C.stable_fraction, ...
 'StableHullVolume',C.stable_hull_volume,'OuterVolume',C.region.outer_volume, ...
 'InnerVolume',C.region.inner_volume,'OuterBadCount',C.region.unstable_inside_outer, ...
 'InnerBadCount',C.region.unstable_inside_inner);
end
end

function R=sweep_empty_region()
R=struct('limit_roll_points',zeros(0,3),'outer_limit_points',zeros(0,3), ...
 'inner_limit_points',zeros(0,3),'outer_vertices',zeros(0,3),'inner_vertices',zeros(0,3), ...
 'outer_volume',NaN,'inner_volume',NaN,'unstable_inside_outer',NaN, ...
 'unstable_inside_inner',NaN,'inner_depth',NaN,'is_valid_inner',false, ...
 'vc_split_center',NaN,'used_stable_mask_fallback',false,'used_support_fallback',false);
end

function sweep_validate_saved_scenario(S,cfg,G,wear_cases,signature)
assert(isfield(S.meta,'signature') && strcmp(S.meta.signature,signature), ...
 'Cached scenario provenance mismatch; preserve it elsewhere before rerunning.');
assert(strcmp(S.meta.run_version,'mu-sweep-v4-kinematic-initialization') && ...
 strcmp(S.cfg.initialization_mode,'kinematic_slip_free_roll') && ...
 strcmp(S.meta.initialization_mode,S.cfg.initialization_mode),'Cached initialization mismatch');
assert(S.meta.case_count==size(wear_cases,1) && ...
 S.meta.healthy_only==(size(wear_cases,1)==1 && all(wear_cases(1,:)==0)), ...
 'Cached selected-wear metadata mismatch');
expected_healthy=find(all(wear_cases==0,2),1);
if isempty(expected_healthy), expected_healthy=NaN; end
assert(isequaln(S.healthy_case_idx,expected_healthy),'Cached healthy case index mismatch');
assert(S.meta.speed_kmh==cfg.speed_kmh && S.meta.mu_road==cfg.mu_road && ...
 isequal(S.meta.wheel_steer_deg,cfg.steer_deg),'Cached metadata mismatch');
assert(isequal(S.grid_coarse,G) && isequal(S.wear_cases,wear_cases),'Cached grid/wear mismatch');
assert(S.cfg.sim_time==cfg.sim_time && S.cfg.Ts==cfg.Ts && S.cfg.Nsub==cfg.Nsub, ...
 'Cached simulation timing mismatch');
assert(numel(S.CASE)==size(wear_cases,1),'Incomplete cached scenario');
[vc,r,phi]=ndgrid(G.vc,G.r,G.phi); points=[vc(:),r(:),phi(:)];
n=numel(vc);
for iw=1:size(wear_cases,1)
 C=S.CASE(iw);
 assert(isequal(C.wear,wear_cases(iw,:)) && isequal(C.grid,G),'Cached case inputs mismatch');
 assert(isequal(size(C.stable_mask),size(vc)) && isequal(size(C.reason_mask),size(vc)), ...
  'Cached mask dimensions mismatch');
 assert(all(C.reason_mask(:)<=31) && isequal(C.stable_mask,C.reason_mask==0), ...
  'Cached reason/stable masks mismatch');
 assert(C.total_count==n && C.stable_count==nnz(C.stable_mask) && ...
  C.unstable_count==nnz(~C.stable_mask) && C.stable_count+C.unstable_count==n, ...
  'Cached classification counts mismatch');
 assert(C.stable_fraction==C.stable_count/n,'Cached stable fraction mismatch');
 assert(isequal(C.stable_points,points(C.stable_mask(:),:)) && ...
  isequal(C.unstable_points,points(~C.stable_mask(:),:)),'Cached point/mask mismatch');
end
assert(numel(S.CaseSummary)==size(wear_cases,1) && numel(S.CaseRanking)==size(wear_cases,1), ...
 'Incomplete cached summary/ranking');
end

function value=sweep_get_option(options,name,default)
if isfield(options,name), value=options.(name); else, value=default; end
end

function sweep_atomic_mat_save(path,name,value)
S=struct(); S.(name)=value; tmp=[path '.tmp'];
save(tmp,'-struct','S','-v7');
[ok,msg]=movefile(tmp,path,'f'); assert(ok,msg);
end

function sweep_write_manifest(outdir,manifest)
sweep_atomic_mat_save(fullfile(outdir,'sweep_manifest.mat'),'manifest',manifest);
fid=fopen(fullfile(outdir,'sweep_manifest.csv.tmp'),'w'); assert(fid>=0);
fprintf(fid,'filename,speed_kmh,mu_road,front_steer_deg,completed');
nwear=numel(manifest(1).stable_counts);
for iw=1:nwear, fprintf(fid,',stable_wear%d',iw); end
fprintf(fid,',elapsed_s\n');
for i=1:numel(manifest)
 m=manifest(i); if isempty(m.filename),continue; end
 fprintf(fid,'%s,%g,%.1f,%g,%d',m.filename,m.speed_kmh,m.mu_road,m.steer_deg,m.completed);
 fprintf(fid,',%d',m.stable_counts);
 fprintf(fid,',%.6f\n',m.elapsed_s);
end
fclose(fid); movefile(fullfile(outdir,'sweep_manifest.csv.tmp'),fullfile(outdir,'sweep_manifest.csv'),'f');
end

function varargout = model_fullcar_model(action,varargin)
% Original full-car equations with explicit finite-state failure handling.
% All units and full-resolution defaults match the supplied second code.
switch action
 case 'defaults'
  [varargout{1:nargout}] = model_model_defaults();
 case 'initial'
  varargout{1} = model_initialize_fullcar_state(varargin{:});
 case 'classify'
  [varargout{1:nargout}] = model_simulate_and_classify_mcode(varargin{:});
 case 'params'
  varargout{1} = model_default_fullcar_params();
 case 'wheelrates'
  varargout{1} = model_spring_to_wheel_rate(varargin{:});
 case 'observe'
  varargout{1} = model_observe_fullcar_stability(varargin{:});
 case 'rhs'
  varargout{1} = model_fullcar_rhs_wrapper_stability(varargin{:});
 case 'step'
  varargout{1} = model_integrate_fullcar_step_stability(varargin{:});
 case 'tire'
  [varargout{1:nargout}] = model_calc_combined_tire_force_scalar(varargin{:});
 otherwise
  error('Unknown fullcar_model action: %s',action);
end
end

function [cfg,gridC,wear_cases,P,gridF] = model_model_defaults()
cfg.mu_road = 0.9;
cfg.initialization_mode = 'kinematic_slip_free_roll';
cfg.grid_generation = 'symmetric_matlab_compatible';
cfg.mu_reference = 0.9; % Original first-code tire calibration is the baseline.
cfg.friction_scaling_mode = 'relative_to_original_mu0p9';
cfg.mu_scale_x = cfg.mu_road/cfg.mu_reference;
cfg.mu_scale_y = cfg.mu_road/cfg.mu_reference;

% Keep output for each mu separate to prevent overwriting existing MAT data.
cfg.save_dir = fullfile(fileparts(mfilename('fullpath')),'results_single_mfile');

if ~exist(cfg.save_dir,'dir')

    mkdir(cfg.save_dir);

end

cfg.final_mat = fullfile(cfg.save_dir, 'stable_point.mat');

cfg.checkpoint_mat = fullfile(cfg.save_dir, 'stable_point_checkpoint.mat');

% Safety save written immediately after the expensive state sweep.
% Even if region construction/ranking later fails, the Stable/Unstable data
% can be reused without repeating all 39,401 simulations.
cfg.raw_mat = fullfile(cfg.save_dir, 'stable_point_raw.mat');

% Fixed operating condition

cfg.speed_kmh = 80.0;

cfg.ux = cfg.speed_kmh/3.6;              % [m/s]

cfg.steer_deg = [2.0 2.0 0.0 0.0];      % [FL FR RL RR] [deg]

% Full-car execution settings: same as supplied MATLAB Function

cfg.Ts = 0.01;                           % [s]

cfg.sim_time = 5.0;                     % [s]

cfg.Nsub = 20;                           % RK4 substeps per 0.01 s

% Suspension settings used during stability-region generation.

% IMPORTANT: change these if your Simulink Constants are different.

cfg.k_spring_N_m = [30790 30790 30790 30790]; % [FL FR RL RR] [N/m]

cfg.I_mA = [950 950 950 950];                    % CDC current [mA]

% Stability limits

cfg.g = 9.81;

cfg.limit_factor = 0.85;  % paper: used for yaw-rate limit only

cfg.r_limit = cfg.limit_factor*cfg.mu_road*cfg.g/cfg.ux; % [rad/s]

cfg.ay_limit = cfg.mu_road*cfg.g;                         % [m/s^2], paper Sect. 3.1

cfg.LTR_limit = 1.0;                                      % rollover boundary, paper Sect. 3.1

% Numerical guards (not physical stability-region boundaries)

cfg.phi_hard_limit_rad = deg2rad(40);

cfg.vc_hard_limit_m_s = 30.0;

cfg.r_hard_limit_rad_s = 2.0;

cfg.ignore_initial_s = 0.00;

% When false, all 5 s are evaluated so reason-bit diagnostics match the

% previous Simulink sweep. Set true only if speed is more important than

% counting all simultaneous violation reasons.

cfg.early_exit_on_unstable = false;

% -------------------------------------------------------------------------

% Main sweep grid: 41 x 31 x 31 = 39,401 initial states / wear case

% -------------------------------------------------------------------------

gridC.vc  = (-20:20)*0.5;

gridC.r   = (-15:15)*(0.8/30);

gridC.phi = (-15:15)*(0.8/30);

% Optional fine re-run after the smallest-volume wear case is found.

% Leave false for the present diagnostic stage.

cfg.refine_selected_cases = false;

gridF.vc  = (-4:4)*0.5;

gridF.r   = (-4:4)*(0.7/8);

gridF.phi = (-4:4)*(0.4/8);

% Paper-method region extraction

cfg.hull_inside_tol = 1e-10;

cfg.paper_vc_split_tol = 1e-12;

cfg.require_inner_no_unstable = true;

% Robust paper-method post-processing.
% depth=1 : outermost stable lateral-speed layer
% depth=2 : first inward layer (default starting point for INNER region)
% depth=3,4,and onward : progressively farther inward until no sampled unstable
%                 point remains inside the candidate hexahedron.
cfg.inner_depth_start = 2;
cfg.allow_stable_mask_fallback = true;
cfg.min_limit_points = 8;
cfg.max_support_depth = 12;

% NOTE:
% The vehicle sweep and the region extraction are separated logically.
% The sweep is expensive; therefore raw classification data are always saved
% before final region ranking/plotting.

% Plot

cfg.plot_stable_points = false;

cfg.save_plot = false;

%% ========================================================================

% 2. FIVE REPRESENTATIVE TIRE-WEAR CASES

% ========================================================================

wear_cases = [
0 0 0 0;
25 25 0 0;
50 50 25 25;
0 0 25 25;
25 25 50 50
];

nWear = size(wear_cases,1);

healthy_idx = find(all(wear_cases==0,2),1,'first');

P = model_default_fullcar_params();
P.mu_scale_x = cfg.mu_scale_x;
P.mu_scale_y = cfg.mu_scale_y;


end

function C = model_empty_case_result()

    C = struct( 'wear',[], 'grid',[], 'stable_mask',[], 'reason_mask',[], 'stable_points',[], 'unstable_points',[], 'total_count',0, 'stable_count',0, 'unstable_count',0, 'stable_fraction',NaN, 'stable_hull_volume',NaN, 'region',model_empty_region());

end

function R = model_empty_region()

    R = struct( 'limit_roll_points',zeros(0,3), 'outer_limit_points',zeros(0,3), 'inner_limit_points',zeros(0,3), 'outer_vertices',zeros(0,3), 'inner_vertices',zeros(0,3), 'outer_volume',NaN, 'inner_volume',NaN, 'unstable_inside_outer',NaN, 'unstable_inside_inner',NaN, 'inner_depth',NaN, 'is_valid_inner',false, 'vc_split_center',NaN, 'used_stable_mask_fallback',false, 'used_support_fallback',false);

end

function C = model_evaluate_wear_case_mcode(cfg,G,wear,P)

    [VC,R,PHI] = ndgrid(G.vc,G.r,G.phi);

    total = numel(VC);

    stable_mask = false(size(VC));

    reason_mask = zeros(size(VC),'uint8');

    count = 0;

    for iv = 1:numel(G.vc)

        for ir = 1:numel(G.r)

            for ip = 1:numel(G.phi)

                count = count + 1;

                vc0 = G.vc(iv);

                r0 = G.r(ir);

                phi0 = G.phi(ip);

                fprintf('  %5d/%5d | vc0=%+7.3f | r0=%+7.4f | phi0=%+7.4f\r', count,total,vc0,r0,phi0);

                [isStable,code] = model_simulate_and_classify_mcode( cfg,wear,P,vc0,r0,phi0);

                stable_mask(iv,ir,ip) = isStable;

                reason_mask(iv,ir,ip) = code;

            end

        end

    end

    fprintf('\n');

    all_points = [VC(:),R(:),PHI(:)];

    stable_points = all_points(stable_mask(:),:);

    unstable_points = all_points(~stable_mask(:),:);

    % -------------------------------------------------------------

    % Diagnostic 1: classification by vc0

    % reason bits: 1 yaw, 2 ay, 4 rollover/LTR, 8 numerical, 16 hard guard

    % -------------------------------------------------------------

    fprintf('\n');

    fprintf('============================================================\n');

    fprintf('Stability classification by vc0\n');

    fprintf('============================================================\n');

    fprintf('     vc0 | Stable/Total |  Yaw |   Ay |  LTR | Num | Hard\n');

    fprintf('------------------------------------------------------------\n');

    for iv = 1:numel(G.vc)

        codes = squeeze(reason_mask(iv,:,:));

        nTotal = numel(codes);

        nStable = nnz(codes==0);

        nYaw = nnz(bitand(codes,uint8(1))~=0);

        nAy = nnz(bitand(codes,uint8(2))~=0);

        nRoll = nnz(bitand(codes,uint8(4))~=0);

        nNum = nnz(bitand(codes,uint8(8))~=0);

        nHard = nnz(bitand(codes,uint8(16))~=0);

        fprintf('%+8.3f | %3d / %3d    | %4d | %4d | %4d | %3d | %4d\n', G.vc(iv),nStable,nTotal,nYaw,nAy,nRoll,nNum,nHard);

    end

    fprintf('============================================================\n');

    % -------------------------------------------------------------

    % Diagnostic 2: overall reason summary

    % -------------------------------------------------------------

    codes_all = reason_mask(:);

    fprintf('\n--- Overall stability reason summary ---\n');

    fprintf('Stable             : %d / %d\n',nnz(codes_all==0),numel(codes_all));

    fprintf('Yaw violation      : %d\n',nnz(bitand(codes_all,uint8(1))~=0));

    fprintf('Ay violation       : %d\n',nnz(bitand(codes_all,uint8(2))~=0));

    fprintf('Rollover / LTR     : %d\n',nnz(bitand(codes_all,uint8(4))~=0));

    fprintf('Numerical failure  : %d\n',nnz(bitand(codes_all,uint8(8))~=0));

    fprintf('Hard guard         : %d\n',nnz(bitand(codes_all,uint8(16))~=0));

    % -------------------------------------------------------------

    % Diagnostic 3: stable-point geometry

    % -------------------------------------------------------------

    fprintf('\n--- Stable-point diagnostics ---\n');

    if isempty(stable_points)

        fprintf('No stable points were found.\n');

        stable_rank = 0;

    else

        fprintf('Stable vc values:\n');

        disp(unique(stable_points(:,1)).');

        fprintf('Stable r values:\n');

        disp(unique(stable_points(:,2)).');

        fprintf('Stable phi values [rad]:\n');

        disp(unique(stable_points(:,3)).');

        fprintf('Stable phi values [deg]:\n');

        disp(rad2deg(unique(stable_points(:,3))).');

        if size(stable_points,1)>=2

            stable_rank = rank(stable_points-mean(stable_points,1));

        else

            stable_rank = 0;

        end

        fprintf('Stable-point rank = %d\n',stable_rank);

    end

    stable_hull_volume = NaN;

    if size(stable_points,1)>=4 && rank(stable_points-mean(stable_points,1))>=3

        try

            [~,stable_hull_volume] = convhulln(stable_points);

        catch

            stable_hull_volume = NaN;

        end

    end

    % Region construction is post-processing.  Never discard the expensive
    % classification result because of a geometry error.
    try
        region = model_build_paper_region( G,stable_mask,stable_points,unstable_points,cfg);
    catch ME
        warning('Region construction failed after sweep: %s',ME.message);
        region = model_empty_region();
    end

    fprintf('Paper points: limit-roll=%d, outer-limit=%d, inner-limit=%d\n', size(region.limit_roll_points,1), size(region.outer_limit_points,1), size(region.inner_limit_points,1));

    C = model_empty_case_result();

    C.wear = wear;

    C.grid = G;

    C.stable_mask = stable_mask;

    C.reason_mask = reason_mask;

    C.stable_points = stable_points;

    C.unstable_points = unstable_points;

    C.total_count = total;

    C.stable_count = size(stable_points,1);

    C.unstable_count = size(unstable_points,1);

    C.stable_fraction = C.stable_count/max(total,1);

    C.stable_hull_volume = stable_hull_volume;

    C.region = region;

end

function x = model_initialize_fullcar_state(cfg,P,vc0,r0,phi0)
% Initialize relaxed slips consistently with the initial wheel-frame velocity.
x = zeros(32,1);
x(1) = vc0;
x(2) = r0;
x(4) = phi0;
delta_w = deg2rad(cfg.steer_deg(:));
[~,alpha0,u_wheel0] = model_calculate_internal_slips( ...
    x,delta_w,cfg.ux,P.lf,P.lr,P.tf,P.tr,P.Rw);
x(21:24) = alpha0;
x(25:28) = max(u_wheel0,0.0)/P.Rw;
x(17:20) = 0;
end

function [isStable,reason] = model_simulate_and_classify_mcode(cfg,wear,P,vc0,r0,phi0)

    delta_w = deg2rad(cfg.steer_deg(:));

    wear_cmd = wear(:);

    Icmd = cfg.I_mA(:);

    kw = model_spring_to_wheel_rate(cfg.k_spring_N_m,P);

    x = model_initialize_fullcar_state(cfg,P,vc0,r0,phi0);

    reason = uint8(0);

    N = round(cfg.sim_time/cfg.Ts)+1;

    for k=1:N

        t = (k-1)*cfg.Ts;

        if any(~isfinite(x))
            reason = bitor(reason,uint8(8));
            break;
        end

        % Same timing as the MATLAB Function block: observe current state first,

        % then integrate to the next 0.01-s sample.

        O = model_observe_fullcar_stability(x,delta_w,cfg.ux,wear_cmd,Icmd,kw,P);

        if t >= cfg.ignore_initial_s

            if abs(O.r_rad_s) > cfg.r_limit

                reason = bitor(reason,uint8(1));

            end

            if abs(O.ay_m_s2) > cfg.ay_limit

                reason = bitor(reason,uint8(2));

            end

            if abs(O.LTR) >= cfg.LTR_limit

                reason = bitor(reason,uint8(4));

            end

            finite_ok = isfinite(O.r_rad_s) && isfinite(O.phi_rad) && isfinite(O.vc) && isfinite(O.ay_m_s2) && isfinite(O.LTR) && all(isfinite(O.Fz_raw_N));

            if ~finite_ok

                reason = bitor(reason,uint8(8));

            end

            if abs(O.phi_rad)>cfg.phi_hard_limit_rad || abs(O.vc)>cfg.vc_hard_limit_m_s || abs(O.r_rad_s)>cfg.r_hard_limit_rad_s

                reason = bitor(reason,uint8(16));

            end

            if cfg.early_exit_on_unstable && reason~=0

                break;

            end

        end

        if k<N

            x = model_integrate_fullcar_step_stability( x,delta_w,cfg.ux,wear_cmd,Icmd,kw,P,cfg.Ts,cfg.Nsub);

        end

    end

    isStable = (reason==0);

end

%% ========================================================================

% LOCAL FUNCTIONS: FULL-CAR WRAPPERS / PARAMETERS

% ========================================================================

function P = model_default_fullcar_params()

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

function kw = model_spring_to_wheel_rate(kSpring,P)

    kw = [kSpring(1)*P.MRf^2; kSpring(2)*P.MRf^2; kSpring(3)*P.MRr^2; kSpring(4)*P.MRr^2];

    for i=1:4

        if ~isfinite(kw(i)) || kw(i)<1000.0

            kw(i)=30000.0;

        end

    end

end

function O = model_observe_fullcar_stability(x,delta_w,ux,wear_cmd,Icmd,kw,P)

    O = struct();

    O.vc = x(1);

    O.r_rad_s = x(2);

    O.phi_rad = x(4);

    [Fz,~,Fz_raw] = model_calculate_internal_fz( x,P.Fz_static,P.kt,P.Fz_force_min,P.Fz_force_max, P.m,P.ms,P.h_cg,P.h_roll,kw,P.tf,P.tr,P.Karb_f,P.Karb_r);

    kappa_eff = x(17:20);

    alpha_eff = x(21:24);

    [~,Fy_body,~,~] = model_calculate_tire_forces( delta_w,kappa_eff,Fz,alpha_eff,wear_cmd,P.Fz0_vec,P.Fz_force_min,P.mu_scale_x,P.mu_scale_y);

    O.ay_m_s2 = sum(Fy_body)/P.m;

    O.Fz_raw_N = Fz_raw;

    % Paper Sect. 3.1: LTR = (FzL-FzR)/(FzL+FzR)

    FzL = Fz_raw(1) + Fz_raw(3);  % FL + RL

    FzR = Fz_raw(2) + Fz_raw(4);  % FR + RR

    den = FzL + FzR;

    if abs(den) < 1e-9

        if abs(FzL-FzR) < 1e-9

            O.LTR = 0.0;

        else

            O.LTR = sign(FzL-FzR)*Inf;

        end

    else

        O.LTR = (FzL-FzR)/den;

    end

end

function xnext = model_integrate_fullcar_step_stability( x,delta_w,ux,wear_cmd,Icmd,kw,P,Ts,Nsub)

    Nsub = max(1,round(Nsub));

    dt = Ts/double(Nsub);

    for n=1:Nsub

        k1 = model_fullcar_rhs_wrapper_stability(x,delta_w,ux,wear_cmd,Icmd,kw,P);

        k2 = model_fullcar_rhs_wrapper_stability(x+0.5*dt*k1,delta_w,ux,wear_cmd,Icmd,kw,P);

        k3 = model_fullcar_rhs_wrapper_stability(x+0.5*dt*k2,delta_w,ux,wear_cmd,Icmd,kw,P);

        k4 = model_fullcar_rhs_wrapper_stability(x+dt*k3,delta_w,ux,wear_cmd,Icmd,kw,P);

        x = x + dt*(k1+2.0*k2+2.0*k3+k4)/6.0;

    end

    % Same numerical protection as supplied Full-car MATLAB Function.

    if any(~isfinite(x))
        xnext = x;
        return;
    end

    for i=17:20

        if x(i)<-0.5; x(i)=-0.5; end

        if x(i)> 0.5; x(i)= 0.5; end

    end

    for i=21:24

        if x(i)<-1.2; x(i)=-1.2; end

        if x(i)> 1.2; x(i)= 1.2; end

    end

    omega_max = 300.0;

    for i=25:28

        if x(i)<0.0; x(i)=0.0; end

        if x(i)>omega_max; x(i)=omega_max; end

    end

    xnext = x;

end

function dx = model_fullcar_rhs_wrapper_stability(x,delta_w,ux,wear_cmd,Icmd,kw,P)

    dx = model_fullcar_derivative_internal( x,delta_w,ux,wear_cmd,Icmd, P.m,P.ms,P.Iz,P.Ixx,P.Iyy,P.lf,P.lr,P.tf,P.tr, P.h_roll,P.h_cg,P.g,P.Rw,P.Iw,P.tau_load, P.Fz_static,P.Fz0_vec,P.Fz_force_min,P.Fz_force_max, P.sigma_kappa,P.sigma_alpha,kw,P.MR_vec,P.mu_u,P.kt, P.Karb_f,P.Karb_r,P.Carb_f,P.Carb_r,P.mu_scale_x,P.mu_scale_y);

end

%% ========================================================================

% LOCAL FUNCTIONS: PAPER SECT. 3.2 3-D REGION

% ========================================================================

function region = model_build_paper_region(G,stable_mask,stable_points,unstable_points,cfg)

    % Robust reconstruction of the paper sequence:
    % Stable -> limit-roll -> lateral-speed layers -> eight vertices
    % -> inner/outer hexahedra.
    %
    % Difference from the previous implementation:
    % The old code selected only one "inner" layer.  In the last run that
    % hexahedron contained 7 sampled unstable points, so the script stopped.
    % Here the inner layer is moved inward discretely (depth=2,3,4,and onward) and
    % the largest candidate containing ZERO sampled unstable points is kept.

    region = model_empty_region();

    if isempty(stable_points) || size(stable_points,1)<8
        return;
    end

    if rank(stable_points-mean(stable_points,1))<3
        return;
    end

    % ---------------------------------------------------------------------
    % STEP 1: LIMIT-ROLL STABLE POINTS
    % For each fixed (vc,r), take the minimum/maximum stable phi.
    % ---------------------------------------------------------------------
    limit_roll = zeros(0,3);

    for iv=1:numel(G.vc)
        for ir=1:numel(G.r)

            idxPhi = find(squeeze(stable_mask(iv,ir,:)));

            if isempty(idxPhi)
                continue;
            end

            ipLo = idxPhi(1);
            ipHi = idxPhi(end);

            limit_roll(end+1,:) = [G.vc(iv),G.r(ir),G.phi(ipLo)]; %#ok<AGROW>

            if ipHi~=ipLo
                limit_roll(end+1,:) = [G.vc(iv),G.r(ir),G.phi(ipHi)]; %#ok<AGROW>
            end
        end
    end

    limit_roll = unique(limit_roll,'rows','stable');
    region.limit_roll_points = limit_roll;

    if size(limit_roll,1)<cfg.min_limit_points
        return;
    end

    % Keep this diagnostic center for compatibility with older MAT files.
    region.vc_split_center = median(stable_points(:,1));

    % ---------------------------------------------------------------------
    % STEP 2A: OUTER LATERAL-SPEED LIMIT
    % depth=1 means the outermost stable vc point on both sides.
    % ---------------------------------------------------------------------
    outer_limit = model_lateral_layer_from_limit_roll(limit_roll,G,1);
    usedFallback = false;

    if size(outer_limit,1)<cfg.min_limit_points && cfg.allow_stable_mask_fallback

        outer_limit = model_lateral_layer_from_stable_mask(G,stable_mask,1);
        usedFallback = true;
    end

    outer_limit = unique(outer_limit,'rows','stable');
    region.outer_limit_points = outer_limit;
    region.used_stable_mask_fallback = usedFallback;

    if size(outer_limit,1)<cfg.min_limit_points
        return;
    end

    Vouter = model_select_ordered_eight_vertices(outer_limit,G,stable_points);

    if size(Vouter,1)~=8
        return;
    end

    [outerVol,okOuter] = model_convex_volume(Vouter);

    if ~okOuter
        return;
    end

    insideOuter = model_points_inside_convex_hull( unstable_points,Vouter,cfg.hull_inside_tol);

    region.outer_vertices = Vouter;
    region.outer_volume = outerVol;
    region.unstable_inside_outer = nnz(insideOuter);

    % ---------------------------------------------------------------------
    % STEP 2B / STEP 3: INNER LATERAL-SPEED LAYER SEARCH
    %
    % depth=1 : outermost layer
    % depth=2 : one stable layer inward
    % depth=3 : two stable layers inward
    % and onward
    %
    % Among all candidate hexahedra containing zero sampled unstable points,
    % keep the LARGEST volume.  Thus we do not shrink more than necessary.
    % ---------------------------------------------------------------------
    maxDepth = max(1,ceil(numel(G.vc)/2));

    bestVol = -Inf;
    best = struct( 'limit',zeros(0,3), 'V',zeros(0,3), 'bad',inf, 'depth',NaN, 'fallback',false);

    for depth = cfg.inner_depth_start:maxDepth

        inner_limit = model_lateral_layer_from_limit_roll(limit_roll,G,depth);
        thisFallback = false;

        if size(inner_limit,1)<cfg.min_limit_points && cfg.allow_stable_mask_fallback

            inner_limit = model_lateral_layer_from_stable_mask( G,stable_mask,depth);
            thisFallback = true;
        end

        inner_limit = unique(inner_limit,'rows','stable');

        if size(inner_limit,1)<cfg.min_limit_points
            continue;
        end

        Vin = model_select_ordered_eight_vertices( inner_limit,G,stable_points);

        if size(Vin,1)~=8
            continue;
        end

        [v,ok] = model_convex_volume(Vin);

        if ~ok || ~(isfinite(v) && v>0)
            continue;
        end

        bad = nnz(model_points_inside_convex_hull( unstable_points,Vin,cfg.hull_inside_tol));

        fprintf('  Inner search depth=%2d | points=%4d | volume=%10.6g | bad=%d\n', depth,size(inner_limit,1),v,bad);

        if bad==0 && v>bestVol
            bestVol = v;
            best.limit = inner_limit;
            best.V = Vin;
            best.bad = bad;
            best.depth = depth;
            best.fallback = thisFallback;
        end
    end

    % If the outer layer itself is already safe, allow it as a valid region.
    if isempty(best.V) && region.unstable_inside_outer==0
        bestVol = outerVol;
        best.limit = outer_limit;
        best.V = Vouter;
        best.bad = 0;
        best.depth = 1;
        best.fallback = usedFallback;
    end

    % ---------------------------------------------------------------------
    % LAST-RESORT GEOMETRIC FALLBACK
    % Search directly among saved stable points with progressively inward
    % support points.  No vehicle simulation is rerun.
    % ---------------------------------------------------------------------
    if isempty(best.V)

        [Vs,bads,vols,depths] = model_support_depth_search( stable_points,unstable_points,G,cfg);

        if ~isempty(Vs)
            [bestVol,jj] = max(vols);

            best.V = Vs{jj};
            best.bad = bads(jj);
            best.depth = depths(jj);
            best.limit = stable_points;
            best.fallback = true;

            region.used_support_fallback = true;
        end
    end

    if isempty(best.V)

        % Important: do not throw an error here.
        % The expensive sweep data must still be returned and saved.
        warning(['No zero-unstable inner hexahedron was found from the ' 'current sampled grid. Raw Stable/Unstable data will ' 'still be saved for post-processing.']);
        return;
    end

    region.inner_limit_points = best.limit;
    region.inner_vertices = best.V;
    region.inner_volume = bestVol;
    region.unstable_inside_inner = best.bad;
    region.inner_depth = best.depth;
    region.is_valid_inner = (best.bad==0);
    region.used_stable_mask_fallback = region.used_stable_mask_fallback || best.fallback;

end

function Pts = model_lateral_layer_from_limit_roll(limit_roll,G,depth)

    % At equal (r,phi), sort limit-roll points by vc.
    % Select the depth-th point from the negative/left and positive/right
    % lateral-speed sides.

    Pts = zeros(0,3);
    tol = 1e-12;

    for ir=1:numel(G.r)

        rr = G.r(ir);

        for ip=1:numel(G.phi)

            ph = G.phi(ip);

            idx = abs(limit_roll(:,2)-rr)<=tol & abs(limit_roll(:,3)-ph)<=tol;

            Q = sortrows(limit_roll(idx,:),1);
            n = size(Q,1);

            if n < depth
                continue;
            end

            Pts(end+1,:) = Q(depth,:); %#ok<AGROW>

            j = n-depth+1;

            if j~=depth && j>=1 && j<=n
                Pts(end+1,:) = Q(j,:); %#ok<AGROW>
            end
        end
    end

    Pts = unique(Pts,'rows','stable');

end

function Pts = model_lateral_layer_from_stable_mask(G,stable_mask,depth)

    % Fallback for a discrete grid:
    % At each fixed (r,phi), select the depth-th stable vc point from both
    % sides using the already-computed stable_mask.

    Pts = zeros(0,3);

    for ir=1:numel(G.r)
        for ip=1:numel(G.phi)

            idxVc = find(squeeze(stable_mask(:,ir,ip)));
            n = numel(idxVc);

            if n < depth
                continue;
            end

            iL = idxVc(depth);
            iR = idxVc(n-depth+1);

            Pts(end+1,:) = [G.vc(iL),G.r(ir),G.phi(ip)]; %#ok<AGROW>

            if iR~=iL
                Pts(end+1,:) = [G.vc(iR),G.r(ir),G.phi(ip)]; %#ok<AGROW>
            end
        end
    end

    Pts = unique(Pts,'rows','stable');

end

function V = model_select_ordered_eight_vertices(Pts,G,stable_points)

    % Eight normalized support directions in [vc,r,phi].
    % Returned order:
    % 1 [- - -], 2 [- - +], 3 [- + -], 4 [- + +]
    % 5 [+ - -], 6 [+ - +], 7 [+ + -], 8 [+ + +]

    V = zeros(0,3);

    if size(Pts,1)<8
        return;
    end

    ranges = [ model_range_nonzero(G.vc), model_range_nonzero(G.r), model_range_nonzero(G.phi)];

    c = median(stable_points,1);
    Q = (Pts-c)./ranges;

    signs = [ -1 -1 -1; -1 -1 +1; -1 +1 -1; -1 +1 +1; +1 -1 -1; +1 -1 +1; +1 +1 -1; +1 +1 +1];

    chosen = false(size(Pts,1),1);
    V = nan(8,3);

    for k=1:8

        s = signs(k,:);

        % Prefer a point in the corresponding octant.
        oct = true(size(Pts,1),1);

        for j=1:3
            if s(j)<0
                oct = oct & (Q(:,j)<=1e-12);
            else
                oct = oct & (Q(:,j)>=-1e-12);
            end
        end

        score = Q*s(:);
        score(chosen) = -Inf;
        score(~oct) = -Inf;

        [best,idx] = max(score);

        % If the exact octant has no unused point, use the best unused point
        % along the same normalized support direction.
        if ~isfinite(best)
            score = Q*s(:);
            score(chosen) = -Inf;
            [best,idx] = max(score);
        end

        if ~isfinite(best)
            V = zeros(0,3);
            return;
        end

        V(k,:) = Pts(idx,:);
        chosen(idx) = true;
    end

    if size(unique(V,'rows'),1)~=8
        V = zeros(0,3);
    end

end

function [Vcells,badList,volList,depthList] = model_support_depth_search( stable_points,unstable_points,G,cfg)

    Vcells = {};
    badList = [];
    volList = [];
    depthList = [];

    if size(stable_points,1)<8
        return;
    end

    ranges = [ model_range_nonzero(G.vc), model_range_nonzero(G.r), model_range_nonzero(G.phi)];

    c = median(stable_points,1);
    Q = (stable_points-c)./ranges;

    signs = [ -1 -1 -1; -1 -1 +1; -1 +1 -1; -1 +1 +1; +1 -1 -1; +1 -1 +1; +1 +1 -1; +1 +1 +1];

    order = cell(8,1);

    for k=1:8
        score = Q*signs(k,:).';
        [~,order{k}] = sort(score,'descend');
    end

    maxDepth = min( cfg.max_support_depth, max(1,floor(size(stable_points,1)/8)));

    for depth=1:maxDepth

        used = false(size(stable_points,1),1);
        V = nan(8,3);
        ok = true;

        for k=1:8

            ord = order{k};
            pick = [];

            for jj=depth:numel(ord)

                ii = ord(jj);

                if ~used(ii)
                    pick = ii;
                    break;
                end
            end

            if isempty(pick)
                ok = false;
                break;
            end

            V(k,:) = stable_points(pick,:);
            used(pick) = true;
        end

        if ~ok || size(unique(V,'rows'),1)~=8
            continue;
        end

        [vol,okVol] = model_convex_volume(V);

        if ~okVol
            continue;
        end

        bad = nnz(model_points_inside_convex_hull( unstable_points,V,cfg.hull_inside_tol));

        fprintf('  Support fallback depth=%2d | volume=%10.6g | bad=%d\n', depth,vol,bad);

        if bad==0
            Vcells{end+1,1} = V; %#ok<AGROW>
            badList(end+1,1) = bad; %#ok<AGROW>
            volList(end+1,1) = vol; %#ok<AGROW>
            depthList(end+1,1) = depth; %#ok<AGROW>
        end
    end

end

function y = model_range_nonzero(x)

    y = max(x)-min(x);

    if ~(isfinite(y) && y>0)
        y = 1;
    end

end

function V = model_select_eight_hexahedron_vertices(Pts,G)

    V = zeros(0,3);

    if size(Pts,1)<8

        return;

    end

    scale = [max(max(G.vc)-min(G.vc),eps), max(max(G.r)-min(G.r),eps), max(max(G.phi)-min(G.phi),eps)];

    vcMid = median(Pts(:,1));

    leftIdx = find(Pts(:,1)<=vcMid);

    rightIdx = find(Pts(:,1)>=vcMid);

    if numel(leftIdx)<4 || numel(rightIdx)<4

        % Fallback: split by sorted lateral speed into two overlapping halves.

        [~,ord] = sort(Pts(:,1));

        n = numel(ord);

        leftIdx = ord(1:max(4,ceil(n/2)));

        rightIdx = ord(min(n-3,floor(n/2)+1):n);

    end

    VL = model_select_four_rphi_extremes(Pts(leftIdx,:),scale);

    VR = model_select_four_rphi_extremes(Pts(rightIdx,:),scale);

    V = unique([VL;VR],'rows','stable');

    if size(V,1)<8

        % Fill deterministically using points farthest from the current center.

        c = median(Pts,1);

        d2 = sum(((Pts-c)./scale).^2,2);

        [~,ord] = sort(d2,'descend');

        for ii=ord(:).'

            p = Pts(ii,:);

            if ~ismember(p,V,'rows')

                V(end+1,:) = p; %#ok<AGROW>

            end

            if size(V,1)>=8

                break;

            end

        end

    end

    if size(V,1)>8

        V = model_reduce_to_max_volume_eight(V);

    end

end

function V4 = model_select_four_rphi_extremes(Pts,scale)

    V4 = zeros(0,3);

    if isempty(Pts)

        return;

    end

    c = median(Pts,1);

    Q = (Pts-c)./scale;

    dirs = [-1 -1; -1 +1; +1 -1; +1 +1];

    chosen = zeros(0,1);

    for k=1:4

        score = Q(:,2)*dirs(k,1) + Q(:,3)*dirs(k,2);

        [~,ord] = sort(score,'descend');

        pick = [];

        for jj=ord(:).'

            if ~ismember(jj,chosen)

                pick=jj;

                break;

            end

        end

        if isempty(pick)

            pick=ord(1);

        end

        chosen(end+1,1)=pick; %#ok<AGROW>

    end

    V4 = unique(Pts(chosen,:),'rows','stable');

    if size(V4,1)<4

        d2 = Q(:,2).^2 + Q(:,3).^2;

        [~,ord] = sort(d2,'descend');

        for jj=ord(:).'

            p=Pts(jj,:);

            if ~ismember(p,V4,'rows')

                V4(end+1,:)=p; %#ok<AGROW>

            end

            if size(V4,1)>=4

                break;

            end

        end

    end

end

function V8 = model_reduce_to_max_volume_eight(V)

    % Normally V already has exactly 8 points.  This deterministic fallback

    % removes points one-by-one while retaining the largest convex volume.

    V8=V;

    while size(V8,1)>8

        bestVol=-Inf;

        bestIdx=1;

        for i=1:size(V8,1)

            T=V8;

            T(i,:)=[];

            [vol,ok]=model_convex_volume(T);

            if ok && vol>bestVol

                bestVol=vol;

                bestIdx=i;

            end

        end

        V8(bestIdx,:)=[];

    end

end

function inside = model_points_inside_convex_hull(P,V,tol)

    inside=false(size(P,1),1);

    if isempty(P) || size(V,1)<4

        return;

    end

    try

        K=convhulln(V);

    catch

        return;

    end

    c=mean(V,1);

    for i=1:size(P,1)

        p=P(i,:);

        isIn=true;

        for f=1:size(K,1)

            tri=V(K(f,:),:);

            n=cross(tri(2,:)-tri(1,:),tri(3,:)-tri(1,:));

            nn=norm(n);

            if nn<eps

                continue;

            end

            n=n/nn;

            if dot(n,c-tri(1,:))>0

                n=-n;

            end

            if dot(n,p-tri(1,:))>tol

                isIn=false;

                break;

            end

        end

        inside(i)=isIn;

    end

end

function [vol,ok] = model_convex_volume(V)

    vol=NaN;

    ok=false;

    if size(V,1)<4 || rank(V-mean(V,1))<3

        return;

    end

    try

        [~,vol]=convhulln(V);

        ok=isfinite(vol) && vol>0;

    catch

        vol=NaN;

        ok=false;

    end

end

function model_plot_region_patch(ax,V,faceColor,faceAlpha,lineWidth,nameStr)

    if isempty(V) || size(V,1)<4

        return;

    end

    try

        K=convhulln(V);

    catch

        return;

    end

    trisurf(K,V(:,2),V(:,3),V(:,1), 'Parent',ax,'FaceColor',faceColor,'FaceAlpha',faceAlpha, 'EdgeColor',faceColor,'LineWidth',lineWidth,'DisplayName',nameStr);

end

function model_plot_region_wire(ax,V,lineColor,lineWidth,nameStr)

    if isempty(V) || size(V,1)<4

        return;

    end

    try

        K=convhulln(V);

    catch

        return;

    end

    trisurf(K,V(:,2),V(:,3),V(:,1), 'Parent',ax,'FaceColor','none','EdgeColor',lineColor, 'LineWidth',lineWidth,'DisplayName',nameStr);

end

%% ========================================================================

% LOCAL FUNCTIONS: EXACT SUPPLIED FULL-CAR DYNAMICS / TIRE / CDC MAPS

% ========================================================================

function dx = model_fullcar_derivative_internal( x,delta_w,ux,wear_cmd,I_cmd, m,ms,Iz,Ixx,Iyy,lf,lr,tf,tr,h_roll,h_cg,g,Rw,Iw,tau_load, Fz_static,Fz0_vec,Fz_force_min,Fz_force_max, sigma_kappa,sigma_alpha,kw,MR_vec,mu_u,kt, Karb_f,Karb_r,Carb_f,Carb_r,mu_scale_x,mu_scale_y)

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

Fd_damper = model_lookup_all_damper_forces(cmp_rate,I_cmd);

Fd_wheel  = MR_vec.*Fd_damper;

Fs = Fspring+Fd_wheel;

Marb_f = -Karb_f*phi-Carb_f*p;

Marb_r = -Karb_r*phi-Carb_r*p;

Fs(1)=Fs(1)+Marb_f/tf;

Fs(2)=Fs(2)-Marb_f/tf;

Fs(3)=Fs(3)+Marb_r/tr;

Fs(4)=Fs(4)-Marb_r/tr;

%% Internal normal loads

[Fz,Fz_elastic] = model_calculate_internal_fz( x,Fz_static,kt,Fz_force_min,Fz_force_max, m,ms,h_cg,h_roll,kw,tf,tr,Karb_f,Karb_r);

%% Internal wheel kinematics and raw slips

[kappa_target,alpha_target,u_wheel] = model_calculate_internal_slips( x,delta_w,ux,lf,lr,tf,tr,Rw);

%% Tire forces used by dynamics

[Fx_body,Fy_body,Fx_tire,~] = model_calculate_tire_forces( delta_w,kappa_eff,Fz,alpha_eff,wear_cmd,Fz0_vec,Fz_force_min,mu_scale_x,mu_scale_y);

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

function [kappa_target,alpha_target,u_wheel] = model_calculate_internal_slips( x,delta_w,ux,lf,lr,tf,tr,Rw)

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

function [Fz,Fz_elastic,Fz_raw] = model_calculate_internal_fz( x,Fz_static,kt,Fz_force_min,Fz_force_max, m,ms,h_cg,h_roll,kw,tf,tr,Karb_f,Karb_r)

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

function [Fx_body,Fy_body,Fx_tire,Fy_tire] = model_calculate_tire_forces( delta_w,kappa_eff,Fz,alpha_eff,wear_cmd,Fz0_vec,Fz_force_min,mu_scale_x,mu_scale_y)

Fx_tire=zeros(4,1);

Fy_tire=zeros(4,1);

Fx_body=zeros(4,1);

Fy_body=zeros(4,1);

for i=1:4

    if Fz(i)>Fz_force_min

        [Fx_tire(i),Fy_tire(i)] = model_calc_combined_tire_force_scalar( kappa_eff(i),alpha_eff(i),Fz(i),Fz0_vec(i),wear_cmd(i),mu_scale_x,mu_scale_y);

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

function [Fx,Fy] = model_calc_combined_tire_force_scalar(kappa,alpha,Fz,Fz0,wear,mu_scale_x,mu_scale_y)

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

Fx0 = model_calc_mf_longitudinal_scalar(kappa,Fz,Fz0,wear,mu_scale_x);

Fy0 = model_calc_mf_lateral_scalar(alpha,Fz,Fz0,wear,mu_scale_y);

mux_peak = model_calc_longitudinal_peak_mu(Fz,Fz0,wear,mu_scale_x);

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

function mux = model_calc_longitudinal_peak_mu(Fz,Fz0,wear,mu_scale_x)

PDX1 = 1.145;

PDX2 = -0.2106;

wear_case = model_normalize_wear_case(wear);

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

function Fx = model_calc_mf_longitudinal_scalar(kappa, Fz, Fz0, wear, mu_scale_x)

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

wear_case = model_normalize_wear_case(wear);

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

function Fy = model_calc_mf_lateral_scalar(alpha, Fz, Fz0, wear, mu_scale_y)

wear_case = model_normalize_wear_case(wear);

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

function wear_case = model_normalize_wear_case(wear)

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

function cmp_rate = model_calculate_damper_rates( x, lf, lr, tf, tr, MRf, MRr)

% Sprung-body velocities

dzs = x(10);

p   = x(11);

q   = x(12);

% Unsprung velocities

dzuFL = x(13);

dzuFR = x(14);

dzuRL = x(15);

dzuRR = x(16);

vBFL = dzs + lf*q + (tf/2.0)*p;

vBFR = dzs + lf*q - (tf/2.0)*p;

vBRL = dzs - lr*q + (tr/2.0)*p;

vBRR = dzs - lr*q - (tr/2.0)*p;

vrelFL = vBFL-dzuFL;

vrelFR = vBFR-dzuFR;

vrelRL = vBRL-dzuRL;

vrelRR = vBRR-dzuRR;

cmp_rate = [ -1000.0*MRf*vrelFL;

             -1000.0*MRf*vrelFR;

             -1000.0*MRr*vrelRL;

             -1000.0*MRr*vrelRR ];

end

%% ========================================================================

% Front and rear CDC lookup-table evaluation

%% ========================================================================

function Fd = model_lookup_all_damper_forces(cmp_rate, I_cmd)

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

Fd(1) = model_interpDamperForce( I_cmd(1), cmp_rate(1), I_axis, v_axis, F_front);

Fd(2) = model_interpDamperForce( I_cmd(2), cmp_rate(2), I_axis, v_axis, F_front);

Fd(3) = model_interpDamperForce( I_cmd(3), cmp_rate(3), I_axis, v_axis, F_rear);

Fd(4) = model_interpDamperForce( I_cmd(4), cmp_rate(4), I_axis, v_axis, F_rear);

end

%% ========================================================================

% Bilinear CDC-map interpolation

%% ========================================================================

function Fd_N = model_interpDamperForce( I_mA, cmp_rate, I_axis, v_axis, F_map)

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

function region = region_stability_region(G,stable_mask,stable_points,unstable_points,cfg)
% Reconstruct the supplied sampled stability region without rerunning dynamics.
% Source: second user-supplied Simulation2_muRoad.m.
% Vertex selection, layer search, tolerances, and validity flags are retained.
% Geometry validity means zero SAMPLED unstable points; it is not a proof of
% continuous-domain safety. Eight support points need not form six faces.

    % Robust reconstruction of the paper sequence:
    % Stable -> limit-roll -> lateral-speed layers -> eight vertices
    % -> inner/outer hexahedra.
    %
    % Difference from the previous implementation:
    % The old code selected only one "inner" layer.  In the last run that
    % hexahedron contained 7 sampled unstable points, so the script stopped.
    % Here the inner layer is moved inward discretely (depth=2,3,4,and onward) and
    % the largest candidate containing ZERO sampled unstable points is kept.

    region = region_empty_region();

    if isempty(stable_points) || size(stable_points,1)<8
        return;
    end

    if rank(stable_points-mean(stable_points,1))<3
        return;
    end

    % ---------------------------------------------------------------------
    % STEP 1: LIMIT-ROLL STABLE POINTS
    % For each fixed (vc,r), take the minimum/maximum stable phi.
    % ---------------------------------------------------------------------
    limit_roll = zeros(0,3);

    for iv=1:numel(G.vc)
        for ir=1:numel(G.r)

            idxPhi = find(squeeze(stable_mask(iv,ir,:)));

            if isempty(idxPhi)
                continue;
            end

            ipLo = idxPhi(1);
            ipHi = idxPhi(end);

            limit_roll(end+1,:) = [G.vc(iv),G.r(ir),G.phi(ipLo)]; %#ok<AGROW>

            if ipHi~=ipLo
                limit_roll(end+1,:) = [G.vc(iv),G.r(ir),G.phi(ipHi)]; %#ok<AGROW>
            end
        end
    end

    limit_roll = unique(limit_roll,'rows','stable');
    region.limit_roll_points = limit_roll;

    if size(limit_roll,1)<cfg.min_limit_points
        return;
    end

    % Keep this diagnostic center for compatibility with older MAT files.
    region.vc_split_center = median(stable_points(:,1));

    % ---------------------------------------------------------------------
    % STEP 2A: OUTER LATERAL-SPEED LIMIT
    % depth=1 means the outermost stable vc point on both sides.
    % ---------------------------------------------------------------------
    outer_limit = region_lateral_layer_from_limit_roll(limit_roll,G,1);
    usedFallback = false;

    if size(outer_limit,1)<cfg.min_limit_points && cfg.allow_stable_mask_fallback

        outer_limit = region_lateral_layer_from_stable_mask(G,stable_mask,1);
        usedFallback = true;
    end

    outer_limit = unique(outer_limit,'rows','stable');
    region.outer_limit_points = outer_limit;
    region.used_stable_mask_fallback = usedFallback;

    if size(outer_limit,1)<cfg.min_limit_points
        return;
    end

    Vouter = region_select_ordered_eight_vertices(outer_limit,G,stable_points);

    if size(Vouter,1)~=8
        return;
    end

    [outerVol,okOuter] = region_convex_volume(Vouter);

    if ~okOuter
        return;
    end

    insideOuter = region_points_inside_convex_hull( unstable_points,Vouter,cfg.hull_inside_tol);

    region.outer_vertices = Vouter;
    region.outer_volume = outerVol;
    region.unstable_inside_outer = nnz(insideOuter);

    % ---------------------------------------------------------------------
    % STEP 2B / STEP 3: INNER LATERAL-SPEED LAYER SEARCH
    %
    % depth=1 : outermost layer
    % depth=2 : one stable layer inward
    % depth=3 : two stable layers inward
    % and onward
    %
    % Among all candidate hexahedra containing zero sampled unstable points,
    % keep the LARGEST volume.  Thus we do not shrink more than necessary.
    % ---------------------------------------------------------------------
    maxDepth = max(1,ceil(numel(G.vc)/2));

    bestVol = -Inf;
    best = struct( 'limit',zeros(0,3), 'V',zeros(0,3), 'bad',inf, 'depth',NaN, 'fallback',false);

    for depth = cfg.inner_depth_start:maxDepth

        inner_limit = region_lateral_layer_from_limit_roll(limit_roll,G,depth);
        thisFallback = false;

        if size(inner_limit,1)<cfg.min_limit_points && cfg.allow_stable_mask_fallback

            inner_limit = region_lateral_layer_from_stable_mask( G,stable_mask,depth);
            thisFallback = true;
        end

        inner_limit = unique(inner_limit,'rows','stable');

        if size(inner_limit,1)<cfg.min_limit_points
            continue;
        end

        Vin = region_select_ordered_eight_vertices( inner_limit,G,stable_points);

        if size(Vin,1)~=8
            continue;
        end

        [v,ok] = region_convex_volume(Vin);

        if ~ok || ~(isfinite(v) && v>0)
            continue;
        end

        bad = nnz(region_points_inside_convex_hull( unstable_points,Vin,cfg.hull_inside_tol));

        fprintf('  Inner search depth=%2d | points=%4d | volume=%10.6g | bad=%d\n', depth,size(inner_limit,1),v,bad);

        if bad==0 && v>bestVol
            bestVol = v;
            best.limit = inner_limit;
            best.V = Vin;
            best.bad = bad;
            best.depth = depth;
            best.fallback = thisFallback;
        end
    end

    % If the outer layer itself is already safe, allow it as a valid region.
    if isempty(best.V) && region.unstable_inside_outer==0
        bestVol = outerVol;
        best.limit = outer_limit;
        best.V = Vouter;
        best.bad = 0;
        best.depth = 1;
        best.fallback = usedFallback;
    end

    % ---------------------------------------------------------------------
    % LAST-RESORT GEOMETRIC FALLBACK
    % Search directly among saved stable points with progressively inward
    % support points.  No vehicle simulation is rerun.
    % ---------------------------------------------------------------------
    if isempty(best.V)

        [Vs,bads,vols,depths] = region_support_depth_search( stable_points,unstable_points,G,cfg);

        if ~isempty(Vs)
            [bestVol,jj] = max(vols);

            best.V = Vs{jj};
            best.bad = bads(jj);
            best.depth = depths(jj);
            best.limit = stable_points;
            best.fallback = true;

            region.used_support_fallback = true;
        end
    end

    if isempty(best.V)

        % Important: do not throw an error here.
        % The expensive sweep data must still be returned and saved.
        warning(['No zero-unstable inner hexahedron was found from the ' 'current sampled grid. Raw Stable/Unstable data will ' 'still be saved for post-processing.']);
        return;
    end

    region.inner_limit_points = best.limit;
    region.inner_vertices = best.V;
    region.inner_volume = bestVol;
    region.unstable_inside_inner = best.bad;
    region.inner_depth = best.depth;
    region.is_valid_inner = (best.bad==0);
    region.used_stable_mask_fallback = region.used_stable_mask_fallback || best.fallback;

end

function Pts = region_lateral_layer_from_limit_roll(limit_roll,G,depth)

    % At equal (r,phi), sort limit-roll points by vc.
    % Select the depth-th point from the negative/left and positive/right
    % lateral-speed sides.

    Pts = zeros(0,3);
    tol = 1e-12;

    for ir=1:numel(G.r)

        rr = G.r(ir);

        for ip=1:numel(G.phi)

            ph = G.phi(ip);

            idx = abs(limit_roll(:,2)-rr)<=tol & abs(limit_roll(:,3)-ph)<=tol;

            Q = sortrows(limit_roll(idx,:),1);
            n = size(Q,1);

            if n < depth
                continue;
            end

            Pts(end+1,:) = Q(depth,:); %#ok<AGROW>

            j = n-depth+1;

            if j~=depth && j>=1 && j<=n
                Pts(end+1,:) = Q(j,:); %#ok<AGROW>
            end
        end
    end

    Pts = unique(Pts,'rows','stable');

end

function Pts = region_lateral_layer_from_stable_mask(G,stable_mask,depth)

    % Fallback for a discrete grid:
    % At each fixed (r,phi), select the depth-th stable vc point from both
    % sides using the already-computed stable_mask.

    Pts = zeros(0,3);

    for ir=1:numel(G.r)
        for ip=1:numel(G.phi)

            idxVc = find(squeeze(stable_mask(:,ir,ip)));
            n = numel(idxVc);

            if n < depth
                continue;
            end

            iL = idxVc(depth);
            iR = idxVc(n-depth+1);

            Pts(end+1,:) = [G.vc(iL),G.r(ir),G.phi(ip)]; %#ok<AGROW>

            if iR~=iL
                Pts(end+1,:) = [G.vc(iR),G.r(ir),G.phi(ip)]; %#ok<AGROW>
            end
        end
    end

    Pts = unique(Pts,'rows','stable');

end

function V = region_select_ordered_eight_vertices(Pts,G,stable_points)

    % Eight normalized support directions in [vc,r,phi].
    % Returned order:
    % 1 [- - -], 2 [- - +], 3 [- + -], 4 [- + +]
    % 5 [+ - -], 6 [+ - +], 7 [+ + -], 8 [+ + +]

    V = zeros(0,3);

    if size(Pts,1)<8
        return;
    end

    ranges = [ region_range_nonzero(G.vc), region_range_nonzero(G.r), region_range_nonzero(G.phi)];

    c = median(stable_points,1);
    Q = (Pts-c)./ranges;

    signs = [ -1 -1 -1; -1 -1 +1; -1 +1 -1; -1 +1 +1; +1 -1 -1; +1 -1 +1; +1 +1 -1; +1 +1 +1];

    chosen = false(size(Pts,1),1);
    V = nan(8,3);

    for k=1:8

        s = signs(k,:);

        % Prefer a point in the corresponding octant.
        oct = true(size(Pts,1),1);

        for j=1:3
            if s(j)<0
                oct = oct & (Q(:,j)<=1e-12);
            else
                oct = oct & (Q(:,j)>=-1e-12);
            end
        end

        score = Q*s(:);
        score(chosen) = -Inf;
        score(~oct) = -Inf;

        [best,idx] = max(score);

        % If the exact octant has no unused point, use the best unused point
        % along the same normalized support direction.
        if ~isfinite(best)
            score = Q*s(:);
            score(chosen) = -Inf;
            [best,idx] = max(score);
        end

        if ~isfinite(best)
            V = zeros(0,3);
            return;
        end

        V(k,:) = Pts(idx,:);
        chosen(idx) = true;
    end

    if size(unique(V,'rows'),1)~=8
        V = zeros(0,3);
    end

end

function [Vcells,badList,volList,depthList] = region_support_depth_search( stable_points,unstable_points,G,cfg)

    Vcells = {};
    badList = [];
    volList = [];
    depthList = [];

    if size(stable_points,1)<8
        return;
    end

    ranges = [ region_range_nonzero(G.vc), region_range_nonzero(G.r), region_range_nonzero(G.phi)];

    c = median(stable_points,1);
    Q = (stable_points-c)./ranges;

    signs = [ -1 -1 -1; -1 -1 +1; -1 +1 -1; -1 +1 +1; +1 -1 -1; +1 -1 +1; +1 +1 -1; +1 +1 +1];

    order = cell(8,1);

    for k=1:8
        score = Q*signs(k,:).';
        [~,order{k}] = sort(score,'descend');
    end

    maxDepth = min( cfg.max_support_depth, max(1,floor(size(stable_points,1)/8)));

    for depth=1:maxDepth

        used = false(size(stable_points,1),1);
        V = nan(8,3);
        ok = true;

        for k=1:8

            ord = order{k};
            pick = [];

            for jj=depth:numel(ord)

                ii = ord(jj);

                if ~used(ii)
                    pick = ii;
                    break;
                end
            end

            if isempty(pick)
                ok = false;
                break;
            end

            V(k,:) = stable_points(pick,:);
            used(pick) = true;
        end

        if ~ok || size(unique(V,'rows'),1)~=8
            continue;
        end

        [vol,okVol] = region_convex_volume(V);

        if ~okVol
            continue;
        end

        bad = nnz(region_points_inside_convex_hull( unstable_points,V,cfg.hull_inside_tol));

        fprintf('  Support fallback depth=%2d | volume=%10.6g | bad=%d\n', depth,vol,bad);

        if bad==0
            Vcells{end+1,1} = V; %#ok<AGROW>
            badList(end+1,1) = bad; %#ok<AGROW>
            volList(end+1,1) = vol; %#ok<AGROW>
            depthList(end+1,1) = depth; %#ok<AGROW>
        end
    end

end

function y = region_range_nonzero(x)

    y = max(x)-min(x);

    if ~(isfinite(y) && y>0)
        y = 1;
    end

end


function R = region_empty_region()

    R = struct( 'limit_roll_points',zeros(0,3), 'outer_limit_points',zeros(0,3), 'inner_limit_points',zeros(0,3), 'outer_vertices',zeros(0,3), 'inner_vertices',zeros(0,3), 'outer_volume',NaN, 'inner_volume',NaN, 'unstable_inside_outer',NaN, 'unstable_inside_inner',NaN, 'inner_depth',NaN, 'is_valid_inner',false, 'vc_split_center',NaN, 'used_stable_mask_fallback',false, 'used_support_fallback',false);

end


function inside = region_points_inside_convex_hull(P,V,tol)
    % Same half-space test as supplied model, with facet geometry computed once.
    % Process all points per facet. The strict >tol rejection deliberately
    % preserves the original inclusive boundary and NaN comparison semantics.
    inside = false(size(P,1),1);
    if isempty(P) || size(V,1)<4
        return;
    end
    try
        K = convhulln(V);
    catch
        return;
    end
    c = mean(V,1);
    inside(:) = true;
    for f=1:size(K,1)
        tri = V(K(f,:),:);
        n = cross(tri(2,:)-tri(1,:),tri(3,:)-tri(1,:));
        nn = norm(n);
        if nn<eps
            continue;
        end
        n = n/nn;
        if dot(n,c-tri(1,:))>0
            n = -n;
        end
        d = (P(:,1)-tri(1,1))*n(1) + ...
            (P(:,2)-tri(1,2))*n(2) + ...
            (P(:,3)-tri(1,3))*n(3);
        inside = inside & ~(d>tol);
    end
end

function [vol,ok] = region_convex_volume(V)

    vol=NaN;

    ok=false;

    if size(V,1)<4 || rank(V-mean(V,1))<3

        return;

    end

    try

        [~,vol]=convhulln(V);

        ok=isfinite(vol) && vol>0;

    catch

        vol=NaN;

        ok=false;

    end

end

function signature = single_stability_run_signature(cfg,G,wear,stage)
%STABILITY_RUN_SIGNATURE Reproducibility key for classification/final caches.
% No paths, timestamps, credentials, or process state are part of the key.
% File contents are hashed directly, without a platform-specific shell tool.
source_path=[mfilename('fullpath') '.m'];
[~,source_name,source_ext]=fileparts(source_path);
files={[source_name source_ext]};
hashes={single_file_sha256(source_path)};
names={'speed_kmh','ux','mu_road','mu_reference','friction_scaling_mode','grid_generation','mu_scale_x','mu_scale_y','steer_deg', ...
 'Ts','sim_time','Nsub','k_spring_N_m','I_mA','g','limit_factor','r_limit', ...
 'ay_limit','LTR_limit','phi_hard_limit_rad','vc_hard_limit_m_s', ...
 'r_hard_limit_rad_s','ignore_initial_s','early_exit_on_unstable','initialization_mode'};
if ~strcmp(stage,'native')
 names=[names,{'hull_inside_tol','inner_depth_start','allow_stable_mask_fallback', ...
  'min_limit_points','max_support_depth','require_inner_no_unstable'}];
end
parameters=struct();
for i=1:numel(names), parameters.(names{i})=cfg.(names{i}); end
parameters.grid=G; parameters.wear=wear;
provenance=struct('schema','mu-sweep-cache-v2','stage',stage, ...
 'source_files',{files},'sha256',{hashes},'parameters',parameters);
signature=jsonencode(provenance);
end

function digest=single_file_sha256(path)
fid=fopen(path,'rb'); assert(fid>=0,'Cannot fingerprint %s',path);
cleanup=onCleanup(@()fclose(fid)); %#ok<NASGU>
if exist('OCTAVE_VERSION','builtin')
 % Octave's built-in hash accepts a binary char vector, including NUL bytes.
 bytes=fread(fid,inf,'*uint8');
 digest=hash('sha256',char(bytes.'));
else
 assert(usejava('jvm'),'File SHA-256 requires the MATLAB Java VM.');
 md=java.security.MessageDigest.getInstance('SHA-256');
 while true
  bytes=fread(fid,1024*1024,'*uint8');
  if isempty(bytes), break; end
  md.update(typecast(bytes,'int8'));
 end
 % Java digest bytes are signed; preserve the bit pattern when converting.
 bytes=typecast(md.digest(),'uint8');
 digest=lower(reshape(dec2hex(bytes,2).',1,[]));
end
end
