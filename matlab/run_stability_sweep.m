function manifest = run_stability_sweep(options)
%RUN_STABILITY_SWEEP Full 210-scenario, 5-wear, 39401-initial-state sweep.
%   run_stability_sweep() uses exactly the original 41x31x31 grid,
%   Ts=0.01 s, Nsub=20, and 5 s observation horizon. Front wheel steer is
%   [angle angle 0 0] degrees, NOT the steering-wheel angle.
%   Files are MATLAB-compatible v7 MAT files containing STABILITY3D.
%   Optional options fields: speeds, mus, steers, output_dir, engine, wear_cases.
%   wear_cases=[0 0 0 0] computes only the healthy tire case.
if nargin < 1, options = struct(); end
root = fileparts(mfilename('fullpath'));
addpath(root);
[cfg0,G,default_wear_cases,P0,GF] = fullcar_model('defaults');
wear_cases = get_option(options,'wear_cases',default_wear_cases);
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
speeds = get_option(options,'speeds',[50 60 70 80 90 100]);
mus = get_option(options,'mus',[0.9 0.8 0.7 0.6 0.5]);
steers = get_option(options,'steers',[0 0.5 1 1.5 2 2.5 3]);
outdir = get_option(options,'output_dir',fullfile(root,'results'));
engine = get_option(options,'engine','native');
assert(isnumeric(speeds) && isnumeric(mus) && isnumeric(steers),'Scenario options must be numeric vectors.');
assert(isvector(speeds) && isvector(mus) && isvector(steers),'Scenario options must be nonempty vectors.');
speeds=speeds(:).'; mus=mus(:).'; steers=steers(:).';
assert(all(isfinite(speeds)) && all(speeds>0) && all(isfinite(mus)) && all(mus>0) && all(isfinite(steers)), ...
 'Scenario options must be finite; speed and mu must be positive.');
assert(numel(unique(speeds))==numel(speeds) && numel(unique(mus))==numel(mus) && numel(unique(steers))==numel(steers), ...
 'Duplicate scenarios are not permitted.');
assert(all(abs(mus*10-round(mus*10))<1e-10),'Filename format supports mu values with one decimal.');
assert(any(strcmp(engine,{'native','mcode'})),'Unknown engine: %s',engine);
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
   scenario_signature=stability_run_signature(cfg,G,wear_cases,['scenario-' engine]);
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
      validate_saved_scenario(S,cfg,G,wear_cases,scenario_signature);
      manifest(idx).completed=true;
      manifest(idx).stable_counts=[S.CASE.stable_count];
      fprintf('Already complete; validated metadata.\n');
      write_manifest(outdir,manifest);
      continue;
    end
    error('Existing file has an unknown format; refusing to overwrite: %s',target);
   end
   CASE = repmat(empty_case(),size(wear_cases,1),1);
   for iw=1:size(wear_cases,1)
    wear=wear_cases(iw,:);
    if strcmp(engine,'native')
      [stable_mask,reason_mask,run_info]=native_stability_masks(cfg,G,wear,cache,tag,iw);
    elseif strcmp(engine,'mcode')
      [stable_mask,reason_mask]=mcode_masks(cfg,G,wear,P);
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
    atomic_mat_save(case_raw_path,'CASE_RAW',CASE_RAW);
    C=analyze_case(cfg,G,wear,stable_mask,reason_mask,run_info);
    CASE(iw)=C;
    CHECKPOINT=struct('cfg',cfg,'grid_coarse',G,'wear_cases',wear_cases, ...
       'last_completed_case',iw,'CASE',CASE);
    atomic_mat_save(cfg.checkpoint_mat,'CHECKPOINT',CHECKPOINT);
    fprintf('Wear %d/%d: stable %d/%d; inner volume %.8g; valid %d\n', ...
      iw,size(wear_cases,1),C.stable_count,C.total_count,C.region.inner_volume,C.region.is_valid_inner);
   end
   RAW=struct('cfg',cfg,'grid_coarse',G,'wear_cases',wear_cases,'CASE',CASE);
   atomic_mat_save(cfg.raw_mat,'RAW',RAW);
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
     'postprocessing_version','stable-grid-box-fallback-v1', ...
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
   STABILITY3D.meta.inner_region_methods=arrayfun(@(c)inner_region_method(c.region),CASE,'UniformOutput',false);
   STABILITY3D.meta.postprocessing_complete=all(arrayfun(@(c) ...
     isempty(c.postprocess.stable_hull_error) && isempty(c.postprocess.region_error),CASE));
   STABILITY3D.cfg=cfg; STABILITY3D.grid_coarse=G; STABILITY3D.grid_fine=GF;
   STABILITY3D.wear_cases=wear_cases; STABILITY3D.healthy_case_idx=healthy_case_idx;
   STABILITY3D.smallest_region_case_idx=worst;
   STABILITY3D.smallest_region_wear=wear_cases(worst,:);
   STABILITY3D.CASE=CASE; STABILITY3D.CASE_FINE=struct();
   % A struct, not MATLAB table: portable between Octave and MATLAB.
   STABILITY3D.CaseSummary=make_summary(CASE);
   rankv=volumes; rankv(~isfinite(rankv))=inf;
   [~,order]=sortrows([~valid(:),rankv(:),[CASE.stable_fraction].'],[1 2 3]);
   STABILITY3D.CaseRanking=STABILITY3D.CaseSummary(order);
   atomic_mat_save(target,'STABILITY3D',STABILITY3D);
   check=load(target,'STABILITY3D');
   validate_saved_scenario(check.STABILITY3D,cfg,G,wear_cases,scenario_signature);
   manifest(idx).completed=true;
   manifest(idx).stable_counts=[CASE.stable_count];
   manifest(idx).elapsed_s=toc(began);
   write_manifest(outdir,manifest);
   fprintf('SAVED %s | %.1f s\n',target,manifest(idx).elapsed_s);
  end
 end
end
assert(all([manifest.completed]),'Sweep did not complete');
fprintf('\nALL %d SCENARIOS COMPLETE\n',numel(manifest));
end

function C=empty_case()
C=struct('wear',[],'grid',[],'stable_mask',[],'reason_mask',[], ...
 'stable_points',[],'unstable_points',[],'total_count',0,'stable_count',0, ...
 'unstable_count',0,'stable_fraction',NaN,'stable_hull_volume',NaN, ...
 'region',empty_region(),'run_info',struct(), ...
 'postprocess',struct('stable_hull_error','','region_error',''));
end

function C=analyze_case(cfg,G,wear,mask,reasons,info)
[vc,r,phi]=ndgrid(G.vc,G.r,G.phi); points=[vc(:),r(:),phi(:)];
assert(isequal(size(mask),size(vc)) && isequal(size(reasons),size(vc)));
assert(isequal(mask,reasons==0),'Reason/stable masks disagree');
C=empty_case(); C.wear=wear; C.grid=G; C.stable_mask=mask; C.reason_mask=reasons;
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
 C.region=stability_region(G,mask,C.stable_points,C.unstable_points,cfg);
catch ME
 C.postprocess.region_error=ME.message;
 warning('Region construction failed; raw masks are preserved: %s',ME.message);
end
end

function [mask,reasons]=mcode_masks(cfg,G,wear,P)
mask=false(numel(G.vc),numel(G.r),numel(G.phi)); reasons=zeros(size(mask),'uint8');
for ip=1:numel(G.phi)
 for ir=1:numel(G.r)
  for iv=1:numel(G.vc)
   [mask(iv,ir,ip),reasons(iv,ir,ip)]=fullcar_model('classify',cfg,wear,P,G.vc(iv),G.r(ir),G.phi(ip));
  end
 end
end
end

function S=make_summary(CASE)
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

function R=empty_region()
R=struct('limit_roll_points',zeros(0,3),'outer_limit_points',zeros(0,3), ...
 'inner_limit_points',zeros(0,3),'outer_vertices',zeros(0,3),'inner_vertices',zeros(0,3), ...
 'outer_volume',NaN,'inner_volume',NaN,'unstable_inside_outer',NaN, ...
 'unstable_inside_inner',NaN,'inner_depth',NaN,'is_valid_inner',false, ...
 'vc_split_center',NaN,'used_stable_mask_fallback',false,'used_support_fallback',false);
end

function validate_saved_scenario(S,cfg,G,wear_cases,signature)
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

function method=inner_region_method(region)
method='paper';
if isfield(region,'inner_method') && ~isempty(region.inner_method)
 method=region.inner_method;
end
end

function value=get_option(options,name,default)
if isfield(options,name), value=options.(name); else, value=default; end
end

function atomic_mat_save(path,name,value)
S=struct(); S.(name)=value; tmp=[path '.tmp'];
save(tmp,'-struct','S','-v7');
[ok,msg]=movefile(tmp,path,'f'); assert(ok,msg);
end

function write_manifest(outdir,manifest)
atomic_mat_save(fullfile(outdir,'sweep_manifest.mat'),'manifest',manifest);
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
