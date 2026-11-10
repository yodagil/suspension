function OUT=cdc_research_generate(o)
% Build H-reference-preserving finite-horizon initial-state regions.
% Grid points outside H are NOT simulated and are NOT labelled unrecoverable.
% All figures use absolute [vy,r] coordinates. The final 2-D section is NOT
% claimed invariant: its trajectories are tested inside H, not inside itself.
validate(o);o.speeds=o.speeds(:)';o.mus=o.mus(:)';o.steers=o.steers(:)';o.horizons=o.horizons(:)';o.grid.vc=o.grid.vc(:)';o.grid.r=o.grid.r(:)';o.I_mA=o.I_mA(:)';root=fileparts(mfilename('fullpath'));
if ~exist(o.output_dir,'dir'),mkdir(o.output_dir);end
OUT=struct('files',{{}},'options',o,'completed',false);
source=struct();for f={'cdc_research_generate.m','cdc_research_simulate.m','cdc_fullcar_model.m','cdc_handling_reference.m','cdc_dynamic_slope.m','cdc_research_codes.m','cdc_run_points.m','cdc_research_fields.m','cdc_research_native.m'}
 source.(strrep(f{1},'.','_'))=fileread(fullfile(root,f{1}));
end
if strcmp(o.engine,'native')
 source.native_driver=fileread(fullfile(root,'native','cdc_research_native.cpp'));
 source.native_physics=fileread(fullfile(root,'native','cdc_physics.hpp'));
end
for speed=o.speeds
 for mu=o.mus
  for steer=o.steers
   c=o;c.speed_kmh=speed;c.ux=speed/3.6;c.mu=mu;c.steer_deg=steer;c.delta=deg2rad([steer;steer;0;0]);
   itag=sprintf('_I%g_%g_%g_%g',o.I_mA);
   tag=sprintf('research_%gkph_mu%g_delta%+g%s_%s',speed,mu,steer,itag,c.slice_mode);
   if ~isempty(c.controller),tag=[tag '_feedback'];end
   target=fullfile(o.output_dir,[tag '.mat']);
   if exist(target,'file')==2&&~o.overwrite
    assert(o.resume,'Existing result would be overwritten: %s',target);
    oldResult=load(target,'RESEARCH2D');
    assert(isfield(oldResult,'RESEARCH2D')&&isfield(oldResult.RESEARCH2D,'source')&&isequaln(oldResult.RESEARCH2D.source,source)&&isequaln(oldResult.RESEARCH2D.cfg,c),'Saved-result provenance mismatch. Use a new output_dir.');
    OUT.files{end+1}=target;fprintf('Validated cached result: %s\n',target);continue;
   end
   CASE=cell(size(o.wear_cases,1),1);
   for iw=1:size(o.wear_cases,1)
    checkpointPath=fullfile(o.output_dir,[tag sprintf('_wear%d_checkpoint.mat',iw)]);
    if exist(checkpointPath,'file')==2&&o.resume&&~o.overwrite
     w=load(checkpointPath,'CHECKPOINT');
     assert(isfield(w,'CHECKPOINT')&&isequaln(w.CHECKPOINT.options,o)&&isequaln(w.CHECKPOINT.source,source),'Checkpoint provenance mismatch. Use a new output_dir.');
     CASE{iw}=w.CHECKPOINT.case_result;fprintf('Validated checkpoint %d/%d\n',iw,size(o.wear_cases,1));continue;
    end
    [Q,T]=cdc_fullcar_model('prepare',c,o.wear_cases(iw,:));
    if ~T.root_found,error('No checked trim for speed=%g mu=%g steer=%g wear row=%d. No fallback hidden state is silently used.',speed,mu,steer,iw);end
    trimH=cdc_handling_reference(T.x(1:2)',Q);
    [V,R]=ndgrid(c.grid.vc,c.grid.r);points=[V(:),R(:)];H=cdc_handling_reference(points,Q);
    ref=reshape(H.inside,size(V));ids=find(ref);p=points(ids,:);
    if isempty(ids),error('Reference mask is empty on this grid. Inspect handling range/model before proceeding.');end
    raw=cdc_run_points(p,Q,c,T);raw.initial_points=p;raw.initial_grid_linear_index_1based=ids;
    M=struct('reference',ref);keys={'recoverable_on_reference','guarded_recoverable_on_reference','final','guarded_final','prepeak_final'};
    fields={'recoverable','guarded_recoverable','handling_preserved_recoverable','guarded_handling_preserved_recoverable','prepeak_handling_preserved_recoverable'};
    for j=1:numel(keys)
     a=false(numel(V),numel(c.horizons));a(ids,:)=logical(raw.(fields{j}));
     if j>=3&&~trimH.inside,a(:)=false;end
     M.(keys{j})=reshape(a,[size(V),numel(c.horizons)]);
    end
    if ~trimH.inside,warning('Trim lies outside H reference. Final region is deliberately empty, not artificially moved.');end
    h=struct('grid',c.grid,'mask',ref,'margin',reshape(H.margin,size(V)),'trace',reshape(H.trace,size(V)),'determinant',reshape(H.determinant,size(V)),'min_tire_restoring_slope',reshape(H.min_tire_restoring_slope,size(V)),'definition',H.definition);
    fm=M.final(:,:,end);cm=fm(1:end-1,1:end-1)&fm(2:end,1:end-1)&fm(1:end-1,2:end)&fm(2:end,2:end);
    ci=find(cm);[iv,ir]=ind2sub(size(cm),ci);vg=c.grid.vc;rg=c.grid.r;
    lo=[reshape(vg(iv),[],1),reshape(rg(ir),[],1)];hi=[reshape(vg(iv+1),[],1),reshape(rg(ir+1),[],1)];cp=(lo+hi)/2;
    valid=true(numel(ci),1);centerRaw=struct();
    if c.check_cell_centers&&~isempty(ci)
     centerRaw=cdc_run_points(cp,Q,c,T);valid=logical(centerRaw.handling_preserved_recoverable(:,end));
    end
    interior=cell(c.random_points_per_cell,1);old=rng;restore=onCleanup(@()rng(old));rng(c.random_seed+iw,'twister');
    for k=1:c.random_points_per_cell
     if isempty(ci),break;end
     ip=lo+rand(size(lo)).*(hi-lo);rr=cdc_run_points(ip,Q,c,T);pass=logical(rr.handling_preserved_recoverable(:,end));valid=valid&pass;interior{k}=struct('points',ip,'pass',pass);
    end
    clear restore;
    cm(ci(~valid))=false;[dv,dr]=ndgrid(diff(vg),diff(rg));areas=dv.*dr;
    region=struct('cell_mask',cm,'cell_area',sum(areas(cm)),'candidate_cells',numel(ci),'accepted_cells',nnz(cm),'center_points',cp,'center_pass',valid,'center_raw',centerRaw,'random_checks',{interior},'certified_continuous',false,'definition','Four passed vertices plus explicitly requested interior tests; NOT a continuous inner-approximation certificate');
    C=struct('wear',Q.wear,'case_label',sprintf('row%d wear[%g %g %g %g]',iw,Q.wear),'grid',c.grid,'handling',h,'trim',T,'trim_in_reference',trimH.inside,'configuration',c,'masks',M,'raw',raw,'region',region,'grid_truncated',edge(fm),'reference_grid_truncated',edge(ref));
    CASE{iw}=C;
    fprintf('%g km/h mu=%g delta=%+g wear[%g %g %g %g]: H=%d recovery=%d final=%d cells=%d area=%.8g\n',speed,mu,steer,Q.wear,nnz(ref),nnz(M.recoverable_on_reference(:,:,end)),nnz(fm),nnz(cm),region.cell_area);
    if edge(ref)||edge(fm),warning('Reference/final mask touches the grid. Expand the domain before using area comparisons.');end
    checkpoint=struct('case_result',C,'options',o,'source',source);
    save_atomic(checkpointPath,'CHECKPOINT',checkpoint);
   end
   meta=struct('version','CDC_RESEARCH_FINAL_1','definition','Conditional finite-horizon H-reference-preserving safe recovery initial-state region','certified_invariant',false,'engine',o.engine,'speed_kmh',speed,'mu_road',mu,'wheel_steer_deg',c.delta'*180/pi,'horizons_s',c.horizons,'dt_s',c.Ts/c.Nsub,'damper_type','CDC','motion_ratios',[.8613 .797],'currents_mA',c.I_mA,'feedback_enabled',~isempty(c.controller),'initialization',[c.initialization '/' c.slice_mode],'hidden_variants',c.hidden_variants,'friction_note','Original mu/0.9 amplitude scaling; not absolute peak-mu normalization','important','Path tested inside H reference, NOT inside final 2-D initial-state subset; only H-initial points simulated','created',datestr(now,30));
   S=struct('meta',meta,'CASE',{CASE},'cfg',c,'source',source);
   save_atomic(target,'RESEARCH2D',S);OUT.files{end+1}=target;
  end
 end
end
OUT.completed=true;
end
function validate(o)
assert(isvector(o.speeds)&&all(o.speeds>0)&&all(isfinite(o.speeds)),'Invalid speeds');
assert(isvector(o.mus)&&all(o.mus>0)&&all(isfinite(o.mus)),'Invalid mus');
assert(isvector(o.steers)&&all(isfinite(o.steers)),'Invalid steers');
assert(all(diff(o.grid.vc)>0)&&all(diff(o.grid.r)>0),'Grid axes must strictly increase');
assert(numel(o.grid.vc)>1&&numel(o.grid.r)>1,'Grid needs >=2 values on each axis');
assert(size(o.wear_cases,2)==4&&all(ismember(o.wear_cases(:),[0 25 50])),'Invalid wear');
assert(numel(o.state_tolerance)==29&&all(o.state_tolerance>0),'Invalid terminal tolerances');
assert(o.Nsub>=1&&o.Nsub==round(o.Nsub)&&o.Ts>0,'Invalid step');
assert(all(diff(o.horizons)>0)&&all(o.horizons>=o.terminal_window),'Invalid horizons');
assert(all(abs(o.horizons/(o.Ts/o.Nsub)-round(o.horizons/(o.Ts/o.Nsub)))<1e-6),'Horizon not on dt grid');
assert(numel(o.I_mA)==4&&all(o.I_mA>=100&o.I_mA<=1800),'Current outside table');
assert(any(strcmp(o.engine,{'mcode','native'})),'Unknown engine');
assert(size(o.hidden_variants,2)==4&&size(o.hidden_variants,1)>=1&&all(isfinite(o.hidden_variants(:))),'hidden_variants needs at least one finite four-column row');
assert(o.batch_size>=1&&mod(o.batch_size,1)==0,'Invalid batch_size');
assert(o.native_threads>=1&&mod(o.native_threads,1)==0,'Invalid native_threads');
assert(o.trace_stride>=0&&mod(o.trace_stride,1)==0,'Invalid trace_stride');
assert(o.current_rate_limit_mA_s>=0&&~isnan(o.current_rate_limit_mA_s),'Invalid current rate limit');
assert(~strcmp(o.engine,'native')||isempty(o.controller),'Feedback requires the mcode backend');
assert(all(o.k_spring_N_m==30790),'This release preserves original springs. Parameter changes require a new audit.');
assert(isempty(o.controller)||isa(o.controller,'function_handle'),'controller must be empty or a function handle');
assert(o.random_points_per_cell>=0&&mod(o.random_points_per_cell,1)==0,'Invalid random cell count');
end
function b=edge(m)
b=any(m(1,:))||any(m(end,:))||any(m(:,1))||any(m(:,end));
end
function save_atomic(path,name,value)
s=struct();s.(name)=value;tmp=[path '.tmp'];save(tmp,'-struct','s','-v7');[ok,msg]=movefile(tmp,path,'f');assert(ok,msg);
end
