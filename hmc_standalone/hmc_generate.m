function OUT=hmc_generate(o)
%HMC_GENERATE Finite-time H-constrained recovery maps for independent HMC.
% Outside H is uncomputed. Empty/invalid trims never silently replace data.
assert(all(o.speeds>0)&&all(o.mus>0)&&all(diff(o.grid.vc)>0)&&all(diff(o.grid.r)>0));
assert(ismember(o.engine,{'native','mcode'}),'Unknown engine');
assert(o.terminal_window<=min(o.horizons)&&o.terminal_window>0);
assert(size(o.wear_cases,2)==4&&all(ismember(o.wear_cases(:),[0 25 50])));
data=load(o.data_file,'D');D=data.D;
root=fileparts(mfilename('fullpath'));source=struct();
files={'hmc_vehicle_model.m','hmc_tire_force.m','hmc_handling_reference.m', ...
 'hmc_classify.m','hmc_generate.m','hmc_options.m','hmc_native.m','hmc_mcode_simulate.m'};
for k=1:numel(files),source.(strrep(files{k},'.','_'))=fileread(fullfile(root,files{k}));end
source.data_manifest=fileread(fullfile(fileparts(o.data_file),'source_manifest.json'));
source.derived_data=D; % Semantic data identity, not just the upstream-file manifest.
if strcmp(o.engine,'native')
 for file={'hmc_native.cpp','hmc_vehicle.hpp','hmc_tire.hpp'}
  source.(strrep(file{1},'.','_'))=fileread(fullfile(root,'native',file{1}));
 end
end
if ~exist(o.output_dir,'dir'),mkdir(o.output_dir);end
OUT=struct('files',{{}},'completed',false,'options',o);
scenario=0;total=numel(o.speeds)*numel(o.mus)*numel(o.steers);
for speed=o.speeds
 for mu=o.mus
  for steer=o.steers
   scenario=scenario+1;tag=sprintf('hmc_%gkph_mu%g_delta%+g',speed,mu,steer);
   target=fullfile(o.output_dir,[tag '.mat']);
   if exist(target,'file')&&o.resume&&~o.overwrite
    old=load(target,'RESEARCH2D');assert(isequaln(old.RESEARCH2D.source,source)&&isequaln(old.RESEARCH2D.options,o),'Changed source/options; choose a new output folder.');
    OUT.files{end+1}=target;continue;
   end
   CASE=cell(size(o.wear_cases,1),1);
   for iw=1:size(o.wear_cases,1)
    checkpoint=fullfile(o.output_dir,sprintf('%s_wear%d_checkpoint.mat',tag,iw));
    if exist(checkpoint,'file')&&o.resume&&~o.overwrite
     old=load(checkpoint,'CHECKPOINT');assert(isequaln(old.CHECKPOINT.source,source)&&isequaln(old.CHECKPOINT.options,o),'Checkpoint source/options changed.');
     CASE{iw}=old.CHECKPOINT.case_result;continue;
    end
    c=o;c.speed_kmh=speed;c.ux=speed/3.6;c.mu=mu;c.steer_deg=steer;c.wear=o.wear_cases(iw,:);
    fprintf('SCENARIO %d/%d WEAR %d/%d: %gkm/h mu%g steer%+g\n',scenario,total,iw,size(o.wear_cases,1),speed,mu,steer);
    [Q,T]=hmc_vehicle_model('prepare',c,c.wear,D);
    assert(T.root_found&&all(isfinite(T.x)),'No checked independent trim; no fallback data used.');
    [~,trimO]=hmc_vehicle_model('rhs',T.x,Q);
    if ~isempty(o.reference_steer_deg),c.reference_steer_deg=o.reference_steer_deg;
    elseif isfield(trimO,'delta'),c.reference_steer_deg=mean(trimO.delta(1:2))*180/pi;
    else,c.reference_steer_deg=steer;end
    [V,R]=ndgrid(c.grid.vc,c.grid.r);points=[V(:),R(:)];
    H=hmc_handling_reference(points,c,c.wear);ref=reshape(H.inside,size(V));ids=find(ref);
    assert(~isempty(ids),'H has no initial points on this grid.');
    trimH=hmc_handling_reference(T.x(1:2)',c,c.wear);
    raw=run_points(points(ids,:),Q,T,c);raw.initial_points=points(ids,:);raw.initial_grid_linear_index_1based=ids;
    M=struct('reference',ref);keys={'final','recoverable_on_reference','guarded_final','model_safe'};
    fields={'passed','recoverable','guarded_final','model_safe'};
    for j=1:numel(keys)
     a=false(numel(V),numel(c.horizons));a(ids,:)=logical(raw.(fields{j}));
     if ismember(keys{j},{'final','guarded_final'})&&~trimH.inside&&~c.allowTrimOutsideH,a(:)=false;end
     M.(keys{j})=reshape(a,[size(V),numel(c.horizons)]);
    end
    F=M.final(:,:,end);cells=F(1:end-1,1:end-1)&F(2:end,1:end-1)&F(1:end-1,2:end)&F(2:end,2:end);
    ci=find(cells);[iv,ir]=ind2sub(size(cells),ci);v=c.grid.vc;r=c.grid.r;
    cp=[(reshape(v(iv),[],1)+reshape(v(iv+1),[],1))/2,(reshape(r(ir),[],1)+reshape(r(ir+1),[],1))/2];
    passed=true(numel(ci),1);center=struct();
    if c.check_cell_centers&&~isempty(ci),center=run_points(cp,Q,T,c);passed=logical(center.passed(:,end));end
    cells(ci(~passed))=false;[dv,dr]=ndgrid(diff(v),diff(r));area=dv.*dr;
    region=struct('cell_mask',cells,'candidate_cells',numel(ci),'accepted_cells',nnz(cells), ...
     'cell_area',sum(area(cells)),'center_points',cp,'center_pass',passed,'center_raw',center,'certified_continuous',false);
    C=struct('wear',c.wear,'trim',T,'trim_in_reference',logical(trimH.inside), ...
     'configuration',c,'grid',c.grid,'handling',struct('mask',ref,'trace',reshape(H.trace,size(V)), ...
     'determinant',reshape(H.determinant,size(V)),'margin',reshape(H.margin,size(V))), ...
     'masks',M,'raw',raw,'region',region,'grid_truncated',edge(F),'reference_grid_truncated',edge(ref));
    CASE{iw}=C;CHECKPOINT=struct('source',source,'options',o,'case_result',C);save_atomic(checkpoint,'CHECKPOINT',CHECKPOINT);
    fprintf('DONE %d/%d WEAR %d/%d: H%d final%d cells%d area%.9g\n',scenario,total,iw,size(o.wear_cases,1),nnz(ref),nnz(F),nnz(cells),region.cell_area);
   end
   meta=struct('version','HMC_INDEPENDENT_1','source','Independent HMC data-driven approximation', ...
    'not_carsim_engine',true,'not_tno_swift_belt_model',true,'speed_kmh',speed,'mu_road',mu, ...
    'steer_target_deg',steer,'engine',o.engine,'horizons_s',o.horizons,'dt_s',o.dt, ...
    'state_count',34,'longitudinal_speed','dynamic PI speed control','currents_mA',o.I_mA, ...
    'certified_invariant',false,'outside_H','not_simulated','created',datestr(now,30));
   RESEARCH2D=struct('meta',meta,'CASE',{CASE},'options',o,'source',source);
   save_atomic(target,'RESEARCH2D',RESEARCH2D);OUT.files{end+1}=target;
  end
 end
end
OUT.completed=true;
end
function raw=run_points(points,Q,T,c)
X=hmc_vehicle_model('lift',points,Q,c,T);
if strcmp(c.engine,'native'),raw=hmc_native(X,Q,T,c,c.wear);
else,raw=hmc_mcode_simulate(X,Q,T,c,c.wear);end
end
function b=edge(m),b=any(m(1,:))||any(m(end,:))||any(m(:,1))||any(m(:,end));end
function save_atomic(path,key,value)
s=struct();s.(key)=value;temp=[path '.tmp'];save(temp,'-struct','s','-v7');[ok,msg]=movefile(temp,path,'f');assert(ok,msg);
end
