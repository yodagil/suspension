function cases=run_requested_40kph(outdir,dt_override,make_plots)
%RUN_REQUESTED_40KPH Audited independent paper model, requested 0/3/7 degrees.
% Run without arguments after placing this file beside the other paper_*.m.
% The initial-state grid is 41x31x31; horizon 5 s; no Full-Car/MF code used.
root=fileparts(mfilename('fullpath'));addpath(root);
if nargin<1 || isempty(outdir),outdir=fullfile(root,'results_requested_40kph');end
if nargin<2,dt_override=[];end
if nargin<3,make_plots=true;end
if ~exist(outdir,'dir'),mkdir(outdir);end
cfg0=paper_config();if ~isempty(dt_override),cfg0.dt=dt_override;end
angles=[0 3 7];cases=struct([]);
sources={'paper_config.m','paper_rhs.m','paper_simulate_grid.m','paper_build_region.m','run_requested_40kph.m'};
hashes=cell(size(sources));
for j=1:numel(sources),hashes{j}=source_sha256(fullfile(root,sources{j}));end
status=struct('phase','running','total',3,'completed_count',0,'current_scenario_index',1, ...
    'current_steer_deg',0,'speed_kmh',40,'mu',.85,'dt',cfg0.dt, ...
    'initial_conditions_per_scenario',39401,'source_files',{sources},'source_sha256',{hashes});
write_status(fullfile(outdir,'job_status.json'),status);
try
 for k=1:3
    cfg=cfg0;cfg.speed_kmh=40;cfg.mu=.85;cfg.steer_deg=angles(k);
    cfg.progress_path=fullfile(outdir,'progress.json');
    status.current_scenario_index=k;status.current_steer_deg=angles(k);
    write_status(fullfile(outdir,'job_status.json'),status);
    write_status(cfg.progress_path,struct('steer_deg',angles(k),'processed_initial_states',0, ...
        'total_initial_states',39401,'elapsed_s',0));
    fprintf('\nPAPER SCENARIO %d/3: 40 km/h, mu=0.85, steer=%g deg, dt=%g\n',k,angles(k),cfg.dt);
    result=paper_simulate_grid(cfg);
    CLASSIFICATION=struct('cfg',cfg,'result',result);
    save(fullfile(outdir,sprintf('classification_%gdeg.mat',angles(k))),'CLASSIFICATION','-v7');
    region=paper_build_region(cfg,result.stable_mask);
    meta=struct('run_version','xiao2022-audited-v2','engine','MATLAB-compatible M-code', ...
        'paper_doi',cfg.paper_doi,'model','Independent five-state paper body and Burckhardt tire model', ...
        'source_files',{sources},'source_sha256',{hashes}, ...
        'published_figure6_angles_deg',[0 3.5 7],'requested_angles_deg',angles, ...
        'paper_exact_reproduction',false,'numerical_solver','RK4', ...
        'completed_local',datestr(now,30), ...
        'assumptions',{{'ms=1170 kg','hphi=0.35 m','Burckhardt shape [1.28 23.99 0.52]', ...
            'Ks=1','lambda_long=0','initial roll rate=0','5-second horizon', ...
            'dynamic longitudinal speed','mu=0.85 for requested comparison', ...
            'discrete inward ranks and support selection for eight inner vertices'}});
    scenario=struct('meta',meta,'cfg',cfg,'result',result,'region',region);
    if isempty(cases),cases=scenario;else,cases(k)=scenario;end
    PAPER_STABILITY3D=scenario;
    filename=sprintf('paper_stable_point_40kph_0.85mu_%gdeg.mat',angles(k));
    temp=fullfile(outdir,[filename '.tmp']);save(temp,'PAPER_STABILITY3D','-v7');
    movefile(temp,fullfile(outdir,filename),'f');
    status.completed_count=k;write_status(fullfile(outdir,'job_status.json'),status);
    fprintf('PAPER SAVED %d/3: stable=%d/39401 inner_volume=%.12g valid=%d\n', ...
        k,result.stable_count,region.volume,region.valid);
 end
 save(fullfile(outdir,'paper_comparison.mat'),'cases','-v7');
 status.phase='completed';status.current_scenario_index=0;
 write_status(fullfile(outdir,'job_status.json'),status);
 if make_plots,paper_plot_region(cases,true);end
catch failure
 status.phase='failed';status.error=failure.message;
 write_status(fullfile(outdir,'job_status.json'),status);rethrow(failure);
end
end

function write_status(path,value)
temp=[path '.tmp'];fid=fopen(temp,'w');assert(fid>=0,'Cannot write progress');
fprintf(fid,'%s',jsonencode(value));fclose(fid);movefile(temp,path,'f');
end

function digest=source_sha256(path)
fid=fopen(path,'rb');assert(fid>=0,'Cannot fingerprint source');
cleanup=onCleanup(@()fclose(fid));bytes=fread(fid,inf,'*uint8'); %#ok<NASGU>
if exist('OCTAVE_VERSION','builtin')
 digest=hash('sha256',char(bytes.'));
else
 assert(usejava('jvm'),'Source hashing requires MATLAB JVM');
 md=java.security.MessageDigest.getInstance('SHA-256');md.update(typecast(bytes,'int8'));
 raw=typecast(md.digest(),'uint8');digest=lower(reshape(dec2hex(raw,2).',1,[]));
end
end
