function signature = stability_run_signature(cfg,G,wear,stage)
%STABILITY_RUN_SIGNATURE Reproducibility key for classification/final caches.
% No paths, timestamps, credentials, or process state are part of the key.
% File contents are hashed directly, without a platform-specific shell tool.
root=fileparts(mfilename('fullpath'));
files={'fullcar_model.m','stability_run_signature.m'};
if ~isempty(strfind(stage,'native')) %#ok<STREMP>
 files=[files,{'native_accel.cpp','native_accel'}];
end
if ~strcmp(stage,'native')
 files=[files,{'stability_region.m','run_stability_sweep.m','native_stability_masks.m'}];
end
hashes=cell(size(files));
for i=1:numel(files)
 hashes{i}=file_sha256(fullfile(root,files{i}));
end
names={'speed_kmh','ux','mu_road','mu_reference','friction_scaling_mode','grid_generation','mu_scale_x','mu_scale_y','steer_deg', ...
 'Ts','sim_time','Nsub','k_spring_N_m','I_mA','g','limit_factor','r_limit', ...
 'ay_limit','LTR_limit','phi_hard_limit_rad','vc_hard_limit_m_s', ...
 'r_hard_limit_rad_s','ignore_initial_s','early_exit_on_unstable'};
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

function digest=file_sha256(path)
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
