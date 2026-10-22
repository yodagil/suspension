function [stable_mask,reason_mask,info] = native_stability_masks(cfg,G,wear,cache,tag,iw)
% Run the compiled reference equations; refuse unsupported parameter changes.
root=fileparts(mfilename('fullpath')); exe=fullfile(root,'native_accel');
assert(exist(exe,'file')==2,'Compile native_accel first with build_native.sh');
assert(cfg.g==9.81 && cfg.ux==cfg.speed_kmh/3.6,'Native gravity/speed mismatch');
assert(cfg.mu_reference==0.9 && strcmp(cfg.friction_scaling_mode,'relative_to_original_mu0p9'),'Unsupported native friction reference');
assert(cfg.mu_scale_x==cfg.mu_road/cfg.mu_reference && cfg.mu_scale_y==cfg.mu_road/cfg.mu_reference,'Native friction scaling mismatch');
assert(cfg.r_limit==cfg.limit_factor*cfg.mu_road*cfg.g/cfg.ux && cfg.ay_limit==cfg.mu_road*cfg.g,'Native constraint mismatch');
assert(numel(wear)==4 && all(ismember(wear,[0 25 50])),'Unsupported native wear input');
assert(isequal(cfg.k_spring_N_m,[30790 30790 30790 30790]));
assert(isequal(cfg.I_mA,[950 950 950 950]));
assert(isequal(cfg.steer_deg,[cfg.steer_deg(1) cfg.steer_deg(1) 0 0]));
assert(cfg.ignore_initial_s==0 && cfg.limit_factor==0.85 && cfg.LTR_limit==1);
assert(cfg.phi_hard_limit_rad==deg2rad(40) && cfg.vc_hard_limit_m_s==30 && cfg.r_hard_limit_rad_s==2);
assert(isequal(G.vc,(-20:20)*0.5),'Native MATLAB-reference vc grid mismatch');
assert(isequal(G.r,(-15:15)*(0.8/30)),'Native MATLAB-reference yaw grid mismatch');
assert(isequal(G.phi,(-15:15)*(0.8/30)),'Native MATLAB-reference roll grid mismatch');
assert(cfg.sim_time==5 && cfg.Ts==.01 && cfg.Nsub==20,'Full-resolution timing required');
n=numel(G.vc)*numel(G.r)*numel(G.phi);
signature=stability_run_signature(cfg,G,wear,'native');
base=fullfile(cache,sprintf('%s_wear%d',tag,iw));
bin=[base '.bin']; meta=[base '.json'];
if exist(bin,'file') && exist(meta,'file')
 info=jsondecode(fileread(meta));
 assert(info.total==n && dir(bin).bytes==n,'Incomplete cache');
 assert(isfield(info,'signature') && strcmp(info.signature,signature), ...
  'Native cache provenance mismatch; preserve it elsewhere before rerunning: %s',base);
else
 args=sprintf(' grid %.17g %.17g %.17g %d %d %d %d %s %.17g %.17g %d %d 4 %d %d %d', ...
  cfg.speed_kmh,cfg.mu_road,cfg.steer_deg(1),wear,quote_shell([bin '.tmp']), ...
  cfg.sim_time,cfg.Ts,cfg.Nsub,cfg.early_exit_on_unstable, ...
  numel(G.vc),numel(G.r),numel(G.phi));
 cmd=[quote_shell(exe) args ' > ' quote_shell([meta '.tmp'])];
 fprintf('Native full grid: speed=%g mu=%.1f steer=%g wear=[%d %d %d %d]\n', ...
  cfg.speed_kmh,cfg.mu_road,cfg.steer_deg(1),wear);
 [status,msg]=system(cmd);
 assert(status==0,'Native engine failed: %s',msg);
 info=jsondecode(fileread([meta '.tmp']));
 assert(info.total==n && dir([bin '.tmp']).bytes==n,'Native output is incomplete');
 info.signature=signature;
 fid=fopen([meta '.tmp'],'w'); assert(fid>=0); fprintf(fid,'%s',jsonencode(info)); fclose(fid);
 movefile([bin '.tmp'],bin,'f'); movefile([meta '.tmp'],meta,'f');
end
fid=fopen(bin,'rb'); assert(fid>=0); raw=fread(fid,inf,'*uint8'); fclose(fid);
assert(numel(raw)==n,'Wrong native result length');
assert(all(raw<=31),'Unknown native reason bits');
reason_mask=reshape(raw,[numel(G.vc),numel(G.r),numel(G.phi)]);
stable_mask=(reason_mask==0);
assert(nnz(stable_mask)==info.stable,'Native summary/mask mismatch');
info.engine='native_accel_cpp_openmp';
if cfg.early_exit_on_unstable, info.reason_semantics='through_first_violation';
else, info.reason_semantics='full_horizon_accumulated'; end
end

function out=quote_shell(s)
% POSIX single-quote escaping. Paths are never interpreted as shell code.
q=char(39); out=[q strrep(s,q,[q char(34) q char(34) q]) q];
end
