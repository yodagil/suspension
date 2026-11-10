function export_cdc_native_case(config_path,points_path,c,wear,T,points)
%EXPORT_CDC_NATIVE_CASE Exact scalar configuration for the CDC 2-D backend.
fid=fopen(config_path,'w'); assert(fid>=0,'Cannot write native configuration');
cleanup=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'FULLCAR2D_V2\n');
fprintf(fid,'%.17g %.17g %.17g\n',c.speed_kmh,c.mu,c.steer_deg);
fprintf(fid,'%.17g %.17g %.17g %.17g\n',wear);
switch c.initialization
 case 'legacy', mode=0;
 case 'kinematic_zero', mode=1;
 case 'trim', mode=2;
 otherwise, error('Unsupported initialization');
end
switch c.slice_mode
 case 'trim', slice=0;
 case 'flat', slice=1;
 case 'legacy_phi0', slice=2;
 otherwise, error('Unsupported V2 slice_mode');
end
fprintf(fid,'%.17g %d %.17g %.17g %d %d %d %d %d\n', ...
 c.Ts,c.Nsub,c.terminal_window,c.derivative_time_scale,c.batch_size,mode,T.root_found,T.usable,slice);
fprintf(fid,'%.17g %.17g %.17g %d\n',c.yaw_factor,c.ay_factor,c.LTR_limit,c.reject_force_clipping);
fprintf(fid,'%.17g %.17g %.17g %.17g %.17g %.17g\n', ...
 c.guard.vc,c.guard.r,c.guard.phi,c.guard.kappa,c.guard.alpha,c.guard.omega);
fprintf(fid,'%d ',numel(c.horizons)); fprintf(fid,'%.17g ',c.horizons); fprintf(fid,'\n');
fprintf(fid,'%.17g ',c.state_tolerance); fprintf(fid,'\n');
x=zeros(32,1); if T.root_found, x=T.x; assert(all(isfinite(x)),'Nonfinite found trim'); end
fprintf(fid,'%.17g ',x); fprintf(fid,'\n');
fprintf(fid,'%d\n',size(c.hidden_variants,1));
fprintf(fid,'%.17g %.17g %.17g %.17g\n',c.hidden_variants');
if ~isempty(points_path)
 if size(points,2)==2
  points=[points,repmat(c.fixed_phi0,size(points,1),1)];
 end
 assert(size(points,2)==3 && all(isfinite(points(:))),'Expected finite native [vc,r,phi] coordinates');
 f=fopen(points_path,'w'); assert(f>=0,'Cannot write native points');
 fprintf(f,'%d\n',size(points,1)); fprintf(f,'%.17g %.17g %.17g\n',points'); fclose(f);
end
end
