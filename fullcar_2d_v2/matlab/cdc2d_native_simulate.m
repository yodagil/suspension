function out=cdc2d_native_simulate(points,Q,c,T)
%CDC2D_NATIVE_SIMULATE Native integration with the attached M-code semantics.
assert(all(c.I_mA(:)==950)&&all(c.k_spring_N_m(:)==30790), ...
 'The native backend supports only 30790 N/m and 950 mA per corner.');
assert(isempty(fieldnames(c.parameter_overrides)),'Native backend does not support parameter_overrides');
assert(size(points,2)==2,'Native 2-D bridge requires [vc,r] points');
assert(isfield(c,'native_executable')&&exist(c.native_executable,'file')==2,'Native executable missing');
work=tempname; mkdir(work); cleanup=onCleanup(@()rmdir(work,'s')); %#ok<NASGU>
config=fullfile(work,'case.txt'); pts=fullfile(work,'points.txt'); raw=fullfile(work,'raw.bin');
export_cdc_native_case(config,pts,c,Q.wear,T,points);
threads=4; if isfield(c,'native_threads'),threads=c.native_threads;end
quote=@(s) ['''' strrep(s,'''','''"''"''') ''''];
command=sprintf('%s --config %s --points %s --output %s --threads %d', ...
 quote(c.native_executable),quote(config),quote(pts),quote(raw),threads);
[status,message]=system(command);assert(status==0,'Native CDC simulation failed: %s',message);
n=size(points,1); nh=numel(c.horizons);
f=fopen(raw,'rb','ieee-le');assert(f>=0,'Missing native raw output');
h=fread(f,4,'*uint32'); assert(isequal(double(h(:))',[hex2dec('46435632') 2 n nh]),'Native raw header mismatch');
out=struct(); names={'safe','yaw_ok','recovered','reason','terminal_error','terminal_derivative'};
for j=1:numel(names)
 a=fread(f,[n nh],'double'); assert(numel(a)==n*nh,'Incomplete native output');
 if j<=3,a=logical(a);elseif j==4,a=uint16(a);end
 out.(names{j})=a;
end
names={'first_physical_violation','first_yaw_violation','peak_r','peak_ay','peak_LTR','min_Fz'};
for j=1:numel(names)
 a=fread(f,[n 1],'double');assert(numel(a)==n,'Incomplete native diagnostics');out.(names{j})=a;
end
assert(isempty(fread(f,1,'uint8')),'Unexpected trailing native data');fclose(f);
out.status=uint8(out.safe);out.status(out.recovered)=uint8(2);
out.status_definition='0 constraint/domain violation; 1 safe but recovery unresolved; 2 safe and recovered';
out.first_yaw_violation_note='Observed until physical/domain failure; not necessarily full-horizon yaw history';
end
