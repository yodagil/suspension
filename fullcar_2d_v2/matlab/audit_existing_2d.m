function TAB=audit_existing_2d(location,make_plot)
%AUDIT_EXISTING_2D Inspect old/new-fullcar STABILITY2D MATs without rerunning.
% T=audit_existing_2d('C:\path\cdc_shift_2d_phi0');
% T=audit_existing_2d('C:\path\one_result.mat',true);
% Binary MATs must already be downloaded. No internet access is attempted.
% Saved safe/recovered labels retain their original constrained semantics.
if nargin<2,make_plot=false;end
if exist(location,'dir')
    files=dir(fullfile(location,'*.mat'));folder=location;
else
    assert(exist(location,'file')==2,'File/directory not found');
    files=dir(location);folder=fileparts(location);if isempty(folder),folder=pwd;end
end
rows=struct([]); plotted=false;
for k=1:numel(files)
    if ~isempty(strfind(files(k).name,'checkpoint')),continue;end %#ok<STREMP>
    path=fullfile(files(k).folder,files(k).name);
    info=whos('-file',path);if ~any(strcmp({info.name},'STABILITY2D')),continue;end
    d=load(path,'STABILITY2D');S=d.STABILITY2D;
    for iw=1:numel(S.CASE)
        if iscell(S.CASE),C=S.CASE{iw};else,C=S.CASE(iw);end
        if ~isfield(C,'raw')||~isfield(C.raw,'recovered'),continue;end
        raw=C.raw;safe=raw.safe(:,end);rec=raw.recovered(:,end);yaw=raw.yaw_ok(:,end);
        selected=C.stable_mask(:);reason=raw.reason(:,end);
        speed=S.meta.speed_kmh;mu=S.meta.mu_road;
        if isfield(S.meta,'wheel_steer_deg'),delta=S.meta.wheel_steer_deg(1);else,delta=NaN;end
        fac=.85;if isfield(S,'cfg')&&isfield(S.cfg,'yaw_factor'),fac=S.cfg.yaw_factor;end
        cap=fac*mu*9.81/(speed/3.6);trim=C.trim;
        actualphi=NaN;if isfield(C,'fixed_initial_roll_rad'),actualphi=C.fixed_initial_roll_rad;end
        row=struct('file',files(k).name,'speed_kmh',speed,'mu',mu,'steer_deg',delta, ...
            'wear_case',iw,'safe',nnz(safe),'recovered',nnz(rec), ...
            'guarded_safe',nnz(safe&yaw),'yaw_compliant_recovered',nnz(rec&yaw), ...
            'selected',nnz(selected),'recovered_removed_by_yaw',nnz(rec&~yaw), ...
            'safe_unresolved',nnz(safe&~rec),'sampled_cell_area',C.region.sampled_area, ...
            'trim_root',trim.root_found,'trim_usable',trim.usable, ...
            'trim_vy',trim.x(1),'trim_r',trim.x(2),'trim_phi',trim.x(4), ...
            'actual_phi0',actualphi,'yaw_cap',cap,'trim_outside_yaw_policy',abs(trim.x(2))>cap, ...
            'selected_touches_grid',touch(reshape(selected,size(C.stable_mask))), ...
            'recovered_touches_grid',touch(reshape(rec,size(C.stable_mask))), ...
            'safe_touches_grid',touch(reshape(safe,size(C.stable_mask))), ...
            'zero_area_but_nonzero_samples',nnz(selected)>0&&C.region.sampled_area==0, ...
            'yaw_reason',nnz(bitand(reason,uint16(1))), ...
            'ay_reason',nnz(bitand(reason,uint16(2))), ...
            'ltr_reason',nnz(bitand(reason,uint16(4))), ...
            'contact_reason',nnz(bitand(reason,uint16(8))), ...
            'axle_ltr_reason',nnz(bitand(reason,uint16(16))), ...
            'numeric_reason',nnz(bitand(reason,uint16(32))), ...
            'domain_reason',nnz(bitand(reason,uint16(64))), ...
            'load_clip_reason',nnz(bitand(reason,uint16(128))));
        rows(end+1)=row; %#ok<AGROW>
        if make_plot&&~plotted
            replot_existing_2d(path,iw);plotted=true;
        end
    end
end
assert(~isempty(rows),'No STABILITY2D simulation cases found');
TAB=struct2table(rows);writetable(TAB,fullfile(folder,'audit_2d_v2.csv'));
fprintf('Audited %d wear/condition records. CSV: %s\n',height(TAB),fullfile(folder,'audit_2d_v2.csv'));
disp(TAB(:,{'speed_kmh','mu','steer_deg','wear_case','recovered','selected','recovered_removed_by_yaw','sampled_cell_area'}));
fprintf('Empty selected despite nonempty recovery: %d records\n',nnz(TAB.selected==0&TAB.recovered>0));
fprintf('Nonzero selected samples but zero cell area: %d records\n',nnz(TAB.zero_area_but_nonzero_samples));
end
function y=touch(m)
y=any(m(1,:))||any(m(end,:))||any(m(:,1))||any(m(:,end));
end
