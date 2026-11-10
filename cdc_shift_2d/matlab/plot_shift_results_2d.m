function figs=plot_shift_results_2d(input,wear_index,horizon_index,kind)
%PLOT_SHIFT_RESULTS_2D Absolute vc/r initial-state sections at fixed initial roll.
%   plot_shift_results_2d(result,1) plots selected mask at the last horizon.
%   plot_shift_results_2d(folder,1,1,'safe') plots the first safety horizon.
% kind: selected, safe, guarded_safe, recoverable, guarded_recoverable,
%       safe_unresolved, or handling (separate frozen-load diagnostic).
% No projection over roll, hull, coordinate shift, or missing-region fallback.
if nargin<1, input=fullfile(fileparts(mfilename('fullpath')),'results_2d_full'); end
if nargin<2, wear_index=1; end
if nargin<3, horizon_index=[]; end
if nargin<4, kind='selected'; end
S={};
if isstruct(input)&&isfield(input,'results')
    z=input.results;
    for j=1:numel(z), if ~isempty(z{j}), S{end+1}=z{j}; end, end %#ok<AGROW>
else
    if isstruct(input)&&isfield(input,'files'), files=input.files;
    elseif iscell(input), files=input;
    elseif ischar(input)&&exist(input,'dir')==7
        d=dir(fullfile(input,'cdc2d_*.mat')); files={};
        for j=1:numel(d)
            if isempty(strfind(d(j).name,'checkpoint')) %#ok<STREMP>
                files{end+1}=fullfile(input,d(j).name); %#ok<AGROW>
            end
        end
    else, files={input};
    end
    for j=1:numel(files)
        a=load(files{j},'STABILITY2D');
        assert(isfield(a,'STABILITY2D'),'Not a fixed-roll 2-D result: %s',files{j});
        S{end+1}=a.STABILITY2D; %#ok<AGROW>
    end
end
assert(~isempty(S),'No results found');
keys=cell(size(S));
for j=1:numel(S)
    keys{j}=sprintf('%g km/h | mu=%g | %s | initial phi=%g rad', ...
        S{j}.meta.speed_kmh,S{j}.meta.mu_road,S{j}.cfg.initialization,S{j}.cfg.fixed_phi0);
end
groups=unique(keys); figs={};
for gg=1:numel(groups)
    ids=find(strcmp(keys,groups{gg}));
    angles=cellfun(@(s)s.cfg.steer_deg,S(ids)); [~,order]=sort(angles); ids=ids(order);
    f=figure('Name',[kind ' | ' groups{gg}]); ax=axes('Parent',f); hold(ax,'on'); colors=get(ax,'ColorOrder');
    for jj=1:numel(ids)
        s=S{ids(jj)}; assert(wear_index<=numel(s.CASE),'wear_index out of range');
        c=s.CASE(wear_index);
        if strcmp(kind,'handling')
            z=c.handling.mask; G=c.handling.grid; horizon_note='Frozen-load local handling diagnostic';
        else
            ih=horizon_index; if isempty(ih), ih=numel(s.cfg.horizons); end
            assert(ih>=1&&ih<=numel(s.cfg.horizons),'horizon_index out of range');
            if strcmp(kind,'selected')
                z=c.masks.recoverable(:,:,ih);
                if s.cfg.use_yaw_policy_for_region, z=z & c.masks.yaw_policy(:,:,ih); end
            else
                assert(isfield(c.masks,kind),'Unknown mask kind: %s',kind);
                a=c.masks.(kind); z=a(:,:,ih);
            end
            G=c.grid; horizon_note=sprintf('T=%g s; fixed initial phi=%g rad',s.cfg.horizons(ih),s.cfg.fixed_phi0);
        end
        color=colors(1+mod(jj-1,size(colors,1)),:);
        label=sprintf('delta=%+g deg | %d/%d selected',s.cfg.steer_deg,nnz(z),numel(z));
        if any(z(:)) && any(~z(:))
            [~,h]=contour(ax,G.vc,G.r,double(z'),[.5 .5],'LineWidth',1.5,'LineColor',color);
            set(h,'DisplayName',label);
        else
            % A missing or grid-filling contour is explicitly labelled, not replaced.
            plot(ax,NaN,NaN,'-','Color',color,'LineWidth',1.5,'DisplayName',label);
            fprintf('%s: delta=%g has %d/%d points; no interior contour.\n',kind,s.cfg.steer_deg,nnz(z),numel(z));
        end
    end
    xlabel(ax,'Initial lateral velocity v_y [m/s]'); ylabel(ax,'Initial yaw rate r [rad/s]');
    title(ax,{['Sampled 2-D ' kind ' mask'],horizon_note,groups{gg}},'Interpreter','none');
    xlim(ax,[G.vc(1) G.vc(end)]); ylim(ax,[G.r(1) G.r(end)]);
    grid(ax,'on'); legend(ax,'show','Location','best'); figs{end+1}=f; %#ok<AGROW>
end
end
