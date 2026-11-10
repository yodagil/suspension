function fig=plot_cdc_research(input,caseIndex,kind)
% Plot saved RESEARCH2D only; no simulation and no data shifting.
% kind: final (accepted-cell edges + H reference), reference, points,
%       guarded_final, prepeak_final, recoverable_on_reference.
if nargin<2,caseIndex=1;end
if nargin<3,kind='final';end
if isstruct(input)&&isfield(input,'files'),files=input.files;
elseif ischar(input)||isstring(input)
 input=char(input);
 if isfolder(input)
  d=dir(fullfile(input,'research_*.mat'));files=arrayfun(@(f)fullfile(f.folder,f.name),d,'UniformOutput',false);
 else,files={input};end
elseif iscell(input),files=input;
else,error('Expected a result, filename, folder, or cell array of filenames');end
assert(~isempty(files),'No research result files');
fig=figure('Color','w','Name','CDC research: saved finite-horizon regions','NumberTitle','off');
ax=axes('Parent',fig);hold(ax,'on');grid(ax,'on');box(ax,'on');
hh=gobjects(0);labels={};
for k=1:numel(files)
 D=load(files{k},'RESEARCH2D');S=D.RESEARCH2D;
 if iscell(S.CASE),C=S.CASE{caseIndex};else,C=S.CASE(caseIndex);end
 name=sprintf('U=%g mu=%g delta=%+g wear[%g %g %g %g]',S.meta.speed_kmh,S.meta.mu_road,S.meta.wheel_steer_deg(1),C.wear);
 if strcmp(kind,'final')
  [~,h]=contour(ax,C.grid.vc,C.grid.r,double(C.handling.mask)',[.5 .5],'LineStyle','--','LineWidth',1.2);hh(end+1)=h;labels{end+1}=[name ' H reference'];
  p=cdc_cell_boundary(C.grid.vc,C.grid.r,C.region.cell_mask);
  if ~isempty(p),h=plot(ax,p(:,1),p(:,2),'LineWidth',1.8);hh(end+1)=h;labels{end+1}=sprintf('%s tested cells (A=%.5g)',name,C.region.cell_area);end
 elseif strcmp(kind,'reference')
  [~,h]=contour(ax,C.grid.vc,C.grid.r,double(C.handling.mask)',[.5 .5],'LineWidth',1.6);hh(end+1)=h;labels{end+1}=[name ' H reference'];
 elseif strcmp(kind,'points')
  [v,r]=ndgrid(C.grid.vc,C.grid.r);m=C.masks.final(:,:,end);h=scatter(ax,v(m),r(m),12,'filled');hh(end+1)=h;labels{end+1}=[name ' passed initial points'];
 else
  assert(isfield(C.masks,kind),'Unknown saved mask');m=C.masks.(kind);if ndims(m)==3,m=m(:,:,end);end
  [~,h]=contour(ax,C.grid.vc,C.grid.r,double(m)',[.5 .5],'LineWidth',1.6);hh(end+1)=h;labels{end+1}=[name ' ' kind];
 end
 plot(ax,C.trim.x(1),C.trim.x(2),'kx','LineWidth',1.5,'MarkerSize',8,'HandleVisibility','off');
end
xlabel(ax,'Lateral velocity V_y [m/s]');ylabel(ax,'Yaw rate r [rad/s]');
title(ax,{'CDC full-car: finite-horizon, reference-preserving recovery','Sampled initial-state section; not an invariant-set certificate'});
if ~isempty(hh),legend(ax,hh,labels,'Interpreter','none','Location','best','FontSize',8);end
end
