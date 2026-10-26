function fig = plot_mf210_final(results_dir,output_stem,visible)
%PLOT_MF210_FINAL Overlay actual hexahedra in a common paper-like camera.
%   plot_mf210_final('results')
%   plot_mf210_final('results','regions','off')
% Coordinates are displayed as x=r, y=vc, z=phi without scale or translation.
% The mathematical region retains its original [vc,r,phi] coordinate order.
  if nargin<1 || isempty(results_dir)
    results_dir=fullfile(fileparts(mfilename('fullpath')),'results');
  end
  if nargin<2 || isempty(output_stem)
    output_stem=fullfile(results_dir,'mf210_paper_roll_regions');
  end
  if nargin<3 || isempty(visible), visible='on'; end
  angles=[0 3.5 7];
  colors=[161 231 172;240 238 49;24 63 229]/255;
  alphas=[0.26 0.32 0.58];
  edges=[1 2;1 3;1 5;2 4;2 6;3 4;3 7;4 8;5 6;5 7;6 8;7 8];
  fig=figure('Color','w','Position',[80 80 1200 800],'Visible',visible);
  ax=axes('Parent',fig,'Position',[0.13 0.22 0.72 0.69]);
  hold(ax,'on');
  handles=[];
  for ii=1:numel(angles)
    tag=sprintf('%g',angles(ii));
    saved=load(fullfile(results_dir,['stable_point_40kph_0.85mu_' tag 'deg.mat']), ...
      'INNER_REGION','PAPER_STABILITY3D');
    region=saved.INNER_REGION;
    assert(region.valid && size(region.vertices,1)==8 && size(region.faces,1)==6, ...
      'Expected a valid eight-vertex, six-face hexahedron');
    vertices=region.vertices(:,[2 1 3]);
    render_faces=region.faces;
    if exist('OCTAVE_VERSION','builtin')~=0
      % Octave's headless gnuplot renderer only fills 3-D triangular patches.
      % Invisible internal diagonals only tessellate the six faces;
      % region.faces and the twelve boundary edges remain unchanged.
      render_faces=[region.faces(:,[1 2 3]);region.faces(:,[1 3 4])];
    end
    h=patch('Parent',ax,'Vertices',vertices,'Faces',render_faces, ...
      'FaceColor',colors(ii,:),'FaceAlpha',alphas(ii),'EdgeColor','none');
    if ii==1
      handles=repmat(h,1,3);
    else
      handles(ii)=h;
    end
    for jj=1:size(edges,1)
      p=vertices(edges(jj,:),:);
      line('Parent',ax,'XData',p(:,1),'YData',p(:,2),'ZData',p(:,3), ...
        'Color',[0.06 0.12 0.17],'LineWidth',1.0,'HandleVisibility','off');
    end
  end
  grid(ax,'on'); box(ax,'on');
  xlabel(ax,'r (rad/s)'); ylabel(ax,'v_c (m/s)'); zlabel(ax,'\phi (rad)');
  title(ax,'40 km/h, \mu = 0.85');
  set(ax,'XDir','reverse','XLim',[-0.72 0.72],'YLim',[-2 2],'ZLim',[-0.3 0.3], ...
    'XTick',[-0.6 -0.3 0 0.3 0.6],'YTick',[-2 -1 0 1 2], ...
    'ZTick',[-0.3 -0.15 0 0.15 0.3],'FontSize',11,'Projection','orthographic');
  pbaspect(ax,[1.3 2.28 1.65*0.6/0.84]);
  view(ax,-72,11);
  legend(ax,handles,{'0 deg','3.5 deg','7 deg'},'Location','northeastoutside');
  annotation(fig,'textbox',[0.10 0.075 0.80 0.04], ...
    'String','Quasisteady MF; dynamic roll-reaction extension of Eq. (11)', ...
    'HorizontalAlignment','center','EdgeColor','none','FontSize',10);
  annotation(fig,'textbox',[0.10 0.035 0.80 0.04], ...
    'String','Independent conservative stable-cell sheared regions; sampled states only', ...
    'HorizontalAlignment','center','EdgeColor','none','FontSize',10);
  print(fig,[output_stem '.png'],'-dpng','-r220');
  savefig(fig,[output_stem '.fig']);
end
