function fig=plot_mf210_eq11_baseline(results_dir,output_stem,visible)
%PLOT_MF210_EQ11_BASELINE Actual sampled sets and accepted Sec3.1 regions.
% No invalid extreme-point hull is filled as an accepted stability region.
  if nargin<1||isempty(results_dir),results_dir=fullfile(fileparts(mfilename('fullpath')),'results');end
  if nargin<2||isempty(output_stem),output_stem=fullfile(results_dir,'eq11_baseline_comparison');end
  if nargin<3||isempty(visible),visible='on';end
  fig=figure('Color','w','Position',[50 100 1500 560],'Visible',visible);
  angles=[0 3.5 7];colors=[.2 .65 .32;.78 .64 .05;.15 .30 .84];
  for ii=1:3
    s=load(fullfile(results_dir,sprintf('stable_point_40kph_0.85mu_%gdeg.mat',angles(ii))));
    p=s.PAPER_STABILITY3D.result.stable_points;region=s.INNER_REGION;
    ax=subplot(1,3,ii,'Parent',fig);hold(ax,'on');
    if ~isempty(p)
      ids=unique(round(linspace(1,size(p,1),min(size(p,1),1800))));
      scatter3(ax,p(ids,2),p(ids,1),p(ids,3),5,colors(ii,:),'filled');
    end
    if region.valid
      faces=region.faces;
      if exist('OCTAVE_VERSION','builtin')~=0,faces=[faces(:,[1 2 3]);faces(:,[1 3 4])];end
      patch('Parent',ax,'Vertices',region.vertices(:,[2 1 3]),'Faces',faces, ...
        'FaceColor',colors(ii,:),'FaceAlpha',.25,'EdgeColor',[.2 .2 .2]);
      region_status=sprintf('Accepted inner V=%.4g',region.volume);
    else
      region_status='No accepted inner hexahedron';
    end
    plot3(ax,0,0,0,'kp','MarkerFaceColor','w','MarkerSize',9);
    set(ax,'XDir','reverse','XLim',[-.72 .72],'YLim',[-10 10],'ZLim',[-.4 .4], ...
      'Projection','orthographic');view(ax,-72,11);pbaspect(ax,[1.3 2.28 1.65]);
    xlabel(ax,'r (rad/s)');ylabel(ax,'v_c (m/s)');zlabel(ax,'\phi (rad)');grid(ax,'on');
    title(ax,{sprintf('%g deg | %d stable',angles(ii),s.PAPER_STABILITY3D.result.stable_count),region_status},'FontSize',10);
  end
  annotation(fig,'textbox',[.05 .02 .90 .06], ...
    'String','40 km/h fixed, mu=0.85 | Printed Eq11 + unchanged MF | Points: five-second sampled stable states | Star: initial origin', ...
    'HorizontalAlignment','center','EdgeColor','none','FontSize',10);
  print(fig,[output_stem '.png'],'-dpng','-r180');savefig(fig,[output_stem '.fig']);
end
