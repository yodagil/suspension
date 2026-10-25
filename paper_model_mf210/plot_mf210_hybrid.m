function fig = plot_mf210_hybrid(result_dir,output_base)
%PLOT_MF210_HYBRID Plot unchanged original-210 inner vertices, common camera.
% Display coordinate order is [r,vc,phi]; data remain in [vc,r,phi].
% This function only loads saved MAT results and never changes their geometry.
  if nargin<1 || isempty(result_dir)
    result_dir=fullfile(fileparts(mfilename('fullpath')),'results');
  end
  if nargin<2 || isempty(output_base)
    output_base=fullfile(result_dir,'mf210_hybrid_regions');
  end
  tags={'0','3.5','7'};
  colors=[.64 .92 .66;.95 .93 .20;.09 .24 .90];
  alpha=[.16 .24 .65];
  fig=figure('Color','w','Position',[100 100 780 760]);
  ax=axes('Parent',fig);hold(ax,'on');
  handles=zeros(1,0);labels={};
  for ii=1:numel(tags)
    region_file=fullfile(result_dir,['region210_' tags{ii} 'deg.mat']);
    if ~exist(region_file,'file')
      region_file=fullfile(result_dir,['stable_point_40kph_0.85mu_' tags{ii} 'deg.mat']);
    end
    loaded=load(region_file,'REGION210');
    selected=loaded.REGION210.region;
    if ~selected.is_valid_inner || size(selected.inner_vertices,1)<4
      warning('No valid inner region at delta=%s degrees; raw classifications remain saved.',tags{ii});
      continue;
    end
    V=selected.inner_vertices(:,[2 1 3]);
    faces=convhulln(V);
    p=patch('Parent',ax,'Vertices',V,'Faces',faces, ...
      'FaceColor',colors(ii,:),'FaceAlpha',alpha(ii),'EdgeColor','none');
    if isempty(handles),handles=p;else,handles(end+1)=p;end %#ok<AGROW>
    labels{end+1}=sprintf('delta = %s deg',tags{ii}); %#ok<AGROW>
    edges=unique(sort([faces(:,[1 2]);faces(:,[2 3]);faces(:,[3 1])],2),'rows');
    % Remove triangulation diagonals shared by coplanar hull triangles.
    for jj=1:size(edges,1)
      e=edges(jj,:);
      adjacent=find(sum(ismember(faces,e),2)==2);
      diagonal=false;
      if numel(adjacent)==2
        T1=V(faces(adjacent(1),:),:);T2=V(faces(adjacent(2),:),:);
        n1=cross(T1(2,:)-T1(1,:),T1(3,:)-T1(1,:));
        n2=cross(T2(2,:)-T2(1,:),T2(3,:)-T2(1,:));
        diagonal=norm(cross(n1,n2))<=1e-10*norm(n1)*norm(n2);
      end
      if ~diagonal
        line('Parent',ax,'XData',V(e,1),'YData',V(e,2),'ZData',V(e,3), ...
          'Color',colors(ii,:)*.65,'LineWidth',1.0,'HandleVisibility','off');
      end
    end
  end
  % Identical fixed axes and orthographic camera for all steering cases.
  set(ax,'XLim',[-.72 .72],'XDir','reverse','YLim',[-5 5], ...
    'ZLim',[-.45 .45],'XTick',-.6:.2:.6,'YTick',[-5 0 5], ...
    'ZTick',-.4:.2:.4,'Projection','orthographic','Box','off');
  view(ax,-72,11);pbaspect(ax,[1.3 2.28 1.65*.9/.84]);grid(ax,'off');
  xlabel(ax,'r (rad/s)');ylabel(ax,'v_c (m/s)');zlabel(ax,'phi (rad)');
  title(ax,'40 km/h fixed, mu = 0.85 | MF210 transient hybrid');
  if ~isempty(handles),legend(ax,handles,labels,'Location','northwest');end
  set(fig,'PaperPositionMode','auto');
  print(fig,[output_base '.png'],'-dpng','-r180');
  if exist('OCTAVE_VERSION','builtin')
    hgsave(fig,[output_base '.ofig']);
  else
    savefig(fig,[output_base '.fig']);
  end
end
