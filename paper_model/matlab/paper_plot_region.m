function paper_plot_region(cases,display_raw)
% Side-by-side and overlaid 3-D comparison of steering scenarios.
if nargin<2
    display_raw=true;
end
n=numel(cases);
colors=lines(max(n,3));
fig=figure('Color','w','Name','Xiao2022 paper-based stability regions');
for k=1:n
    subplot(1,n,k);
    hold on;
    C=cases(k);
    if display_raw && ~isempty(C.result.stable_points)
        S=C.result.stable_points;
        skip=max(1,ceil(size(S,1)/5000));
        if exist('OCTAVE_VERSION','builtin')
            scatter3(S(1:skip:end,1),S(1:skip:end,2),S(1:skip:end,3),5,colors(k,:),'filled');
        else
            scatter3(S(1:skip:end,1),S(1:skip:end,2),S(1:skip:end,3),5,colors(k,:),'filled','MarkerFaceAlpha',0.12,'MarkerEdgeAlpha',0.12);
        end
    end
    if C.region.valid
        patch('Vertices',C.region.vertices,'Faces',C.region.faces,'FaceColor',colors(k,:),'FaceAlpha',0.18,'EdgeColor',colors(k,:),'LineWidth',1.3);
    end
    xlabel('v_c (m/s)'); ylabel('r (rad/s)'); zlabel('\phi (rad)');
    title(sprintf('%g deg | %d stable | V=%.3g',C.cfg.steer_deg,C.result.stable_count,C.region.volume));
    view(45,18); grid on; axis tight;
    xlim([-10 10]); ylim([-0.4 0.4]); zlim([-0.4 0.4]);
end
set(fig,'Position',[80 140 max(1050,390*n) 450]);
fig2=figure('Color','w','Name','Xiao2022 overlay of inner regions');
hold on;
legendHandles={};
labels={};
for k=1:n
    C=cases(k);
    if C.region.valid
        h=patch('Vertices',C.region.vertices,'Faces',C.region.faces,'FaceColor',colors(k,:),'FaceAlpha',0.12,'EdgeColor',colors(k,:),'LineWidth',2);
        legendHandles{end+1}=h;
        labels{end+1}=sprintf('\\delta=%g deg, V=%.3g',C.cfg.steer_deg,C.region.volume);
    end
end
if ~isempty(legendHandles)
    legend([legendHandles{:}],labels,'Location','northeast');
else
    text(0,0,0,'No valid sampled-grid inner hexahedron for these conditions');
end
xlabel('v_c (m/s)'); ylabel('r (rad/s)'); zlabel('\phi (rad)');
xlim([-10 10]); ylim([-0.4 0.4]); zlim([-0.4 0.4]);
view(42,20); grid on;
title('Paper-based reconstruction, NOT guaranteed exact published Fig. 6');
end
