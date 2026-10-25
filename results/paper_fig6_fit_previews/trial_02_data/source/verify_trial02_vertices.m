function verify_trial02_vertices()
% Reintegrate every chosen vertex and exact equilibrium with half time step.
addpath('/workspace/suspension/paper_model_dynamic_roll/matlab');
root='/workspace/suspension/results/paper_fig6_fit_previews/trial_02_data';
cases=struct([]);
for angle=[0 3.5 7]
    src=load(fullfile(root,sprintf('paper_dynamic_roll_40kph_0.85mu_%gdeg.mat',angle)));
    s=src.PAPER_STABILITY3D;cfg=s.cfg;
    points=[s.region.vertices;reshape(s.region.equilibrium,1,3)];
    cfg.grid.vc=unique(points(:,1)).';cfg.grid.r=unique(points(:,2)).';cfg.grid.phi=unique(points(:,3)).';
    cfg.dt=.005;cfg.progress_path='';cfg.batch_size=3500;
    r=paper_simulate_grid(cfg);selected=false(size(points,1),1);
    for j=1:size(points,1)
        match=all(abs(bsxfun(@minus,r.initial_points,points(j,:)))<1e-10,2);
        assert(nnz(match)==1);selected(j)=r.stable_mask(find(match));
    end
    assert(all(selected),'A selected vertex or equilibrium failed dt=0.005 reintegration');
    c=struct('steer_deg',angle,'dt',cfg.dt,'selected_points',size(points,1), ...
        'all_selected_stable',all(selected),'nonfinite_failure_count',r.nonfinite_failure_count, ...
        'grid_test_count',r.total_count,'maximum_load_residual_N',r.maximum_load_equilibrium_residual_N);
    if isempty(cases),cases=c;else,cases(end+1)=c;end
    fprintf('HALF-DT CHECK steer=%g passed %d/%d vertices and equilibrium\n',angle,nnz(selected),numel(selected));
end
report=struct('passed',true,'all_24_vertices_and_3_equilibria_stable',true,'cases',cases, ...
    'scope','Half-step reintegration of selected boundary vertices and equilibria, not a continuous-region proof');
file='/workspace/suspension/validation/fig6_dynamic_roll/trial02_vertex_half_dt_checks.json';
fid=fopen(file,'w');assert(fid>=0);fprintf(fid,'%s',jsonencode(report));fclose(fid);
end
