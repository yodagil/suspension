function paper_selftest()
% Fast MATLAB smoke checks before running 39,401-point sweeps.
cfg=paper_config();
assert(isequal(cfg.grid.vc,(-20:20)*0.5));
assert(isequal(cfg.grid.r,(-15:15)*(0.8/30)));
assert(isequal(cfg.grid.phi,(-15:15)*(0.8/30)));
assert(numel(cfg.grid.vc)*numel(cfg.grid.r)*numel(cfg.grid.phi)==39401);
assert(abs(cfg.lf+cfg.lr-cfg.L)<1e-12);
x=[cfg.speed_kmh/3.6 0 0 0 0];
[dx,ob]=paper_rhs(x,cfg);
assert(all(isfinite(dx(:))));
assert(max(abs(dx(:)))<1e-7,'Zero-steer neutral state should be static.');
assert(abs(sum(ob.Fz_raw)-cfg.m*cfg.g)<1e-7);
cfg.steer_deg=3.5;
[dx,ob]=paper_rhs(x,cfg);
assert(all(isfinite(dx(:))));
assert(all(isfinite(ob.Fy_tires(:))));
assert(any(abs(ob.Fy_tires(:))>0));
assert(abs(dx(3))>0);
% Independent short-run smoke grid: do NOT mutate cfg default grid values.
cfg.grid.vc=[-0.5 0 0.5];
cfg.grid.r=[-0.1 0 0.1];
cfg.grid.phi=[-0.05 0 0.05];
cfg.horizon_s=0.06;
cfg.dt=0.02;
cfg.batch_size=100;
R=paper_simulate_grid(cfg);
assert(R.total_count==27);
assert(numel(R.stable_mask)==27);
assert(all(isfinite(R.max_abs_ay(:))));
G=paper_build_region(cfg,R.stable_mask);
assert(~G.valid || G.grid_unstable_inside==0);
fprintf('paper_selftest PASSED: published Table 1, exact grid, RHS, short sweep, region validation\n');
end
