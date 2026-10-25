function x=paper_initial(points,cfg)
% Five body states only; algebraic tire variables have no initial conditions.
x=[repmat(cfg.speed_kmh/3.6,size(points,1),1),points,zeros(size(points,1),1)];
assert(size(x,2)==5);
end
