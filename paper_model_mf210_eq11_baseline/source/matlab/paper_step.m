function next=paper_step(x,cfg)
% Five-state RK4, 20 substeps per .01s observation; no hidden tire clamps.
h=cfg.dt/cfg.Nsub;next=x;
for j=1:cfg.Nsub
    k1=paper_rhs(next,cfg);k2=paper_rhs(next+.5*h*k1,cfg);
    k3=paper_rhs(next+.5*h*k2,cfg);k4=paper_rhs(next+h*k3,cfg);
    next=next+(h/6)*(k1+2*k2+2*k3+k4);
end
end
