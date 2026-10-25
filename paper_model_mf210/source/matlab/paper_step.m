function next=paper_step(x,cfg)
% Same .01s observation, 20 RK4 substeps and tire-state clamps as MF210.
h=cfg.dt/cfg.Nsub;next=x;
for j=1:cfg.Nsub
    k1=paper_rhs(next,cfg);k2=paper_rhs(next+.5*h*k1,cfg);
    k3=paper_rhs(next+.5*h*k2,cfg);k4=paper_rhs(next+h*k3,cfg);
    next=next+(h/6)*(k1+2*k2+2*k3+k4);
end
finite=all(isfinite(next),2);
next(finite,6:9)=min(.5,max(-.5,next(finite,6:9)));
next(finite,10:13)=min(1.2,max(-1.2,next(finite,10:13)));
next(finite,14:17)=min(300,max(0,next(finite,14:17)));
end
