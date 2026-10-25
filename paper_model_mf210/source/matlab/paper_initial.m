function x=paper_initial(points,cfg)
% Reference09 original tire initialization, deliberately not kinematic reset.
n=size(points,1);d=cfg.steer_deg*pi/180;delta=[d d 0 0];
omega=max(0,(cfg.speed_kmh/3.6)*cos(delta))/cfg.mf_Rw;
x=[repmat(cfg.speed_kmh/3.6,n,1),points,zeros(n,1), ...
    zeros(n,4),repmat(-delta,n,1),repmat(omega,n,1),zeros(n,1)];
assert(size(x,2)==18);
end
