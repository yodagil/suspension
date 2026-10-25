function cfg=paper_config()
% PRIVATE comparison: quasisteady MF free rolling with dynamic roll reaction.
% This replaces the hidden tire transients, not the original MF force kernel.
% It is neither original210 full tire dynamics nor a paper-exact reproduction.
cfg.paper_doi='10.1016/j.aei.2022.101764';
cfg.m=1300;cfg.ms=1300;cfg.Izz=2500;cfg.Ixx=700;
cfg.L=2.4;cfg.lf=1.2;cfg.lr=1.2;cfg.df=.7;cfg.dr=.7;
cfg.h=.54;cfg.hphi=.54;cfg.g=9.81;cfg.Kphi=43100;cfg.Cphi=1400;
cfg.deltaFz=[0 0 0 0];cfg.mu=.85;cfg.speed_kmh=40;cfg.steer_deg=0;
cfg.speed_mode='fixed';cfg.horizon_s=5;cfg.dt=.01;cfg.method='rk4';
cfg.Nsub=20;cfg.integration_dt=cfg.dt/cfg.Nsub;cfg.batch_size=3500;
cfg.limit_factor=.85;cfg.min_u=.5;cfg.filename_digits=3;
cfg.grid.vc=(-40:40)*.25;cfg.grid.r=(-27:27)*(.8/30);cfg.grid.phi=(-15:15)*(.8/30);
cfg.tire_model='Original210 healthy MF kernel with algebraic free-rolling slip';
cfg.mf_Fz0=5425;cfg.mf_force_min=100;cfg.mf_force_max=16000;
cfg.mf_mu_reference=.9;cfg.mf_mu_scaling_rule='mu_road / 0.9';cfg.mf_wear=[0 0 0 0];
cfg.slip_angle_mode='Instantaneous exact original210 wheel-frame velocity';
cfg.tire_dynamics='Quasisteady free rolling: solve kappa=-Shx, Fx=0';cfg.model_dimension=5;
cfg.free_rolling_constraint='kappa=0.001635-0.0025297*((clippedFz-5425)/5425), original Vx offset=0';
cfg.load_solver='explicit dynamic roll-reaction extension; zero baseline ay coefficient';
cfg.load_transfer_model='Dynamic roll-reaction extension of printed paper Eq11';
cfg.roll_reaction_equation='D=(m*h-ms*hphi)*ay+Cphi*p+Kphi*phi+ms*g*hphi*(sin(phi)-phi)';
cfg.roll_reaction_ay_coefficient=cfg.m*cfg.h-cfg.ms*cfg.hphi;
cfg.paper_exact_reproduction=false;
cfg.initialization='u=speed, vc/r/phi=grid, p=0; instantaneous algebraic tire forces';
cfg.reconstruction_version='PRIVATE-XiaoBody-MF210-QuasisteadyFreeRolling-DynamicRollReaction-v1';
end
