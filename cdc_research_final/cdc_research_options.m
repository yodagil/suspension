function o = cdc_research_options()
% Explicit engineering choices; original tire/mass/CDC parameters unchanged.
o=struct();
o.speeds=50; o.mus=[0.8 0.9]; o.steers=[0 3];
o.wear_cases=[0 0 0 0;25 25 0 0;50 50 25 25;0 0 25 25;25 25 50 50];
o.grid=struct('vc',-3:0.1:3,'r',-1.5:0.025:1.5);
o.handling_grid=o.grid;
o.horizons=[5 10];o.Ts=0.01;o.Nsub=20;o.terminal_window=1;o.derivative_time_scale=1;
o.I_mA=[950 950 950 950];o.k_spring_N_m=[30790 30790 30790 30790];
o.initialization='trim';o.slice_mode='trim';o.fixed_phi0=0;
o.hidden_variants=[0 0 0 0]; % [delta roll-rate, front/rear alpha, load ay]
o.yaw_factor=0.85;o.ay_factor=1;o.LTR_limit=1;o.reject_force_clipping=true;
o.guard=struct('vc',30,'r',2,'phi',deg2rad(40),'kappa',0.5,'alpha',1.2,'omega',300);
o.state_tolerance=[0.10;0.005;0.002;deg2rad(0.2);deg2rad(0.2);0.002*ones(4,1);0.01;0.01;0.01;0.01*ones(4,1);0.001*ones(4,1);0.002*ones(4,1);0.2*ones(4,1);0.05];
o.trim_step_deg=0.25;o.trim_residual_tol=1e-6;o.eigenvalue_tolerance=1e-5;
o.batch_size=256;o.check_cell_centers=true;
o.random_points_per_cell=0;o.random_seed=20261111;
o.engine='mcode'; % 'native' after cdc_build_native; feedback requires mcode.
o.native_executable=fullfile(fileparts(mfilename('fullpath')),'native','cdc_research_native');
if ispc,o.native_executable=[o.native_executable '.exe'];end
o.native_threads=4;
o.controller=[]; % Optional @(t,X,Q) -> 4xN current [mA], sampled every Ts.
o.current_rate_limit_mA_s=Inf; % Identified rate limit can be supplied; no invented actuator dynamics.
o.trace_stride=0; % 0: off; otherwise integration steps per saved trace sample.
o.output_dir=fullfile(fileparts(mfilename('fullpath')),'results_new');
o.resume=true;o.overwrite=false;o.verbose=true;
end
