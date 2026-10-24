function cfg = paper_config()
% Xiao et al., Advanced Engineering Informatics 54 (2022) 101764.
% Published quantities are marked PUBLISHED. Other quantities are explicit
% RECONSTRUCTION ASSUMPTIONS, because numerical values were not reported.
% Independent reimplementation: original MF/Full-Car model is not used.
cfg.paper_doi = '10.1016/j.aei.2022.101764';
cfg.m = 1300;                    % kg PUBLISHED Table 1
cfg.Izz = 2500;                 % kg m^2 PUBLISHED Table 1
cfg.Ixx = 700;                  % kg m^2 PUBLISHED Table 1
cfg.L = 2.4;                    % m PUBLISHED Table 1
cfg.lf = 1.2;                   % m PUBLISHED Table 1
cfg.lr = cfg.L-cfg.lf;         % m inferred from published L, lf
cfg.h = 0.54;                   % m PUBLISHED Table 1
cfg.df = 0.7;                   % m PUBLISHED Table 1 (half-track)
cfg.dr = 0.7;                   % m PUBLISHED Table 1 (half-track)
cfg.Kphi = 43100;              % Nm/rad PUBLISHED Table 1
cfg.Cphi = 1400;               % Nm/(rad/s) PUBLISHED Table 1
cfg.g = 9.81;
% MISSING PAPER NUMBERS: user-adjustable, never presented as published.
cfg.ms = 1170;                  % kg ASSUMPTION: 90% of total mass
cfg.hphi = 0.35;                % m ASSUMPTION: CG above roll center
cfg.Ks = 1.0;                   % ASSUMPTION: lateral/longitudinal factor
cfg.burckhardt_shape = [1.28 23.99 0.52]; % ASSUMPTION: standard example shape
cfg.scale_burckhardt_to_mu = true; % ASSUMPTION: calibrate curve peak to road mu
cfg.lambda_long = [0 0 0 0];    % ASSUMPTION: freely rolling tires (no driven slip)
cfg.deltaFz = [0 0 0 0];        % no active suspension while mapping open-loop region
cfg.force_sign_y = -1;         % ASSUMPTION: restorative physical sign convention
% Change force_sign_y to +1 to evaluate the literal sign of printed Eq (10).
cfg.mu = 0.85;                  % PUBLISHED for Fig. 2, ASSUMED for Fig. 6
cfg.speed_kmh = 40;            % PUBLISHED for Fig. 6
cfg.steer_deg = 0;              % front tire angle (not steering wheel)
cfg.speed_mode = 'dynamic';     % Eq (1): dynamic. 'fixed' is an optional approximation.
cfg.horizon_s = 5.0;            % ASSUMPTION: publication does not specify time
cfg.dt = 0.01;                 % ASSUMPTION: refined integration/observation interval
cfg.method = 'rk4';            % ASSUMPTION: numerical solver
cfg.batch_size = 3500;         % working memory bound
cfg.load_solver = 'analytic piecewise-affine contact equilibrium';
cfg.reconstruction_version = 'xiao2022-audited-v2';
cfg.limit_factor = 0.85;      % Eq (13), used also as oversteer threshold
cfg.min_u = 0.5;               % numerical safety only
cfg.filename_digits = 3;
% EXACT ORIGINAL GRID (41 x 31 x 31 = 39,401 states)
cfg.grid.vc = (-20:20)*0.5;
cfg.grid.r = (-15:15)*(0.8/30);
cfg.grid.phi = (-15:15)*(0.8/30);
end
