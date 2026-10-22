# 80 km/h, mu=0.9, wheel steer=2 deg: reference reproduction

This retest uses the original first-code tire calibration at the reference road setting `mu_reference=0.9`.

The previous second-code normalization used `mu/1.145` longitudinally and `mu/1.0278` laterally. At mu=0.9 these are 0.7860262009 and 0.8756567426, so it changed the tire-force curves relative to the first code, whose mu parameter only set classification limits.

The corrected settings are:

```matlab
cfg.mu_road = 0.9;
cfg.mu_reference = 0.9;
cfg.mu_scale_x = cfg.mu_road/cfg.mu_reference;
cfg.mu_scale_y = cfg.mu_road/cfg.mu_reference;
```

This is a relative road-friction calibration. The original MF coefficients are preserved. At mu=0.9 both force scale factors are 1; it does not redefine the original MF peak coefficients as exactly 0.9.

The state grid uses MATLAB-compatible symmetric construction:

```matlab
gridC.vc  = (-20:20)*0.5;
gridC.r   = (-15:15)*(0.8/30);
gridC.phi = (-15:15)*(0.8/30);
gridF.vc  = (-4:4)*0.5;
gridF.r   = (-4:4)*(0.7/8);
gridF.phi = (-4:4)*(0.4/8);
```

An Octave linspace rounding difference of 5.55e-17 changed a tied support-point choice in wear case 5. The symmetric grids match the reference MATLAB values exactly.

The retest preserves all five wear cases, 39,401 initial conditions per wear case, a 5-second horizon, Ts=0.01 s, and 20 RK4 substeps. `early_exit_on_unstable=false` matches the reference accumulated reason-mask diagnostics. Nonfinite states are marked as numerical failure; the reference contains no numerical-failure samples.

The original 210-scenario computation and automatic publisher were stopped. Only the 80-kph reference retest and the subsequently requested 50-kph retest are authorized to run; the global sweep remains stopped. Final comparison evidence is stored in `reference_comparison.json` after the full retest passes.
