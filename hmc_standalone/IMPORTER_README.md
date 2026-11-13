# HMC runtime-data importer

`hmc_import_data.py` reads the uploaded expanded CarSim runtime echo, end-state echo, five-tire ZIP, and the active CDC MATLAB Function embedded in the Simulink SLX. It does not execute CarSim, Simulink callbacks or workbook macros.

Generated data:

- `data/hmc_data.mat`: MATLAB variable `D`.
- `data/vehicle.json`: SI vehicle parameters, retained table interpolation labels/native points, compliance data and one clearly marked benchmark trim.
- `data/tires.json`: all 262 numeric coefficients per supplied TIR, preserving front/rear25K/50K distinctions.
- `data/damper.json`: active Simulink integer force maps, current[mA], velocity[mm/s], force[N].
- `data/source_manifest.json`: original input file and individual tire-file SHA-256 values.

Run on this workspace using default attachment locations:

```sh
python hmc_import_data.py
```

Alternate locations are accepted with `--echo`, `--end`, `--tires`, `--slx` and `--output`. Python requires NumPy and SciPy.

MATLAB:

```matlab
load('data/hmc_data.mat','D');
V = D.vehicle;
P = D.tires.new;
map = D.damper.front_N; %18currents ×17velocities
```

`V.tables.<kind>.<FL|FR|RL|RR>` has `x_m` and `y_N`, `y_rad` or `y_m`. Kind names are `spring_comp`, `spring_ext`, `jounce_stop`, `rebound_stop`, `camber`, `toe`, `dive`, `susp_x`, `susp_lat`. Front steering kinematics use `V.tables.steering.FL/FR.x_rad/y_rad` for pitman-arm to wheel angle. `V.gear_rot_ratio` converts steering-wheel input to pitman-arm angle. `V.tables.steering_wheel.x_s/y_rad` is the *inactive internal time maneuver*, superseded by the external steering import; it is not a steering gear map.

The runtime echo is authoritative. Its four spring curves have slope30,790N/m and front/rear intercepts4,462.655/4,492.153N. These differ from the nonlinear spring curves in the earlier assembly-only PAR. `V.reference_trim.time_s=12` is the completed trim timestamp (`TSTART` of generated continuation echo); that echo's `TSTOP=24` is the next continuation end setting, not the timestamp of its saved state.

Standard gravity9.80665m/s² times supplied `R_GRAVITY` agrees with total benchmark vertical load divided by model mass to1.3e−8m/s². Antiroll coefficients documented in Nm/deg convert by180/pi toNm/rad. Wheel spin inertias include both supplied wheel and tire contributions.

The importer preserves data, not proprietary solver equations. Original `SPLINE` labels do not identify the exact VS interpolation algorithm. TNO tire dynamics and CarSim throttle/power implementation are not provided by parameter files. The benchmark trim must not be silently reused for all conditions. No parameters are fitted to stable-point classifications or region geometry. Uploaded customer data are private and are not automatically published.
