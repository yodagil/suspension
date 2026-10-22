# Stability sweep MAT results

Completed scenarios: **4/210**.

Each MAT file contains `STABILITY3D` with all five tire-wear cases, the original 41 x 31 x 31 initial-condition grid, and the 5-second classification horizon (Ts=0.01 s, 20 RK4 substeps). Front wheel steering is [angle, angle, 0, 0] degrees.

Stable points are finite-horizon constraint-compliant samples, not a proof of asymptotic stability. Cases where a valid inner polytope could not be extracted retain stable/unstable samples and record NaN geometry values. Results were computed using the validated GNU Octave/C++ engine.

Open a MAT file and use GitHub’s raw-file download button, or use its **Download** link below. New completed files are uploaded while the cloud computation and publisher remain active.

| Speed (km/h) | Road mu | Front wheel steer (deg) | MAT file |
|---|---|---|---|
| 50 | 0.9 | 0 | [Download](https://raw.githubusercontent.com/yodagil/suspension/main/results/stable_point_50kph_0.9mu_0deg.mat) |
| 50 | 0.9 | 0.5 | [Download](https://raw.githubusercontent.com/yodagil/suspension/main/results/stable_point_50kph_0.9mu_0.5deg.mat) |
| 50 | 0.9 | 1 | [Download](https://raw.githubusercontent.com/yodagil/suspension/main/results/stable_point_50kph_0.9mu_1deg.mat) |
| 50 | 0.9 | 1.5 | [Download](https://raw.githubusercontent.com/yodagil/suspension/main/results/stable_point_50kph_0.9mu_1.5deg.mat) |
