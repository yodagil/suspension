# V2 Full-car native backend

This backend implements the uploaded `stability_fullcar_2d_v2.m` equations and classification, including all 32 states and CDC damping. It is not the planar/no-roll model.

Compile from the repository root:

```bash
g++ -std=c++17 -O3 -fopenmp fullcar_2d_v2/native/fullcar2d_v2_native.cpp -o fullcar_2d_v2/native/fullcar2d_v2_native
```

No fast-math is used. `frozen_native_model.cpp` preserves the previously validated 32-state equations. The adapter changes initial-state lifting to match V2's explicit `trim`, `flat`, and `legacy_phi0` slices and checks constraints at every RK4 node (`dt=0.0005 s` for the production configuration). There is no post-step state projection.

Use the MATLAB bridge/config exporter from **`fullcar_2d_v2/matlab`**. The text configuration starts with `FULLCAR2D_V2`; the initial configuration line includes the slice enum `0=trim`, `1=flat`, `2=legacy_phi0`. The distinct binary output header is `[0x46435632, 2, n, nh]` (four little-endian uint32 values), followed by the same raw diagnostic arrays as the uploaded M-code. These schemas intentionally reject the previous fixed-phi0 engine.

The executable supports `--config FILE --points FILE --output FILE --threads 4`, and independent checks through `--probe rhs`, `step`, `initial`, or `trajectory`. `rhs` and `step` read 32 state values from stdin; all other probes take a point file. Points are absolute `[vc,r]` coordinates in the MATLAB interface; the exporter supplies the fixed-phi parameter separately as an internal transport coordinate, which `trim` correctly ignores when a trim root exists.

Only the original fixed default vehicle parameters, 30790 N/m springs and 950 mA CDC currents per corner are supported by this compiled adapter. Parameter overrides are rejected. Pure MATLAB execution remains the default and needs no executable.
