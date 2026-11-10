# CDC 2-D native backend

The backend integrates the uploaded 32-state Full-Car equations using plain
RK4 at `dt=Ts/Nsub`. It observes the uploaded physical/domain constraints at
every integration substep and preserves all 5/10-second masks separately.
There is no post-step state projection.

`frozen_native_model.cpp` is an exact copy of the previously validated model;
the new classifier, initialization, and output protocol are in
`cdc2d_native.cpp`. Only the attached default physical parameters, four
30790 N/m spring commands, and four 950 mA CDC currents are supported.

Build before starting the calculation:

```bash
g++ -O3 -std=c++17 -fopenmp cdc2d_native.cpp -o cdc2d_native
```

The production MATLAB driver uses the **2-D helpers in `../matlab`** and an
absolute `native_executable` path. Do not shadow them by adding this directory
after the MATLAB directory on the MATLAB/Octave search path. This directory's
export helper also supports explicit three-coordinate probe points.

Batch CLI:

```text
cdc2d_native --config case.txt --points points.txt --output raw.bin --threads 4
```

Configuration serialization is specified by `export_cdc_native_case.m`.
Points are absolute `[vc, r, phi]`; production fixes `phi=0`. The binary starts
with four little-endian uint32 values `[0x43444332, 1, n, nh]`, followed by
float64 column-major arrays: `safe`, `yaw_ok`, `recovered`, `reason`,
`terminal_error`, `terminal_derivative` (each `n x nh`), then
`first_physical_violation`, `first_yaw_violation`, `peak_r`, `peak_ay`,
`peak_LTR`, `min_Fz` (each `n x 1`). IEEE infinities are preserved.

Independent probes use `--probe rhs` or `--probe step` with 32 stdin state
values, or `--probe initial` / `--probe trajectory` with a points file. The
step probe is **one plain RK4 substep**, not one observation interval.

The uploaded batch routine has an early break when an entire point batch
fails before a horizon. The backend reproduces its resulting infinite future
terminal diagnostics using the same configured `batch_size`; this behavior
does not change any classification mask.
