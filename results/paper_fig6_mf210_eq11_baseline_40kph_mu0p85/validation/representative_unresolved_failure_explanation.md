# Bounded first-failed-stage audit

The six requested benchmark cases were reproduced with the frozen production
RHS and exactly the same RK4 substep/stage order, without modifying that RHS.
Two initial states were checked at each of 0/3.5/7 degrees:

```text
[vc,r,phi] = [-6.75, -16*(0.8/30), -14*(0.8/30)]
[vc,r,phi] = [-4.00, +12*(0.8/30), -15*(0.8/30)].
```

The first failed *stage* states, preceding successful states and root hints,
exact stage times and global-root diagnostics are retained in
`failed_stage_root_audit.json`. They should not be confused with the last
completed observation state written to the production CSV.

| Steering | Initial state | First failed stage time, s | Failed-stage phi, rad |
|---|---|---:|---:|
| 0 | first | 0.37375 | 0.438180535 |
| 0 | second | 0.37400 | 0.466963899 |
| 3.5 | first | 0.38125 | 0.444691880 |
| 3.5 | second | 0.40350 | 0.490115238 |
| 7 | first | 0.38775 | 0.449340016 |
| 7 | second | 0.43750 | 0.498278906 |

For each failed stage, an independent evaluator scanned every smooth segment
between the preserved MF 100 N activation and 16000 N clipping breakpoints.
It used 4097 acceleration values per segment, independent SciPy Brent solves
for all sign changes, actual cut evaluations, and scalar absolute-residual
minimization to check for tangent roots. The search interval [-70,70] covers
the source force-derived acceleration bound at all three steering angles.

Every case had the same result: no residual-certified root was found, and
the residual `f(a)=a-Fy(Fz(a))/m` jumped across zero at the 100 N force
activation transition. For example, the first 0-degree case has a transition
at `a=8.1554958903 m/s²`, with one-sided residuals approximately
`-0.00177599` and `+0.16222653 m/s²`. A sign-bracketing solver that ignored
the force jump could falsely return that transition as a solution.

This is strong independent numerical evidence of a genuinely unsolved
algebraic closure caused by the inherited force-activation discontinuity,
rather than a simple missed smooth root in the engine's 16-sample fallback.
It is not a symbolic global root-exclusion theorem. The six already-recorded
wide RHS parity failures were checked with the same independent reference
and exhibited the same no-root/jump behavior.

Those trajectories must remain explicitly **unknown / unresolved model
closure**, rather than being relabeled as physical instability or stability.
Changing the 100 N activation gate or adding a load lag would define another
model and is not an authorized silent repair to this literal-Eq11 baseline.
The baseline can still be reported with its unknown mask, source hashes and
the reasons for these unresolved classifications.

No production source, parameter, classifier or existing result was changed
by this audit. `capture_first_failed_stage.cpp` and `check_failed_stages.py`
provide the read-only instrumentation and independent reference.
