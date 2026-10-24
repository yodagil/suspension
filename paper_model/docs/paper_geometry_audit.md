# Independent geometry and stability-definition audit

Paper: Xiao et al., Advanced Engineering Informatics 54 (2022) 101764, DOI 10.1016/j.aei.2022.101764. Page numbers below are the printed/PDF page numbers.

## Published procedure and observed trend

Section 3.1, pp. 4–5, describes a stable initial state as one that can continue operating without excessive yaw, sideslip, or rollover. The paper does not specify a finite classification horizon. Figure 4 and the boxed four-step procedure on p. 5 select (1) extreme stable roll angles at each fixed lateral speed and yaw rate, (2) inner and outer lateral-speed limit stable points within those roll-limit points, (3) eight extreme points from each limit set to form inner/outer approximations, and (4) the inner region, because the outer region contains unstable points.

The exact inner-branch selection, support directions, tie rules, candidate search, and numerical tolerances are not supplied. They must be declared as reconstruction choices rather than claimed to be the paper's unpublished algorithm.

Figure 6 explicitly uses 40 km/h and steering angles 0, 3.5, and 7 degrees. The current user requests 0, 3, and 7 degrees, so that is a qualitative comparison rather than an exact Figure 6 replication. The plotted hexahedra become smaller and retreat toward a positive-yaw subregion as steering increases. Their centers/bounds need not each decrease in absolute value. The horizontal yaw axis reaches approximately +/-0.6 rad/s, and the largest plotted region extends beyond +/-0.4. Thus the requested inherited yaw grid +/-0.4 can clip the zero-steering region; boundary contact must be reported rather than hidden.

## ZIP geometry mismatch

The supplied paper_build_region.m computes roll_limit_points, inner_limit_points, and outer_limit_points, but its final vertices do not use either lateral-limit set. Instead it PCA-aligns the entire stable cloud and contracts one oriented box about a median-nearest stable anchor. Consequently it is not the stated Step 3 extreme-inner-limit construction.

The contracted PCA vertices are generally not actual tested grid states. The validity test only requires positive volume and no sampled unstable point inside; a tiny positive-volume box surrounding one stable anchor can pass this test. This does not make the box a proof of stability between grid points, nor does it reproduce the paper's selected boundary vertices. A change in its volume may result from PCA orientation or contraction geometry rather than a change in the underlying stable-state set.

## Concrete boundary-derived reconstruction

1. Keep the minimum and maximum stable roll-angle points at each fixed (vc,r).
2. At each fixed (r,phi) in that roll-limit point set, sort actual vc values. Depth 1 supplies outer endpoints. For preset depths d=2,...,ceil(n/2), choose the d-th endpoint from either side to obtain candidate inner-limit sets. This inward-rank interpretation is an explicit discrete reconstruction assumption, not a published formula.
3. Normalize using fixed original-grid ranges and center each candidate cloud at its mean. Use eight signed support directions/octants to select eight distinct actual stable grid points; deterministic score ties use original grid linear index. If necessary, search a fixed support-rank range within the same inner-limit cloud, not the entire stable cloud. Reject missing octants, duplicates, rank-deficient sets, or selections that do not have eight convex-hull vertices. Build triangle faces using convhulln rather than assuming that arbitrary vertices have the PCA cube's fixed adjacency.
4. Check every sampled unstable point against the inclusive hull halfspaces. Select the largest positive-volume candidate with zero sampled unstable points. Report a construction failure as invalid/NaN and retain raw data; do not identify it with an empty stable-state set. Do not shrink arbitrary boxes or tune geometry separately by steering angle to impose monotonic volumes.

Report raw stable count/fraction, raw stable-cloud convex-hull volume, coordinate spans and centers, boundary-contact counts, and accepted inner-polytope volume. A trend claim should be based on these jointly, with the inner-region approximation method identified. Optionally simulate each newly selected off-grid geometry vertex if an interpolated method is retained.

## Classification/runtime notes

The supplied early exit after a first violation preserves the finite-horizon stable mask. Its max-yaw/max-acceleration/min-load diagnostics, however, are observed only until first failure, not across a full trajectory for unstable cases. Bit 8 combines nonfinite numerical failure with u<0.5 m/s domain termination. The latter is a modeling-domain guard, not necessarily a numerical integration failure.

The paper's rollover wording concerns wheels on one side leaving the ground. The code rejects any individual wheel lift. Under the supplied baseline load model, equal front/rear half tracks, zero active suspension forces, and proportional front/rear load split make the two wheels on a side reach lift together, so the two conditions coincide in that baseline; they need not coincide after parameter changes.

The 5-second horizon, 0.02-second RK4 step, initial zero roll rate, and other omitted parameters remain assumptions. Check at least a finer integration step and longer horizon before treating an observed trend as robust. Exact paper vertices and numerical Figure 6 values cannot be established from the supplied information alone.
