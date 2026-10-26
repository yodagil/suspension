# Independent conservative cell-cover review

The first point-only oblique candidate passed every existing grid-point test,
but its new off-grid tests failed at 0 and 3.5 degrees. The recorded failures
are model trajectory failures, not a rasterization defect. That candidate must
remain rejected as a validated inner-region deliverable.

The follow-up source `empty_sheared_cell_box.cpp`, SHA-256
`b1b44f2049faeed3bc89d6b4d125931c9bd76255938608dc8d8ac7e17f13c0ff`,
adds a clearly stronger, common geometric rule. A physical unit grid cell is
eligible only when all eight existing corner classifications are stable.
For each ineligible cell, all inverse-sheared coordinates are bounded by

```text
qv_low  = v   - max(sv*t0, sv*t1)
qv_high = v+1 - min(sv*t0, sv*t1)
qp_low  = p   - max(sp*t0, sp*t1)
qp_high = p+1 - min(sp*t0, sp*t1).
```

These bounds are correct extrema of linear coordinates on a physical cell.
They are exact marginal bounds, but their Cartesian product is generally a
superset of the actual correlated projection. Excluding this complete bounding
box is consequently conservative. The search should not claim a maximum over
all regions lying in the eligible-cell cover; it finds a maximum in its stated
finite, conservatively filtered candidate family.

The native rule marks q-unit cells with open-interior overlap with this box.
The integer indices `floor(low)` through `ceil(high)-1` are correct, including
negative shifts and exact integer endpoints. An ineligible physical cell that
merely touches an external candidate face does not erode a zero-shear box. The
original inclusive bad-node test remains necessary on boundaries. Zero shear
therefore retains every original all-stable axis-aligned grid cuboid.

`independent_stable_cell_check.py` does not use those floor/ceil formulas,
rasterization, or a histogram. It separately projects all eight corners of
every ineligible physical cell, directly checks strict interval overlap for
every candidate rectangle, and independently tests inclusive raw bad points.
On 112 cases, with 27,693 candidate rectangles, the native optimum volume,
lexicographic selected bounds, all maximum-volume tie counts and tied roll-span
ranges matched the exhaustive reference. The JSON report binds source,
independent reference and compiled test binary hashes.

This rule covers geometric interpolation risk more cautiously; it does not
turn classified stable corners into a theorem about nonlinear cell interiors.
Off-grid trajectories must be retested. No cross-angle inclusion, desired
roll extent, or desired image appears in this candidate filter.
