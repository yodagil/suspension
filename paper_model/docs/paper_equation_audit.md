# Xiao 2022 independent equation audit

Paper: Xiao et al., *A novel integrated control framework of AFS, ASS, and DYC based on ideal roll angle to improve vehicle stability*, Advanced Engineering Informatics 54 (2022) 101764, DOI 10.1016/j.aei.2022.101764.

The uploaded ZIP was audited without modifying it. No full-grid trajectories were run for this audit. Page references below are printed paper page numbers.

## Verified correspondence

- Table 1 (p.2): `m=1300`, `Izz=2500`, `Ixx=700`, `L=2.4`, `lf=1.2`, `h=0.54`, `df=dr=0.7`, `Kphi=43100`, `Cphi=1400` are copied correctly. `lr=L-lf=1.2` is a direct inference.
- Eqs. (1)–(4), p.3: longitudinal `udot=Fx/m+v*r`, lateral `vdot=Fy/m-u*r`, yaw `rdot=Mz/Izz`, roll `pdot=(ms*hphi*(vdot+u*r)-Cphi*p+(ms*g*hphi-Kphi)*phi-deltaMx)/Ixx` correspond algebraically.
- Tire-force rotation and yaw moment accompanying Eqs. (1)–(4), p.3, match the uploaded formulas, including track-force differences and front steering terms.
- Eq. (5), p.3, prints a velocity ratio minus steering, rather than atan/atan2. The ZIP implements the printed approximation. The denominator guard is an explicit numerical addition.
- Eqs. (7)–(9), p.3, match the combined-slip construction and Burckhardt functional form. Zero longitudinal slip is an assumption, not a paper-provided simulation input.
- Eq. (11), p.3, has the same static, lateral-acceleration, and sin(roll)-dependent load-transfer signs and coefficients as the code.
- Section 3.1, p.4, says excessive yaw **or** excessive lateral acceleration **or** rollover makes a point unstable. Therefore OR-combined constraint rejection is supported. Replacing OR by AND merely to increase stability would contradict the text.

## Clear numerical defect: Eq. (11) is not actually solved to consistency

`paper_rhs.m` performs exactly five damped fixed-point updates for acceleration before evaluating final loads and tire forces. It has no convergence/residual check. Consequently the reported `obs.Fz_raw` does not generally equal Eq. (11) evaluated using the reported final acceleration `obs.ay=Fy/m`.

Actual uploaded MATLAB/Octave RHS evaluations at 40 km/h, with states `[u,v,r,phi,p]` using `v=[0,1,-2,5,-5]`, `r=[0,.2,-.3,.4,-.4]`, `phi=[0,.1,-.2,.25,-.25]`, `p=0`, yielded maximum Eq. (11) load residuals:

| Steering | Maximum load inconsistency |
|---|---:|
| 0 deg | 64.9312053782 N |
| 3 deg | 64.1808143073 N |
| 7 deg | 62.9101599030 N |

This can affect wheel-lift classification near zero load. It is a numerical equation-consistency issue, not a parameter calibration target.

For a fixed current state the tire-force-per-unit-load coefficients are known. Write raw loads as `Fz_raw=A+B*ay`, where

`B=[-m*h*lr/(2*df*L), +m*h*lr/(2*df*L), -m*h*lf/(2*dr*L), +m*h*lf/(2*dr*L)]`.

Let `tx_i=muRes_i*lambdaL_i/lambdaRes_i` and `ty_i=force_sign_y*Ks*muRes_i*lambdaS_i/lambdaRes_i` with the existing zero-slip safeguards. The body lateral coefficient is `q_i=sin(delta)*tx_i+cos(delta)*ty_i` for front tires and `q_i=ty_i` for rear tires. For a presumed positive-load tire set I, the exact solution is

`ay=sum(q(I).*A(I))/(m-sum(q(I).*B(I)))`.

Check that its raw-load signs agree with I. The all-four-loaded branch is a single exact solve; wheel-lift cases can use active-set updates/enumeration of 16 tire sets, or a residual-controlled convergent solver. Do not change the physical equations or per-angle parameters to eliminate residuals.

## Paper sign inconsistency: retain an explicit, justified convention

The printed Eq. (10), p.3, and introductory text before Eq. (B.4), p.18, show a positive `Ks` factor. However Eq. (B.4), Eq. (B.6), and Eq. (B.8) explicitly contain `-Ks`; Appendix C's linearized lateral-force model (p.19) also uses `FyF=-2*Ksf*((v+lf*r)/u-delta)`. With the printed Eq. (5) slip-angle convention, a positive lateral coefficient would be anti-restoring near straight travel. The uploaded `force_sign_y=-1` choice is physically coherent and supported by the appendices, but should remain disclosed as resolution of a paper inconsistency. Flipping it simply to force a desired region trend is not justified.

## Figure 6 comparison limits and faithful trend assessment

- Figure 6, p.5, is at 40 km/h with steering **0, 3.5, 7 degrees**. The user's requested middle angle is **3 degrees**, so it is a nearby qualitative comparison rather than exact reproduction of all three plotted conditions.
- The rendered figure shows shrinking hexahedral regions as steering increases, together with a shift toward positive yaw. Compare stable-point counts, estimated volumes, yaw-location/extent, and roll extent, rather than enforcing only a sorted volume list.
- The figure's yaw plotting range extends to about +/-0.6 rad/s, whereas the inherited requested grid stops at +/-0.4. This grid can clip the paper's region. Preserve it as requested and report boundary contact.
- `mu_peak=0.85` is explicitly stated for Figure 2 (80 km/h, 0 deg) on p.4. Figure 6 does not explicitly restate its friction value. Here 0.85 is a user-requested condition and reasonable reconstruction assumption, not a proven Figure 6 parameter.
- The paper does not give numerical `C1,C2,C3`, `Ks`, sprung mass, roll-center moment arm, mapping horizon, initial roll rate, or exact longitudinal slip/drive/constant-speed regulation used for Figure 6. The ZIP's corresponding values must remain labeled assumptions.
- Equation (1) is dynamic, but a region indexed by a stated speed may use held speed. With zero longitudinal slip and positive steering, dynamic-u open-loop trajectories can decelerate. Whether speed was held during the paper's Figure 6 mapping is not established. A common-parameter dynamic/fixed-speed sensitivity comparison is reasonable; changing speed mode only for selected angles would be cherry-picking.
- Five-second finite-horizon constraint compliance is not proof of all-time stability. Check timestep/horizon sensitivity where feasible, without claiming an unreported paper duration.

## Geometry implementation is a reconstruction, not the paper's actual vertex procedure

The uploaded `paper_build_region.m` computes roll-limit, inner-limit, and outer-limit sets in Steps 1–2. Step 3 then ignores those sets and performs PCA on **all stable points**. Its eight box corners are interpolated PCA corners, not necessarily eight observed extreme stable points from the inner-limit set specified by the paper's Steps 2–3 (p.5).

Sampled-unstable exclusion can accept a small box containing only its stable anchor and untested corners. This is a defensible explicitly limited grid reconstruction, but should not be presented as the original inner-limit eight-stable-vertex algorithm. Prefer actual candidate stable vertices and independent inside-unstable checks, or clearly disclose a conservative reconstruction fallback. Keep the classification mask and geometry-volume interpretation separate.

## Diagnostic-label issue

`failNumeric` includes `u<min_u` (0.5 m/s). This is a low-speed model-domain exit, not necessarily a NaN or integration failure. Preserve a necessary model validity guard, but report it separately or label its reason clearly. Otherwise steering-induced open-loop slowing may be misreported as a numerical failure.

## Recommended order

1. Correct the implicit-load residual, with independent equation residual and force-balance tests.
2. Keep all published constants and one common set of disclosed assumptions across steering angles.
3. Retain the physically supported restoring lateral-force convention.
4. Run the requested 0,3,7 conditions on the exact inherited grid, retaining all raw masks and reason counts.
5. Build regions with a documented paper-based candidate procedure; report geometry-dependent and grid-clipping limitations.
6. Evaluate trend, timestep convergence, and justified model variants transparently. If disagreement remains because the source omits parameters, report it rather than forcing the result.
