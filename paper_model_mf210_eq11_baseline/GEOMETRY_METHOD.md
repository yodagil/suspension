# Paper §3.1 extraction baseline

`paper_sec31_region.m` (base MATLAB) and `paper_sec31_region.py` implement the
published extreme-point **sequence** of Xiao et al., *Advanced Engineering
Informatics* 54 (2022) 101764, PDF pages 4–5, Figures 4–5. They do not claim to
recover the authors' unpublished executable corner/branch algorithm.

The routines operate on one independent `stable_mask(vc,r,phi)` and its three
ascending grids. No other steering angle or reference image is an input.

1. For each fixed `(vc,r)`, retain minimum and maximum classified stable roll.
   Both surfaces, all selected coordinates, disconnected roll slices and
   samples touching the roll-grid endpoints are retained as diagnostics.
2. At each fixed `(r,phi)` in this limit-roll cloud, split lateral-speed hits
   into contiguous grid-index bands. Outer limits are the farthest endpoints.
   For two bands, inner limits are their facing endpoints, **provided every
   sampled vc node between them is stable**. This is the declared discrete
   interpretation of the central black branches in Figure 4(d). One band
   yields coincident inner/outer endpoints and is labeled unresolved at grid
   resolution; more than two bands are ambiguous and omitted from the inner
   cloud. A gap containing a nonstable node is also omitted. These omissions
   and counts are reported; no branch is chosen using a vehicle equilibrium.
3. From the inner cloud, select the first and last yaw sections admitting
   four distinct noncollinear extrema. If a section's convex hull already has
   four vertices, keep them. Otherwise select its four cardinal vc/phi
   extrema. Coordinate ties use closest orthogonal-coordinate range midpoint,
   then lexicographic coordinates. This tie rule does not translate or scale
   returned coordinates. No inward yaw-section search is performed to make
   an invalid candidate pass. The two caps give eight **actual limit points**.
4. Keep the candidate as an accepted sampled inner hexahedron only if its
   convex hull has exactly eight extreme vertices, six planar quadrilateral
   faces, positive volume, and no excluded node inside/on the boundary. Save
   an invalid candidate and its reason rather than substituting another
   geometry. Outer-candidate diagnostics use the identical corner rule.

Step 3's exact yaw/corner rule, the treatment of quantized bands in Step 2,
and numerical tolerances are explicit reconstruction choices; the publication
does not specify them. No maximum-volume search, equilibrium anchor, inward
rank erosion, stable-box fallback, prescribed aspect ratio, roll-angle cap,
coordinate correction or cross-angle inclusion is used.

`roll_domain_censored_slices>0` means the grid ends, rather than a simulated
unstable sample, terminate at least one retained roll bound. A geometrically
valid region may therefore be a grid-truncated inner approximation with an
unresolved physical roll boundary. This flag must accompany any plot or claim.

Validation excludes **every node not classified stable**, including numerical
unresolved nodes if the physical runner stores them as `stable_mask=false`.
The legacy field `unstable_nodes_inside_inclusive` consequently counts all
such excluded nodes; `excluded_not_stable_nodes_inside_inclusive` and
`excluded_node_definition` make that scope explicit. The caller may separately
count physically unstable and unresolved nodes from its raw classifications.

`test_sec31_port.py` runs analytic boxes, diamond/sheared-diamond prisms,
roll holes, disconnected vc lobes, ambiguous multiple branches, grid censoring,
degenerate data, and random masks, comparing Python and GNU Octave outputs.
The analytical accepted volumes are 64, 32, 32 and 96 for the corresponding
synthetic examples. Rejected examples exercise holes and ambiguity; tests do
not mirror only successful execution. This is sampled finite-horizon geometry,
not continuous stability or a reproduction guarantee.

The rendered source-publication pages in this private experiment folder are
inspection aids. They should not be copied into a public code/results package.
