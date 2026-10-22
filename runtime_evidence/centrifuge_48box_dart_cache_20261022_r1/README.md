# DART exact-order constraint cache r1

This is a private, non-system experiment for the new r343 Gazebo process only.
It does not overwrite libdart, formal models, old overlays, or successful runs.

The algorithm follows DART v6.13.2 BoxedLcpConstraintSolver. The complete original
source and its BSD-style copyright, conditions and disclaimer are retained in
`UpstreamBoxedLcpConstraintSolver.cpp`, downloaded from the official repository:
https://raw.githubusercontent.com/dartsim/dart/v6.13.2/dart/constraint/BoxedLcpConstraintSolver.cpp
SHA-256: 38ec5eccfe9bf05112647affdf06e2461e879a03e0f42c97d442e16eaf6d66f1.
That license applies to the derived solver implementation. No system installation occurs.

The solver uses the installed 6.13.2 class layout, default Dantzig and PGS objects,
original numerical operations and fallback policy. It caches group-owned raw pointers
and dimensions for one non-mutating solve call and copies the same lower triangle in
the same order without repeated shared_ptr retrieval. No fast-math or time-step change.

Qualification compares the actual installed function by explicit dlsym against the
candidate function, not a source reimplementation pretending to be the baseline:
105 matrix/input/output/callback-order cases including empty groups, fallback and NaN;
1200 real contact-world steps with exact position and velocity bytes, 1044 contact steps.
The cheap synthetic 240-dimensional component is about 1.1534x faster. This is not a
full robot speed or stability certificate. Whole-task evidence belongs to r343.

The sampled log observes one group per 1024 nonempty calls, so it is not a complete
constraint census and can be biased by periodic group order. Inclusive native CPU
profiling remains separate. Do not infer individual collider cost from contact messages.

The isolated build's installed library is a symlink into build/unique_prefix. r343 binds
the resolved path and hash; do not delete that build directory as disposable cache.
