# DART contact-response experiment

This new isolated library tests moving the pure reactive-body predicate before
the skeleton shared-pointer lookup in ContactConstraint::getVelocityChange.
No constraint, contact, friction, impulse arithmetic, timestep, solver bound,
or robot model is removed or changed. It does not replace the system library.

The function is derived from official DART v6.13.2 ContactConstraint.cpp.
The complete unmodified upstream file including its BSD terms and notices is
retained as UpstreamContactConstraint.cpp:
https://raw.githubusercontent.com/dartsim/dart/v6.13.2/dart/constraint/ContactConstraint.cpp
Those upstream license terms apply to the derived function.

The qualifier uses the actual installed symbol as baseline, checks two
independent three-body contact worlds byte-for-byte for 2400 steps, then checks
normal/friction response with and without CFM and alternates component timing.
Only a qualified and measurably useful candidate may be bound into another
new full-task identity. No full robot or real-time claim follows from this test.
