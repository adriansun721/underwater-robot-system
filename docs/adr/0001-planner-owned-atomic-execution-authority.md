# Planner-owned profile and atomic execution authority

The planner exclusively owns the immutable joint ExecutionProfile, and the
ExecutionAuthority publishes it with plan, dependencies, approved remainder,
and lease as one AuthorizedExecutionBundle. Control adapters may sample,
convert, and conservatively limit but cannot recalculate targets. This avoids
control-side changes invalidating cable/track validation and prevents split
plan/lease races; a changed situation revokes and replans.

