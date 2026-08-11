# Duplicate TF audit

Evidence: `runtime-unverified-4pc`

PC3 owns `map → base_link` and every required static transform on a per-transform basis. The live
`/tf` and `/tf_static` broadcaster list, timestamps, parent/child pairs, and authorities must be
captured from all four PCs. Duplicate static broadcasters or conflicting parentage are blocking;
renaming a broadcaster does not resolve the conflict.
