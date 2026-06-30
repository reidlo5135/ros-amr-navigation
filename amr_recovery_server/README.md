# amr_recovery_server

Lifecycle recovery command planner.

Service:

- `/plan_recovery`

The server returns `MotionCommand` recovery behaviors such as wait, backup, and
spin. It does not own localization or TF publication; the navigator passes the
current pose obtained from TF.
