# amr_recovery_server

Recovery behavior command generator.

## Role

- receives recovery behavior requests from `amr_bt_navigator`
- returns executable motion commands for:
  - `wait`
  - `backup`
  - `spin`

## Service

- `/amr/recovery_server/plan_recovery`
