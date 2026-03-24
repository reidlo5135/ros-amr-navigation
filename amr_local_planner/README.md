# amr_local_planner

Local planning package for near-term path generation and escape behavior.

## Role

- slices the active global path into a local tracking path
- checks the dynamic local costmap for blocked segments
- provides local escape planning as a service

## Inputs

- `/amr/motion/command`
- `/amr/localization/pose`
- `/amr/costmap/local`

## Outputs

- `/amr/planner/local`
- service `/amr/local_planner/plan_local_escape`
