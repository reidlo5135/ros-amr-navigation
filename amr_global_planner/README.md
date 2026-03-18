# amr_global_planner

`amr_global_planner` provides occupancy-grid A* planning on the global costmap owned by `amr_costmap_server`.

## Interfaces

- subscribes: `/amr/costmap/global`
- serves: `/amr/global_planner/plan_segment`
- serves: `/amr/global_planner/plan_route`
- publishes: `/amr/planner/global`

## Planning Model

- search space: 2D occupancy grid cells
- planner: A*
- planning grid: global costmap
- connectivity: 4 or 8
- heuristic:
  - Manhattan for 4-connectivity
  - diagonal distance for 8-connectivity
- extra cost:
  - optional `turn_penalty` when direction changes

## Costmap Consumption

- consumes the global costmap published by `amr_costmap_server`
- can keep unknown space blocked or traversable based on `planner.allow_unknown`
- relocates start or goal to the nearest free cell when the request lands inside blocked costmap space

## Global Planning Pipeline

```mermaid
flowchart LR
    A[/amr/costmap/global/] --> B[A* search]
    B --> C[/amr/planner/global/]
```

## Post-processing

- returns a dense cell-by-cell path converted back to world poses
- publishes the same path for visualization and downstream local planning
