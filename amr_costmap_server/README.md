# amr_costmap_server

`amr_costmap_server` owns the global and local costmaps used by the planners.

## Responsibilities

- Subscribe to the official map
- Build the global inflated costmap
- Overlay dynamic obstacle reports into the local costmap
- Publish `/amr/costmap/global` and `/amr/costmap/local`

## Does Not Own

- Global or local path generation
- Obstacle detection from raw scans
- Motion command generation

## Interface Graph

```mermaid
flowchart LR
    A[/amr/map/data] --> B[amr_costmap_server]
    C[/amr/obstacle/report] --> B
    B --> D[/amr/costmap/global]
    B --> E[/amr/costmap/local]
```
