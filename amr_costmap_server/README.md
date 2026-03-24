# amr_costmap_server

Costmap producer for the AMR stack.

## Role

- publishes static inflated global costmap from SLAM map
- publishes scan-based dynamic inflated local costmap
- applies footprint-aware inflation scaling
- exposes `clear_costmap` service for recovery

## Costmap Model

- `global_costmap`: static map obstacles with inflation
- `local_costmap`: current local scan hits that differ from static map, inflated for near-term avoidance

## Important Interfaces

- input: `/amr/map/data`
- input: `/amr/localization/pose`
- input: `/scan`
- output: `/amr/costmap/global`
- output: `/amr/costmap/local`
- service: `/amr/costmap_server/clear_costmap`
