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

## Publish Behavior

- global costmap is rebuilt and published when the static map is received or a full clear/rebuild is requested
- scan updates rebuild and publish only the local costmap by default
- `publish.global_on_scan` can restore legacy scan-driven global publishing when needed
- `publish.local_min_period_ms` throttles local costmap publishing to reduce DDS load for remote visualization subscribers
- `local_window.enabled` publishes `/amr/costmap/local` as a robot-centered window instead of the full global-sized grid

## Important Interfaces

- input: `/amr/map/data`
- input: `/amr/localization/pose`
- input: `/scan`
- output: `/amr/costmap/global`
- output: `/amr/costmap/local`
- service: `/amr/costmap_server/clear_costmap`
