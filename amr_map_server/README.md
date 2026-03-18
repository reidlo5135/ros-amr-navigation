# amr_map_server

`amr_map_server` loads a map YAML file, converts the referenced image into an occupancy grid, publishes the map, and serves `GetMap`.

## Interfaces

- publishes: `/amr/map/data`
- serves: `/amr/map/get`

## Map Loading Model

- input:
  - YAML file path from `files.yaml`
- reads:
  - image path
  - resolution
  - origin
  - occupied threshold
  - free threshold
  - negate mode
- output:
  - `nav_msgs/msg/OccupancyGrid`

## Notes

- the node is lifecycle-managed
- the loaded map is reused by localization and both planners
