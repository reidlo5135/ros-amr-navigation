# amr_map_server

Map lifecycle server for official and temporary occupancy maps.

## Role

- loads and publishes the static SLAM map
- accepts a temporary SLAM map from `amr_slam_mapper`
- can evaluate and save a temporary mapping result
- publishes the official map topic

## Important Topics

- `/amr/map/data`
- `/amr/map/temp`

## Important Services

- `/amr/map_server/get_map`
- `/amr/map_server/freeze_temporary_map`
- `/amr/map_server/evaluate_temporary_map`
- `/amr/map_server/save_temporary_map`
