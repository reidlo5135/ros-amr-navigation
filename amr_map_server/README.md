# amr_map_server

Map source for both localization and navigation.

## Role

- loads and publishes the static SLAM map
- supports optional mapping mode
- can evaluate and save a temporary mapping result
- publishes map and temporary map topics

## Important Topics

- `/amr/map/data`
- `/amr/map/temporary`

## Important Services

- `/amr/map_server/get_map`
- `/amr/map_server/freeze_temporary_map`
- `/amr/map_server/evaluate_temporary_map`
- `/amr/map_server/save_temporary_map`
