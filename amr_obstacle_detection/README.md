# amr_obstacle_detection

`amr_obstacle_detection` detects forward obstacles from `/scan`, classifies them against the
official map, and publishes a compact obstacle report for higher-level decision making.

## Responsibilities

- Detect forward obstacles from live laser scans
- Estimate obstacle distance and bearing in the robot heading frame
- Classify hits as likely static-map matches or dynamic obstacles
- Publish a lightweight report to the navigator and costmap server

## Does Not Own

- Global or local replanning decisions
- Costmap inflation
- Direct `/cmd_vel` control

## Output

- `/amr/obstacle/report`

```mermaid
flowchart LR
    A[/scan] --> B[amr_obstacle_detection]
    C[/amr/localization/pose] --> B
    D[/amr/map/data] --> B
    B --> E[/amr/obstacle/report]
```
