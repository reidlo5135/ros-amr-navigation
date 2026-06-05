# AMR Structured Logging Schema

이 문서는 ros-amr-navigation v0.17.5 기준 현장 운용 및 디버깅 로그 규칙을 정의한다. 목적은 토픽을 늘리지 않고도 ROS2 console log, `ROS_LOG_DIR`, `nohup` 출력, grep/awk 기반 분석에서 같은 기준으로 navigation 상태를 볼 수 있게 하는 것이다.

## 공통 로그 형식

정형 로그는 message 본문을 아래 형식으로 작성한다.

```text
AMR_LOG schema=v1 component=controller event=tracking_state phase=tracking target_idx=18 target_x=1.240 target_y=0.820 dist_goal_m=1.420 heading_err_rad=0.210 blocked=false recovery=false
AMR_LOG schema=v1 component=bt_navigator event=recovery_decision goal_id=3 reason=blocked recovery_type=backup recovery_skipped=false planner_ok=false controller_ok=false
AMR_LOG schema=v1 component=controller event=goal_state phase=final_heading_align xy_reached=true yaw_reached=false dist_goal_m=0.041 heading_err_rad=0.180 cmd_lin=0.000 cmd_ang=0.120
```

규칙:

- Prefix는 반드시 `AMR_LOG`를 사용한다.
- `schema=v1`, `component=...`, `event=...`는 필수다.
- 나머지는 `key=value` 형식으로 추가한다.
- 값에는 공백을 넣지 않는다. 공백이 필요한 값은 `_` 또는 짧은 reason code로 표준화한다.
- Boolean은 `true` 또는 `false` 소문자를 사용한다.
- 거리 단위는 `_m`, 각도 단위는 `_rad`, 시간 단위는 `_sec` suffix를 사용한다.
- ROS timestamp, logger name, severity는 ROS2 logging system이 제공하므로 message 본문에 중복 timestamp를 넣지 않는다.
- 코드 내부 로그의 key/value는 영어만 사용한다. 문서와 운영 절차 설명은 한국어로 작성할 수 있다.
- 고주파 루프의 상태 요약은 throttle logging을 사용한다.
- 상태 전이와 decision event는 즉시 남기되 반복 출력은 피한다.

## 공통 필드 사전

필수 공통 필드:

| Field | Meaning |
| --- | --- |
| `schema` | 로그 schema version. 현재 값은 `v1` |
| `component` | 로그 책임 component. 예: `controller`, `bt_navigator` |
| `event` | 이벤트 이름. 예: `tracking_state`, `recovery_decision` |

권장 공통 필드:

| Field | Meaning |
| --- | --- |
| `node` | ROS node 이름 |
| `phase` | 현재 phase 또는 state group |
| `goal_id` | action goal 또는 내부 command sequence id |
| `route_id` | route identifier |
| `seq` | component 내부 sequence |
| `state` | 상태명 |
| `result` | 결과. 예: `success`, `failed`, `skipped` |
| `reason` | reason code. 공백 없는 snake_case 권장 |
| `duration_sec` | 수행 시간, seconds |

Navigation 필드:

| Field | Meaning |
| --- | --- |
| `target_idx` | 선택된 tracking target index |
| `nearest_idx` | current pose와 가장 가까운 local plan index |
| `candidate_idx` | lookahead/min-distance 기준 candidate index |
| `selected_idx` | 실제 선택된 tracking target index |
| `target_x` | 선택된 target x position |
| `target_y` | 선택된 target y position |
| `target_dist_m` | current pose에서 선택 target까지 거리 |
| `target_jump_m` | 이전 target과 새 target 사이의 거리 |
| `lookahead_m` | lookahead distance |
| `path_points` | path pose count |
| `path_length_m` | path length |
| `dist_goal_m` | 현재 pose에서 goal까지 거리 |
| `heading_err_rad` | heading error |
| `steering_err_rad` | deadband/final-align suppression 이후 실제 steering에 쓰는 heading error |
| `selection_reason` | tracking target 선택 이유. 예: `candidate_min_distance`, `retained_target_too_close` |
| `xy_reached` | goal XY tolerance 도달 여부 |
| `yaw_reached` | goal yaw tolerance 도달 여부 |
| `blocked` | local/controller blocked 상태 |
| `safety_blocked` | safety gate blocked 상태 |
| `rejoin` | path rejoin phase 여부 |
| `recovery` | recovery 상태 여부 |
| `recovery_type` | `wait`, `backup`, `spin`, `local_escape`, `global_replan` 등 |
| `recovery_skipped` | recovery 생략 여부 |

Command 품질 필드:

| Field | Meaning |
| --- | --- |
| `cmd_lin` | linear command x |
| `cmd_ang` | angular command z |
| `cmd_flip_count` | command sign flip count |
| `cmd_zero_count` | zero command count |
| `oscillation_score` | component-local oscillation score |
| `last_cmd_age_sec` | 마지막 command 이후 경과 시간 |

TF/Localization 필드:

| Field | Meaning |
| --- | --- |
| `frame_from` | source frame |
| `frame_to` | target frame |
| `pose_frame` | current pose frame |
| `plan_frame` | local/global plan frame |
| `target_frame` | selected tracking target frame |
| `tf_ok` | TF 상태 |
| `tf_age_sec` | TF age |
| `pose_x` | pose x |
| `pose_y` | pose y |
| `pose_yaw` | pose yaw |
| `cov_xy` | XY covariance summary |
| `cov_yaw` | yaw covariance summary |

MQTT/External Bridge 필드:

| Field | Meaning |
| --- | --- |
| `robot_id` | robot id |
| `topic` | MQTT or ROS topic |
| `command` | command name or type |
| `accepted` | command accepted 여부 |
| `rejected` | command rejected 여부 |
| `reject_reason` | rejection reason code |

## 로그 레벨 기준

DEBUG:

- 매주기 상세값
- 계산 중간값
- 너무 자주 발생하는 내부 상태

INFO:

- 상태 전이
- goal 시작, 성공, 취소
- planner, controller, recovery의 주요 decision
- 1초 이상 throttle된 runtime state

WARN:

- blocked 상태 진입
- recovery 진입
- goal 접근 중 oscillation 의심
- target jump 과다
- TF 지연
- local/global plan 없음
- command timeout

ERROR:

- 필수 TF 조회 실패로 동작 불가
- action/service 실패
- lifecycle activate/configure 실패
- 필수 topic/message 미수신으로 navigation 불가

## 패키지별 로그 책임

| Package | Component | 책임 | 주요 event |
| --- | --- | --- | --- |
| `amr_controller_server` | `controller` | local planner, tracking target selection, motion command generation, goal approach, final heading alignment, local blocked/rejoin 판단 | `tracking_state`, `tracking_heading_debug`, `tracking_frame_mismatch`, `tracking_target_selected`, `target_jump_detected`, `goal_state`, `cmd_quality`, `local_blocked_state`, `rejoin_state`, `motion_command` |
| `amr_bt_navigator` | `bt_navigator` | goal orchestration, planner/controller/recovery decision, recovery 진입/스킵 판단, action result 관리 | `goal_received`, `goal_started`, `goal_succeeded`, `goal_failed`, `goal_canceled`, `recovery_decision`, `recovery_skipped`, `recovery_started`, `recovery_finished`, `bt_phase_transition` |
| `amr_global_planner` | `global_planner` | global plan generation, plan request/result/failure reason | `plan_requested`, `plan_succeeded`, `plan_failed`, `plan_quality` |
| `amr_recovery_server` | `recovery_server` | wait/backup/spin recovery command generation | `recovery_plan_requested`, `recovery_plan_selected`, `recovery_plan_failed`, `recovery_command` |
| `amr_costmap_server` | `costmap_server` | global/local costmap, obstacle/local blocked context, clear request/result | `costmap_state`, `obstacle_state`, `costmap_clear_requested`, `costmap_clear_done`, `costmap_error` |
| `amr_localization` | `localization` | pose estimate, map to odom TF, initial pose | `localization_state`, `initial_pose_received`, `tf_state`, `tf_error` |
| `amr_runtime_observation` | `runtime_observation` | cross-package 상태 요약과 event 집계. 기존 `/amr/observation/runtime/summary`, `/amr/observation/runtime/events` 우선 활용 | `runtime_summary`, `runtime_event`, `nav_quality`, `goal_quality`, `recovery_summary` |
| `amr_lifecycle_manager` | `lifecycle_manager` | lifecycle state transition | `lifecycle_configure`, `lifecycle_activate`, `lifecycle_deactivate`, `lifecycle_error` |
| `amr_mqtt_server` | `mqtt_server` | external command/telemetry bridge | `mqtt_command_received`, `mqtt_command_accepted`, `mqtt_command_rejected`, `mqtt_publish_result`, `mqtt_connection_state` |
| `amr_visualization` | `visualization` | operator visualization app | `ui_state`, `subscription_state`, `render_error` |

## Runtime Observation 원칙

- 여러 package를 가로지르는 상태 요약은 `amr_runtime_observation`에서 담당한다.
- 기본 light mode는 기존 command/status/action feedback/summary 중심으로만 집계한다.
- `/scan`, full costmap, `/tf` 같은 heavy topic은 기본 구독 대상이 아니다.
- heavy topic 관측이 필요하면 parameter로 opt-in한다.
- 새 topic을 추가하기 전에 기존 `/amr/observation/runtime/summary`, `/amr/observation/runtime/events`, motion status, local plan status, action feedback/status 재사용을 우선 검토한다.

## 구현 지침

- 매 control cycle마다 INFO 로그를 찍지 않는다.
- 상태 요약은 throttle 로그로 제한한다.
- 상태 전이는 즉시 INFO/WARN으로 남긴다.
- `target_idx`, `target_x`, `target_y`, `target_dist_m`, `target_jump_m`, `dist_goal_m`, `heading_err_rad`, `cmd_lin`, `cmd_ang`는 같은 이름을 유지한다.
- plan은 존재하지만 로봇이 직진만 하는 경우 `tracking_heading_debug`의 `nearest_idx`, `candidate_idx`, `selected_idx`, `target_dist_m`, `pose_frame`, `plan_frame`, `target_frame`, `heading_err_rad`, `steering_err_rad`, `selection_reason`을 우선 확인한다.
- 직선 plan에서 좌우 흔들림이 있으면 `tracking_state`의 `rejoin`, `rejoin_context_active`와 `tracking_heading_debug`의 `straight_segment`, `path_curvature_score`, `lateral_error_m`, `heading_error_raw_rad`, `heading_error_filtered_rad`, `steering_deadband_active`, `steering_hysteresis_state`, `cmd_ang_sign`, `cmd_ang_flip_count`, `output_ang_sign`, `output_ang_flip_count`를 함께 확인한다.
- recovery 실행 또는 skip 판단은 `reason`, `recovery_type`, `recovery_skipped`, `planner_ok`, `controller_ok`를 포함한다.
- costmap grid, scan ranges, map data 등 대량 데이터는 log에 직접 출력하지 않는다.
- rosbag2 profile에서 필요한 heavy topic을 선택적으로 기록한다.

## Straight-Line Tracking Oscillation Diagnostics

직선 plan 구간에서 `/cmd_vel.angular.z`가 작은 값으로 좌우 반복되면 controller 로그를 다음 순서로 본다.

1. `cmd_quality`에서 `cmd_ang_sign_flip_count`와 `output_ang_sign_flip_count`를 비교한다. `cmd_ang`부터 흔들리면 tracking heading/deadband 문제이고, `output_ang`만 흔들리면 velocity controller 응답이나 derivative 영향이 크다.
2. `tracking_state`에서 `rejoin=false`, `rejoin_context_active=false`인지 확인한다. `rejoin=true`는 정상 tracking이 아니라 recovery/escape 이후 path 복귀 컨텍스트만 의미한다.
3. `tracking_heading_debug`에서 `target_dist_m`과 `lookahead_m`을 확인한다. 직선 구간인데 target이 너무 가까우면 local plan point noise에 민감하다.
4. `straight_segment=true`인데 `heading_error_raw_rad`가 작고 `heading_error_filtered_rad`가 deadband 근처라면 `straight_heading_deadband`와 `straight_heading_release_threshold`를 먼저 조정한다.
5. `path_curvature_score` 또는 `lateral_error_m`이 threshold보다 크면 local plan 자체가 grid/refinement 영향으로 미세 zigzag일 수 있다.
6. `straight_segment=true`에서 `cmd_ang_abs_avg`, `output_ang_abs_avg`, sign flip count가 줄었는지 `scripts/extract_nav_quality.sh`의 `nav_quality_summary` row로 수정 전후를 비교한다.
7. deadband 조정 후에도 흔들리면 `straight_angular_gain`, `straight_max_angular_speed`, `straight_tracking_lookahead_distance`를 보수적으로 조정한다.
8. 마지막으로 `velocity_controller.angular.kd`를 0.00과 비교해 derivative가 작은 부호 전환을 키우는지 확인한다.

정상 직선 주행 기대값은 `phase=tracking`, `rejoin=false`, `rejoin_context_active=false`, `straight_segment=true`, `steering_deadband_active=true` 또는 `steering_hysteresis_state=suppressed`, `cmd_ang`과 `output_ang`이 0에 가까운 상태다.

관련 parameter는 `/amr/motion_controller`의 `control.straight_tracking_enabled`, `control.straight_tracking_lookahead_distance`, `control.straight_curvature_threshold`, `control.straight_lateral_error_threshold`, `control.straight_heading_deadband`, `control.straight_heading_release_threshold`, `control.straight_angular_gain`, `control.straight_max_angular_speed`, `control.straight_heading_filter_alpha`, `control.tracking_heading_release_threshold`, `control.rejoin_context_timeout_sec`, `control.rejoin_context_distance_m`, `control.rejoin_target_jump_threshold_m`이다. Straight mode는 tracking 구간에서만 적용되며 goal approach, final heading alignment, recovery command에는 적용하지 않는다.
