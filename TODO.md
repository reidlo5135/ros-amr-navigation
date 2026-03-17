# 2026-03-16

- 완료: `amr_bringup/params/amr.yaml`의 토픽 이름 정리
  - 예: `motion_controller/command` -> `motion/command`
- 완료: `.hpp` / `.cpp`에 남아 있는 토픽 주소 하드코딩 요소를 `amr_bringup/params/amr.yaml` 기준으로 파라미터화
- 완료: `amr_bringup/rviz/amr.rviz` 구현
- 진행: localization 오류 수정
  - `/odom` 연결 문제 수정
  - dead reckoning 기반 구조에서 `AMCL-lite` 방향으로 1차 전환
  - 추가 안정화 및 튜닝 필요
- 보류: `localization/verb`, `local_planner/verb`처럼 혼용되어 있는 토픽 주소 체계 통합

# 2026-03-17

- `amr_localization` AMCL-lite 안정화 및 파라미터 튜닝
- `amr_local_planner` local plan 생성 안정화
  - 턴 구간, 경로 추종 중 local plan 비정상 생성 확인 및 보정
- `NavigateToPose` 액션/feedback 단순화 반영 확인
  - 불필요한 `route_id`, waypoint 관련 항목 제거 후 연동 점검
- 실제 주행 기준 bringup 연동 점검
  - `goal -> global plan -> local plan -> cmd_vel` 흐름 재확인
- `amr_local_planner` 고도화
  - 현재 global plan slice 수준의 local plan에서 벗어나도록 구조 개선
  - costmap / inflation 개념을 도입해 `global_plan -> inflated local planning` 흐름 검토
  - 주행 방향성, clearance, lookahead target 품질 개선
- obstacle 감지 이후 local 재계산 / 회피 주행 검토
  - 단순 정지에서 끝나지 않고 obstacle 상황에서 local plan 재생성 가능성 검토
  - 재계산 후 우회 또는 회피 주행의 최소 구현 방향 설계
  - 난이도를 고려해 `정지 -> 재계산 -> 재시도`부터 단계적으로 접근

# 2026-03-18

- `amr_motion_controller` local plan tracking issue 해결
  - local plan 마지막 점만 따라가며 corner cutting 하는 현상 수정
  - 현재 위치 기준 nearest point 이후의 lookahead target 추종 방식 검토
  - 실제 odom 궤적이 local/global costmap clearance를 유지하도록 보정
- obstacle 감지 시 local plan 재계산 및 회피 주행 구현
  - `obstacle_detected` 발생 시 단순 정지에서 끝나지 않도록 local replan 흐름 추가
  - `정지 -> local recompute -> 재시도` 최소 동작 먼저 구현
  - 필요 시 회피 실패/재시도 횟수 제한 및 fallback 동작 정의
- Non-SLAM 상황의 임의 주행 및 동시 SLAM 매핑 검토
  - 저장된 map 없이도 일정 시간 임의 주행 가능한 흐름 설계
  - 주행 중 `slam_toolbox` 기반 실시간 매핑 연동 가능성 검토
  - 저가형 로봇 청소기 수준의 "막 주행하며 맵 생성" 운용 흐름 정리
- global localization 구현 검토
  - 수동 `initial_pose` 없이 시작 가능한 global localization 흐름 설계
  - map 전체 particle 분포 초기화 및 scan 기반 수렴 절차 검토
  - 자동 초기 위치 추정 성공 조건과 실패 시 fallback 동작 정의

# 로드맵

## 1단계. 현재 MVP 안정화

- localization 안정화
  - `AMCL-lite` pose 흔들림, yaw drift, initial pose 적용 후 수렴성 개선
  - `map -> odom -> base_link` TF 일관성 점검
  - `scan`, `odom`, `map` 타임스탬프 및 frame 정합성 확인
- global/local planning 안정화
  - A* 경로 품질 개선
  - local plan 생성 품질 개선
  - 회전만 반복하는 케이스, 턴 구간 oscillation 제거
- motion control 안정화
  - `P / PI / PID` 파라미터 튜닝
  - 저속 주행에서 overshoot, goal 근처 떨림, 제자리 회전 과다 현상 보정
  - 정지 명령 이후 residual `cmd_vel` 제거
- bringup/rviz/debug 체계 정리
  - 주요 토픽, TF, 상태 로그 기준 확립
  - 운영 중 확인할 최소 디버그 포인트 문서화

## 2단계. 실내 자율주행 기본기 완성

- obstacle 대응 강화
  - `scan` 기반 전방/측방 위험 구역 판단
  - local path 상 충돌 예상 시 감속 또는 정지
  - 단순 obstacle_detected 상태만이 아니라 정지/재시도 흐름 추가
- recovery behavior 추가
  - 정지
  - 후진
  - 제자리 회전
  - replan
- navigation 실행 안정화
  - goal 취소, 중단, 재시작 처리
  - planner/controller timeout 처리
  - stuck 판단 기준 추가

## 3단계. 저가형 로봇 청소기 수준의 실내 주행 품질

- 좁은 실내 공간 대응
  - 복도, 문틀, 가구 사이 통로에서 안정적으로 통과
  - 급회전 구간에서 local plan / control oscillation 억제
- 동적 장애물 대응
  - 사람, 의자, 작은 물체 등으로 인해 path가 막히는 상황에서 정지 후 재시도
  - 단순 충돌 회피보다 "멈춤 -> 대기 -> 재계획" 흐름 우선 구현
- 근거리 안전 주행
  - 벽을 과하게 긁지 않도록 clearance 유지
  - obstacle inflation 또는 safety margin 개념 도입
- goal 도달 품질 향상
  - 목표 근처 overshoot 최소화
  - heading alignment와 goal tolerance 세분화
  - 도착 판정 후 불필요한 추가 회전 억제

## 4단계. 맵 운용 현실화

- mapping / localization 전략 결정
  - 자체 SLAM 구현은 범위상 제외
  - 맵 생성은 `slam_toolbox` 사용 검토
  - 운영 모드에서는 저장된 map + 자체 localization 사용
  - 필요 시 localization도 완전 자체 구현 고집보다 안정성 우선으로 판단
- map 관리 기능
  - 맵 저장/로드 절차 정리
  - 진입 금지 구역, 가상벽, 운영 금지 구역 표현 방식 정의
  - occupancy map 기반의 기본 운용 절차 문서화
- map 업데이트 정책
  - 환경이 조금 바뀌었을 때 재매핑할지, localization만 유지할지 기준 정의
  - 맵 버전 관리, 배포, 교체 절차 정리

## 5단계. 제품성 기능 추가

- 도킹/복귀
  - 저전압 또는 미션 종료 시 홈 복귀
  - 초기에는 단순 goal 복귀 방식으로 구현
- battery/mission 관리
  - 배터리 상태 수신
  - 주행 중단 후 복귀 조건 정의
- 사용자 운용 기능
  - 주행 시작, 정지, 일시정지
  - 수동 goal 이동과 자동 주행 모드 분리
  - 저장된 위치로 이동하는 단순 mission 기능 검토

## 6단계. 최종 품질 목표

- 장시간 주행 안정성 검증
  - 20분 이상 연속 주행 시 pose drift, oscillation, stuck 빈도 확인
- 저가형 로봇 청소기 수준의 주행/맵 핸들링 기준 정리
  - 벽/가구에 과하게 부딪히지 않을 것
  - 단일 실내 공간에서 localization을 잃지 않을 것
  - goal 주행과 자동 주행 모드 전환이 끊기지 않을 것
  - 재실행 시 initial pose 및 map 로드 절차가 단순할 것

## 구현 원칙

- 가능하면 직접 구현 유지
  - localization, planner, controller, cleaning coverage는 자체 구현 우선
- 예외적으로 사용 가능한 외부 패키지
  - `slam_toolbox`: 맵 생성 단계 대체
- 되도록 피할 것
  - Nav2 전체 의존
  - 과한 behavior tree/plugin 체계 도입
  - 초기 단계에서 고급 MPC/TEB/DWB 수준 복잡도 구현
