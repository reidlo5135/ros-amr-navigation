# 2026-03-20

- `amr_viz`를 순수 React Web 프로젝트로 전환
  - Electron 관련 shell / 실행 경로 제거
  - host PC 브라우저 접속 기준 운영 viz 구조로 정리
  - visualization UI와 ROS bridge 책임 완전 분리
- `amr_mqtt_bridge` 신규 ROS 2 패키지 추가
  - `amr_viz` 내부 bridge 구현을 별도 패키지로 분리
  - viz 전용이 아닌 범용 ROS <-> MQTT bridge 역할로 설계
  - 향후 다른 UI / 운영 툴 / 외부 시스템에서도 재사용 가능하도록 패키지 경계 정의
- `amr_navigation` 인터페이스를 MQTT 기준으로 재모델링
  - 기존 ROS topic / service / action을 그대로 노출하지 말고 필요한 데이터만 선별
  - visualization / control / telemetry 용 메시지 스키마 별도 정의
  - publish / subscribe 채널 구조, topic naming, QoS 대응 전략 설계
- React Web client를 MQTT 기반 통신 구조로 전환
  - WebSocket direct bridge 대신 MQTT client 사용
  - low-latency 상태 갱신, command 송신, initial pose / goal / status 흐름 재정리
  - host browser 렌더링 + VM/Ubuntu Server bridge/hosting 역할 분리 유지
- 성능 고도화 최우선
  - Python bridge 고CPU 문제 제거
  - launch / logging / serialization overhead 최소화
  - 실사용 기준으로 RViz 대비 가벼운 운영 viz 목표 재설정

# 2026-03-23

- `amr_viz` React + MQTT 완전 전환
  - 현재 과도기적 viz/bridge 흔적 정리
  - browser 기반 운영 화면을 MQTT client 전제로 재구성
  - `goal`, `status`, `feedback`, `map`, `path`, `costmap` 확인 흐름 재정리
- `amr_mqtt_bridge`, `amr_mqtt_robot_plugin` 소스 구조 개편
  - `rcl` 관련 로직과 `MQTT` 관련 로직을 별도 `.h/.c` 파일로 분리
  - 예시: `node.h`, `node.c`, `mqtt.h`, `mqtt.c`
  - 직렬화/역직렬화, topic routing, ROS interface init/fini 책임도 파일 단위로 분리
- local escaping replan 명확화
  - 현재는 dynamic obstacle을 미리 보고 replan은 하지만 좌/우 oscillation이 심함
  - escape 진입 조건, 유지 조건, 종료 조건을 명확히 나눌 필요 있음
  - obstacle release 전 기존 corridor 복귀를 제한하는 hysteresis와 확실한 판단 기준 추가
- `amr_bt_navigator` 현업 표준 BT 적용
  - 현재의 형식적 BT 사용에서 벗어나 실제 의사결정 트리로 재구성
  - 동적 장애물 판단과 recovery 선택은 반드시 `amr_bt_navigator`가 담당
  - 각 feature package는 판단이 아닌 수행 역할만 하도록 책임 분리
  - 3번 local escaping replan 정책과 직접 연계

# 2026-03-18

- dynamic obstacle escaping 고도화
  - 현재 local replan이 escape보다 기존 global plan 복귀를 너무 빨리 시도해 어색한 주행이 발생함
  - 목표 동작은 `dynamic obstacle 감지 -> dynamic inflation 반영 -> global A* detour replan -> local plan은 obstacle release 전까지 escaping 수행 -> detour 경유 후 goal 복귀` 흐름으로 재정의
  - `amr_bt_navigator`가 dynamic obstacle 상황에서 global detour replan을 트리거하고, `amr_local_planner`는 release 조건 전까지 escape-centric local plan을 유지하도록 역할 정리
  - dynamic obstacle이 해제되기 전에는 기존 global corridor로 즉시 재복귀하지 않도록 조건과 hysteresis 추가 검토
- `0.3.1` R&R refactoring continuation
  - `amr_bt_navigator`가 obstacle report를 바탕으로 wait / local replan / global replan / recovery를 실제로 판단하도록 확장
  - `amr_motion_controller`를 path tracking + 최종 근접 safety gate만 남기는 방향으로 추가 축소
  - `amr_global_planner`, `amr_local_planner`에서 남아 있는 legacy self-costmap 가정 완전 제거
  - `amr_costmap_server`를 기준으로 fixed/dynamic obstacle layer 정책 정교화
- `amr_motion_controller` local plan tracking issue 해결
  - local plan 마지막 점만 따라가며 corner cutting 하는 현상 수정
  - 현재 위치 기준 nearest point 이후의 lookahead target 추종 방식 적용
  - 실제 odom 궤적이 local/global costmap clearance를 유지하도록 보정
- obstacle 감지 시 local plan 재계산 및 회피 주행 구현
  - `obstacle_detected` 발생 시 단순 정지에서 끝나지 않도록 local replan 흐름 추가
  - `정지 -> local recompute -> 재시도` 최소 동작 먼저 구현
  - 필요 시 회피 실패/재시도 횟수 제한 및 fallback 동작 정의
- custom mapping workflow 확장
  - `mapping mode -> static navigation mode` 전환 절차 설계
  - bootstrap mapping용 local scan matching 파라미터 튜닝
  - mapping-mode corrected `map -> odom` TF 안정화 및 시각 정합성 검증
  - map quality 기준값 튜닝 및 실제 환경별 threshold 표준화
  - auto-save 이후 자동 `navigation mode` 전환 절차 정리
  - 저장된 official map을 바로 navigation launch에 재사용하는 절차 정리
  - temporary map 기반 localization correction은 mapping 초기 구간 이후에만 붙는 구조 검토
- 최초 환경 자동 탐색 전략 정리
  - teleop 없이도 미지 환경을 훑으며 map을 쌓는 탐색 주행 전략 검토
  - 벽 따라가기, 회전-전진, 장애물 회피 기반 탐색 방식 비교
  - coverage보다 "충돌 없이 공간을 훑으며 map 생성"을 우선 목표로 정의
- global localization 구현 검토
  - 수동 `initial_pose` 없이 시작 가능한 global localization 흐름 설계
  - map 전체 particle 분포 초기화 및 scan 기반 수렴 절차 검토
  - kidnapped 상황 대응용 relocalization 조건과 fallback 동작 정의

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

- custom mapping / localization 전략 구체화
  - 자체 mapping mode와 static navigation mode의 역할 분리
  - map save/load/freeze 절차 정리
  - mapping 완료 후 자동 전환 기준 정의
- map 관리 기능
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
  - localization, planner, controller, mapping, relocalization은 자체 구현 우선
- 예외적으로 사용 가능한 외부 패키지
  - 없음
- 되도록 피할 것
  - Nav2 전체 의존
  - 과한 behavior tree/plugin 체계 도입
  - 초기 단계에서 고급 MPC/TEB/DWB 수준 복잡도 구현
