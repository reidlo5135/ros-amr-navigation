# amr_slam_mapper TODO

## 목적

- `amr_slam_mapper`를 `slam_toolbox` 대체품으로 한 번에 만들려 하지 않고,
  현재 AMR 서비스에 필요한 수준까지 점진적으로 고도화한다.
- 특히 다음 시나리오에서 map 왜곡을 줄이는 것을 우선 목표로 둔다.
  - 직진 후 큰 우회전으로 loop 생성
  - loop return 후 기존 직진 구간 재합류
  - 회전 중 다른 자세/각도에서 같은 구조를 재관측하는 상황

## 현재 주요 현상

- 직진 구간은 상대적으로 양호하지만 회전이 시작되면 map이 회전 방향으로 휜다.
- loop closure 이후 `y` 오차는 일부 줄어들어도 전체 직진축이 기울어진다.
- pure rotation 또는 회전 비중이 큰 구간에서 keyframe과 graph가 지저분해진다.
- global OGM 윤곽이 `slam_toolbox` 대비 거칠고 벽 두께/일관성이 떨어진다.

## 로드맵

### 1. Scan Matcher 강화

- 목표
  - 현재 occupied/free endpoint 중심 score를 더 geometry-aware한 방식으로 개선
  - 회전 중 다른 각도에서 본 동일 구조를 더 안정적으로 정합
- 방향
  - distance-field 기반 obstacle proximity score 검토
  - correlative scan matching 또는 beam consistency score 강화
  - 애매한 구간에서는 correction gain을 낮추고 `odom + imu` 예측을 더 보수적으로 유지
- pass 기준
  - 큰 우회전 구간에서 map이 회전 방향으로 휘는 정도가 현재 대비 눈에 띄게 감소
  - loop 전 구간에서 `map -> odom` 또는 corrected pose jump가 줄어듦
  - 동일 주행 패턴에서 직진 복귀 후 기존 벽 라인이 덜 벌어짐
- fail 기준
  - 회전 구간 왜곡이 거의 그대로이거나 더 심해짐
  - scan matching correction이 줄었는데 전체 위치 추종이 더 불안정해짐
  - 직진 구간 품질까지 같이 나빠짐

### 2. 회전 구간 Keyframe 정책 분리

- 목표
  - pure rotation 또는 저병진 고회전 구간에서 keyframe 과생성을 줄임
  - 거의 같은 위치에서 yaw만 다른 node가 graph를 오염시키는 것을 방지
- 방향
  - translation이 작고 yaw만 큰 구간은 keyframe threshold를 다르게 적용
  - pure spin은 일반 keyframe이 아니라 별도 억제/축약 정책 검토
- pass 기준
  - 회전 구간에서 keyframe 밀도가 줄고 graph node 분포가 더 균질해짐
  - loop 이후 graph가 한쪽으로 눕는 현상이 완화됨
- fail 기준
  - keyframe 수는 줄었지만 loop closure 성공률까지 같이 크게 떨어짐
  - 회전 구간 이후 pose drift가 오히려 더 커짐

### 3. Loop Descriptor / Candidate Search 강화

- 목표
  - 다른 heading에서 같은 장소를 다시 봤을 때 loop 후보를 더 robust하게 찾음
- 방향
  - 현재 단순 downsampled range vector를 회전 shift 허용 방식으로 개선
  - sector histogram / circular shift 비교 등 회전 불변성 강화 검토
- pass 기준
  - 동일 루프 주행에서 loop candidate가 더 일찍/안정적으로 발견됨
  - 같은 장소 재방문 시 false negative가 줄어듦
- fail 기준
  - loop 후보 수만 늘고 false positive가 크게 늘어 optimizer를 더 망침
  - 연산량 증가 대비 체감 개선이 없음

### 4. Pose Graph Optimizer 보강

- 목표
  - loop closure 후 `y` 축만 맞고 전체 직진축이 기울어지는 문제를 줄임
  - loop를 닫는 동시에 기존 직선 구조도 보존
- 방향
  - 첫 keyframe anchor 강화
  - odom edge / loop edge weight 재조정
  - robust kernel 또는 correction step 보수화 검토
- pass 기준
  - loop 이후 직진 복귀 구간이 현재보다 더 평행하게 정렬됨
  - 전체 graph가 한쪽으로 눕는 현상이 줄어듦
- fail 기준
  - loop는 닫히지만 global orientation 왜곡이 그대로 남음
  - graph correction이 과도하게 커져 map이 오히려 더 찢어짐

### 5. Submap 기반 구조 전환 검토

- 목표
  - `scan -> global grid` 직결 구조의 한계를 줄이고 장기적으로 안정성 향상
- 방향
  - local submap 누적
  - scan은 submap에 먼저 정합
  - loop 후 submap pose graph 최적화
  - 최종 global map rebuild
- pass 기준
  - 장거리 주행과 loop return에서 global OGM 찢김/번짐이 줄어듦
  - front-end와 global map rebuild 책임이 더 명확해짐
- fail 기준
  - 구조는 복잡해졌지만 현재 증상 개선이 체감되지 않음
  - 구현 복잡도만 커지고 운영 검증이 더 어려워짐

## 우선순위

1. Scan Matcher 강화
2. 회전 구간 Keyframe 정책 분리
3. Loop Descriptor / Candidate Search 강화
4. Pose Graph Optimizer 보강
5. Submap 기반 구조 전환 검토

## 실험 원칙

- 한 번에 여러 축을 같이 바꾸지 않는다.
- 각 단계는 동일 주행 패턴으로 비교한다.
  - 직진
  - 큰 우회전 loop 생성
  - 원래 직진 코스로 복귀
- 개선 기준은 “예쁜 map”뿐 아니라 다음을 함께 본다.
  - corrected pose 안정성
  - loop closure 성공 시점
  - 직진축 보존 여부
  - 회전 구간 map 휨 감소 여부
