# 2026-03-16

- `amr_bringup/params/amr.yaml`의 토픽 이름 정리
  - 예: `motion_controller/command` -> `motion/command`
- `.hpp` / `.cpp`에 남아 있는 토픽 주소 하드코딩 요소를 전부 `amr_bringup/params/amr.yaml`로 파라미터화
- `amr_bringup/rviz/amr.rviz` 구현
- localization 오류 수정
- `localization/verb`, `local_planner/verb`처럼 혼용되어 있는 토픽 주소 체계 통합
