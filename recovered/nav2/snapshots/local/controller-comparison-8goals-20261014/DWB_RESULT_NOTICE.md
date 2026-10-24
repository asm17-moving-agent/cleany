# DWB 결과 공지

공통 8개 목표의 좌표·world·map·planner 입력을 잠그고 원격 common 전송 및 READY를 완료했다. 측정 후에도 로컬 common 입력과 원격 204개 파일의 해시 일치를 확인했다.

DWB는 목표마다 새 world에서 1회씩, 총 8회 실행했다. Nav2 action status는 SUCCEEDED(4) 5회, timeout 후 CANCELED(5) 2회, ABORTED(6) 1회다. 완료된 목표를 재실행하거나 제한 margin을 사후 변경하지 않았다.

**제한 준수와 완전한 원자료 요건을 만족하는 캠페인으로 분류할 수 없다.**

- 실제 Gazebo wheel joint 5 rad/s 제한을 turn_arrival, far_heading, distant에서 넘었다. 최대값은 5.1284976327385 rad/s다. 예측 command cap 4.99는 실제 joint cap 준수 증명이 아니다.
- narrow_seat(seat43)는 Nav2 status4지만 GT 위치 오차가 0.329012654 m로 30 cm를 넘었다. Action 상태와 GT 허용 범위를 분리해서 해석한다.
- chair_gap와 distant recorder는 SQLite `database is locked` 오류로 중단되어 raw bag tail이 없다. 실행 중 제가 추가한 SQLite 읽기 진단과 오류 시점이 맞아, 이 진단이 recorder writer를 막았을 가능성이 높다. 해당 진단은 제거했다. Offline reindex는 metadata만 복구했고 원본 DB 해시는 그대로이며, 잃은 메시지를 복원하지 못한다.
- 독립 Probe velocity/command 기록은 8개 모두 주행 시간 구간을 포함한다. 이것은 모든 DDS 메시지의 무손실 수신이나 raw bag 복원을 뜻하지 않는다. Recorder 중단에 따른 host IO/RTF 영향도 남는다.

공통 설정, 초기 pose, actual scene/map, controller/planner parameter 일치 및 bag SQLite 무결성을 검증했다. 해당 run의 정확한 partition에 속한 프로세스는 모두 종료했다. 기존 main 변경과 이전 자료를 보존했으며 push하지 않았다. 실물 검증 및 GUI 검증은 수행하지 않았다. 접촉 관측은 연속적인 무충돌 증명이 아니며 penetration depth와 연속 최소 clearance는 미확인이다.

결과 파일:

- [전체 보고서](dwb/20261014T0035_5bfca0/REPORT.md)
- [측정 JSON](dwb/20261014T0035_5bfca0/metrics.json)
- [측정 CSV](dwb/20261014T0035_5bfca0/metrics.csv)
- [최종 검증](dwb/20261014T0035_5bfca0/final_validation.json)
- [완료 상태와 제한 미충족 표시](dwb/20261014T0035_5bfca0/COMPLETE.json)
- [독립 Probe 시간 coverage](dwb/20261014T0035_5bfca0/independent_probe_coverage.json)
- [파일 SHA256 목록](dwb/20261014T0035_5bfca0/artifact_sha256.json)
