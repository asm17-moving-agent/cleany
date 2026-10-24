# DWB 최적화 결과 — 보수적·완화 단계

전용 branch codex/dwb-optimization-20261013. local commits conservative538e923, relaxed44c083a. navigation194 tests 통과. push/물리 로봇 실험 없음. dirty main은 보존했다.

| 목표 | baseline sim초 | 보수적 sim초 / GT cm | 완화 sim초 / GT cm |
|---|---|---|---|
| straight | 29.43 | 19.91 / 12.30 | 9.13 / 13.14 |
| turn_arrival | 38.92 | 50.63 / 11.29 | 18.49 / 12.32 |
| chair_gap | 51.30 | 64.35 / 18.97 | 20.83 / 16.36 |
| far_heading | 57.93 | 57.77 / 7.93 | 25.47 / 9.81 |
| distant | 168.59 | 142.82 / 11.70 | 45.69 / 11.67 |


두 최종 단계는 각5개를1회씩 실행했다. Nav2 status4와 GT 허용오차/무접촉을 구분한다. 보수적 설정은 common command box와 safety chain을 유지했다. 완화는 결합wheel5rad/s, 외부 safety bypass, 장기 watchdog을 사용한 Gazebo 전용 실험이며 제약 조건이 다르다. 원래 baseline은 의도된 command box 위반이 있어서 엄격한 동일조건A/B로 해석하지 않는다. 단일 관측을 통계적 성공률로 해석하지 않고 합산 점수는 없다.

완화 compute exception 총0, >50ms 0. 관측 wheel 최대4.992796rad/s. 접촉은 physics telemetry와 body positive-control로 관측 경로를 확인했다. penetration depth/continuous minimum clearance는 null이며 이유를 기록했다. common localization/arrival 정책을 유지했고 chair-gap GT 오차16.36cm가 남았다.

- [보수적 상세 보고서](conservative_REPORT.md)
- [완화 상세 보고서](relaxed61_20261013/REPORT.md)
- [전체 비교 JSON](comparison_metrics.json), [CSV](comparison_metrics.csv)
- [보수적 재현](REPRODUCE.md), [완화 재현](relaxed61_20261013/REPRODUCE.md)
- 원시 결과: final61_20261013_v1 및 relaxed61_20261013/final5_v1
- 초기 HANDOFF, 중단·실패·infrastructure 진단을 보존했다.

최종 source 전체 diff는 `combined_source.patch`, base/단계 commit·설정·test hash는 `source_bundle_manifest.json`에 있다.
