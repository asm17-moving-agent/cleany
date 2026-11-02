# 경로 계획과 주행 제어

기본 AMCL 및 Omni 프로필은 `nav2_smac_planner::SmacPlannerLattice`와
`cleany_navigation::MPPIController`를 사용한다. MPPI는 `Omni` 모델로 전후와 횡이동을 허용한다.
명시적인 `axis_motion:=true`는 기존 축 전용 DWB 프로필을 선택한다.

launch는 설치된 `config/lattice/omni-r10cm.json`을 사용하며,
`GridBased.lattice_filepath`를 명시하면 해당 경로를 우선한다.
이 lattice는 공식 Nav2 1.1.20 Generator의 5cm 격자, 10cm 반경 설정,
16개 heading과 144개 primitive를 사용한다. 반경 설정은 실물 최소 회전 반경 측정값이 아니다.

커스텀 MPPI와 critic은 `mppi/src` 및 `include/cleany_navigation/mppi`에서 빌드한다.
기본 Nav2 MPPI와 구분하도록 controller는 `cleany_navigation::MPPIController`,
critic은 `cleany_navigation::mppi::critics`로 등록한다.
원본 라이선스와 설명은 `mppi/LICENSE.md`와 `mppi/README.upstream.md`에 보존한다.
