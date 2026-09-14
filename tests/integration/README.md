# integration tests

이 디렉터리는 향후 패키지 간 end-to-end 테스트를 둘 scaffold이며 현재 실행 파일은 없다.

실행 가능한 통합 검사는 각 ROS 패키지의 `test/`에 있다.
`make test-grasp-pregrasp-runtime`은 MoveIt 계획과 MuJoCo pregrasp 실행을,
`make test`는 등록된 전체 패키지 검사를 실행한다. 범위는 [상위 안내](../README.md)를 따른다.

Mission Manager, Planner, Robot Interface를 하나로 연결한 mission end-to-end 검증은
아직 구현되지 않았다.
