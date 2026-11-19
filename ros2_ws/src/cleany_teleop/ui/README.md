# 휴대폰 조종 UI

React와 Vite, Tailwind neutral 테마를 사용한다. `src/components/ui`는 shadcn CLI 4.21.4로
가져온 shadcn/ui 컴포넌트이며 upstream은 [shadcn/ui](https://github.com/shadcn-ui/ui), MIT license다.
나머지 라이브러리의 출처, 버전과 라이선스는 package-lock.json에 보존한다.
빌드와 설치는 루트 `make build-teleop-ui`, `make build-hardware-mapping`을 사용한다.
실행 방법과 HTTP/ROS 계약은 상위 패키지의 docs와 루트 RUNNING 문서를 따른다.
