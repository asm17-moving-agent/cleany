# ROLY P1G210M 사진 기반 재구성

공식 제품 사진과 ROLY 국문 카탈로그를 참고해 직접 만든 정적 시뮬레이션 모델이다.
제조사 CAD나 현장 실측 모델은 아니다. 코드와 직접 생성한 질감의 라이선스는
저장소와 같은 Apache-2.0이며, 참고 사진·카탈로그를 asset으로 배포하지 않는다.

- 제품: https://bestuhlmall.com/product/detail.html?product_no=28
- 카탈로그: https://bestuhl.co.kr/wp-content/uploads/2023/11/ROLY-카다로그_국문-압축됨.pdf
- 카탈로그 p.30의 P1G210M 치수: 697×572×955 mm, 좌고 428–504 mm.
  현재 모델의 좌고는 460 mm로 설정했다. 부품별 치수·곡률은 사진 기반 추정이며
  카탈로그 전체 치수와 동일하다고 주장하지 않는다. P1G110의 가동 팔걸이나
  L 모델의 헤드레스트를 추가하지 않았다.

`world/roly_geometry.py`는 둥근 쿠션·좌판 하부 쉘, 허리 굴곡이 있는 등판,
연속 테두리, 요추 패드, U자 하부 프레임, 고정 팔걸이, 테이퍼가 있는 5발 베이스,
이중 캐스터를 만든다. `config/furniture/roly_p1g210m.yaml`로 치수와 색상을 조절한다.
흰 프레임·회색 등판·녹색 좌판을 기본값으로 사용하며 현장 설치 색상은 미확인이다.

smooth normal과 UV를 포함한 6개 COLLADA mesh를 재질별로 합쳐 생성한다.
설정과 소스 hash로 캐시를 구분하고 48개 의자가 동일한 mesh를 공유한다.
`tools/facility/generate_roly_textures.py`는 로컬의 좌판·등판 직물 질감을 재생성한다.
새 모델 생성에 Blender나 외부 서비스는 필요하지 않다.

충돌은 좌판 box·등판 5개 panel·다리·캐스터·프레임의 단순 근사다. 바닥을 막는
원판 collision은 없다. `gpu_lidar`는 상세 visual geometry를 관측한다.
등판의 작은 망 구멍은 질감으로 표현하며 광학 투과, 리클라이닝과 3D 모션,
캐스터 회전·탄성은 재현하지 않는다.

단품 확인은 `tools/facility/validate_runtime.py --scene chair`로 수행한다.
정면(face)·사선(front)·측면(side)·후면(rear)·상부(top) PNG와 실제 scan을 저장한다.
