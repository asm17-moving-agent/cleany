# ROLY P1G210M 사진 기반 재구성

공식 제품 사진과 ROLY 국문 카탈로그를 참고해 직접 만든 정적 시뮬레이션 모델입니다. 제조사 CAD나 현장 실측 모델은 아닙니다. 코드와 직접 생성한 질감은 저장소와 같은 Apache-2.0을 따르며, 참고 사진과 카탈로그는 asset으로 배포하지 않습니다.

## 출처와 치수

| 출처, 항목 | 값 |
|---|---|
| 제품 | [ROLY P1G210M](https://bestuhlmall.com/product/detail.html?product_no=28) |
| 카탈로그 | [ROLY 국문 카탈로그](https://bestuhl.co.kr/wp-content/uploads/2023/11/ROLY-카다로그_국문-압축됨.pdf), p.30 |
| 제품 전체 치수 | 697×572×955 mm |
| 제품 좌고 범위 | 428–504 mm |
| 모델 좌고 | 460 mm |

부품별 치수, 곡률은 사진 기반 추정이며 카탈로그 전체 치수와 일치한다고 보장하지 않습니다. P1G110의 가동 팔걸이와 L 모델의 헤드레스트는 포함하지 않습니다.

## 생성과 조정

`world/roly_geometry.py`는 쿠션, 좌판 하부 쉘, 굴곡 등판, 테두리, 요추 패드, U자 하부 프레임, 고정 팔걸이, 5발 베이스, 이중 캐스터를 생성합니다. `config/furniture/roly_p1g210m.yaml`에서 치수와 색상을 조절합니다. 기본 색상은 흰 프레임, 회색 등판, 녹색 좌판이며, 현장 설치 색상은 미확인입니다.

Smooth normal과 UV를 포함한 6개 COLLADA mesh를 재질별로 생성합니다. 캐시는 설정, 소스 hash로 구분하고 48개 의자가 같은 mesh를 공유합니다. `ros2 run cleany_gazebo_sim generate_roly_textures.py --output <출력 폴더>`로 로컬 좌판, 등판 직물 질감을 재생성할 수 있습니다. Blender나 외부 서비스는 필요하지 않습니다.

## 근사 범위와 확인

Collision은 좌판 box, 등판 5개 panel, 다리, 캐스터, 프레임을 단순 근사하며 바닥을 막는 원판은 없습니다. `gpu_lidar`는 상세 visual geometry를 관측합니다. 망 구멍은 질감으로 표현하므로 광학 투과, 리클라이닝, 3D 모션, 캐스터 회전, 탄성은 재현하지 않습니다.

단품 확인은 `ros2_ws/src/cleany_gazebo_sim/runtime_test/validate_facility.py --scene chair`로 수행합니다. 정면(face), 사선(front), 측면(side), 후면(rear), 상부(top) PNG와 실제 scan을 저장합니다.
