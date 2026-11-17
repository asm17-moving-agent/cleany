# 3D 모델과 라이브러리 출처

STEP 파일은 원본 그대로 보관하고 `${KIPRJMOD}/libraries/3dmodels/`로 참조한다. 단위는 mm이며 배율은 모두 1이다.

## 적용 모델

| Reference | 모델 | 출처와 범위 | 풋프린트 변환 |
|---|---|---|---|
| L1–L5 | `L_Wurth_WE-XHMI-6060.step` | Würth 공식 WE-XHMI-6060 패키지 모델. 제조사 [WE-XHMI 제품 페이지](https://www.we-online.com/en/components/products/WE-XHMI)의 74439346033 패키지에 대응 | 이동 없음, Z축 90° 회전 |
| U3–U7 | `SOIC-8-1EP_3.9x4.9mm_P1.27mm_EP2.29x3mm.step` | KiCad의 일반 PowerPAD SOIC-8 모델. LMR33640DDDAR 전용 제조사 모델은 아님 | 이동과 회전 없음 |
| U9 | `easyeda/TCA9548ADGSR.step` | LCSC C41730132에 연결된 EasyEDA VSSOP-24 공유 패키지 모델 | Z축 +0.6 이동, Z축 90° 회전 |
| J7–J10 | `molex/022057035.step` | Newark의 정확한 22-05-7035 품목에 연결된 Farnell 제조사 CAD | X +2.5, Y −3.65, Z +2.45 이동, Z축 180° 회전 |
| J11–J14 | `molex/022057045.step` | Newark의 정확한 22-05-7045 품목에 연결된 Farnell 제조사 CAD | X +3.75, Y −3.65, Z +2.45 이동, Z축 180° 회전 |

Würth 모델의 기본 몸체는 6.45 × 6.65 × 5.8이다. 90° 회전하여 로컬 풋프린트의 6.65 × 6.45 몸체와 Y축 방향 단자에 맞춘다. 모델 참조는 커스텀 풋프린트와 PCB 인스턴스에 모두 저장한다.

SOIC 모델은 3.9 × 4.9 몸체, 1.27 피치, 6.0 리드 전체 폭, 1.55 높이와 2.29 × 3.0 노출 패드를 표현한다. 이 치수는 [TI 데이터시트](https://www.ti.com/lit/ds/symlink/lmr33640.pdf)의 DDA0008J 기계 치수 범위에 들어간다. 기구 간섭 검토에는 모델의 명목 높이 대신 제조사 최대 높이 1.7을 사용한다. 기존의 누락된 `HTSOP-...Pitch1.27mm.step`와 이름이 비슷한 1.1 높이, 5.4 리드 폭의 HTSOP 모델은 이 패키지의 대체 모델이 아니다.

U3–U7와 J7–J14의 모델 참조는 PCB 인스턴스에 저장한다. 기본 라이브러리에서 해당 풋프린트를 다시 갱신할 때 모델 참조를 재확인한다.

U9 모델은 24핀, 0.5 피치이며 몸체는 약 6.1 × 3.0, 리드 전체 폭은 4.9, 높이는 1.1이다. 원본 STEP의 Z 범위는 −0.6에서 +0.5이므로 +0.6 이동하여 실장면에 맞춘다. 기존 DGS0024A 풋프린트에는 Z축 90° 회전으로 맞추며, 핀 1 표식은 실크의 핀 1 방향과 일치한다. 모델 참조는 커스텀 풋프린트와 PCB 인스턴스에 모두 저장한다.

Molex 모델은 각각 3핀과 4핀, 2.50 피치의 직각 헤더다. 원본의 핀 행은 Y = −3.65이고 몸체 실장면은 Z = −2.45이다. 위 변환으로 모든 핀 중심을 풋프린트 패드에 맞추며 몸체는 보드 위 0–4.90, 납땜 꼬리는 보드 아래 −3.50까지 위치한다. 몸체 폭은 각각 9.90과 12.40이며 전면은 풋프린트의 +Y 방향이다.

### 원본과 이용 조건

- Würth: [공식 KiCad 라이브러리의 원본 STEP](https://github.com/WurthElektronik/KiCad-Library/blob/215595ffc7c98649394598bc6bac81ddc421fcc6/3dmodels/Inductor_SMD_Wurth.3dshapes/L_Wurth_WE-XHMI-6060.step), commit `215595ffc7c98649394598bc6bac81ddc421fcc6`. Würth Elektronik eiSos GmbH & Co. KG 제공. [이용 약관](License_Terms_WE_KiCad_library.pdf)과 [면책 고지](Disclaimer_WE_KiCad_library.pdf)를 함께 보관한다. 독자 약관이며 CC/MIT 라이선스가 아니다.
- KiCad: [원본 STEP](https://gitlab.com/kicad/libraries/kicad-packages3D/-/blob/afa6aab16f43a736cd062b8c853863e753653921/Package_SO.3dshapes/SOIC-8-1EP_3.9x4.9mm_P1.27mm_EP2.29x3mm.step), commit `afa6aab16f43a736cd062b8c853863e753653921`. KiCad 제공. [CC-BY-SA 4.0과 전자 설계 예외](LICENSE.KiCad-packages3D.md)를 따른다.
- U9: **JLCEDA/EasyEDA Official Library** ([JLCEDA](https://lceda.cn/), [EasyEDA](https://easyeda.com/)), LCSC 소유 레코드 [C41730132](https://easyeda.com/api/products/C41730132/components). 모델 UUID `6f8d08e6fe1d4497a9774c553e01fa50`, [원본 STEP](https://modules.easyeda.com/qAxj6KHrDKw4blvCG8QJPs7Y/6f8d08e6fe1d4497a9774c553e01fa50). 제조사 TI의 전용 CAD로 인증된 모델은 아니다.
- Molex: [22-05-7035 Newark 품목](https://www.newark.com/molex/22-05-7035/connector-header-3pos-1row-2-5mm/dp/57R9151)과 [22-05-7045 Newark 품목](https://canada.newark.com/molex/22-05-7045/connector-header-4pos-1row-2-54mm/dp/07P8724)의 `CAD Multiple Formats EN` 링크에서 각각 [1990352.zip](https://www.farnell.com/cad/1990352.zip)과 [1976541.zip](https://www.farnell.com/cad/1976541.zip)을 받았다. ZIP에는 `022057035.stp` 또는 `022057045.stp` 한 파일만 있으며, 내용은 그대로 두고 확장자만 `.step`으로 저장했다. 원본 헤더의 작성일은 2014-03-21이다. 다운로드와 Chromium은 타임아웃이 발생했으며 Playwright Firefox로 다운로드했다. 원본 해시, 부품 번호와 변환은 [출처 메타데이터](molex/sources.json)에 보관한다.

Molex CAD는 KiCad 라이브러리 라이선스의 적용 대상이 아니다. ZIP에는 별도 이용 고지가 없다. [Molex Legal Notice](https://www.molex.com/en-us/about/legal)는 제품의 응용 평가를 위한 다운로드와 사용을 허용하며, 공개 재배포 허가는 확인되지 않았다. 공개 저장소나 배포물에 포함하기 전 권리자의 허가를 확인한다.

공개 저장소에는 Molex STEP 2개를 포함하지 않는다. 이용 조건을 확인한 사용자는 위 원본 ZIP에서 각각 `022057035.stp`, `022057045.stp`를 받아 내용 변경 없이 `molex/022057035.step`, `molex/022057045.step`으로 저장할 수 있다. 해당 파일은 Git에서 제외하며, 파일이 없으면 원본 해시 검사 1건을 건너뛴다. 출처 메타데이터와 PCB 배치 검사는 항상 실행한다. 이 모델의 유무는 회로도와 PCB의 전기적 연결 검사에 영향을 주지 않는다.

EasyEDA STEP와 원본 저작권 및 이용 고지가 담긴 [메타데이터](easyeda/TCA9548ADGSR.source.json)를 저장소에 함께 보관한다. 변환기의 AGPL 라이선스는 다운로드한 CAD의 라이선스가 아니다.

## EasyEDA 다운로드와 풋프린트

[easyeda2kicad.py](https://github.com/uPesy/easyeda2kicad.py) 1.0.1로 개별 부품을 내려받는다. 변환 심볼과 풋프린트는 검사용이며 프로젝트 라이브러리에 자동 등록하지 않는다.

| 부품 | 검색 키와 공개 링크 | 확인 범위 |
|---|---|---|
| Assmann A-2004-3-4-LP-N-R | [EasyEDA 공개 레코드](https://easyeda.com/modules/A-2004-3-4-LP-N-R-RJ11_43ee4cb605614b2eb2bb52d979074b97), [C6113817](https://lcsc.com/product-detail/Ethernet-Connectors-Modular-Connectors-RJ45-RJ11_Assmann-WSW-Components-A-2004-3-4-LP-N-R_C6113817.html) | 부품 코드 API는 `Component not found`. 공개 레코드는 docType 14 모듈이며 4패드와 홀만 있고 3D 참조 없음. 패드 번호별 상대 위치도 기존 풋프린트와 달라 대체하지 않음 |
| TI TCA9548ADGSR | [C41730132](https://www.lcsc.com/product-detail/C41730132.html) | 심볼, 24패드 풋프린트와 STEP 다운로드 가능. U9의 3D 참조에 사용 |
| On Shore USB-B1HSB6 | [C3197678](https://www.lcsc.com/product-detail/C3197678.html) | 카탈로그 등재와 달리 부품 API는 `Component not found` |
| Molex 0022057045 | [C9900017324](https://jlcpcb.com/partdetail/JLCPCBAssembly-0022057045/C9900017324) | 부품 API는 `Component not found`. 카탈로그 제조사 필드도 `JLCPCB Assembly`이므로 Molex 원본 모델로 간주하지 않음 |
| TI LMR33640DDDAR | [C1852154](https://jlcpcb.com/partdetail/TexasInstruments-LMR33640DDDAR/C1852154) | 심볼, 9패드 풋프린트와 STEP 다운로드 가능. 원본 UUID `3936dbd423424b148317d27cdee29b93`. U3–U7에는 기존 KiCad PowerPAD 모델 사용 |

변환된 U9과 LMR 풋프린트의 courtyard는 패드 전체를 포함하지 않는다. 다운로드 성공만으로 기존 풋프린트의 대체품으로 사용하지 않는다. Molex 0022057035/5268-03A의 정확한 공개 EasyEDA 레코드는 확보하지 못했다.

### U9 원본 다시 받기

아래 명령은 보관 중인 모델의 원본을 다시 받을 때 사용한다. 프로젝트 폴더에서 실행하며, `uv`가 필요하다. 원본 메타데이터와 이용 고지를 함께 보존한다.

```bash
OUT=$(mktemp -d)
(
  cd "$OUT"
  uvx --from easyeda2kicad==1.0.1 easyeda2kicad \
    --full --lcsc_id=C41730132 --output "$OUT/imported" --use-cache
)
mkdir -p libraries/3dmodels/easyeda
cp "$OUT/imported.3dshapes/VSSOP-24_L6.1-W3.0-P0.50-LS4.9-BL.step" \
  libraries/3dmodels/easyeda/TCA9548ADGSR.step
cp "$OUT/.easyeda_cache/C41730132.json" \
  libraries/3dmodels/easyeda/TCA9548ADGSR.source.json
sha256sum libraries/3dmodels/easyeda/TCA9548ADGSR.step
```

확인한 원본 SHA-256은 `271ba4897aad65006349c54c4a2f47a401f9dfb15fb8c0b1943b6141cfa99ce3`이다. 서버에서 모델이 바뀌면 해시 불일치 원인을 확인하고 치수와 정렬을 다시 검토한다.

## 미확보 모델

| Reference | 필요한 모델과 확보 경로 |
|---|---|
| SW3 | Molex 5268-02A 2P 헤더. 지정된 표준 KiCad STEP 파일이 로컬 설치에 없음 |
| J1 | 정확한 XT60 MPN과 성별 확정 후 해당 제조사 모델 |
| J2 | USB-B1HSB6. [제조사 페이지](https://www.on-shore.com/part/usb-b1h-interconnect-usb-type-b/)는 모델 요청 양식을 제공한다. EasyEDA C3197678 API는 `Component not found` |
| J15–J18 | Assmann A-2004-3-4-LP-N-R. EasyEDA 공개 레코드에는 3D 참조 없음. SnapMagic은 모델 제작 요청 화면이며, 제조사 TraceParts의 정확한 MPN 경로는 Playwright에서도 HTTP 403 |

J2와 동일한 파일명의 [과거 KiCad 초안](https://github.com/KiCad/kicad-packages3D/blob/d92682a71a1eb0c57cf0c3f44212cee267a01642/Connector_USB.3dshapes/USB_B_OST_USB-B1HSxx_Horizontal.step)은 실드 탭 중심 간격이 약 11.40으로 현재 풋프린트의 12.04와 다르다. 이 초안은 J2에 적용하지 않는다.

[Leako의 추가 USB-B1HSB6 STEP 후보](https://github.com/barbeque/leako/blob/10f17095319e50987e920cf7e21cdf39c934c567/USB-B1HSB6/USB-B1HSB6.step)는 제조사 원본으로 확인되지 않았다. 실장면의 실드 탭 단면 중심 간격은 약 11.77이며, 보드 아래에서 바깥쪽으로 벌어지는 형상이다. 12.04 간격의 현재 홀과 제조사 도면에 대한 추가 대조가 필요하다. 이 저장소는 CC-BY-NC 4.0을 선언하므로 이용 조건도 확인해야 한다. 프로젝트에는 복사하거나 연결하지 않았다.

위 7개 부품의 모델은 미확보 상태이며 J7–J14의 Molex 모델은 로컬 설치 항목이다. 나머지 모델 중 표준 KiCad 경로는 설치된 3D 라이브러리에 따라 해석된다. 장착 홀 H1–H4는 PCB 부품 BOM에서 제외한 기구 요소다.

## 로컬 검사

STEP 원본 해시와 PCB의 모델 경로, 표시 상태, 배율, 이동과 회전을 검사한다. 기존 제조 규칙 검사도 함께 실행한다.

```bash
/usr/bin/python3 -m unittest discover -s tests -p 'test_*.py' -v
```
