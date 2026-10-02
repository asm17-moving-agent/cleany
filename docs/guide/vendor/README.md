# 브라우저 라이브러리

문서 뷰어가 인터넷 연결 없이 Markdown을 표시하도록 배포 파일만 보관한다. `build_standalone.py`가 이 배포 파일과 라이선스를 단일 `index.html`에 포함한다. 가이드를 읽는 팀원에게 빌드나 npm 설치는 필요 없다.

| 파일 | 버전 | 원본 | 라이선스 |
| --- | --- | --- | --- |
| `marked.umd.js` | 18.0.14 | [Marked](https://github.com/markedjs/marked), npm `lib/marked.umd.js` | MIT, `marked.LICENSE` |
| `mermaid.tiny.js` | 12.0.0 | [Mermaid Tiny](https://github.com/mermaid-js/mermaid/tree/develop/packages/mermaid-tiny), npm `@mermaid-js/tiny`의 `dist/mermaid.tiny.js` | MIT, `mermaid.LICENSE` |
| `purify.min.js` | 3.4.16 | [DOMPurify](https://github.com/cure53/DOMPurify), npm `dist/purify.min.js` | Apache-2.0 OR MPL-2.0, `dompurify.LICENSE` |

2026-10-01에 npm registry의 해당 버전 tarball을 내려받고 metadata의 SHA-512 integrity를 비교했다. 원본 파일은 수정하지 않았다. Marked 결과는 DOMPurify로 정리한 뒤 표시한다.

갱신할 때는 버전을 고정하고 같은 integrity 확인 후 파일과 라이선스를 함께 교체한다. 브라우저에서 표, 코드, 링크, 이미지와 HTML 제거 동작을 확인한다.


## Mermaid 고정 배포 파일

공식 [사용 안내](https://mermaid.js.org/config/usage.html)의 Tiny IIFE 배포판을 사용한다. flowchart·state·sequence 도식은 지원하며, 사용하지 않는 mindmap·architecture·KaTeX·ELK는 포함하지 않는다. 단일 로컬 스크립트이므로 추가 다운로드와 빌드는 필요 없다.

- npm tarball: `https://registry.npmjs.org/@mermaid-js/tiny/-/tiny-12.0.0.tgz`
- npm SHA-512 integrity(다운로드 후 실제 비교): `sha512-ZW9YXy+3tSLPscBT+NnQO0K0qYH7BGES5nExl6Snoygey7xd2N0cO7Nq30N5k/u7gIeYoCrldyVy+VTCMWrNDQ==`
- vendored 파일 SHA-256: `9f2807e402479d2864bfd9a95052076fc69d0ff45a0b25148b84b70fc33425b9`

뷰어는 `startOnLoad: false`, `securityLevel: strict`와 무채색 테마로 초기화한다. fenced `mermaid` 블록을 SVG로 렌더하고 원문은 접을 수 있게 유지한다. 비동기 렌더가 끝난 뒤 페이지 generation을 확인하여 이전 페이지의 결과가 최신 본문을 덮어쓰지 않게 한다. 렌더 실패 시 원문과 짧은 안내를 표시한다. SVG 결과에도 DOMPurify를 적용한다.
