"""Render the bounded retreat/home handoff result without inventing cycle timing."""
import argparse
import json
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, required=True)
    args = parser.parse_args()
    result = json.loads((args.root/'verification.json').read_text())
    assert all(result[key]['hashes_match'] for key in ('native', 'container'))
    assert result['regression']['failures'] == result['regression']['errors'] == 0
    report = f'''<!doctype html><html lang="ko"><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>6번 후퇴와 본 복귀 준비 공유</title><style>
*{{box-sizing:border-box}}body{{margin:0;background:#f2f5f9;color:#1b2b42;font:16px/1.75 system-ui,-apple-system,sans-serif}}
main{{max-width:1020px;margin:auto;padding:40px 24px}}h1{{font-size:30px;line-height:1.4}}h2{{font-size:22px;margin:0 0 14px}}
section{{background:white;border:1px solid #d9e2ee;border-radius:14px;padding:25px;margin:22px 0}}
.status{{color:#136655;font-weight:750}}.muted{{color:#55677d}}.scroll{{overflow:auto}}
table{{width:100%;border-collapse:collapse}}th,td{{padding:12px;text-align:left;border-bottom:1px solid #dde5ed;vertical-align:top}}th{{background:#edf3f8}}
a{{color:#175aa0}}.flow{{border-left:4px solid #136655;background:#eff8f4;padding:14px 18px}}
</style><main><div class="status">JETSON 실행본 반영 완료 · 2026-10-29</div>
<h1>6번: 놓은 뒤 후퇴와 본 복귀의 준비 공유</h1>
<p>후퇴에서 확인한 측정 결과를 한 번 넘겨 중복 FK를 줄이고, 본 복귀의 새 조회를 다른 준비 작업과 겹치도록 적용했습니다.</p>
<section><h2>무엇이 바뀌었나</h2><div class="scroll"><table>
<tr><th>대상</th><th>이전</th><th>현재</th></tr>
<tr><td>후퇴 직후 TCP 확인</td><td>도착 검사의 FK 후 다시 측정 FK 요청</td><td>전체 관절·목표·모델이 같고 최신이면 추가 요청 1회 → 0회</td></tr>
<tr><td>본 복귀 모델/상태 조회</td><td>본 복귀 실행 준비에서 시작</td><td>후퇴 도착 확인 후 시작, 남은 확인과 경로 준비 중 처리</td></tr>
<tr><td>조회 횟수</td><td>본 복귀용 새 조회 4개</td><td>새 조회 4개 유지. 오래되면 폐기 후 새로 조회</td></tr>
<tr><td>후퇴와 본 복귀</td><td>후퇴 완료 확인 → 본 복귀</td><td>순서와 측정 도착 기준, 안정화 조건 유지</td></tr>
</table></div><p class="muted">공유 범위는 놓기가 확인된 임시 집기 모드의 빈 팔 복귀 한 번입니다. 두 동작을 하나의 연속 궤적으로 합친 변경은 아닙니다.</p></section>
<section><h2>실행 흐름</h2><div class="flow">후퇴 이동 → 안정화·측정 도착 확인 → 본 복귀용 새 조회 시작<br>→ 같은 상태의 측정 FK 재사용 → 본 복귀 경로 준비 → 조회 응답 검증 → 본 복귀 이동</div>
<p>기존 최대 나이 <b>0.25초</b> 안에서만 공유합니다. 관절이 변하면 FK를 다시 계산합니다. 추가 명령, 통신 복구, 잘못된 팔 또는 응답 만료 시 선행 조회를 버리고 정상 조회합니다. 후퇴 후 모델이 바뀌면 본 복귀를 막습니다. 예외·취소·종료 시 미사용 요청을 정리합니다.</p>
<p><code>return_endpoint_handoff.reused</code>로 FK 재사용 여부를, <code>return_model_prefetch.outcome</code>으로 선행 조회의 소비·폐기 여부를 확인할 수 있습니다.</p></section>
<section><h2>얼마나 줄었나</h2><p><b>새 실물 사이클의 초 단위 단축량은 아직 측정하지 않았습니다.</b> 모터 대체 포트를 연결한 검사에서 같은 상태의 추가 측정 FK 호출이 1회에서 0회로 줄었고, 다음 단계 조회를 먼저 보낸 뒤 응답을 소비하는 것을 확인했습니다.</p>
<p>팔의 물리적 이동 속도·가속도·중간 자세와 궤적 시간은 바꾸지 않았습니다. 기존 후퇴 최소 시간과 시작 대기 단축을 이번 절감량으로 다시 계산하지 않습니다. 상태가 달라지거나 경로 준비가 길어지면 재사용 효과가 없을 수 있습니다.</p></section>
<section><h2>검증과 반영</h2><p><b>집중 검사 {result['focused']['passed']}개, 관련 회귀 검사 {result['regression']['passed']}개 통과.</b> 고정 CUDA 환경이 필요한 기존 GPU 검사 1개는 제외했습니다. 처음 실패한 42개는 이전 기록 궤적 API를 따라가지 못한 모의 실행기 문제였으며, 변경 전 설치본에서도 재현했습니다. 모의 실행기와 빠진 설정을 보완한 뒤 재검사를 통과했습니다.</p>
<p>native 2개 패키지와 motion container를 빌드했습니다. 실제 import된 native 실행 모듈 5개, container 모듈 3개의 hash를 배포 명세와 대조했고 <code>hardware_share_return_preparation=true</code>를 확인했습니다. 실행 중인 로봇 프로세스가 없는 상태에서 갱신했으며 다음 시작부터 사용합니다.</p>
<p class="muted">로컬 707538d / Jetson 4f92bc0 기반 미커밋 변경. 실물 모터 명령과 전체 workspace 검사는 실행하지 않았습니다. 기존 NumPy/SciPy 및 container setuptools_scm 경고가 남습니다. 보고서 구조와 링크는 정적 검사했으며 브라우저 화면 검증은 하지 않았습니다.</p>
<p><a href="verification.json">검증 요약</a> · <a href="regression.xml">회귀 검사</a> · <a href="deployment.json">배포 명세</a> · <a href="native-installed.json">native 설치 확인</a> · <a href="container-installed.json">container 설치 확인</a> · <a href="baseline-fixture-failure.log">기존 모의 실행기 실패 재현</a></p>
</section></main></html>'''
    (args.root/'report.html').write_text(report)


if __name__ == '__main__':
    main()
