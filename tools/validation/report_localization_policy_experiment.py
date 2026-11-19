#!/usr/bin/env python3
"""Render reference-relative recovery comparisons without claiming physical truth."""
from __future__ import annotations

import argparse
from collections import Counter
import csv
from datetime import datetime, timezone, timedelta
import html
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

NAMES={'floor_0.65':'기존 65%', 'floor_0.50':'A · 하한만 50%', 'structured':'B · 복구 흐름 분리'}
ENGLISH=['Existing 65%', 'A: threshold 50%', 'B: structured']
KST=timezone(timedelta(hours=9))


def summarize(cases: list[dict], policy: str) -> dict:
    baseline=[c['policies'][policy]for c in cases if not any(c['offset']) and c['reference_stable']]
    injected=[c['policies'][policy]for c in cases if any(c['offset']) and c['reference_stable']]
    admitted=[r for r in injected if r['accepted']]
    large=[r for r in admitted if r['reference_translation_error_m']>.2 or r['reference_yaw_error_rad']>.15]
    difficult=[c['policies'][policy]for c in cases if not any(c['offset']) and not c['reference_stable']]
    return {'baseline_count':len(baseline),'baseline_accepted':sum(r['accepted']for r in baseline),
            'baseline_reseed':sum(r['operation']=='reseed'for r in baseline),
            'injected_count':len(injected),'admitted':len(admitted),'large_residual':len(large),
            'within_tolerance':len(admitted)-len(large),'rejected':len(injected)-len(admitted),
            'large_residual_keep':sum(r['operation']=='keep'for r in large),
            'difficult_base_count':len(difficult),'difficult_admitted':sum(r['accepted']for r in difficult),
            'difficult_keep':sum(r['operation']=='keep'for r in difficult),
            'rejection_reasons':dict(Counter(r['reason']for r in injected if not r['accepted']))}


def main() -> None:
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('run',type=Path)
    args=parser.parse_args();out=args.run
    cases=json.loads((out/'results.json').read_text());inputs=json.loads((out/'inputs.json').read_text())
    totals={split:{policy:summarize([c for c in cases if split=='all'or c['split']==split],policy)
                   for policy in cases[0]['policies']}for split in ('all','development','holdout')}
    (out/'summary.json').write_text(json.dumps(totals,indent=2))
    with (out/'comparison.csv').open('w')as stream:
        writer=csv.writer(stream);writer.writerow(['bag','split','window','time_kst','reference_stable','offset_x','offset_y','offset_yaw',
                'policy','accepted','operation','reason','translation_residual_m','yaw_residual_deg'])
        for c in cases:
            for key,r in c['policies'].items():
                writer.writerow([c['bag'],c['split'],c['window'],datetime.fromtimestamp(c['time'],KST).isoformat(),
                    c['reference_stable'],*c['offset'],key,r['accepted'],r['operation'],r['reason'],
                    r['reference_translation_error_m'],None if r['reference_yaw_error_rad']is None else np.degrees(r['reference_yaw_error_rad'])])
    plt.rcParams.update({'font.size':11,'axes.spines.top':False,'axes.spines.right':False})
    fig,axes=plt.subplots(1,2,figsize=(14,5),layout='constrained')
    x=np.arange(3)
    for ax,split,title in zip(axes,('all','holdout'),('All six bags','Held-out three bags')):
        values=[totals[split][p]for p in NAMES]
        corrected=np.array([v['within_tolerance']for v in values])
        bad=np.array([v['large_residual']for v in values])
        reject=np.array([v['rejected']for v in values])
        ax.bar(x,corrected,color='#209989',label='Admitted: within reference tolerance')
        ax.bar(x,bad,bottom=corrected,color='#d95659',label='Admitted: large reference residual')
        ax.bar(x,reject,bottom=corrected+bad,color='#cbd4df',label='Rejected')
        ax.set_xticks(x,ENGLISH);ax.set_ylabel('Injected-offset trials');ax.set_title(title)
        for i,v in enumerate(values):
            if corrected[i]:ax.text(i,corrected[i]/2,str(int(corrected[i])),ha='center',va='center',color='white')
            if bad[i]:
                ax.text(i,corrected[i]+max(bad[i]/2,2),str(int(bad[i])),ha='center',
                        va='center',color='white'if bad[i]>3 else'#a12433')
            if reject[i]:ax.text(i,corrected[i]+bad[i]+reject[i]/2,str(int(reject[i])),ha='center',va='center')
    axes[0].legend(loc='upper center',bbox_to_anchor=(.5,-.15),fontsize=9)
    fig.savefig(out/'comparison.png',dpi=160);plt.close(fig)
    fig,axes=plt.subplots(1,3,figsize=(14,4.5),layout='constrained')
    for ax,policy,label in zip(axes,NAMES,ENGLISH):
        for kind,marker in [('keep','o'),('reseed','x')]:
            points=[c['policies'][policy]for c in cases if c['reference_stable']and any(c['offset'])
                    and c['policies'][policy]['accepted']and c['policies'][policy]['operation']==kind]
            ax.scatter([p['reference_translation_error_m']for p in points],
                       [np.degrees(p['reference_yaw_error_rad'])for p in points],label=kind,marker=marker,alpha=.7)
        ax.axvline(.2,color='#c54f55',ls='--');ax.axhline(np.degrees(.15),color='#c54f55',ls='--')
        ax.set(xlabel='Distance from recorded reference (m)',ylabel='Yaw difference (deg)',title=label,xlim=(-.02,.65),ylim=(-1,25))
        ax.grid(alpha=.15);ax.legend()
    fig.savefig(out/'residuals.png',dpi=160);plt.close(fig)
    def table(split: str) -> str:
        body=''
        for p,label in NAMES.items():
            s=totals[split][p]
            body+=f'<tr><th>{label}</th><td>{s["baseline_accepted"]}/{s["baseline_count"]}</td><td>{s["baseline_reseed"]}</td><td>{s["within_tolerance"]}/{s["injected_count"]}</td><td class="bad">{s["large_residual"]}/{s["injected_count"]}</td><td>{s["rejected"]}</td></tr>'
        return '<div class="scroll"><table><thead><tr><th>방법</th><th>정상 참조 복귀 허용</th><th>정상 참조 재지정</th><th>주입 오차 시험: 허용·잔차 이내</th><th>허용했지만 큰 잔차</th><th>거부</th></tr></thead><tbody>'+body+'</tbody></table></div>'
    sensitivity=''
    for p in cases[0]['policies']:
        if p=='structured':continue
        s=totals['all'][p]
        sensitivity+=f'<tr><th>{float(p.split("_")[1])*100:.0f}%</th><td>{s["baseline_accepted"]}/{s["baseline_count"]}</td><td>{s["within_tolerance"]}</td><td>{s["large_residual"]}</td><td>{s["rejected"]}</td></tr>'
    bags=''
    for m in inputs:
        matched=m['score_crosscheck']['matched_scans']
        bags+=f'<tr><td>{m["bag"][:8]}</td><td>{m["split"]}</td><td>{(m["end"]-m["start"])/60:.2f}</td><td>{m["observations"]}</td><td>{m["windows"]}</td><td>{matched}</td></tr>'
    # Show B's most important regression: stable normal scores can retain an injected wrong seed.
    regressions=[c for c in cases if c['reference_stable']and any(c['offset'])and
        c['policies']['structured']['operation']=='keep'and
        (c['policies']['structured']['reference_translation_error_m']>.2 or
         c['policies']['structured']['reference_yaw_error_rad']>.15)]
    example=''
    if regressions:
        c=regressions[0];a=c['policies']['floor_0.50'];b=c['policies']['structured'];s=c['seed_score']
        example=f'<p>예: bag {c["bag"][:8]}, {datetime.fromtimestamp(c["time"],KST).strftime("%H:%M:%S")}. 주입 오차는 {html.escape(str(c["offset"]))}(m, m, rad). 시작 점수는 일치율 {s["inlier_ratio"]*100:.1f}%, 평균 잔차 {s["mean_distance"]*100:.1f}cm였다. B는 이를 안정된 정상 위치로 받아들여 그대로 유지했고, 기록 참조 대비 {b["reference_translation_error_m"]*100:.1f}cm / {np.degrees(b["reference_yaw_error_rad"]):.1f}° 차이가 남았다. A의 판정은 {html.escape(a["operation"])}였다.</p>'
    duration=sum(m['end']-m['start']for m in inputs)
    doc=f'''<!doctype html><html lang="ko"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Cleany · 위치 복구 정책 비교</title><style>
*{{box-sizing:border-box}}body{{margin:0;background:#eef2f7;color:#1b2c40;font:16px/1.75 system-ui,sans-serif}}main{{max-width:1180px;margin:auto;padding:44px 24px}}h1{{font-size:clamp(28px,4vw,42px);line-height:1.3;letter-spacing:-1px}}h2{{font-size:23px}}.eyebrow{{font-size:12px;letter-spacing:2px;color:#007d74;font-weight:bold}}section,.note{{background:white;border:1px solid #dce5ee;border-radius:16px;padding:25px;margin:20px 0}}.note{{border-left:5px solid #d58f34}}.grid{{display:grid;grid-template-columns:repeat(3,1fr);gap:14px}}.card{{background:#173d4b;color:white;border-radius:14px;padding:20px}}.card b{{display:block;font-size:30px}}.card small{{color:#c4dfe7}}img{{width:100%;height:auto}}.scroll{{overflow-x:auto}}table{{width:100%;border-collapse:collapse;font-size:14px}}td,th{{padding:12px 8px;border-bottom:1px solid #e3eaf0;text-align:right}}th:first-child,td:first-child{{text-align:left}}thead{{background:#eef5f7}}.bad{{color:#b42b39;font-weight:bold}}a{{color:#007d74}}code{{font-size:13px}}@media(max-width:720px){{.grid{{grid-template-columns:1fr}}main{{padding:24px 12px}}section{{padding:18px}}}}</style>
<main><div class="eyebrow">CLEANY / OFFLINE RECOVERY POLICY EXPERIMENT</div><h1>복구 검색 분기 비교<br>실행 코드 전체 비교의 누락 정정</h1>
<div class="note"><b>후속 코드 대조에 따른 정정:</b> 기존 실행 코드에도 현재 score가 정상이고 상태가 available이면 already_aligned로 복구를 완료하는 분기가 있다. 이 실험의 기존/A 모델은 그 분기를 포함하지 않고 후보 검색부터 평가했다. 따라서 아래 정상 참조의 1/17/18 허용 및 1/17/0 재지정 수치로 실제 기존 코드와 B의 우열을 주장할 수 없다. ‘B가 불필요한 재지정을 줄였다’는 실행 코드 대비 결론을 철회한다. 주입 시험도 이 명시된 모델에 대한 결과이며 전체 실행 정책 비교는 기존 분기를 포함해 다시 해야 한다.</div>
<p>6개 bag의 약 {duration/60:.1f}분에서 정지 관측 창 28개를 골라 196개 조건을 비교했다. 기존 65%와 A(50% 하한), B(현재 위치 정상 복귀와 새 후보 검증 분리)를 같은 지도·스캔·검색 결과로 평가했다. 실물 서버 설정은 변경하지 않았다.</p>
<div class="grid"><div class="card"><small>정상 참조 복귀 허용 · 기존 / A / B</small><b>1 / 17 / 18</b><small>정상 점수가 유지된 참조 18개 중</small></div><div class="card"><small>허용했지만 큰 잔차 · A / B</small><b>7 / 9</b><small>108개 주입 오차 시험 중</small></div><div class="card"><small>별도 3개 bag · A / B</small><b>44 / 53</b><small>54개 중 허용·잔차 이내, 큰 잔차 0 / 0</small></div></div>
<div class="note"><b>실측 정답이나 실물 복구 성공률은 아니다.</b> 기록된 AMCL 위치를 참조로 잡고 X/Y ±40cm 또는 yaw ±0.35rad(약 20°)를 주입했다. ‘큰 잔차’는 허용한 위치가 기록 참조에서 20cm 또는 0.15rad(약 8.6°) 넘게 떨어진 경우다. 참조 위치 자체가 틀릴 수 있으므로 실제 오복구라고 확정할 수 없다. 반대로 이 시험을 통과했다고 실제 위치가 맞는 것도 아니다.</div>
<section><h2>방법을 먼저 고정한 비교</h2><p><b>기존 / A:</b> 복구 하한만 각각 65% / 50%. 검색 반경 0.8m, yaw ±0.7rad, 필요 시 평균 잔차 4cm 개선, 대안 후보와 2.5cm 점수 차이, 분리한 검증 빔과 후속 새 스캔 검증은 동일하다.</p><p><b>B:</b> 현재 위치가 정상 기준(30%·18cm)을 1.5초 유지하면 재지정 없이 복귀를 허용한다. 그렇지 않으면 후보 하한 30%에 기존 개선·모호성 검사를 적용하고, 후속 스캔에서도 1.5초 안정성을 요구한다. 정합 점수 산식 자체는 바꾸지 않았다.</p><p>모든 방법의 후보 탐색 결과를 공유해 하한과 판정 흐름의 차이만 비교했다. 새 위치를 AMCL에 실제로 넣은 뒤의 검증은 수행하지 않았으므로 이 결과는 <b>복구 후보 적용 전 허용 판단</b>이다.</p></section>
<section><h2>전체 결과</h2>{table('all')}<p>정상 참조 18개에는 인위적 오차를 넣지 않았다. 나머지 108개는 그 18개에 각각 6가지 오차를 넣은 시험이다. 원래부터 정상 판정이 안정적이지 않은 10개 창과 그 파생 시험은 정답 판정에서 제외하고 원본 JSON에 별도로 남겼다.</p><img src="comparison.png"></section>
<section><h2>따로 남겨 둔 bag의 결과</h2>{table('holdout')}<p>앞 3개 bag과 뒤 3개 bag을 분리했고, 실험 결과를 본 뒤 수치를 바꾸지 않았다. 뒤 3개에서는 B가 A보다 9개 더 많은 주입 오차 시험을 허용하면서 참조 잔차 이내에 들어왔다. 그러나 앞쪽 bag에서의 실패가 사라지는 것은 아니다. 같은 로봇·지도·장소의 반복 기록이라 독립된 환경 검증도 아니다.</p></section>
<section><h2>B에서 드러난 결함</h2><p>정상 점수가 일정 시간 유지된다는 사실만으로는 현재 위치의 정확성을 증명할 수 없었다. B가 큰 잔차를 남긴 9개 중 4개는 새 후보로 옮긴 경우가 아니라 <b>틀어 놓은 현재 위치를 그대로 유지한 경우</b>다. 따라서 ‘정상 관측 지속 → 복구 성공’ 분기는 추가 근거 없이 실행본에 넣으면 안 된다.</p>{example}<img src="residuals.png"></section>
<section><h2>수치만 낮췄을 때의 민감도</h2><div class="scroll"><table><thead><tr><th>후보 하한</th><th>정상 참조 허용</th><th>주입 시험 허용·잔차 이내</th><th>허용·큰 잔차</th><th>거부</th></tr></thead><tbody>{sensitivity}</tbody></table></div><p>하한을 낮추면 허용은 늘지만, 위치를 구별하지 못하는 사례가 제거되지는 않는다. 65%도 실제 위치 정확도의 보증은 아니다.</p><p>원래 점수가 불안정했던 10개 비주입 창에서는 기존/A/B가 각각 0/8/7개를 허용했다. 이 창에는 참조 위치의 정상성도 확인되지 않아 성공률로 계산하지 않았다.</p></section>
<section><h2>판단과 다음 개선점</h2><p><b>현 단계에서는 B가 무조건 더 낫다고 결론낼 수 없다.</b> B는 정상 위치의 불필요한 재지정을 없애고 별도 bag에서 허용 범위를 넓혔다. 하지만 전체 주입 시험에서는 A보다 큰 잔차를 남긴 허용이 2개 많았다. A 역시 큰 잔차 허용이 7개라, 숫자만 50%로 낮추는 것을 검증된 해결책으로 추천하지 않는다.</p><p>다음 후보는 현재 위치 유지 분기에도 주변 위치·방향 후보와의 구별 가능성을 확인하고, ‘다시 정상 점수가 됨’과 ‘올바른 위치가 확인됨’을 분리해야 한다. 그 후 같은 실패 사례와 새로운 실측 기준 구간으로 다시 검증해야 한다. 이번 실험에서는 원래 실행 설정을 유지했다.</p></section>
<section><h2>입력과 검증 경계</h2><div class="scroll"><table><thead><tr><th>bag</th><th>분할</th><th>분</th><th>관측</th><th>창</th><th>기록 점수 대조</th></tr></thead><tbody>{bags}</tbody></table></div><p>현재 기록 중인 bag은 SQLite 읽기 스냅샷을 만들고 09:27:30까지로 잘랐다. 각 bag의 기록된 지도를 사용하고 평가 시각까지 수신된 TF만 사용했다. /scan_raw의 0.5–8m 끝점을 최대 240개로 처리했다. 스캔 저장 공백, pose 초기화와 odom 이동이 있는 창은 제외했다. 정지 선택은 wheel odom 근거여서 실제 정지를 독립 측정한 것은 아니다.</p><p>정책 간 비교는 같은 창을 사용하며, 주입한 오차를 전체 pose 스트림에 유지해 미래의 원래 AMCL pose가 오차를 자동으로 지우지 못하게 했다. 후보 위치는 후속 스캔에 기록된 odom 상대 변화만 적용해 검증했다. 처리 시간과 관측 대기 시간은 실제 복구 지연을 뜻하지 않는다.</p>
<p><a href="protocol.json">사전 고정 조건</a> · <a href="summary.json">집계</a> · <a href="comparison.csv">전체 CSV</a> · <a href="results.json">후보와 판정 원본</a> · <a href="inputs.json">입력·해시</a> · <a href="comparison.png">그래프 PNG</a></p></section></main></html>'''
    (out/'index.html').write_text(doc)
    print(json.dumps({'all':totals['all'],'holdout':totals['holdout']},ensure_ascii=False))


if __name__=='__main__':main()
