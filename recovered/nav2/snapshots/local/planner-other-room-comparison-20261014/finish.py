from pathlib import Path
import json
O=Path(__file__).resolve().parent;p=O/'results.json';d=json.loads(p.read_text())
for a in d['results']:
 a['failure_reason']=('maximum iterations (1,000,000)' if a['planner'] in ['SmacLattice','SmacHybrid'] and a['status']==6 else ('no_valid_lattice_path;5s budget' if a['planner']=='StraightFirst' and a['status']==6 else None))
p.write_text(json.dumps(d,indent=2))
p=O/'render.py';s=p.read_text().replace("elif r['status']==6:detail=f\"NO PATH RETURNED  |  ABORTED  |  {r['wall_s']:.2f} s\"", "elif r['status']==6:detail=f\"NO PATH  |  {r['wall_s']:.2f} s  |  \"+('iteration cap 1M' if r['planner'] in ['SmacLattice','SmacHybrid'] else '5s budget')")
p.write_text(s)
(O/'README.md').write_text('''# 다른방 목표 플래너 경로 비교

2행3열PNG는StraightFirstPlanner/NavFn(Dijkstra)/Smac2D/Theta*/SmacLattice(Omni)/SmacHybrid(Reeds-Shepp)의실제ComputePathToPose결과다. 같은5cm정적지도, 시작[-1.865,-4.705,1.5708], 목표[-16.775,1.775,1.570796], 목적지M14의자제거, polygon/padding.02/inflation.8입력. 각planner독립fresh서버1회호출. 2D계열은입력polygon이있어도차체방향에따른polygon충돌을검색에쓰지않는다. 주행은수행하지않았다.

NavFn/Smac2D/Theta*는경로반환. StraightFirst는5초예산의no_valid_lattice_path. SmacLattice/Hybrid는native최대100만iteration에도달해중단. 경로없음을물리통과불가능증명으로해석하지않는다. 정적경로반환은dynamicdepth/memory/BT/TF와실제차체주행성공을보장하지않는다. 5초검색시간은Smac/custom설정이며NavFn/Theta는native종료조건. 시간은단일호출관측이다.

이전8goal캠페인2.5cm와달리표준5cmOmni primitive에맞춘동일5cm지도를모두사용. 지도는2.5cm원본2x2점유cell을보수적으로합친파생본. 기존커스텀80초예산과도달라원래실패의동일조건재현은아니다.

첫multi-plugin서버는Lattice호출에서segfault종료. multi-plugin-partial.json/run.py/planner.log에인프라실패보존,최종6결과에는미사용. run_isolated.py/개별YAML/log/results.json이최종근거. PNG는render.py생성. 실제로봇/Gazebo주행/gitpush없음.
''')
