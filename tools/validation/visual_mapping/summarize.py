"""Build the Korean review page and an integrity manifest for the offline results."""

import argparse
import zipfile
import hashlib
import json
import sqlite3
from pathlib import Path

import numpy as np
import yaml
from PIL import Image


def verify(root):
    out = root / "results"
    metadata = yaml.safe_load((root / "input/source-metadata.yaml").read_text())[
        "rosbag2_bagfile_information"
    ]
    # This illustrated evaluation uses hand-selected wall regions and a confirmed fixed head.
    if metadata["starting_time"]["nanoseconds_since_epoch"] != 1795294569769105273:
        raise ValueError("The illustrated wall report requires the 2026-11-22 05:56 Floor 19 recording")
    fs = sorted((root / "input/manifest").glob("*.json"))
    assert len(fs) == len(metadata["files"])
    manifests = [json.loads(f.read_text()) for f in fs]
    topics = {
        entry["topic_metadata"]["name"]: entry["message_count"]
        for entry in metadata["topics_with_message_count"]
    }
    prefixes = {
        "rgb": "/camera/sdk/color/image_raw/compressed",
        "depth": "/camera/aligned_depth_to_color/image_raw",
        "left": "/camera/infra1/image_rect_raw",
        "right": "/camera/infra2/image_rect_raw",
    }
    counts = {}
    for kind, topic in prefixes.items():
        actual = sum(m["counts"].get(kind, 0) for m in manifests)
        if topics[topic] != actual:
            raise ValueError(f"Source count mismatch {topic}: {actual} / {topics[topic]}")
        counts[kind] = actual
    pair_summary = {}
    for mode, a, b in (
        ("rgbd", "rgb", "depth"),
        ("stereo", "left", "right"),
        ("stereo-filtered", "left", "right"),
        ("stereo-depth", "left", "right"),
    ):
        names = [{p.stem for p in (root / "input" / k).iterdir()} for k in (a, b)]
        tracking = np.loadtxt(out / mode / "tracking.csv", delimiter=",", skiprows=1, dtype="i8")
        assert names[0] == names[1] == {str(v) for v in tracking[:, 1]}
        assert np.all(np.diff(tracking[:, 1]) > 0)
        pair_summary[mode] = len(names[0])
        with sqlite3.connect((out / mode / "map.db").as_uri() + "?mode=ro", uri=True) as c:
            if c.execute("PRAGMA quick_check").fetchone()[0] != "ok":
                raise ValueError(f"Database quick_check failed {mode}")
        tum = np.atleast_2d(np.loadtxt(out / mode / "trajectory.tum"))
        assert np.all(np.isfinite(tum)) and np.all(np.diff(tum[:, 0]) > 0)
        assert np.max(abs(np.linalg.norm(tum[:, 4:], axis=1) - 1)) < 1e-5
    for sub in ("fused", "lidar"):
        p = out / "fusion" / sub
        m = yaml.safe_load((p / "map.yaml").read_text())
        im = np.asarray(Image.open(p / m["image"]))
        assert m["resolution"] == 0.05 and set(np.unique(im)) == {0, 205, 254}
    for p in (out / "rgbd").glob("*.ply"):
        with p.open("rb") as f:
            head = []
            while True:
                line = f.readline()
                if not line:
                    raise ValueError("Truncated PLY header")
                head.append(line)
                if line == b"end_header\n":
                    break
            n = int(next(x.split()[-1] for x in head if x.startswith(b"element vertex ")))
            if p.stat().st_size - f.tell() != 15 * n or n < 100:
                raise ValueError(f"Invalid vertex records {p}")
    meta = dict(
        source_messages=metadata["message_count"],
        source_counts=counts,
        exact_timestamp_pairs_processed=pair_summary,
        bag_splits=len(fs),
        checks=[
            "paired timestamps and per-sensor counts",
            "database quick_check",
            "strictly increasing valid TUM transforms",
            "Nav2 trinary PGM/YAML",
            "PLY payload length",
        ],
    )
    (out / "verification.json").write_text(json.dumps(meta, indent=2))
    return meta


def page(root):
    out = root / "results"
    check = verify(root)
    v = json.loads((out / "visual-metrics.json").read_text())
    f = json.loads((out / "fusion/metrics.json").read_text())
    links = json.loads((out / "fusion/visit-constraints.json").read_text())
    passes = json.loads((out / "fusion/illustrated-passes.json").read_text())
    landmarks = json.loads((out / "fusion/wall-landmarks.json").read_text())
    primary = v["primary_stereo"]
    sa, sb = v["rgbd"], v[primary]
    names = {
        "stereo": "원본 IR",
        "stereo-filtered": "투사점 억제 IR",
        "stereo-depth": "스테레오 시차 깊이와 저주파 IR",
    }

    def median_text(x):
        return f"{x:.2f} m" if x is not None else "비교 구간 없음"

    variants = "".join(
        f"<tr><td>{names[m]}</td><td>{v[m]['moving_windows']['available']} / {v[m]['moving_windows']['eligible']}</td><td>{v[m]['moving_windows']['within_25cm_and_7deg']}</td><td>{median_text(v[m]['moving_windows']['translation_p50_m'])}</td></tr>"
        for m in names
    )
    ma, mb = f["test"]["lidar"], f["test"]["fused"]

    def number(x):
        return f"{x:,}"

    metrics = "".join(
        f"<tr><td>{label}</td><td>{a}</td><td>{b}</td></tr>"
        for label, a, b in [
            (
                "추적된 영상 쌍",
                f"{sa['tracked']:,} / {sa['total_frames']:,}",
                f"{sb['tracked']:,} / {sb['total_frames']:,}",
            ),
            ("추적 성공률", f"{sa['tracking_pct']}%", f"{sb['tracking_pct']}%"),
            ("VO 구간 수", sa["odometry_segments"], sb["odometry_segments"]),
            ("최대 연결 성분에 위치가 있는 키프레임", max(sa["component_nodes"]), max(sb["component_nodes"])),
            ("40초 이상 떨어진 시각 연결", sa["long_visual_links"], sb["long_visual_links"]),
            (
                "LiDAR 기준 궤적과의 중앙 거리",
                f"{sa['relative_to_lidar']['p50_m']:.2f} m",
                f"{sb['relative_to_lidar']['p50_m']:.2f} m",
            ),
        ]
    )
    walls = f["walls"]["scores"]
    rows = "".join(
        f"<tr><td>{key}</td><td>{w['pairs']:,}</td><td>{w['median_cm']:.1f} cm</td><td>{walls['fused'][key]['median_cm']:.1f} cm</td><td>{len(landmarks.get(key,[]))}</td></tr>"
        for key, w in walls["lidar"].items()
    )
    ed = passes["match"]
    landmark = passes.get("landmarks")
    depth = json.loads((out / "fusion/depth-walls.json").read_text())
    depth_rows = "".join(
        f"<tr><td>{key}</td><td>{m['pair_count']}</td>"
        + (
            "".join(
                f"<td>{m['normal_separation_cm'][kind]:.1f} cm</td>" for kind in ("field", "lidar", "fused")
            )
            if m["pair_count"]
            else "<td colspan=3>왕복 관측 불충분</td>"
        )
        + "</tr>"
        for key, m in depth.items()
    )
    depth_img = (
        '<img src="fusion/depth-wall-passes.jpg" alt="서로 다른 시각에 카메라가 관측한 벽 표면">'
        if (out / "fusion/depth-wall-passes.jpg").is_file()
        else ""
    )

    if landmark:
        gap = landmark["normal_gap_cm"]
        image_note = (
            f"벽 {passes['wall']} 주변에서 두 영상의 RGB-D 특징점 {landmark['matches_on_wall']}개가 지정한 벽 구간에 투영됐다. "
            f"이 점들의 벽 법선 방향 왕복 위치 차이 중앙값은 현장 지도 {gap['field']:.1f} cm, "
            f"LiDAR 재처리 {gap['lidar']:.1f} cm, 결합 지도 {gap['fused']:.1f} cm다."
        )
    else:
        image_note = f"벽 {passes['wall']} 근처에서 확인한 동일 장소 영상이다. 표시된 특징점이 벽면 위에 있다는 직접 근거는 부족하다."
    votes = json.loads((out / "fusion/used-edges.json").read_text())
    bag_start = yaml.safe_load((root / "input/source-metadata.yaml").read_text())[
        "rosbag2_bagfile_information"
    ]["starting_time"]["nanoseconds_since_epoch"]

    def mmss(ns):
        seconds = round((ns - bag_start) / 1e9)
        return f"{seconds//60}분 {seconds%60:02d}초"

    vote_times = " / ".join(
        f"{mmss(min(e[key] for e in votes))}–{mmss(max(e[key] for e in votes))}" for key in ("a", "b")
    )
    example_a = next(e for e in passes["examples"] if e["wall"] == "A")
    example_text = " → ".join(f"{100*x:.1f}" for x in example_a["median_separation_m"])
    with zipfile.ZipFile(out / "fusion/floor19-nav2.zip", "w", compression=zipfile.ZIP_DEFLATED) as bundle:
        for name in ("map.yaml", "map.pgm"):
            bundle.write(out / "fusion/fused" / name, arcname=name)
    provenance = dict(
        source=root.name,
        bag_name=Path(json.loads((root / "input/manifest/00.json").read_text())["source"]).parent.name,
        source_metadata_sha256=hashlib.sha256((root / "input/source-metadata.yaml").read_bytes()).hexdigest(),
        compression_format=yaml.safe_load((root / "input/source-metadata.yaml").read_text())[
            "rosbag2_bagfile_information"
        ]["compression_format"],
        environment="Fedora laptop, Ubuntu 22.04 Distrobox, ROS 2 Humble, RTAB-Map core 0.23.14, OpenCV 4.5.4",
        fixed_head="operator confirmation; CAD runtime transform used as nominal extrinsic",
        sampling="exact ROS header timestamp pairs, up to one pair per 200 ms bin per recorder split",
        baseline="holdout-rotation; 6 m search, global wheel rotation weight 10000; every eighth scan withheld",
        motion_commands_sent=0,
    )
    (out / "provenance.json").write_text(json.dumps(provenance, indent=2))

    p = f"""<!doctype html><html lang="ko"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Cleany 19층 · 카메라와 LiDAR 재처리</title><style>
*{{box-sizing:border-box}}body{{margin:0;color:#1d302e;background:#f6f6f0;font:16px/1.65 system-ui,sans-serif}}main{{max-width:1080px;margin:auto;padding:44px 24px 72px}}h1{{font-size:clamp(30px,5vw,48px);line-height:1.15;margin:0 0 12px}}h2{{margin:48px 0 8px;line-height:1.25}}p{{max-width:88ch}}img{{max-width:100%;display:block;margin:16px 0;background:white}}table{{border-collapse:collapse;width:100%;background:#fff}}td,th{{padding:8px 10px;text-align:left;border-bottom:1px solid #d7ded8}}a{{color:#006b67;font-weight:600}}small{{color:#4a605e}}.links{{display:flex;gap:14px;flex-wrap:wrap}}.scroll{{overflow:auto}}</style><main>
<h1>19층 복도 재구성</h1><p>2026년 11월 22일 05:56 주행 · 21분 6초 · 고정 헤드 · 22개 bag 분할</p>
<h2>RGB-D 컬러 3D 지도</h2><p>결합 궤적에 RGB-D {number(v['global_cloud']['frames'])}장을 투영했다. 0.35–5 m에서 관측한 표면을 4 cm 복셀로 모으고 두 프레임 이상 관측된 {number(v['global_cloud']['vertices'])}점을 남겼다. 센서가 보지 못한 공간은 비어 있다.</p>
<div class="links"><a href="rgbd/viewer.html">3D 지도 회전·확대</a><a href="rgbd/floor19-color.ply">전체 컬러 PLY</a><a href="rgbd/visual-component.ply">RGB-D 단독 PLY</a></div><a href="rgbd/viewer.html"><img src="rgbd/overview.png" alt="19층 RGB-D 컬러 점군"></a>
<h2>카메라 단독 궤적</h2><p>RTAB-Map에서 RGB-D와 좌우 IR 스테레오를 독립 처리했다. 프레임 시각이 정확히 같은 쌍을 초당 최대 5쌍 선별했다. 깊이는 원본 mm 값을 보존해 2픽셀마다 표본 추출했고 IR은 원래 영상 크기의 JPEG 품질 93으로 전송했다. 스테레오 기준선은 기록된 투영 행렬에서 계산한 5.01 cm다.</p>
<div class="scroll"><table><thead><tr><th>측정 항목</th><th>RGB-D</th><th>{names[primary]}</th></tr></thead><tbody>{metrics}</tbody></table></div>
<p>IR 원본에는 적외선 프로젝터 점무늬가 강하게 찍혔다. 원본 스테레오의 최대 연결 성분에서 추정한 수평 이동거리는 {v['stereo']['relative_to_lidar']['visual_path_m']:.1f} m, 같은 시각 구간의 LiDAR 이동거리는 {v['stereo']['relative_to_lidar']['reference_path_same_intervals_m']:.1f} m다. 밝은 점 제거 조건과 원본 좌우 시차로 깊이를 구해 저주파 IR로 추적하는 조건도 재실행했다. 대표 스테레오의 같은 비교는 {sb['relative_to_lidar']['visual_path_m']:.1f} m와 {sb['relative_to_lidar']['reference_path_same_intervals_m']:.1f} m다. <a href="stereo/trajectory.tum">IR 원본 결과</a></p>
<p>5초간 LiDAR 변위가 40 cm 이상인 구간에서 같은 시각의 이동량을 비교했다. 유효 구간 분모에는 추적 유실도 포함했다. 25 cm·7도 이내로 일치한 구간 수가 가장 많은 조건을 대표 결과로 표시했다. LiDAR는 같은 주행에서 얻은 비교 기준이다. 세 조건 모두 입력은 좌우 IR 영상과 고정된 장착 정보다.</p><div class="scroll"><table><tr><th>IR 처리</th><th>비교 가능 / 이동 구간</th><th>25 cm·7도 이내</th><th>위치 오차 중앙값</th></tr>{variants}</table></div><img src="trajectories.png" alt="시각 궤적과 LiDAR 궤적 비교"><p>그림은 각 최대 연결 성분을 LiDAR 궤적과 XY 평면에서 회전·이동해 겹쳤다. 실제 자세 기준기는 사용하지 않았다. 본체와 카메라 간 변환에는 주행 중 고정된 헤드와 기록된 CAD 모델 자세를 사용했다.</p>
<div class="links"><a href="{primary}/trajectory.tum">스테레오 원좌표 궤적</a><a href="{primary}/map.db">스테레오 RTAB-Map DB</a><a href="rgbd/trajectory.tum">RGB-D 원좌표 궤적</a><a href="rgbd/map.db">RGB-D RTAB-Map DB</a></div>
<h2>왕복 구간의 이중벽</h2><p>{image_note} 아래 첫 그림은 {abs(ed['b']-ed['a'])/1e9:.0f}초 간격으로 촬영한 실제 영상이다. 이 쌍의 RGB-D 인라이어는 {ed['inliers']}개, 양방향 LiDAR 일치 비율은 {ed['scan_overlap']:.0%}다.</p><img src="fusion/revisit.jpg" alt="왕복 RGB-D 대응점">
<p>벽 구간은 현장 PGM에서 고정했다. 같은 벽의 같은 쪽을 주행 거리 5 m, 시각 45초 이상 떨어져 다시 본 스캔끼리 비교했다. 표는 벽을 따라 75 cm 구간별로 구한 법선 방향 차이의 중앙값이다. 마지막 열은 각 벽에 실제 특징점이 세 개 이상 투영된 영상 쌍 수다.</p>
<div class="scroll"><table><tr><th>벽</th><th>스캔 구간 쌍</th><th>LiDAR</th><th>결합</th><th>벽 영상 쌍</th></tr>{rows}</table></div>
<p>중앙 복도 A에서 같은 두 스캔 구간의 간격은 현장 → LiDAR 재처리 → 결합 순으로 {example_text} cm였다. 다른 구간까지 포함한 대표값은 위 표에서 확인할 수 있다.</p><img src="fusion/wall-passes.png" alt="동일 LiDAR 관측의 세 지도상 벽 법선 위치 비교">
<h3>깊이 카메라로 직접 본 벽</h3><p>색상 특징이 부족한 구간도 확인하기 위해 RGB-D에서 수직 표면을 골랐다. 같은 벽의 같은 쪽·같은 폭 75 cm 구간으로 묶었다. 표면의 왕복 간격은 동일 물체 특징점의 거리를 보장하지 않는다. 초록 점은 해당 구간에 실제로 들어온 깊이 관측이다.</p>{depth_img}<div class="scroll"><table><tr><th>벽</th><th>왕복 구간 쌍</th><th>현장</th><th>LiDAR 재처리</th><th>결합</th></tr>{depth_rows}</table></div>
<h2>카메라와 LiDAR를 결합한 2D 지도</h2><p>LiDAR 기준 궤적으로 제안한 재방문 {number(links['proposed'])}쌍에서 RGB-D 기하 검증 {number(links['visual_verified'])}쌍, LiDAR 일치 검증 {number(links['scan_verified'])}쌍을 얻었다. 같은 시각대의 중복을 줄여 최종 {f['accepted_edges']}개 제약을 강건 자세 그래프 최적화에 사용했다. 채택된 제약의 시각·위치는 궤적과 재방문 대응점 파일에 보관했다. 기준 궤적은 같은 주행에서 별도로 선택한 Cartographer 설정(재방문 거리 6 m, 휠 회전 가중치 10,000)으로 만들었다. 실제 채택 시각은 1차 통과와 재방문 {vote_times}에 집중됐다. 최신 결과에서 기준 궤적의 최대 위치 변화는 {100*f['solver']['maximum_shift_m']:.1f} cm였다. 출력 해상도는 5 cm/픽셀이다.</p>
<div class="links"><a href="fusion/floor19-nav2.zip">Nav2 지도 ZIP</a><a href="fusion/fused/map.pgm">결합 지도 PGM</a><a href="fusion/fused/map.yaml">Nav2 YAML</a><a href="fusion/trajectory.csv">결합 궤적</a><a href="fusion/comparison.png">지도 비교 확대</a></div><img src="fusion/comparison.png" alt="동일 렌더러로 생성한 LiDAR 지도 및 결합 지도">
<p>8번째마다 보류한 LiDAR 스캔 중 공통 시각 구간 {number(f['heldout_scans'])}개, {number(mb['beams'])}개 유효 빔을 두 지도에서 똑같이 평가했다. 인접 프레임이므로 이 평가는 같은 주행 내부의 일관성을 측정한다. 10 cm 이내 점유 셀에 도달한 비율은 LiDAR {ma['within_10cm_pct']:.2f}%, 결합 {mb['within_10cm_pct']:.2f}%, 거리 90백분위는 각각 {ma['p90_cm']:.2f} cm와 {mb['p90_cm']:.2f} cm다.</p>
<p>Nav2 실물 주행과 도면 실측 일치도는 아직 검증하지 않았다. 이번 지도는 오프라인 후보로 저장했다.</p>
<div class="links"><a href="../reproduction/README.md">재현 자료</a><a href="fusion/metrics.json">지도 평가 수치</a><a href="fusion/used-edges.json">채택한 재방문 제약</a><a href="fusion/wall-landmarks.json">벽별 영상 근거</a><a href="fusion/depth-walls.json">깊이 표면 왕복 비교</a><a href="visual-metrics.json">카메라 처리 수치</a><a href="verification.json">원본 대조 및 파일 검증</a><a href="provenance.json">입력과 실행 조건</a></div>
</main></html>"""
    (out / "index.html").write_text(p)
    return check


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("root", type=Path)
    print(page(p.parse_args().root))
