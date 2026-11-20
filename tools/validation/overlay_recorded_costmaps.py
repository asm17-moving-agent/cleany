"""여러 실물 bag의 장애물 셀을 기록 시각 TF로 지도에 누적합니다.

LiDAR와 Depth가 합쳐진 costmap의 100 셀만 사용하고 inflation은 제외합니다.
수신 및 TF 시각 조건을 만족하지 않는 프레임은 외삽하지 않고 제외합니다.
"""

from __future__ import annotations

import argparse
from collections import Counter, deque
import heapq
import json
from pathlib import Path
import sqlite3

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties
from matplotlib.patches import Patch
import numpy as np
from PIL import Image
from rosbags.typesys import Stores, get_typestore
import yaml

STORE = get_typestore(Stores.ROS2_HUMBLE)


def stamp(message) -> float:
    return message.header.stamp.sec + message.header.stamp.nanosec * 1e-9


def planar(pose) -> np.ndarray:
    """Quaternion의 yaw와 평면 병진을 3×3 동차 행렬로 변환합니다."""
    p, q = (pose.translation, pose.rotation) if hasattr(pose, "translation") else (pose.position, pose.orientation)
    yaw = np.arctan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))
    c, s = np.cos(yaw), np.sin(yaw)
    return np.array([[c, -s, p.x], [s, c, p.y], [0, 0, 1]])


def records(path: Path):
    with sqlite3.connect(path.as_uri() + "?mode=ro", uri=True) as db:
        wanted = {"/tf", "/tf_static", "/odom", "/map", "/local_costmap/costmap"}
        topics = {i: (n, t) for i, n, t in db.execute("SELECT id,name,type FROM topics") if n in wanted}
        if not topics:
            return
        placeholders = ",".join("?" for _ in topics)
        query = f"SELECT timestamp,topic_id,data FROM messages WHERE topic_id IN ({placeholders}) ORDER BY timestamp"
        for ns, ident, raw in db.execute(query, tuple(topics)):
            name, typ = topics[ident]
            yield ns, name, typ, raw


def at(history: deque, t: float, max_age: float):
    # 수신 시각 순으로 처리하므로 아직 도착하지 않은 TF는 후보에 포함되지 않습니다.
    candidates = [(ts, value) for ts, value in history if 0 <= t-ts <= max_age]
    return max(candidates, key=lambda pair: pair[0]) if candidates else None


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", type=Path, action="append", required=True)
    parser.add_argument("--map-yaml", type=Path, required=True)
    parser.add_argument("--mask-yaml", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--font", type=Path, required=True)
    parser.add_argument("--tf-max-age", type=float, default=1.5)
    parser.add_argument("--moving-linear", type=float, default=.02)
    parser.add_argument("--moving-angular", type=float, default=.03)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    config = yaml.safe_load(args.mask_yaml.read_text())
    source = yaml.safe_load(args.map_yaml.read_text())
    assert config["resolution"] == source["resolution"] and config["origin"] == source["origin"]
    assert config["origin"][2] == 0 and config["negate"] == 0 and config["mode"] == "trinary"
    raw = np.array(Image.open(args.map_yaml.parent / source["image"]).convert("L"))
    mask = np.array(Image.open(args.mask_yaml.parent / config["image"]).convert("L"))
    assert raw.shape == mask.shape
    blocked = np.flipud((255.0-mask)/255 > config["occupied_thresh"])
    h, w = blocked.shape
    ox, oy, _ = config["origin"]
    resolution = config["resolution"]
    total = {key: np.zeros((h,w), dtype=np.uint32) for key in ("hits", "coverage", "moving_hits", "moving_coverage")}
    reports = []
    tracks = []
    for bag in args.bag:
        dbs = sorted(bag.rglob("*.db3"))
        assert dbs
        histories = {"map_odom": deque(maxlen=300), "odom_base": deque(maxlen=300), "motion": deque(maxlen=300)}
        counts = Counter()
        first, last = None, None
        ages = []
        local = {key: np.zeros((h,w), dtype=np.uint32) for key in total}
        map_checked = False
        for ns, name, typ, binary in heapq.merge(*(records(p) for p in dbs), key=lambda row: row[0]):
            msg = STORE.deserialize_cdr(binary, typ)
            if name in ("/tf", "/tf_static"):
                for tf in msg.transforms:
                    pair = (tf.header.frame_id.lstrip("/"), tf.child_frame_id.lstrip("/"))
                    key = {("map","odom"): "map_odom", ("odom","base_link"): "odom_base"}.get(pair)
                    if key:
                        histories[key].append((stamp(tf), planar(tf.transform)))
                continue
            if name == "/odom":
                v = msg.twist.twist
                histories["motion"].append((stamp(msg), bool(np.hypot(v.linear.x,v.linear.y)>args.moving_linear or abs(v.angular.z)>args.moving_angular)))
                continue
            if name == "/map":
                info = msg.info
                assert (info.height,info.width) == (h,w)
                assert np.allclose([info.resolution, info.origin.position.x, info.origin.position.y], [resolution,ox,oy])
                assert np.allclose(planar(info.origin), [[1,0,ox],[0,1,oy],[0,0,1]])
                occupancy = (255.0-raw)/255
                expected = np.where(occupancy>source["occupied_thresh"],100,np.where(occupancy<source["free_thresh"],0,-1))
                mismatch = int(np.sum(msg.data.reshape(h,w) != np.flipud(expected)))
                counts["recorded_map_pixel_mismatches"] = mismatch
                assert mismatch == 0, "원본 지도와 기록 지도의 점유 값 불일치"
                map_checked = True
                continue
            t = stamp(msg)
            counts["costmap_frames"] += 1
            first = t if first is None else first
            last = t
            frame = msg.header.frame_id.lstrip("/")
            if frame == "map":
                tf = np.eye(3); age = 0.0
            elif frame == "odom":
                match = at(histories["map_odom"],t,args.tf_max_age)
                if match is None:
                    counts["skipped_no_timely_map_odom"] += 1
                    continue
                ts, tf = match; age = t-ts
            else:
                counts["skipped_unknown_frame"] += 1
                continue
            ages.append(age)
            motion = at(histories["motion"],t,.5)
            moving = motion is not None and motion[1]
            counts["used_frames"] += 1
            counts["moving_frames"] += int(moving)
            counts["motion_unknown_frames"] += int(motion is None)
            cells = msg.data.reshape(msg.info.height,msg.info.width)
            rr, cc = np.nonzero(cells >= 0)
            xy = np.column_stack(((cc+.5)*msg.info.resolution,(rr+.5)*msg.info.resolution,np.ones(len(rr))))
            world = xy @ (tf @ planar(msg.info.origin)).T
            columns = np.floor((world[:,0]-ox)/resolution).astype(int)
            rows = np.floor((world[:,1]-oy)/resolution).astype(int)
            valid = (rows>=0)&(rows<h)&(columns>=0)&(columns<w)
            occupied = cells[rr,cc] == 100
            counts["occupied_cell_samples_outside_map"] += int(np.sum(occupied & ~valid))
            # 회전으로 같은 지도 셀에 겹친 관측은 프레임당 한 번만 집계합니다.
            for key, select in (("coverage",valid),("hits",valid & occupied)):
                indices = np.unique(rows[select]*w+columns[select])
                local[key].ravel()[indices] += 1
                if moving:
                    local["moving_"+key].ravel()[indices] += 1
            body = at(histories["odom_base"],t,args.tf_max_age)
            if moving and body is not None:
                position = tf @ body[1]
                tracks.append([float(t),float(position[0,2]),float(position[1,2])])
            if counts["costmap_frames"] % 10000 == 0:
                print(bag.name, dict(counts), flush=True)
        assert map_checked, "기록된 원본 지도 확인 불가"
        assert all(np.all(local[a]<=local[b]) for a,b in (("hits","coverage"),("moving_hits","moving_coverage")))
        for key in total:
            total[key] += local[key]
        np.savez_compressed(args.output/f"{bag.name}.npz",**local)
        report=dict(bag=str(bag), databases=[dict(path=str(p),bytes=p.stat().st_size) for p in dbs],counts=dict(counts),
                    first_stamp=first,last_stamp=last,tf_age_p50_p95_max=np.percentile(ages,[50,95,100]).tolist() if ages else [])
        reports.append(report)
        print(json.dumps(report,ensure_ascii=False),flush=True)
    np.savez_compressed(args.output/"all-costmaps.npz",**total,track=np.array(tracks))
    hit = total["hits"]
    moving = total["moving_hits"]
    stats=dict(unique_hit_cells=int((hit>0).sum()),unique_moving_hit_cells=int((moving>0).sum()),
               outside_mask_hit_cells=int(((hit>0)&~blocked).sum()),
               outside_mask_moving_hit_cells=int(((moving>0)&~blocked).sum()))
    font=FontProperties(fname=str(args.font), weight="medium")
    extent=[ox,ox+w*resolution,oy,oy+h*resolution]
    xx=ox+(np.arange(w)+.5)*resolution; yy=oy+(np.arange(h)+.5)*resolution
    n_used=sum(x["counts"].get("used_frames",0) for x in reports)
    n_all=sum(x["counts"]["costmap_frames"] for x in reports)
    n_moving=sum(x["counts"].get("moving_frames",0) for x in reports)
    for filename,zoom in (("all-costmaps-overlay.png",False),("tables-costmaps-overlay.png",True)):
        fig,axes=plt.subplots(1,2,figsize=(14,10),dpi=150)
        fig.patch.set_facecolor("#f3f7f5")
        for ax in axes:
            ax.imshow(np.flipud(raw),origin="lower",extent=extent,cmap="gray",vmin=0,vmax=255,alpha=.35,interpolation="nearest")
            rgba=np.zeros((h,w,4));rgba[blocked]=(.95,.2,.2,.20)
            ax.imshow(rgba,origin="lower",extent=extent,interpolation="nearest")
            ax.contour(xx,yy,blocked.astype(float),levels=[.5],colors=["#dd3434"],linewidths=1)
            ax.set_axis_off();ax.set_aspect("equal")
            if zoom:ax.set(xlim=(-8,6),ylim=(-22,-7))
        rgba=np.zeros((h,w,4));rgba[hit>0]=(.0,.45,.5,.8)
        axes[0].imshow(rgba,origin="lower",extent=extent,interpolation="nearest")
        rgba=np.zeros((h,w,4));rgba[moving>0]=(.3,.7,.7,.5);rgba[moving>=3]=(.0,.3,.4,.95)
        axes[1].imshow(rgba,origin="lower",extent=extent,interpolation="nearest")
        axes[0].set_title("전체 기록: 한 번이라도 잡힌 장애물",fontproperties=font,fontsize=17,pad=15)
        axes[1].set_title("이동 중 기록: 반복 관측 강조",fontproperties=font,fontsize=17,pad=15)
        fig.suptitle("Keepout + 지도 좌표가 확인된 모든 장애물 기록",fontproperties=font,fontsize=22,y=.97)
        fig.text(.5,.92,f"11월 8일 {n_used:,}장 누적 / 이동 중 {n_moving:,}장 · 11월 9–10일은 map → odom 기록 부재로 제외",ha="center",fontproperties=font,fontsize=12)
        fig.subplots_adjust(left=.025,right=.975,top=.86,bottom=.16,wspace=.10)
        fig.legend(handles=[Patch(fc="#ef7777",label="빨강: 수동 Keepout"),Patch(fc="#007380",label="청록: 기록된 장애물"),
                            Patch(fc="#004d66",label="오른쪽 진한색: 이동 중 3장 이상")],
                   prop=font,loc="lower center",bbox_to_anchor=(.5,.09),ncol=3,frameon=False)
        fig.text(.5,.035,"LiDAR+Depth 통합 costmap의 장애물 셀(100)만 사용 / inflation 제외 / 순간 장애물과 위치 추정 오차도 누적\n"
                 "원본 마스크 변경 없음 · TF 없는 프레임 제외 · 실제 관측 표면과 고정 책상 영역은 구분 필요",ha="center",fontproperties=font,fontsize=10,color="#475569")
        fig.savefig(args.output/filename);plt.close(fig)
    result=dict(recordings=reports,summary=stats,arguments={k:str(v) for k,v in vars(args).items()},
                method="All costmaps, lethal=100 only, grid cell centers, receive-order streaming; map->odom TF at/before grid header <=1.5s old; motion by fresh odom",
                limitations=["Combined LiDAR/depth, no sensor attribution", "Stationary dwell biases counts; moving-only counts also supplied",
                             "Moving objects and localization errors accumulate; not a static obstacle truth map", "No mask edits or runtime changes"])
    (args.output/"manifest.json").write_text(json.dumps(result,ensure_ascii=False,indent=2)+"\n")
    print(json.dumps(stats),flush=True)


if __name__=="__main__":
    main()
