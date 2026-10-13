import { useEffect, useRef, useState } from "react";
import { Button } from "./ui/button";
import { Badge } from "./ui/badge";
import { type Data, type Row, time } from "@/lib/monitor";
import "./slam-view.css";

const labels: Record<string, string> = {
  submaps: "서브맵",
  boundaries: "경계·번호",
  scan: "현재 스캔",
  matched_points: "매칭 점군",
  trajectory: "궤적",
  constraints: "제약선",
};
const colors: Record<string, string> = {
  scan: "#67dfbd",
  matched_points: "#f7bb64",
  trajectory: "#8bbaff",
  constraints: "#b7a1fa",
};
const list = (value: unknown): Data[] => (Array.isArray(value) ? value : []);
const point = (value: any): value is number[] =>
  Array.isArray(value) && value.length >= 2 && value.every(Number.isFinite);
const compose = (a: number[], b: number[]) => [
  a[0] + Math.cos(a[2]) * b[0] - Math.sin(a[2]) * b[1],
  a[1] + Math.sin(a[2]) * b[0] + Math.cos(a[2]) * b[1],
  a[2] + b[2],
];
// Cartographer texture columns point along -Y and rows along -X.
export const texturePoint = (pose: number[], x: number, y: number, r: number) =>
  compose(pose, [-y * r, -x * r, 0]);

export function SlamView({
  rows,
  config,
  timeAt,
  replay,
  sessionId,
  seek,
}: {
  rows: Row[];
  config: Data;
  timeAt: number;
  replay: boolean;
  sessionId: string;
  seek: (at: number) => void;
}) {
  const atRows = rows.filter((r) => r.received_at <= timeAt + 0.001);
  const stream = atRows.find((r) => r.kind === "slam" && r.key === "submaps");
  const epoch = stream?.data.epoch;
  const submaps = stream?.data.valid ? list(stream.data.submaps) : [];
  const textures = atRows.filter(
    (r) => r.kind === "slam_texture" && r.data.epoch === epoch,
  );
  const texture = (s: Data) =>
    textures.find((r) => r.key === s.id && r.data.version <= s.version)?.data;
  const geometry = (key: string) =>
    atRows.find(
      (r) => r.kind === "slam" && r.key === key && r.data.epoch === epoch,
    );
  const scan = atRows.find((r) => r.kind === "navigation" && r.key === "scan");
  const status = atRows.find(
    (r) => r.kind === "slam" && r.key === "status" && r.data.epoch === epoch,
  )?.data;
  const stale = !stream || timeAt - stream.received_at > 3;
  const [enabled, setEnabled] = useState<Record<string, boolean>>(() =>
    Object.fromEntries(Object.keys(labels).map((k) => [k, true])),
  );
  const [selected, setSelected] = useState("");
  const selectedMap = submaps.find((s) => s.id === selected);
  const [view, setView] = useState<{
    x: number;
    y: number;
    span: number;
  } | null>(null);
  const canvas = useRef<HTMLCanvasElement>(null);
  const images = useRef(
    new Map<string, { src: string; image: HTMLImageElement }>(),
  );
  const [imageRevision, setImageRevision] = useState(0);
  const [size, setSize] = useState([1000, 600]);
  const drag = useRef<{
    x: number;
    y: number;
    view: { x: number; y: number; span: number };
  } | null>(null);
  const events = atRows
    .filter(
      (r) =>
        r.kind === "event" &&
        r.data.kind === "slam_pose_adjustment" &&
        r.data.epoch === epoch,
    )
    .slice(-40)
    .reverse();
  const points = submaps.flatMap((s) => {
    const t = texture(s);
    if (!point(s.pose)) return [];
    if (!t || !point(t.slice_pose)) return [s.pose];
    const pose = compose(s.pose, t.slice_pose);
    return [
      [0, 0],
      [t.width, 0],
      [t.width, t.height],
      [0, t.height],
    ].map(([x, y]) => texturePoint(pose, x, y, t.resolution));
  });
  if (!points.length)
    for (const marker of list(geometry("trajectory")?.data.markers))
      points.push(
        ...(list(marker.points).filter(point) as unknown as number[][]),
      );
  const bounds = points.length
    ? points
    : [
        [-5, -5],
        [5, 5],
      ];
  const xs = bounds.map((p) => p[0]),
    ys = bounds.map((p) => p[1]);
  const fitted = {
    x: (Math.min(...xs) + Math.max(...xs)) / 2,
    y: (Math.min(...ys) + Math.max(...ys)) / 2,
    span:
      Math.max(
        3,
        Math.max(...xs) - Math.min(...xs),
        ((Math.max(...ys) - Math.min(...ys)) * size[0]) / size[1],
      ) * 1.2,
  };
  const currentView = view || fitted;
  const scale = size[0] / currentView.span;
  const screen = (p: number[]) => [
    size[0] / 2 + (p[0] - currentView.x) * scale,
    size[1] / 2 - (p[1] - currentView.y) * scale,
  ];

  useEffect(() => {
    setSelected("");
    setView(null);
  }, [sessionId, epoch]);
  useEffect(() => {
    const c = canvas.current;
    if (!c) return;
    const observer = new ResizeObserver(([entry]) =>
      setSize([
        Math.max(1, entry.contentRect.width),
        Math.max(1, entry.contentRect.height),
      ]),
    );
    observer.observe(c);
    return () => observer.disconnect();
  }, []);
  const textureKey = textures.map((r) => `${r.key}:${r.seq}`).join("|");
  useEffect(() => {
    let disposed = false;
    const active = new Set<string>();
    for (const row of textures) {
      const d = row.data;
      for (const channel of ["image", "light_image"]) {
        const key = `${d.epoch}:${d.id}:${d.version}:${channel}`;
        const src = d[channel];
        active.add(key);
        if (images.current.get(key)?.src === src) continue;
        if (
          typeof src !== "string" ||
          !src.startsWith("data:image/png;base64,")
        )
          continue;
        const image = new Image();
        image.onload = () => {
          if (!disposed) setImageRevision((v) => v + 1);
        };
        images.current.set(key, { src, image });
        image.src = src;
      }
    }
    for (const key of images.current.keys())
      if (!active.has(key)) images.current.delete(key);
    return () => {
      disposed = true;
    };
  }, [textureKey, epoch]);

  useEffect(() => {
    const c = canvas.current;
    const ctx = c?.getContext("2d");
    if (!c || !ctx) return;
    const dpr = Math.min(2, window.devicePixelRatio || 1);
    c.width = Math.round(size[0] * dpr);
    c.height = Math.round(size[1] * dpr);
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.fillStyle = "#182129";
    ctx.fillRect(0, 0, size[0], size[1]);
    const path = (
      pts: number[][],
      color: string,
      width = 1,
      closed = false,
    ) => {
      if (!pts.length) return;
      ctx.beginPath();
      pts.forEach((p, i) => {
        const [x, y] = screen(p);
        if (i) ctx.lineTo(x, y);
        else ctx.moveTo(x, y);
      });
      if (closed) ctx.closePath();
      ctx.strokeStyle = color;
      ctx.lineWidth = width;
      ctx.stroke();
    };
    const step = Math.pow(10, Math.floor(Math.log10(currentView.span / 8)));
    ctx.strokeStyle = "#26343e";
    ctx.lineWidth = 1;
    for (
      let x = Math.floor((currentView.x - currentView.span / 2) / step) * step;
      x < currentView.x + currentView.span / 2;
      x += step
    )
      path(
        [
          [x, currentView.y - currentView.span],
          [x, currentView.y + currentView.span],
        ],
        "#26343e",
      );
    for (
      let y = Math.floor((currentView.y - currentView.span) / step) * step;
      y < currentView.y + currentView.span;
      y += step
    )
      path(
        [
          [currentView.x - currentView.span, y],
          [currentView.x + currentView.span, y],
        ],
        "#26343e",
      );
    for (const s of submaps) {
      if (!point(s.pose)) continue;
      const t = texture(s);
      if (!t || !point(t.slice_pose)) continue;
      const pose = compose(s.pose, t.slice_pose),
        [x, y] = screen(pose);
      const image = images.current.get(
        `${t.epoch}:${t.id}:${t.version}:image`,
      )?.image;
      const light = images.current.get(
        `${t.epoch}:${t.id}:${t.version}:light_image`,
      )?.image;
      if (enabled.submaps && image?.complete && image.naturalWidth) {
        ctx.save();
        const r = t.resolution * scale,
          angle = pose[2];
        ctx.transform(
          Math.sin(angle) * r,
          Math.cos(angle) * r,
          -Math.cos(angle) * r,
          Math.sin(angle) * r,
          x,
          y,
        );
        ctx.imageSmoothingEnabled = false;
        ctx.globalAlpha = selected && selected !== s.id ? 0.3 : 0.95;
        ctx.drawImage(image, 0, 0);
        if (light?.complete && light.naturalWidth) {
          ctx.globalCompositeOperation = "lighter";
          ctx.drawImage(light, 0, 0);
        }
        ctx.restore();
      }
      if (enabled.boundaries) {
        const corners = [
          [0, 0],
          [t.width, 0],
          [t.width, t.height],
          [0, t.height],
        ].map(([u, v]) => texturePoint(pose, u, v, t.resolution));
        path(
          corners,
          selected === s.id ? "#f7bb64" : "#718699",
          selected === s.id ? 2 : 1,
          true,
        );
        ctx.font = "12px system-ui";
        ctx.fillStyle = selected === s.id ? "#f7bb64" : "#c5d5e4";
        ctx.fillText(s.id, x + 6, y - 6);
      }
    }
    for (const key of ["constraints", "trajectory"]) {
      const row = geometry(key);
      if (!enabled[key] || !row?.data.valid) continue;
      for (const marker of list(row.data.markers)) {
        const pts = list(marker.points).filter(point) as unknown as number[][];
        const color =
          marker.color?.length === 4
            ? `rgba(${marker.color
                .slice(0, 3)
                .map((v: number) => Math.round(v * 255))
                .join(",")},${Math.max(0, Math.min(1, marker.color[3]))})`
            : colors[key];
        if (marker.type === 5)
          for (let i = 0; i + 1 < pts.length; i += 2)
            path(pts.slice(i, i + 2), color, 1);
        else if (marker.type === 4)
          path(pts, color, key === "trajectory" ? 2.5 : 1);
        else if (marker.type === 8) {
          ctx.fillStyle = color;
          for (const p of pts) {
            const [x, y] = screen(p);
            ctx.fillRect(x - 1.5, y - 1.5, 3, 3);
          }
        }
      }
    }
    for (const [key, row] of [
      ["scan", scan],
      ["matched_points", geometry("matched_points")],
    ] as const) {
      if (!enabled[key] || !row?.data.valid || timeAt - row.received_at > 1)
        continue;
      ctx.fillStyle = colors[key];
      for (const p of list(row.data.points).filter(point)) {
        const [x, y] = screen(p as unknown as number[]);
        ctx.fillRect(x - 1.5, y - 1.5, 3, 3);
      }
    }
    const robot = atRows.find(
      (r) => r.kind === "pose" && r.data.valid && timeAt - r.received_at <= 2,
    )?.data.pose;
    if (point(robot)) {
      const [x, y] = screen(robot);
      ctx.save();
      ctx.translate(x, y);
      ctx.rotate(-robot[2]);
      ctx.fillStyle = "#8bbaff";
      ctx.beginPath();
      ctx.moveTo(10, 0);
      ctx.lineTo(-7, -6);
      ctx.lineTo(-7, 6);
      ctx.closePath();
      ctx.fill();
      ctx.restore();
    }
    // Hold the initial camera while poses optimize; explicit fit remains available.
    if (!view && submaps.some((s) => texture(s))) setView(fitted);
  }, [rows, timeAt, enabled, selected, view, size, imageRevision]);

  const readyCount = submaps.filter((s) => texture(s)).length;
  const laggingCount = submaps.filter(
    (s) => texture(s) && texture(s)?.version < s.version,
  ).length;
  const capture = () => {
    if (!canvas.current) return;
    const a = document.createElement("a");
    a.download = `slam-${Math.floor(timeAt)}.png`;
    a.href = canvas.current.toDataURL("image/png");
    a.click();
  };
  return (
    <section className="slam-view" aria-label="SLAM 분석 화면">
      <div className="slam-toolbar">
        <div className="slam-layer-controls">
          {Object.entries(labels).map(([key, label]) => (
            <label key={key}>
              <input
                type="checkbox"
                checked={enabled[key]}
                onChange={(e) =>
                  setEnabled((v) => ({ ...v, [key]: e.target.checked }))
                }
              />
              {label}
            </label>
          ))}
        </div>
        <div className="slam-actions">
          <Button variant="outline" size="sm" onClick={() => setView(fitted)}>
            전체 보기
          </Button>
          <Button
            variant="outline"
            size="sm"
            aria-label="SLAM 확대"
            onClick={() =>
              setView({
                ...currentView,
                span: Math.max(0.5, currentView.span / 1.4),
              })
            }
          >
            ＋
          </Button>
          <Button
            variant="outline"
            size="sm"
            aria-label="SLAM 축소"
            onClick={() =>
              setView({
                ...currentView,
                span: Math.min(10000, currentView.span * 1.4),
              })
            }
          >
            −
          </Button>
          <Button variant="outline" size="sm" onClick={capture}>
            이미지 저장
          </Button>
        </div>
      </div>
      <div className="slam-workspace">
        <div className="slam-map-wrap">
          <canvas
            ref={canvas}
            className="slam-map"
            aria-label="서브맵·스캔·궤적·제약선 지도"
            data-submaps={readyCount}
            data-epoch={epoch || ""}
            data-selected={selectedMap?.id || ""}
            onPointerDown={(e) => {
              e.currentTarget.setPointerCapture(e.pointerId);
              drag.current = { x: e.clientX, y: e.clientY, view: currentView };
            }}
            onPointerMove={(e) => {
              if (!drag.current) return;
              const d = drag.current,
                s = size[0] / d.view.span;
              setView({
                ...d.view,
                x: d.view.x - (e.clientX - d.x) / s,
                y: d.view.y + (e.clientY - d.y) / s,
              });
            }}
            onPointerUp={() => {
              drag.current = null;
            }}
            onPointerCancel={() => {
              drag.current = null;
            }}
          />
          <div className="slam-map-caption">
            <span>{config.map_frame || "map"} · 2D</span>
            <span>{currentView.span.toFixed(1)} m · 드래그하여 이동</span>
          </div>
          {!submaps.length && (
            <div className="slam-map-empty">
              {stream?.data.valid === false
                ? `서브맵 표시 불가: ${stream.data.reason}`
                : replay
                  ? "이 시점에 기록된 SLAM 데이터가 없습니다."
                  : "Cartographer 서브맵 수신 대기"}
            </div>
          )}
        </div>
        <aside className="slam-details">
          <div className="slam-state">
            <Badge variant="secondary">
              {replay ? "기록 재생" : stale ? "수신 지연 / 대기" : "실시간"}
            </Badge>
            <span>{time(timeAt)}</span>
          </div>
          <div className="slam-count">
            <strong>
              {readyCount}
              <small> / {submaps.length}</small>
            </strong>
            <span>서브맵 이미지</span>
          </div>
          <p className="slam-status" role="status">
            {status?.reason ||
              (readyCount < submaps.length
                ? "서브맵 이미지 조회 중"
                : submaps.length
                  ? "서브맵 버전과 위치를 함께 표시합니다."
                  : "SLAM 실행 후 자동으로 연결됩니다.")}
          </p>
          {laggingCount > 0 && (
            <p className="slam-status">
              {laggingCount}개 이미지 갱신 중 · 마지막 관측 버전 표시
            </p>
          )}
          {stream?.data.truncated && (
            <p className="slam-status">
              전체 {stream.data.source_count}개 중 최근 {submaps.length}개 표시
            </p>
          )}
          <label className="slam-selection">
            서브맵 선택
            <select
              value={selectedMap?.id || ""}
              onChange={(e) => setSelected(e.target.value)}
            >
              <option value="">전체 서브맵</option>
              {submaps.map((s) => (
                <option key={s.id} value={s.id}>
                  {s.id} · v{s.version}
                </option>
              ))}
            </select>
          </label>
          {selectedMap && (
            <dl className="slam-facts">
              <dt>버전</dt>
              <dd>{selectedMap.version}</dd>
              <dt>위치</dt>
              <dd>
                {selectedMap.pose
                  .slice(0, 2)
                  .map((v: number) => v.toFixed(2))
                  .join(", ")}{" "}
                m
              </dd>
              <dt>상태</dt>
              <dd>{selectedMap.frozen ? "고정 궤적" : "활성 궤적"}</dd>
              <dt>이미지</dt>
              <dd>
                {texture(selectedMap)
                  ? texture(selectedMap)?.version === selectedMap.version
                    ? "현재 버전 수신"
                    : `표시 v${texture(selectedMap)?.version} · 최신 v${selectedMap.version}`
                  : "현재 버전 미수신"}
              </dd>
            </dl>
          )}
          <h3>위치 보정 관측</h3>
          <small>
            루프 클로저 확정 여부와 별개로, 관측된 서브맵 위치 변화를
            표시합니다.
          </small>
          <div className="slam-events">
            {events.length ? (
              events.map((r) => (
                <button key={r.seq} onClick={() => seek(r.received_at)}>
                  <span>
                    {time(r.received_at)} · {r.data.id}
                  </span>
                  <strong>{r.data.distance_m.toFixed(3)} m</strong>
                </button>
              ))
            ) : (
              <p>이 시점까지 기록된 위치 보정 없음</p>
            )}
          </div>
          {stale && submaps.length > 0 && (
            <p className="slam-status">마지막 지도 유지 · 새 SLAM 관측 대기</p>
          )}
          {(["trajectory", "constraints", "matched_points"] as const).map(
            (key) =>
              geometry(key)?.data.truncated || geometry(key)?.data.sampled ? (
                <small key={key}>{labels[key]}: 제한된 표본 표시</small>
              ) : null,
          )}
        </aside>
      </div>
    </section>
  );
}
