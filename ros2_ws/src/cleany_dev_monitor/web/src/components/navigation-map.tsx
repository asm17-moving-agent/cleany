import { useEffect, useRef, useState } from "react";
import { Button } from "@/components/ui/button";
import { Panel } from "@/components/panel";
import { type Data, type Row } from "@/lib/monitor";

const labels: Record<string, string> = {
  map: "지도",
  path: "계획 경로",
  trail: "실제 궤적",
  footprint: "차체 외곽",
};

// One overview in the map frame; sensor and planner geometry belongs in RViz.
export function NavigationMap({
  rows,
  config,
  timeAt,
}: {
  rows: Row[];
  config: Data;
  timeAt: number;
}) {
  const canvas = useRef<HTMLCanvasElement>(null);
  const [zoom, setZoom] = useState(1);
  const [enabled, setEnabled] = useState<Record<string, boolean>>(
    Object.fromEntries(Object.keys(labels).map((key) => [key, true])),
  );
  const layer = (key: string) =>
    rows.find((r) => r.kind === "layer" && r.key === key)?.data;
  const map = layer("map"),
    path = layer("path");
  const poseRow = rows.find((r) => r.kind === "pose" && r.key === "robot");
  const robot = poseRow?.data;
  const robotValid = robot?.valid && timeAt - poseRow!.received_at <= 2;
  const summary = rows.find(
    (r) => r.kind === "navigation" && r.key === "summary",
  )?.data;
  const footprint = rows.find(
    (r) => r.kind === "navigation" && r.key === "footprint",
  );
  const navigation = rows.find((r) => r.kind === "runtime")?.data.navigation;

  useEffect(() => {
    const target = canvas.current;
    if (!target) return;
    const frame = document.createElement("canvas");
    frame.width = 1100;
    frame.height = 620;
    const ctx = frame.getContext("2d")!;
    ctx.fillStyle = "#181818";
    ctx.fillRect(0, 0, frame.width, frame.height);
    const world = (x: number, y: number, origin: number[]) => [
      origin[0] + Math.cos(origin[2]) * x - Math.sin(origin[2]) * y,
      origin[1] + Math.sin(origin[2]) * x + Math.cos(origin[2]) * y,
    ];
    const bounds: number[][] = [];
    const background = map?.valid ? map : null;
    if (background) {
      for (const [x, y] of [
        [0, 0],
        [background.width, 0],
        [0, background.height],
        [background.width, background.height],
      ])
        bounds.push(
          world(
            x * background.resolution,
            y * background.resolution,
            background.origin,
          ),
        );
    } else {
      if (path?.valid) bounds.push(...path.points);
      if (Array.isArray(summary?.trail))
        bounds.push(...summary.trail.map((p: number[]) => p.slice(1, 3)));
      if (robotValid) bounds.push(robot!.pose.slice(0, 2));
      if (!bounds.length) bounds.push([-5, -5], [5, 5]);
    }
    const minX = Math.min(...bounds.map((p) => p[0])),
      maxX = Math.max(...bounds.map((p) => p[0]));
    const minY = Math.min(...bounds.map((p) => p[1])),
      maxY = Math.max(...bounds.map((p) => p[1]));
    const scale =
      Math.min(
        (frame.width - 80) / Math.max(1, maxX - minX),
        (frame.height - 80) / Math.max(1, maxY - minY),
      ) * zoom;
    const ox = frame.width / 2 - ((minX + maxX) / 2) * scale;
    const oy = frame.height / 2 + ((minY + maxY) / 2) * scale;
    const screen = (x: number, y: number) => [ox + x * scale, oy - y * scale];

    if (enabled.map && map?.valid) {
      const grid = map;
      const image = document.createElement("canvas");
      image.width = grid.width;
      image.height = grid.height;
      const gc = image.getContext("2d")!;
      const pixels = gc.createImageData(grid.width, grid.height);
      grid.cells.forEach((value: number, i: number) => {
        const offset = i * 4;
        const shade = value < 0 ? 78 : Math.round(235 - value * 1.9);
        pixels.data[offset] = shade;
        pixels.data[offset + 1] = shade;
        pixels.data[offset + 2] = shade;
        pixels.data[offset + 3] = 255;
      });
      gc.putImageData(pixels, 0, 0);
      ctx.save();
      ctx.translate(ox, oy);
      ctx.scale(scale, -scale);
      ctx.translate(grid.origin[0], grid.origin[1]);
      ctx.rotate(grid.origin[2]);
      ctx.imageSmoothingEnabled = false;
      ctx.drawImage(
        image,
        0,
        0,
        grid.width * grid.resolution,
        grid.height * grid.resolution,
      );
      ctx.restore();
    }
    const line = (points: number[][], color: string, closed = false) => {
      if (!points?.length) return;
      ctx.strokeStyle = color;
      ctx.lineWidth = 2;
      ctx.beginPath();
      points.forEach(([x, y], i) => {
        const [a, b] = screen(x, y);
        i ? ctx.lineTo(a, b) : ctx.moveTo(a, b);
      });
      if (closed) ctx.closePath();
      ctx.stroke();
    };
    if (enabled.path && path?.valid) line(path.points, "#4ae7ba");
    if (enabled.trail && Array.isArray(summary?.trail))
      line(
        summary.trail.map((p: number[]) => p.slice(1, 3)),
        "#80aaff",
      );
    if (
      enabled.footprint &&
      footprint?.data.valid &&
      timeAt - footprint.received_at <= 2
    )
      line(footprint.data.points, "#f8f8f8", true);
    const goal = navigation?.goal_pose;
    if (goal && navigation.frame === config.map_frame) {
      const [x, y] = screen(goal.x, goal.y);
      ctx.strokeStyle = "#ffd584";
      ctx.beginPath();
      ctx.moveTo(x - 9, y);
      ctx.lineTo(x + 9, y);
      ctx.moveTo(x, y - 9);
      ctx.lineTo(x, y + 9);
      ctx.stroke();
    }
    if (robotValid) {
      const [x, y] = screen(robot!.pose[0], robot!.pose[1]);
      ctx.save();
      ctx.translate(x, y);
      ctx.rotate(-robot!.pose[2]);
      ctx.fillStyle = "#fff";
      ctx.strokeStyle = "#262626";
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(15, 0);
      ctx.lineTo(-10, -9);
      ctx.lineTo(-6, 0);
      ctx.lineTo(-10, 9);
      ctx.closePath();
      ctx.fill();
      ctx.stroke();
      ctx.strokeStyle = "#fff";
      ctx.beginPath();
      ctx.moveTo(15, 0);
      ctx.lineTo(34, 0);
      ctx.lineTo(27, -5);
      ctx.moveTo(34, 0);
      ctx.lineTo(27, 5);
      ctx.stroke();
      ctx.restore();
    }
    ctx.fillStyle = "#b8b8b8";
    ctx.font = "13px monospace";
    ctx.fillText(
      `frame: ${config.map_frame || "map"}, ${zoom.toFixed(1)}×`,
      20,
      frame.height - 20,
    );
    if (target.width !== frame.width) target.width = frame.width;
    if (target.height !== frame.height) target.height = frame.height;
    target.getContext("2d")!.drawImage(frame, 0, 0);
  }, [rows, timeAt, zoom, enabled, config.map_frame]);

  return (
    <Panel
      title="전체 경로와 주행 진행"
      aside={
        <div className="inline">
          <Button
            variant="outline"
            aria-label="지도 축소"
            onClick={() => setZoom(Math.max(0.5, zoom - 0.25))}
          >
            −
          </Button>
          <Button
            variant="outline"
            aria-label="지도 확대"
            onClick={() => setZoom(Math.min(8, zoom + 0.25))}
          >
            ＋
          </Button>
          <Button variant="outline" onClick={() => setZoom(1)}>
            맞춤
          </Button>
        </div>
      }
    >
      <div className="map-tools">
        {Object.entries(labels).map(([key, label]) => (
          <label key={key}>
            <input
              type="checkbox"
              checked={enabled[key]}
              onChange={(e) =>
                setEnabled({ ...enabled, [key]: e.target.checked })
              }
            />
            {label}
          </label>
        ))}
        <span>△ 차체, 방향, ＋ 목표, 파랑 실제 궤적, 초록 계획 경로</span>
      </div>
      {!map?.valid && (
        <p className="nav-map-unavailable">
          배경 지도 표시 불가, {map?.reason || "지도 토픽 미수신"}
        </p>
      )}
      {!robotValid && (
        <p className="nav-map-unavailable">
          차체 위치, 방향 표시 불가,{" "}
          {robot?.reason || "TF 미수신 또는 수신 지연"}
        </p>
      )}
      <canvas
        ref={canvas}
        aria-label="ROS 2D navigation map"
        className="map"
        data-robot-visible={robotValid ? "true" : "false"}
      />
      <details className="nav-map-sources">
        <summary>지도 수신 상태와 출처</summary>
        <div className="layer-status">
          {[
            ["map", map],
            ["path", path],
          ].map(([key, data]) => (
            <div key={String(key)}>
              <b>{String(key)}</b>
              <small>
                {(data as Data)?.valid ? "수신" : "표시 불가"},{" "}
                {(data as Data)?.topic || config[`${key}_topic`] || "토픽 대기"}
              </small>
            </div>
          ))}
        </div>
      </details>
    </Panel>
  );
}
