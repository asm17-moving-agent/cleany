import React, { useEffect, useMemo, useRef, useState } from "react";
import { createRoot } from "react-dom/client";
import {
  ReactFlow,
  Background,
  Handle,
  BaseEdge,
  getSmoothStepPath,
  type NodeProps,
  type EdgeProps,
  Controls,
  MarkerType,
  Position,
  useReactFlow,
  useStore,
  type Node,
  type Edge,
} from "@xyflow/react";
import "@xyflow/react/dist/style.css";
import { CameraView } from "@/components/camera-view";
import { ThemeToggle } from "@/components/theme-toggle";
import "./theme.css";
import "./style.css";
import { Button } from "@/components/ui/button";
import { Badge as UiBadge } from "@/components/ui/badge";
import { Card, CardHeader, CardTitle, CardContent } from "@/components/ui/card";
import { Input } from "@/components/ui/input";
import { Tabs, TabsList, TabsTrigger } from "@/components/ui/tabs";
import { Separator } from "@/components/ui/separator";
import { TooltipProvider } from "@/components/ui/tooltip";
import {
  Activity,
  Camera,
  GitBranch,
  Network,
  Navigation2,
  Logs as LogsIcon,
  History,
  Bot,
  ArrowUpRight,
  Radio,
  Unplug,
  PanelLeftClose,
  PanelLeftOpen,
} from "lucide-react";
import { FoxglovePanel } from "@/components/foxglove-panel";

type Data = Record<string, any>;
type Row = {
  seq: number;
  kind: string;
  key: string;
  data: Data;
  received_at: number;
  ros_time: number | null;
};
type Packet = {
  rows: Row[];
  boot_id?: string;
  sequence?: number;
  initial?: boolean;
  gap?: boolean;
  recording?: Data;
  error?: string;
  selection?: string[];
};
function applyRuntimeEvents(rows: Row[]): Row[] {
  const base = rows.find((r) => r.kind === "runtime");
  if (!base) return rows;
  const data = structuredClone(base.data);
  for (const row of rows
    .filter((r) => r.kind === "event" && r.seq > base.seq)
    .sort((a, b) => a.seq - b.seq)) {
    if (row.data.boot_id !== data.boot_id) continue;
    const event = row.data;
    if (event.kind === "transition") {
      data.state = event.data.to;
      data.active_request = event.data.active_request;
      data.ready = false;
      data.reason = "전이 후 준비 상태 확인 중";
    }
    if (event.kind === "bt_tick") {
      const node = data.bt_nodes?.find(
        (n: Data) => n.id === event.data.node_id,
      );
      if (node) {
        node.status = event.data.status;
        node.visited = true;
      }
    }
    if (event.kind === "mission_result") data.last_result = event.data;
  }
  return rows.map((r) => (r === base ? { ...base, data } : r));
}
const critical = new Set(["event", "log", "result"]);
const navIcons = [GitBranch, Network, Navigation2, LogsIcon, History, Camera];
const tabs = [
  ["execution", "실행 흐름", "01"],
  ["ros", "ROS 탐색기", "02"],
  ["navigation", "자율주행", "03"],
  ["logs", "로그", "04"],
  ["recordings", "기록 & 재생", "05"],
  ["camera", "카메라", "06"],
];
const time = (t: number) =>
  new Date(t * 1000).toLocaleTimeString("ko-KR", { hour12: false });
const pretty = (v: unknown) => JSON.stringify(v, null, 2);
const statusColor = (s: string) =>
  s === "RUNNING"
    ? "var(--info-text)"
    : s === "SUCCESS"
      ? "var(--good-text)"
      : s === "FAILURE" || s === "ERROR"
        ? "var(--bad-text)"
        : "var(--muted-foreground)";
function Json({ value }: { value: unknown }) {
  return <pre>{pretty(value)}</pre>;
}
function Empty({ children }: { children: React.ReactNode }) {
  return (
    <div className="empty">
      <Radio size={24} strokeWidth={1.5} />
      <p>{children}</p>
    </div>
  );
}
function Badge({
  children,
  tone = "",
}: {
  children: React.ReactNode;
  tone?: string;
}) {
  return (
    <UiBadge variant="secondary" className={"badge " + tone}>
      {children}
    </UiBadge>
  );
}
function Panel({
  title,
  aside,
  children,
  className = "",
}: {
  title: string;
  aside?: React.ReactNode;
  children: React.ReactNode;
  className?: string;
}) {
  return (
    <Card className={"panel " + className}>
      <CardHeader className="panel-title">
        <CardTitle>
          <h2>{title}</h2>
        </CardTitle>
        {aside}
      </CardHeader>
      <CardContent className="panel-content">{children}</CardContent>
    </Card>
  );
}

function FitGraphToViewport({ topology = "" }: { topology?: string }) {
  const { fitView } = useReactFlow();
  const fit = useRef(fitView);
  fit.current = fitView;
  const width = useStore((state) => state.width);
  const height = useStore((state) => state.height);
  useEffect(() => {
    const timer = setTimeout(
      () => void fit.current({ padding: 0.18, maxZoom: 1, duration: 0 }),
      100,
    );
    return () => clearTimeout(timer);
  }, [width, height, topology]);
  return null;
}

// Presentation only: never delay runtime readiness, results, or safety indicators.
function useVisibleFsmState(state: string): string {
  const [visible, setVisible] = useState(state);
  const playback = useRef({
    current: state,
    since: performance.now(),
    queue: [] as string[],
    timer: undefined as ReturnType<typeof setTimeout> | undefined,
  });
  useEffect(() => {
    const p = playback.current;
    if (state !== (p.queue.at(-1) ?? p.current)) p.queue.push(state);
    const advance = () => {
      p.timer = undefined;
      if (!p.queue.length) return;
      const remaining = 500 - (performance.now() - p.since);
      if (remaining > 0) {
        p.timer = setTimeout(advance, remaining);
        return;
      }
      p.current = p.queue.shift()!;
      p.since = performance.now();
      setVisible(p.current);
      if (p.queue.length) p.timer = setTimeout(advance, 500);
    };
    if (p.timer === undefined) advance();
  }, [state]);
  useEffect(() => () => clearTimeout(playback.current.timer), []);
  return visible;
}

function Tree({
  runtime,
  onSelect,
  bt = false,
}: {
  runtime: Data;
  onSelect: (id: string) => void;
  bt?: boolean;
}) {
  const visibleState = useVisibleFsmState(runtime.state);
  const spec = JSON.stringify(
    bt
      ? {
          bt_nodes: (runtime.bt_nodes || []).map((n: Data) => ({
            id: n.id,
            parent: n.parent,
            status: n.status,
            visited: n.visited,
          })),
        }
      : {
          state: visibleState,
          fsm_states: runtime.fsm_states,
          fsm_edges: runtime.fsm_edges,
        },
  );
  return (
    <>
      {!bt && visibleState !== runtime.state && (
        <span className="fsm-playback-note">
          전이 재생 중 · 실제 {runtime.state}
        </span>
      )}
      <MemoDiagram spec={spec} onSelect={onSelect} bt={bt} />
    </>
  );
}

function FsmNode({ data }: NodeProps) {
  return (
    <>
      {[Position.Left, Position.Right, Position.Top, Position.Bottom].map(
        (position) => (
          <React.Fragment key={position}>
            <Handle
              id={`${position}-in`}
              type="target"
              position={position}
              style={
                position === Position.Top || position === Position.Bottom
                  ? { left: "42%" }
                  : { top: position === Position.Left ? "42%" : "58%" }
              }
            />
            <Handle
              id={`${position}-out`}
              type="source"
              position={position}
              style={
                position === Position.Top || position === Position.Bottom
                  ? { left: "58%" }
                  : { top: position === Position.Right ? "42%" : "58%" }
              }
            />
          </React.Fragment>
        ),
      )}
      {data.label as React.ReactNode}
    </>
  );
}

function FsmEdge(props: EdgeProps) {
  const [path, labelX, labelY] = getSmoothStepPath({
    ...props,
    borderRadius: 0,
    offset: Number(props.data?.offset ?? 24),
    centerY: props.data?.centerY as number | undefined,
  });
  return (
    <BaseEdge
      path={path}
      markerEnd={props.markerEnd}
      style={props.style}
      label={props.label}
      labelX={labelX}
      labelY={labelY}
      labelStyle={{ fill: "var(--foreground)", fontSize: 14, fontWeight: 450 }}
      labelBgStyle={{ fill: "var(--card)" }}
      labelBgPadding={[6, 4]}
      labelBgBorderRadius={2}
    />
  );
}
const runtimeNodeTypes = { fsm: FsmNode };
const runtimeEdgeTypes = { fsm: FsmEdge };

// Telemetry timestamps and unrelated fields must not rebuild the graph.
const MemoDiagram = React.memo(function Diagram({
  spec,
  onSelect,
  bt,
}: {
  spec: string;
  onSelect: (id: string) => void;
  bt: boolean;
}) {
  const runtime: Data = useMemo(() => JSON.parse(spec), [spec]);
  const states: string[] = runtime.fsm_states || [];
  const btNodes: Data[] = runtime.bt_nodes || [];
  // Center each parent over its leaves; keep malformed/cyclic trees bounded.
  const positions = new Map<string, { x: number; y: number }>();
  const visited = new Set<string>();
  let leaf = 0;
  const place = (id: string, depth: number): number => {
    if (visited.has(id)) return positions.get(id)?.x ?? 0;
    visited.add(id);
    const children = btNodes.filter(
      (n) => n.parent === id && !visited.has(n.id),
    );
    const xs = children.map((n) => place(n.id, depth + 1));
    const x = xs.length ? (xs[0] + xs[xs.length - 1]) / 2 : leaf++ * 240;
    positions.set(id, { x, y: depth * 135 });
    return x;
  };
  btNodes.filter((n) => !n.parent).forEach((n) => place(n.id, 0));
  btNodes.filter((n) => !visited.has(n.id)).forEach((n) => place(n.id, 0));
  const mainFlow = [
    "IDLE",
    "NAVIGATE_TO_TARGET",
    "WORKING",
    "POST_MISSION",
    "RETURN_HOME",
  ];
  // Fixed small offsets preserve direction without a rigid grid.
  const mainPositions = [
    { x: 0, y: 3.6 },
    { x: 310, y: -3.2 },
    { x: 623, y: 7 },
    { x: 932, y: -0.4 },
    { x: 1245, y: 4.8 },
  ];
  const exceptional = [
    "CANCELLING",
    "ERROR",
    ...states.filter(
      (id) => !mainFlow.includes(id) && !["CANCELLING", "ERROR"].includes(id),
    ),
  ];
  const transitionLabels: Record<string, string> = {
    "IDLE-NAVIGATE_TO_TARGET": "미션 수락",
    "NAVIGATE_TO_TARGET-WORKING": "도착 · 정지",
    "WORKING-POST_MISSION": "청소 완료",
    "POST_MISSION-RETURN_HOME": "복귀 정책",
    "POST_MISSION-IDLE": "복귀 생략",
    "RETURN_HOME-IDLE": "홈 도착 · 정지",
    "CANCELLING-RETURN_HOME": "정지 확인 · 복귀",
    "CANCELLING-IDLE": "정지 확인 · 종료",
    "CANCELLING-ERROR": "안전 미확인 / 치명 오류",
    "IDLE-ERROR": "안전 오류",
    "ERROR-IDLE": "안전 확인 · 리셋",
  };
  const topology = JSON.stringify(
    bt ? btNodes.map((n) => [n.id, n.parent]) : [states, runtime.fsm_edges],
  );
  const nodes: Node[] = bt
    ? btNodes.map((n) => {
        return {
          id: n.id,
          position: positions.get(n.id)!,
          data: {
            label: (
              <>
                <strong>{n.id}</strong>
                <small>
                  {n.status} {n.visited ? "· ticked" : ""}
                </small>
              </>
            ),
          },
          style: {
            borderColor: statusColor(n.status),
            background:
              n.status === "RUNNING" ? "var(--info-bg)" : "var(--card)",
            opacity: n.status === "INVALID" ? 0.65 : 1,
          },
        };
      })
    : states.map((id) => ({
        id,
        type: "fsm",
        position: mainFlow.includes(id)
          ? mainPositions[mainFlow.indexOf(id)]
          : {
              x: 580 + exceptional.indexOf(id) * 420,
              y: 340 + (exceptional.indexOf(id) % 2) * 7,
            },
        ariaLabel: runtime.state === id ? `${id} (활성)` : id,
        sourcePosition: Position.Right,
        targetPosition: Position.Left,
        data: {
          label: <strong>{id}</strong>,
        },
        style: {
          borderColor:
            runtime.state === id
              ? id === "ERROR"
                ? "var(--bad-text)"
                : "var(--primary)"
              : "var(--input)",
          background:
            runtime.state === id
              ? id === "ERROR"
                ? "var(--bad-bg)"
                : "var(--info-bg)"
              : "var(--card)",
          borderWidth: runtime.state === id ? 2 : 1,
        },
      }));
  const edges: Edge[] = bt
    ? btNodes
        .filter((n) => n.parent)
        .map((n) => ({
          id: n.parent + "-" + n.id,
          source: n.parent,
          target: n.id,
          animated: false,
          type: "smoothstep",
        }))
    : (runtime.fsm_edges || []).map(([a, b]: string[]) => {
        const ai = mainFlow.indexOf(a);
        const bi = mainFlow.indexOf(b);
        const forward = ai >= 0 && bi === ai + 1;
        const upperReturn = ai >= 0 && bi >= 0 && bi < ai;
        const lowerReturn = bi === 0 && ai < 0;
        const lowerForward = ai < 0 && bi < 0;
        const source =
          forward || lowerForward
            ? Position.Right
            : upperReturn
              ? Position.Top
              : ai < 0 && !lowerReturn
                ? Position.Top
                : Position.Bottom;
        const target =
          forward || lowerForward
            ? Position.Left
            : upperReturn
              ? Position.Top
              : bi < 0 && !lowerReturn
                ? Position.Top
                : Position.Bottom;
        return {
          id: a + "-" + b,
          source: a,
          target: b,
          sourceHandle: `${source}-out`,
          targetHandle: `${target}-in`,
          type: "fsm",
          label:
            transitionLabels[`${a}-${b}`] ??
            (b === "CANCELLING" ? "취소 / 실패" : undefined),
          data: {
            offset: upperReturn
              ? 44 + (ai - 3) * 48
              : lowerReturn
                ? 44 + exceptional.indexOf(a) * 48
                : 24,
            centerY:
              !forward && !upperReturn && !lowerReturn && !lowerForward
                ? 120 + Math.max(ai, bi, 0) * 36 + (ai < 0 ? 36 : 0)
                : undefined,
          },
          style: {
            stroke: "var(--graph-edge)",
            strokeWidth: forward ? 1.5 : 1.2,
            opacity: forward ? 1 : 0.8,
          },
          markerEnd: {
            type: MarkerType.Arrow,
            width: 18,
            height: 18,
            color: "var(--graph-edge)",
          },
        };
      });
  return nodes.length ? (
    <div className={"flow " + (bt ? "bt" : "fsm-diagram")}>
      <ReactFlow
        nodeTypes={runtimeNodeTypes}
        edgeTypes={runtimeEdgeTypes}
        nodes={nodes.map((n) => ({
          ...n,
          className: "runtime-node react-flow__node-default",
          width: 200,
          height: 76,
          style: { ...n.style, width: 200, height: 76 },
        }))}
        edges={edges}
        fitView
        fitViewOptions={{ padding: 0.18 }}
        minZoom={0.1}
        nodesDraggable={false}
        nodesConnectable={false}
        onNodeClick={(_, n) => onSelect(n.id)}
      >
        <FitGraphToViewport topology={topology} />
        {bt && <Background gap={20} color="var(--graph-dot)" />}
        <Controls showInteractive={false} />
      </ReactFlow>
    </div>
  ) : (
    <Empty>{bt ? "BT 생성 대기" : "Runtime 대기"}</Empty>
  );
});

function Navigation({
  rows,
  config,
  timeAt,
}: {
  rows: Row[];
  config: Data;
  timeAt: number;
}) {
  const canvas = useRef<HTMLCanvasElement>(null);
  const [enabled, setEnabled] = useState<Record<string, boolean>>({
    map: true,
    local_costmap: true,
    global_costmap: true,
    path: true,
  });
  const [zoom, setZoom] = useState(1);
  const navigation = rows.find((r) => r.kind === "runtime")?.data.navigation;
  const layers = rows.filter((r) => r.kind === "layer");
  const poseRow = rows.find((r) => r.kind === "pose");
  const poseAge = poseRow
    ? Math.max(0, timeAt - poseRow.received_at)
    : Infinity;
  const robot: Data | undefined = poseRow
    ? {
        ...poseRow.data,
        valid: poseRow.data.valid && poseAge <= 2,
        reason: poseAge > 2 ? "TF 관측 수신 지연" : poseRow.data.reason,
      }
    : undefined;
  useEffect(() => {
    const c = canvas.current;
    if (!c) return;
    const ctx = c.getContext("2d");
    if (!ctx) return;
    const width = (c.width = 1100),
      height = (c.height = 620);
    ctx.fillStyle = "#181818";
    ctx.fillRect(0, 0, width, height);
    const map = layers.find((r) => r.key === "map" && r.data.valid)?.data;
    const points: number[][] = [];
    const world = (x: number, y: number, o: number[]) => [
      o[0] + Math.cos(o[2]) * x - Math.sin(o[2]) * y,
      o[1] + Math.sin(o[2]) * x + Math.cos(o[2]) * y,
    ];
    if (map)
      for (const [x, y] of [
        [0, 0],
        [map.width, 0],
        [0, map.height],
        [map.width, map.height],
      ])
        points.push(world(x * map.resolution, y * map.resolution, map.origin));
    if (!points.length) points.push([-5, -5], [5, 5]);
    const minX = Math.min(...points.map((p) => p[0])),
      maxX = Math.max(...points.map((p) => p[0]));
    const minY = Math.min(...points.map((p) => p[1])),
      maxY = Math.max(...points.map((p) => p[1]));
    const scale =
      Math.min(
        (width - 80) / Math.max(1, maxX - minX),
        (height - 80) / Math.max(1, maxY - minY),
      ) * zoom;
    const ox = width / 2 - ((minX + maxX) / 2) * scale,
      oy = height / 2 + ((minY + maxY) / 2) * scale;
    const screen = (x: number, y: number) => [ox + x * scale, oy - y * scale];
    for (const name of ["map", "global_costmap", "local_costmap"]) {
      const layer = layers.find((r) => r.key === name)?.data;
      if (!enabled[name] || !layer?.valid) continue;
      const off = document.createElement("canvas");
      off.width = layer.width;
      off.height = layer.height;
      const oc = off.getContext("2d")!;
      const pixels = oc.createImageData(layer.width, layer.height);
      layer.cells.forEach((value: number, i: number) => {
        const p = i * 4;
        const mapLayer = name === "map";
        const shade = value < 0 ? 78 : Math.round(235 - value * 1.9);
        pixels.data[p] = mapLayer
          ? shade
          : name === "global_costmap"
            ? 230
            : 63;
        pixels.data[p + 1] = mapLayer ? shade : 110;
        pixels.data[p + 2] = mapLayer
          ? shade
          : name === "global_costmap"
            ? 95
            : 232;
        pixels.data[p + 3] = mapLayer
          ? 255
          : value <= 0
            ? 0
            : Math.min(175, value * 1.75);
      });
      oc.putImageData(pixels, 0, 0);
      ctx.save();
      ctx.translate(ox, oy);
      ctx.scale(scale, -scale);
      ctx.translate(layer.origin[0], layer.origin[1]);
      ctx.rotate(layer.origin[2]);
      ctx.imageSmoothingEnabled = false;
      ctx.drawImage(
        off,
        0,
        0,
        layer.width * layer.resolution,
        layer.height * layer.resolution,
      );
      ctx.restore();
    }
    const path = layers.find((r) => r.key === "path")?.data;
    if (enabled.path && path?.valid && path.points.length) {
      ctx.strokeStyle = "#4ae7ba";
      ctx.lineWidth = 3;
      ctx.beginPath();
      path.points.forEach(([x, y]: number[], i: number) => {
        const [a, b] = screen(x, y);
        i ? ctx.lineTo(a, b) : ctx.moveTo(a, b);
      });
      ctx.stroke();
      const [x, y] = screen(...(path.points.at(-1) as [number, number]));
      ctx.strokeStyle = "#ffd584";
      ctx.beginPath();
      ctx.arc(x, y, 8, 0, Math.PI * 2);
      ctx.stroke();
    }
    const goal = navigation?.goal_pose;
    if (goal && navigation.frame === config.map_frame) {
      const [x, y] = screen(goal.x, goal.y);
      ctx.strokeStyle = "#ffd584";
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(x - 9, y);
      ctx.lineTo(x + 9, y);
      ctx.moveTo(x, y - 9);
      ctx.lineTo(x, y + 9);
      ctx.stroke();
      ctx.fillStyle = "#ffd584";
      ctx.font = "12px monospace";
      ctx.fillText(navigation.target, x + 12, y - 8);
    }
    if (robot?.valid) {
      const [x, y] = screen(robot.pose[0], robot.pose[1]);
      ctx.save();
      ctx.translate(x, y);
      ctx.rotate(-robot.pose[2]);
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
      ctx.restore();
    }
    ctx.fillStyle = "#b8b8b8";
    ctx.font = "13px monospace";
    ctx.fillText(
      `frame: ${config.map_frame || "map"} · ${zoom.toFixed(1)}×`,
      20,
      height - 20,
    );
  }, [rows, enabled, zoom, Math.floor(poseAge)]);
  return (
    <>
      <Panel
        title="2D Navigation"
        aside={
          <div className="inline">
            <Button
              variant="outline"
              onClick={() => setZoom(Math.max(0.5, zoom - 0.25))}
            >
              −
            </Button>
            <Button variant="outline" onClick={() => setZoom(zoom + 0.25)}>
              ＋
            </Button>
            <Button variant="outline" onClick={() => setZoom(1)}>
              맞춤
            </Button>
          </div>
        }
      >
        <div className="map-tools">
          {Object.keys(enabled).map((k) => (
            <label key={k}>
              <input
                type="checkbox"
                checked={enabled[k]}
                onChange={(e) =>
                  setEnabled({ ...enabled, [k]: e.target.checked })
                }
              />
              {k}
            </label>
          ))}
          <span>△ 로봇 · ＋ 미션 목표 · ○ 경로 끝점</span>
        </div>
        <canvas
          ref={canvas}
          aria-label="ROS 2D navigation map"
          className="map"
        />
        <div className="layer-status">
          {["map", "path", "local_costmap", "global_costmap"].map((k) => {
            const l = layers.find((r) => r.key === k);
            return (
              <div key={k}>
                <b>{k}</b>
                <Badge tone={l?.data.valid ? "good" : "warn"}>
                  {l?.data.valid ? "수신" : "표시 불가"}
                </Badge>
                <small>{l?.data.reason || l?.data.topic || "토픽 대기"}</small>
              </div>
            );
          })}
          <div>
            <b>robot TF</b>
            <Badge tone={robot?.valid ? "good" : "warn"}>
              {robot?.valid ? "수신" : "표시 불가"}
            </Badge>
            <small>
              {robot?.reason || `age ${robot?.age?.toFixed(2) ?? "—"}s`}
            </small>
          </div>
        </div>
      </Panel>
      <Panel title="속도 명령 관측">
        <div className="velocity">
          {(config.velocity_topics || []).map((topic: string) => {
            const sample = rows.find(
              (r) => r.key === topic && r.kind === "sample",
            );
            return (
              <div key={topic}>
                <code>{topic}</code>
                <small>
                  {sample
                    ? `마지막 수신 ${Math.max(0, timeAt - sample.received_at).toFixed(1)}초 전`
                    : "미수신"}
                </small>
                {sample && timeAt - sample.received_at > 2 && (
                  <Badge tone="warn">오래된 명령</Badge>
                )}
                <Json value={sample?.data || "수신 대기"} />
              </div>
            );
          })}
        </div>
      </Panel>
    </>
  );
}

function RosView({
  rows,
  graph,
  selected,
  select,
  replay,
  focusTopic,
}: {
  rows: Row[];
  graph: Data;
  selected: string[];
  select: (s: string[]) => void;
  replay: boolean;
  focusTopic: string;
}) {
  const [query, setQuery] = useState("");
  const [topic, setTopic] = useState(focusTopic);
  const [view, setView] = useState("topics");
  const topics: Data[] = graph.topics || [];
  const item = topics.find((t) => t.name === topic);
  const sample = rows.find((r) => r.key === topic && r.kind === "sample");
  const connections = useMemo(() => {
    const chosen = topic
      ? topics.filter((t) => t.name === topic)
      : topics.slice(0, 12);
    const nodes: Node[] = [];
    const edges: Edge[] = [];
    const names = new Map<string, string>();
    chosen.forEach((t, i) => {
      nodes.push({
        id: t.name,
        position: { x: 340, y: i * 100 },
        data: { label: t.name },
        style: { background: "var(--info-bg)" },
        initialWidth: 180,
        initialHeight: 58,
      });
      for (const [kind, endpoints] of [
        ["pub", t.publishers],
        ["sub", t.subscribers],
      ] as [string, Data[]][]) {
        endpoints.forEach((ep) => {
          const key = kind + ep.node;
          if (!names.has(key)) {
            names.set(key, key);
            nodes.push({
              id: key,
              position: {
                x: kind === "pub" ? 0 : 700,
                y:
                  [...names.keys()].filter((k) => k.startsWith(kind)).length *
                    100 -
                  100,
              },
              data: { label: ep.node },
              initialWidth: 180,
              initialHeight: 58,
            });
          }
          edges.push({
            id: key + t.name,
            source: kind === "pub" ? key : t.name,
            target: kind === "pub" ? t.name : key,
            markerEnd: { type: MarkerType.ArrowClosed },
          });
        });
      }
    });
    return { nodes, edges };
  }, [graph, topic]);
  return (
    <>
      <Tabs value={view} onValueChange={setView} className="subtabs">
        <TabsList>
          {[
            ["topics", "Topics"],
            ["graph", "연결 그래프"],
            ["nodes", "Nodes"],
            ["actions", "Actions"],
            ["services", "Services"],
          ].map(([id, label]) => (
            <TabsTrigger key={id} value={id}>
              {label}
            </TabsTrigger>
          ))}
        </TabsList>
      </Tabs>
      {view === "topics" ? (
        <div className="ros-grid">
          <Panel title={`Topics · ${topics.length}`}>
            <Input
              className="search"
              placeholder="토픽 이름 또는 타입 검색"
              value={query}
              onChange={(e) => setQuery(e.target.value)}
            />
            <div className="topic-list">
              <table>
                <thead>
                  <tr>
                    <th>관측</th>
                    <th>Topic / Type</th>
                    <th>수신 Hz</th>
                    <th>상태</th>
                  </tr>
                </thead>
                <tbody>
                  {topics
                    .filter((t) =>
                      (t.name + " " + t.types.join(" "))
                        .toLowerCase()
                        .includes(query.toLowerCase()),
                    )
                    .map((t) => (
                      <tr
                        key={t.name}
                        className={topic === t.name ? "selected" : ""}
                        onClick={() => setTopic(t.name)}
                      >
                        <td>
                          <input
                            aria-label={`구독 ${t.name}`}
                            type="checkbox"
                            disabled={replay || t.fixed}
                            checked={t.fixed || selected.includes(t.name)}
                            onClick={(e) => e.stopPropagation()}
                            onChange={(e) =>
                              select(
                                e.target.checked
                                  ? [...selected, t.name]
                                  : selected.filter((n) => n !== t.name),
                              )
                            }
                          />
                        </td>
                        <td>
                          <strong>{t.name}</strong>
                          <small>{t.types.join(", ")}</small>
                        </td>
                        <td>{t.received_hz?.toFixed(1) ?? "미측정"}</td>
                        <td>
                          <Badge
                            tone={
                              t.state === "receiving"
                                ? "good"
                                : t.state === "error"
                                  ? "bad"
                                  : ""
                            }
                          >
                            {t.state}
                          </Badge>
                        </td>
                      </tr>
                    ))}
                </tbody>
              </table>
            </div>
          </Panel>
          <Panel
            title="Message inspector"
            aside={item && <Badge>{item.state}</Badge>}
          >
            {item ? (
              <>
                <code className="topic-name">{topic}</code>
                {item.error && <div className="notice bad">{item.error}</div>}
                <Json
                  value={
                    sample?.data ||
                    rows
                      .filter((r) => r.key === topic)
                      .sort((a, b) => b.seq - a.seq)[0]?.data ||
                    "메시지 없음"
                  }
                />
                <details open>
                  <summary>Endpoints & QoS</summary>
                  <Json
                    value={{
                      publishers: item.publishers,
                      subscribers: item.subscribers,
                    }}
                  />
                </details>
              </>
            ) : (
              <Empty>토픽 선택</Empty>
            )}
          </Panel>
        </div>
      ) : view === "graph" ? (
        <Panel
          title="Publisher → Topic → Subscriber"
          aside={
            <span>
              {topic || "앞 12개 토픽"}{" "}
              <Button variant="outline" onClick={() => setTopic("")}>
                선택 해제
              </Button>
            </span>
          }
        >
          <div className="flow large">
            <ReactFlow
              {...connections}
              fitView
              nodesDraggable={false}
              nodesConnectable={false}
            >
              <Background color="var(--graph-dot)" />
              <Controls showInteractive={false} />
            </ReactFlow>
          </div>
        </Panel>
      ) : (
        <Panel title={view}>
          <Json value={graph[view] || []} />
        </Panel>
      )}
      {graph.missing_fixed_topics?.length > 0 && (
        <details className="missing">
          <summary>미발견 토픽 · {graph.missing_fixed_topics.length}</summary>
          <Json value={graph.missing_fixed_topics} />
        </details>
      )}
    </>
  );
}

function Logs({
  rows,
  filter,
  setFilter,
}: {
  rows: Row[];
  filter: string;
  setFilter: (s: string) => void;
}) {
  const [level, setLevel] = useState(0);
  const logs = rows
    .filter(
      (r) =>
        critical.has(r.kind) &&
        pretty(r.data).toLowerCase().includes(filter.toLowerCase()) &&
        (r.kind !== "log" || (r.data.level || 0) >= level),
    )
    .sort((a, b) => b.seq - a.seq);
  return (
    <Panel title="이벤트 타임라인" aside={<span>{logs.length}개</span>}>
      <div className="filters">
        <Input
          placeholder="로그 검색"
          value={filter}
          onChange={(e) => setFilter(e.target.value)}
        />
        <select value={level} onChange={(e) => setLevel(+e.target.value)}>
          <option value={0}>전체 수준</option>
          <option value={20}>INFO 이상</option>
          <option value={30}>WARN 이상</option>
          <option value={40}>ERROR 이상</option>
        </select>
      </div>
      <div className="logs">
        {logs.map((r) => (
          <div className="log-row" key={r.seq}>
            <div>
              <time>{time(r.received_at)}</time>
              <small>ROS {r.ros_time?.toFixed(3) ?? "—"}</small>
            </div>
            <Badge tone={r.data.level >= 40 ? "bad" : ""}>
              {r.data.kind || r.kind}
            </Badge>
            <div>
              <strong>{r.data.name || r.data.mission_id || r.key}</strong>
              <span>{r.data.msg || pretty(r.data.data || r.data)}</span>
            </div>
          </div>
        ))}
        {!logs.length && <Empty>결과 없음</Empty>}
      </div>
    </Panel>
  );
}

function App() {
  const [graphTab, setGraphTab] = useState("fsm");
  const [collapsed, setCollapsed] = useState(false);
  const [tab, setTab] = useState(location.hash.slice(1) || "execution");
  const [rows, setRows] = useState<Row[]>([]);
  const live = useRef<Row[]>([]);
  const [connected, setConnected] = useState(false);
  const [now, setNow] = useState(Date.now() / 1000);
  const [notice, setNotice] = useState("");
  const [selected, setSelected] = useState<string[]>([]);
  const selectedRef = useRef<string[]>([]);
  const socket = useRef<WebSocket | null>(null);
  const lastBoot = useRef("");
  const lastSequence = useRef(0);
  const [recording, setRecording] = useState<Data>({});
  const [sessions, setSessions] = useState<Data[]>([]);
  const [replayId, setReplayId] = useState("");
  const replayRef = useRef("");
  const [at, setAt] = useState(0);
  const seekPending = useRef(false);
  const [seekRevision, setSeekRevision] = useState(0);
  const [playing, setPlaying] = useState(false);
  const [filter, setFilter] = useState("");
  const [node, setNode] = useState("");
  const [error, setError] = useState("");
  useEffect(() => {
    let stopped = false;
    let retry: ReturnType<typeof setTimeout>;
    let ws: WebSocket;
    const connect = () => {
      ws = new WebSocket(
        `${location.protocol === "https:" ? "wss" : "ws"}://${location.host}/ws`,
      );
      socket.current = ws;
      ws.onopen = () => {
        setConnected(true);
        if (selectedRef.current.length)
          ws.send(
            JSON.stringify({ op: "select", topics: selectedRef.current }),
          );
      };
      ws.onclose = () => {
        setConnected(false);
        if (!stopped) retry = setTimeout(connect, 1500);
      };
      ws.onmessage = (e) => {
        const p: Packet = JSON.parse(e.data);
        if (p.error) {
          setError(p.error);
          return;
        }
        if (p.selection) {
          selectedRef.current = p.selection;
          setSelected(p.selection);
          setError("");
          return;
        }
        if (!p.rows) return;
        if (p.gap) setNotice("이벤트 유실");
        if (p.initial && lastBoot.current) {
          setNotice(
            p.boot_id === lastBoot.current
              ? "재연결 · 누락 가능"
              : "Monitor 재시작",
          );
        }
        if (p.boot_id !== lastBoot.current) {
          live.current = [];
          lastSequence.current = 0;
        }
        lastBoot.current = p.boot_id || "";
        const merged = p.initial ? [...p.rows] : [...live.current, ...p.rows];
        const latest = new Map<string, Row>();
        const events = new Map<number, Row>();
        for (const r of merged) {
          if (critical.has(r.kind)) events.set(r.seq, r);
          else latest.set(r.kind + ":" + r.key, r);
        }
        const topicGraph = latest.get("graph:ros")?.data.topics || [];
        for (const [key, row] of latest) {
          if (
            row.kind === "sample" &&
            !topicGraph.some(
              (topic: Data) =>
                topic.name === row.key && topic.state !== "unmeasured",
            )
          )
            latest.delete(key);
        }
        live.current = [
          ...latest.values(),
          ...[...events.values()].sort((a, b) => a.seq - b.seq).slice(-500),
        ];
        lastSequence.current = p.sequence || 0;
        if (!replayRef.current) setRows(applyRuntimeEvents(live.current));
        if (p.recording) setRecording(p.recording);
      };
    };
    connect();
    const clock = setInterval(() => setNow(Date.now() / 1000), 500);
    return () => {
      stopped = true;
      clearTimeout(retry);
      clearInterval(clock);
      ws.close();
    };
  }, []);
  const refreshSessions = () =>
    fetch("/api/recordings")
      .then((r) => r.json())
      .then((d) => {
        setSessions(d.sessions);
        setRecording({ id: d.active, reason: d.reason });
      })
      .catch((e) => setError(String(e)));
  useEffect(() => {
    if (tab === "recordings") refreshSessions();
  }, [tab]);
  useEffect(() => {
    replayRef.current = replayId;
    if (!replayId) {
      setRows(applyRuntimeEvents(live.current));
      setPlaying(false);
      return;
    }
    const controller = new AbortController();
    fetch(`/api/recordings/${replayId}/replay?at=${at}`, {
      signal: controller.signal,
    })
      .then((r) => {
        if (!r.ok) throw Error("기록 읽기 실패");
        return r.json();
      })
      .then((p) => {
        setRows(applyRuntimeEvents(p.rows));
        if (seekPending.current) {
          seekPending.current = false;
          setSeekRevision((value) => value + 1);
        }
        if (p.metadata.gap) setNotice(p.metadata.gap);
      })
      .catch((e) => {
        if (e.name !== "AbortError") setError(String(e));
      });
    return () => controller.abort();
  }, [replayId, at]);
  const session = sessions.find((s) => s.id === replayId);
  useEffect(() => {
    if (!playing || !session) return;
    const timer = setInterval(
      () =>
        setAt((t) => {
          if (t + 0.2 >= session.end) {
            setPlaying(false);
            return session.end;
          }
          return t + 0.2;
        }),
      200,
    );
    return () => clearInterval(timer);
  }, [playing, session]);
  const go = (id: string) => {
    setTab(id);
    location.hash = id;
  };
  const runtimeRow = rows.find((r) => r.kind === "runtime");
  const runtime = runtimeRow?.data || {};
  const graph = rows.find((r) => r.kind === "graph")?.data || {};
  const config = rows.find((r) => r.kind === "config")?.data || {};
  const age = runtimeRow
    ? Math.max(0, (replayId ? at : now) - runtimeRow.received_at)
    : null;
  const fresh = age !== null && age < 3 && (!!replayId || connected);
  const select = (topics: string[]) => {
    if (socket.current?.readyState === WebSocket.OPEN)
      socket.current.send(JSON.stringify({ op: "select", topics }));
  };
  const [focusTopic, setFocusTopic] = useState("");
  const moduleName =
    (
      {
        ObserveBefore: "perception",
        Reobserve: "perception",
        PlanNext: "planning",
        ValidateProposal: "planning",
        ExecuteOne: "execution",
        NAVIGATE_TO_TARGET: "navigation",
        RETURN_HOME: "navigation",
      } as Record<string, string>
    )[node] || node;
  const moduleInfo = runtime.modules?.[moduleName] || {};
  const clockRow = rows.find((r) => r.kind === "clock");
  const correlated = rows
    .filter((r) => critical.has(r.kind) && pretty(r.data).includes(node))
    .slice(-12)
    .reverse();
  return (
    <div className={"app " + (collapsed ? "sidebar-collapsed" : "")}>
      <aside className="sidebar">
        <div className="brand">
          <span className="brand-icon">
            <Bot size={24} />
          </span>
          <div>
            cleany<span className="brand-subtitle">Developer console</span>
          </div>
        </div>
        <nav>
          {tabs.map(([id, label], index) => {
            const Icon = navIcons[index];
            return (
              <Button
                variant="ghost"
                key={id}
                onClick={() => go(id)}
                className={tab === id ? "active" : ""}
                aria-label={label}
                aria-current={tab === id ? "page" : undefined}
                title={label}
              >
                <Icon size={18} />
                <span className="nav-label">{label}</span>
              </Button>
            );
          })}
        </nav>
        <div className="sidebar-bottom">
          <Separator />
          <Button
            variant="ghost"
            className="collapse-button"
            aria-label={collapsed ? "사이드바 펼치기" : "사이드바 접기"}
            onClick={() => setCollapsed(!collapsed)}
          >
            {collapsed ? (
              <PanelLeftOpen size={17} />
            ) : (
              <>
                <PanelLeftClose size={17} />
                <span>사이드바 접기</span>
              </>
            )}
          </Button>
        </div>
      </aside>
      <main>
        <header>
          <div className="breadcrumb">읽기 전용</div>
          <div className="header-status">
            <ThemeToggle />
            <Badge tone={replayId ? "info" : connected ? "good" : "bad"}>
              {replayId ? "REPLAY" : connected ? "LIVE" : "연결 끊김"}
            </Badge>
            <span>Domain {config.domain ?? "—"}</span>
            {clockRow && (
              <span title="ROS clock">
                sim {clockRow.data.time.toFixed(1)}s{" "}
                {clockRow.data.unchanged_for > 1 ||
                (!replayId && now - clockRow.received_at > 2)
                  ? "· 정지/수신 지연"
                  : ""}
              </span>
            )}
            <code>{config.runtime_namespace || "/"}</code>
            <span>{time(now)}</span>
          </div>
        </header>
        <div className="content">
          <div className="page-title">
            <div>
              <h1>{tabs.find((t) => t[0] === tab)?.[1]}</h1>
            </div>
            <div className="page-actions">
              <Button variant="outline" asChild>
                <a
                  href="https://app.foxglove.dev"
                  target="_blank"
                  rel="noreferrer"
                >
                  <Activity size={16} />
                  Foxglove
                  <ArrowUpRight size={14} />
                </a>
              </Button>
            </div>
          </div>
          {!connected && !replayId && (
            <div className="connection-banner" role="status">
              <Unplug size={19} />
              <div>
                <strong>서버 연결 끊김 · 재연결 중</strong>
              </div>
            </div>
          )}
          {notice && (
            <div className="notice warn">
              {notice}
              <Button variant="outline" onClick={() => setNotice("")}>
                닫기
              </Button>
            </div>
          )}
          {error && (
            <div className="notice bad">
              {error}
              <Button variant="outline" onClick={() => setError("")}>
                닫기
              </Button>
            </div>
          )}
          {recording.reason && (
            <div className="notice warn">기록 중단 · {recording.reason}</div>
          )}
          {replayId && (
            <div className="replay-bar">
              <strong>기록 재생</strong>
              <span>{time(at)}</span>
              <Button variant="outline" onClick={() => setPlaying(!playing)}>
                {playing ? "일시정지" : "재생"}
              </Button>
              <input
                aria-label="재생 시각"
                type="range"
                min={session?.start || 0}
                max={session?.end || 0}
                step="any"
                value={at}
                onChange={(e) => {
                  seekPending.current = true;
                  setAt(+e.target.value);
                }}
              />
              <Button variant="outline" onClick={() => setReplayId("")}>
                실시간으로
              </Button>
            </div>
          )}
          {tab === "execution" &&
            (!runtimeRow ? (
              <div className="runtime-wait">
                <Radio size={24} />
                <strong>
                  {replayId
                    ? session?.runtime_count === 0
                      ? "Runtime 기록 없음"
                      : "이 시점의 Runtime 기록 없음"
                    : "Runtime 대기"}
                </strong>
              </div>
            ) : (
              <>
                <div className="execution-summary">
                  <div>
                    <span>상태</span>
                    <Badge
                      tone={
                        !fresh
                          ? "warn"
                          : runtime.state === "ERROR"
                            ? "bad"
                            : "info"
                      }
                    >
                      {runtime.state || "UNKNOWN"}
                    </Badge>
                  </div>
                  <div className="mission-summary">
                    <span>미션</span>
                    <code title={runtime.active_request?.mission_id}>
                      {runtime.active_request?.mission_id || "—"}
                    </code>
                  </div>
                  <div>
                    <span>목표</span>
                    <strong>{runtime.active_request?.target_id || "—"}</strong>
                  </div>
                  <div>
                    <span>주행</span>
                    <strong>
                      {!fresh
                        ? "확인 불가"
                        : runtime.safe_to_drive
                          ? "주행 조건 충족"
                          : "주행 차단"}
                    </strong>
                  </div>
                  {runtime.bt_stage && (
                    <div>
                      <span>청소</span>
                      <strong>{runtime.bt_stage}</strong>
                    </div>
                  )}
                </div>
                {(!fresh || runtime.drive_block_reason || runtime.reason) && (
                  <p className="execution-reason">
                    {!fresh
                      ? `상태 미확인 · ${age?.toFixed(1) ?? "—"}초 전`
                      : runtime.drive_block_reason || runtime.reason}
                  </p>
                )}
                <div
                  className={
                    "execution-workspace " + (node ? "with-detail" : "")
                  }
                >
                  <section
                    className="execution-canvas"
                    aria-label="실행 그래프"
                  >
                    <div className="graph-toolbar">
                      <Tabs
                        value={graphTab}
                        onValueChange={(value) => {
                          setGraphTab(value);
                          setNode("");
                        }}
                      >
                        <TabsList>
                          <TabsTrigger value="fsm">FSM</TabsTrigger>
                          <TabsTrigger value="bt">청소 BT</TabsTrigger>
                        </TabsList>
                      </Tabs>
                      <Badge tone={!fresh ? "warn" : ""}>
                        {!fresh
                          ? "상태 미확인"
                          : replayId
                            ? "기록 시점"
                            : "LIVE"}
                      </Badge>
                    </div>
                    <h2 className="sr-only">
                      {graphTab === "fsm"
                        ? "Mission FSM"
                        : "Cleaning Behavior Tree"}
                    </h2>
                    <Tree
                      key={`${graphTab}:${replayId}:${seekRevision}:${runtime.boot_id ?? ""}`}
                      runtime={runtime}
                      onSelect={setNode}
                      bt={graphTab === "bt"}
                    />
                  </section>
                  {node && (
                    <section
                      className="execution-detail"
                      aria-label="노드 상세"
                    >
                      <div className="detail-title">
                        <h2>{node}</h2>
                        <Button
                          variant="ghost"
                          size="sm"
                          onClick={() => setNode("")}
                          aria-label="상세 닫기"
                        >
                          닫기
                        </Button>
                      </div>
                      <dl className="facts">
                        <div>
                          <dt>작업 대상</dt>
                          <dd>{runtime.proposal?.object_id || "—"}</dd>
                        </div>
                        <div>
                          <dt>조작</dt>
                          <dd>
                            {runtime.manipulation?.status ||
                              runtime.manipulation?.outcome ||
                              "—"}
                          </dd>
                        </div>
                      </dl>
                      <div className="detail-section">
                        <h3>모듈</h3>
                        {Object.entries(runtime.modules || {}).map(
                          ([name, value]) => {
                            const v = value as Data;
                            return (
                              <div className="module-row" key={name}>
                                <div>
                                  <strong>{name}</strong>
                                  <Badge>
                                    {runtime.execution_profile?.[name] ||
                                      "unknown"}
                                  </Badge>
                                </div>
                                <small>
                                  {!fresh
                                    ? "상태 미확인"
                                    : v.ready
                                      ? "Ready"
                                      : v.reason || "Not ready"}{" "}
                                  · 정지{" "}
                                  {v.stopped == null
                                    ? "미확인"
                                    : String(v.stopped)}
                                </small>
                              </div>
                            );
                          },
                        )}
                      </div>
                      <details>
                        <summary>원본</summary>
                        <Json
                          value={{
                            proposal: runtime.proposal || null,
                            manipulation: runtime.manipulation || null,
                            navigation: runtime.navigation || null,
                          }}
                        />
                      </details>
                      <div className="detail-section">
                        <div className="detail-title">
                          <h3>실행 기록</h3>
                          <Button
                            variant="outline"
                            size="sm"
                            onClick={() => {
                              setFilter(node);
                              go("logs");
                            }}
                          >
                            로그 →
                          </Button>
                        </div>
                        <div className="related-topics">
                          {(moduleInfo.topics || []).map((topic: string) => (
                            <Button
                              variant="outline"
                              key={topic}
                              onClick={() => {
                                setFocusTopic(topic);
                                go("ros");
                              }}
                            >
                              {topic} →
                            </Button>
                          ))}
                          {(moduleInfo.actions || []).map((action: string) => (
                            <small key={action}>Action: {action}</small>
                          ))}
                        </div>
                        {correlated.map((r) => (
                          <div className="mini-event" key={r.seq}>
                            <small>
                              {time(r.received_at)} · {r.data.kind || r.kind}
                            </small>
                            <Json value={r.data.data || r.data} />
                          </div>
                        ))}
                        {!correlated.length && <small>이벤트 없음</small>}
                      </div>
                    </section>
                  )}
                </div>
                {runtime.last_result && (
                  <details className="execution-result">
                    <summary>
                      마지막 결과 <Badge>{runtime.last_result.outcome}</Badge>
                    </summary>
                    <Json value={runtime.last_result} />
                  </details>
                )}
              </>
            ))}
          {tab === "camera" && (
            <CameraView topics={graph.topics || []} replay={!!replayId} />
          )}
          {tab === "ros" && (
            <RosView
              rows={rows}
              graph={graph}
              selected={selected}
              select={select}
              replay={!!replayId}
              focusTopic={focusTopic}
            />
          )}
          {tab === "navigation" && (
            <>
              <FoxglovePanel />
              <Navigation
                rows={rows}
                config={config}
                timeAt={replayId ? at : now}
              />
            </>
          )}
          {tab === "logs" && (
            <Logs rows={rows} filter={filter} setFilter={setFilter} />
          )}
          {tab === "recordings" && (
            <>
              <Panel
                title="저장된 관측 세션"
                aside={
                  <Button variant="outline" onClick={refreshSessions}>
                    새로고침
                  </Button>
                }
              >
                <table>
                  <thead>
                    <tr>
                      <th>세션</th>
                      <th>시작 / 종료</th>
                      <th>레코드</th>
                      <th>FSM</th>
                      <th>크기</th>
                      <th>열기</th>
                    </tr>
                  </thead>
                  <tbody>
                    {sessions.map((s) => (
                      <tr key={s.id}>
                        <td>
                          <code>{s.id.slice(0, 8)}</code>{" "}
                          {s.id === recording.id && (
                            <Badge tone="good">
                              기록 중{recording.reason ? "단" : ""}
                            </Badge>
                          )}
                          {s.metadata.gap && (
                            <Badge tone="warn">유실 있음</Badge>
                          )}
                        </td>
                        <td>
                          {s.start ? time(s.start) : "—"} →{" "}
                          {s.end ? time(s.end) : "—"}
                        </td>
                        <td>{s.count}</td>
                        <td>
                          {s.runtime_count == null
                            ? "—"
                            : s.runtime_count > 0
                              ? "있음"
                              : "없음"}
                        </td>
                        <td>{(s.bytes / 1024 ** 2).toFixed(2)} MiB</td>
                        <td>
                          <Button
                            variant="outline"
                            disabled={!s.count}
                            onClick={() => {
                              setReplayId(s.id);
                              setAt(s.runtime_start ?? s.start);
                              setPlaying(true);
                              go("execution");
                            }}
                          >
                            재생
                          </Button>{" "}
                          <a href={`/api/recordings/${s.id}/export`}>
                            내보내기 ↓
                          </a>
                        </td>
                      </tr>
                    ))}
                  </tbody>
                </table>
                {!sessions.length && <Empty>기록 없음</Empty>}
              </Panel>
            </>
          )}
        </div>
      </main>
    </div>
  );
}
createRoot(document.getElementById("root")!).render(
  <TooltipProvider>
    <App />
  </TooltipProvider>,
);
