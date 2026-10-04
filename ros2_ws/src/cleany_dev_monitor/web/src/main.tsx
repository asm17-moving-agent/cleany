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
import { NavigationDashboard } from "@/components/navigation-dashboard";
import { CameraView } from "@/components/camera-view";
import { ThemeToggle } from "@/components/theme-toggle";
import "./theme.css";
import "./style.css";
import { Button } from "@/components/ui/button";
import { Badge as UiBadge } from "@/components/ui/badge";
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
import { Panel } from "@/components/panel";
import { type Data, type Row, time } from "@/lib/monitor";

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

function NavigationMap({
  rows,
  config,
  timeAt,
  mode = "overview",
}: {
  rows: Row[];
  config: Data;
  timeAt: number;
  mode?: string;
}) {
  const canvas = useRef<HTMLCanvasElement>(null);
  const projection = useRef({ ox: 0, oy: 0, scale: 1 });
  const [picked, setPicked] = useState<number[] | null>(null);
  const mapViews: Record<
    string,
    { title: string; layers: string[]; defaults: string[] }
  > = {
    overview: {
      title: "전체 경로와 주행 진행",
      layers: ["map", "path", "trail", "footprint"],
      defaults: ["map", "path", "trail", "footprint"],
    },
    safety: {
      title: "주변 장애물과 안전 영역",
      layers: [
        "map",
        "local_costmap",
        "scan",
        "depth_points",
        "footprint",
        "stop_polygon",
        "slow_polygon",
        "risk_points",
        "obstacle_memory",
        "lidar_memory",
        "depth_memory",
      ],
      defaults: [
        "map",
        "local_costmap",
        "scan",
        "depth_points",
        "footprint",
        "stop_polygon",
        "slow_polygon",
        "risk_points",
      ],
    },
    control: {
      title: "계획 경로와 후보 궤적",
      layers: [
        "map",
        "path",
        "local_path",
        "local_costmap",
        "global_costmap",
        "footprint",
        "trail",
      ],
      defaults: ["map", "path", "local_path", "local_costmap", "footprint"],
    },
  };
  const view = mapViews[mode] || mapViews.overview;
  const [enabled, setEnabled] = useState<Record<string, boolean>>(() =>
    Object.fromEntries(
      view.layers.map((key) => [key, view.defaults.includes(key)]),
    ),
  );
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
    const nearby = mode === "safety" || mode === "control";
    const localBounds = layers.find(
      (r) => r.key === "local_costmap" && r.data.valid,
    )?.data;
    if (nearby && robot?.valid) {
      const radius = localBounds
        ? Math.max(
            1,
            Math.min(
              4,
              (Math.max(localBounds.width, localBounds.height) *
                localBounds.resolution) /
                2,
            ),
          )
        : 2.5;
      points.push(
        [robot.pose[0] - radius, robot.pose[1] - radius],
        [robot.pose[0] + radius, robot.pose[1] + radius],
      );
    } else if (nearby && localBounds) {
      for (const [x, y] of [
        [0, 0],
        [localBounds.width, 0],
        [0, localBounds.height],
        [localBounds.width, localBounds.height],
      ])
        points.push(
          world(
            x * localBounds.resolution,
            y * localBounds.resolution,
            localBounds.origin,
          ),
        );
    } else if (map)
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
    projection.current = { ox, oy, scale };
    const screen = (x: number, y: number) => [ox + x * scale, oy - y * scale];
    for (const name of [
      "map",
      "obstacle_memory",
      "lidar_memory",
      "depth_memory",
      "global_costmap",
      "local_costmap",
    ]) {
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
    const summary = rows.find(
      (r) => r.kind === "navigation" && r.key === "summary",
    )?.data;
    const drawLine = (points: number[][], color: string, closed = false) => {
      if (!points.length) return;
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
    if (enabled.trail && Array.isArray(summary?.trail))
      drawLine(
        summary.trail.map((p: number[]) => p.slice(1, 3)),
        "#80aaff",
      );
    const local = layers.find((r) => r.key === "local_path")?.data;
    if (enabled.local_path && local?.valid && Array.isArray(local.points))
      drawLine(local.points, "#ffb85c");
    const colors: Record<string, string> = {
      scan: "#5ed7ee",
      depth_points: "#bf95ef",
      risk_points: "#ff5252",
      footprint: "#f8f8f8",
      stop_polygon: "#ff7272",
      slow_polygon: "#ffce70",
    };
    for (const key of Object.keys(colors)) {
      const row = rows.find((r) => r.kind === "navigation" && r.key === key);
      const d = row?.data;
      if (
        !enabled[key] ||
        !d?.valid ||
        !Array.isArray(d.points) ||
        !row ||
        timeAt - row.received_at > 2
      )
        continue;
      if (["footprint", "stop_polygon", "slow_polygon"].includes(key))
        drawLine(d.points, colors[key], true);
      else {
        ctx.fillStyle = colors[key];
        for (const [x, y] of d.points) {
          const [a, b] = screen(x, y);
          ctx.fillRect(a - 1, b - 1, 3, 3);
        }
      }
    }
    if (mode === "control") {
      const evaluation = rows.find(
        (r) => r.kind === "navigation" && r.key === "controller_evaluation",
      );
      const e = evaluation?.data;
      if (
        evaluation &&
        timeAt - evaluation.received_at <= 2 &&
        Array.isArray(e?.trajectories) &&
        e?.transform
      )
        for (const t of e.trajectories)
          if (Array.isArray(t.points))
            drawLine(
              t.points.map((p: number[]) => world(p[0], p[1], e.transform)),
              t.selected ? "#4ae7ba" : "#737c87",
            );
    }
    const goal = navigation?.goal_pose;
    if (mode !== "safety" && goal && navigation.frame === config.map_frame) {
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
      const arrow = (angle: number, color: string, length: number) => {
        ctx.save();
        ctx.translate(x, y);
        ctx.rotate(-angle);
        ctx.strokeStyle = color;
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.moveTo(0, 0);
        ctx.lineTo(length, 0);
        ctx.lineTo(length - 7, -4);
        ctx.moveTo(length, 0);
        ctx.lineTo(length - 7, 4);
        ctx.stroke();
        ctx.restore();
      };
      const panRow = rows.find(
        (r) => r.kind === "navigation" && r.key === "pan",
      );
      if (
        mode === "safety" &&
        panRow &&
        timeAt - panRow.received_at <= 2 &&
        Number.isFinite(panRow.data.pan)
      )
        arrow(robot.pose[2] + panRow.data.pan, "#d69aff", 45);
      if (
        summary?.measured &&
        Math.hypot(...(summary.measured.slice(0, 2) as [number, number])) >
          0.005
      )
        arrow(
          robot.pose[2] + Math.atan2(summary.measured[1], summary.measured[0]),
          "#5ed7ee",
          35,
        );
    }
    ctx.fillStyle = "#b8b8b8";
    ctx.font = "13px monospace";
    ctx.fillText(
      `frame: ${config.map_frame || "map"} · ${zoom.toFixed(1)}×`,
      20,
      height - 20,
    );
  }, [rows, enabled, zoom, Math.floor(poseAge), timeAt, mode]);
  return (
    <>
      <Panel
        title={view.title}
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
          {view.layers.map((k) => (
            <label key={k}>
              <input
                type="checkbox"
                checked={enabled[k]}
                onChange={(e) =>
                  setEnabled({ ...enabled, [k]: e.target.checked })
                }
              />
              {(
                {
                  map: "지도",
                  path: "계획 경로",
                  local_costmap: "local costmap",
                  global_costmap: "global costmap",
                  local_path: "제어 경로",
                  scan: "LiDAR",
                  depth_points: "Depth 장애물",
                  footprint: "로봇 외곽",
                  trail: "실제 궤적",
                  stop_polygon: "정지 영역",
                  slow_polygon: "감속 영역",
                  risk_points: "위험점",
                  obstacle_memory: "합성 메모리",
                  lidar_memory: "LiDAR 메모리",
                  depth_memory: "Depth 메모리",
                } as Record<string, string>
              )[k] || k}
            </label>
          ))}
          <span>
            {mode === "safety"
              ? "△ 차체 · 빨강 정지 영역 · 노랑 감속 영역 · 보라 카메라"
              : mode === "control"
                ? "△ 차체 · 청록 선택 후보 · 회색 다른 후보 · 주황 제어 경로"
                : "△ 차체 · ＋ 목표 · 파랑 실제 궤적 · 초록 계획 경로"}
          </span>
        </div>
        {(mode === "safety" || mode === "control") &&
          !robot?.valid &&
          !layers.some((r) => r.key === "local_costmap" && r.data.valid) && (
            <small>
              주변 범위를 결정할 위치·local costmap이 없어 전체 지도를
              표시합니다.
            </small>
          )}
        {!layers.some((r) => r.key === "map" && r.data.valid) && (
          <p className="nav-map-unavailable">
            배경 지도 표시 불가 ·{" "}
            {layers.find((r) => r.key === "map")?.data.reason ||
              "지도 토픽 미수신"}
          </p>
        )}
        <canvas
          ref={canvas}
          aria-label="ROS 2D navigation map"
          className="map"
          onClick={(event) => {
            const c = canvas.current;
            if (!c) return;
            const rect = c.getBoundingClientRect();
            const { ox, oy, scale } = projection.current;
            setPicked([
              (((event.clientX - rect.left) * c.width) / rect.width - ox) /
                scale,
              (oy - ((event.clientY - rect.top) * c.height) / rect.height) /
                scale,
            ]);
          }}
        />
        {picked && (
          <div className="map-tools" aria-label="선택한 지도 셀">
            <span>
              map ({picked[0].toFixed(2)}, {picked[1].toFixed(2)}) · 게시
              OccupancyGrid 값
            </span>
            {layers
              .filter((r) => r.data.valid && r.data.cells && enabled[r.key])
              .map((r) => {
                const g = r.data,
                  dx = picked[0] - g.origin[0],
                  dy = picked[1] - g.origin[1];
                const x = Math.floor(
                  (Math.cos(g.origin[2]) * dx + Math.sin(g.origin[2]) * dy) /
                    g.resolution,
                );
                const y = Math.floor(
                  (-Math.sin(g.origin[2]) * dx + Math.cos(g.origin[2]) * dy) /
                    g.resolution,
                );
                const value =
                  x >= 0 && y >= 0 && x < g.width && y < g.height
                    ? g.cells[y * g.width + x]
                    : null;
                return (
                  <span key={r.key}>
                    {r.key}:{" "}
                    {value === null ? "영역 밖" : value < 0 ? "미관측" : value}
                  </span>
                );
              })}
          </div>
        )}
        <details className="nav-map-sources">
          <summary>지도 수신 상태와 출처</summary>
          <div className="layer-status">
            {view.layers
              .filter((k) =>
                [
                  "map",
                  "path",
                  "local_path",
                  "local_costmap",
                  "global_costmap",
                ].includes(k),
              )
              .map((k) => {
                const l = layers.find((r) => r.key === k);
                return (
                  <div key={k}>
                    <b>{k}</b>
                    <Badge tone={l?.data.valid ? "good" : "warn"}>
                      {l?.data.valid ? "수신" : "표시 불가"}
                    </Badge>
                    <small>
                      {l?.data.reason || l?.data.topic || "토픽 대기"}
                    </small>
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
        </details>
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
  const [navigationTab, setNavigationTab] = useState("overview");
  const [collapsed, setCollapsed] = useState(false);
  const [tab, setTab] = useState(location.hash.slice(1) || "execution");
  const [rows, setRows] = useState<Row[]>([]);
  const live = useRef<Row[]>([]);
  const [connected, setConnected] = useState(false);
  const [now, setNow] = useState(Date.now() / 1000);
  const [notice, setNotice] = useState("");
  const [selected, setSelected] = useState<string[]>([]);
  const selectedRef = useRef<string[]>([]);
  const navigationTopicsRef = useRef<string[]>([]);
  const [navigationTopics, setNavigationTopics] = useState<string[]>([]);
  const socket = useRef<WebSocket | null>(null);
  const lastBoot = useRef("");
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
        if (
          !replayRef.current &&
          (selectedRef.current.length || navigationTopicsRef.current.length)
        )
          ws.send(
            JSON.stringify({
              op: "select",
              topics: [
                ...new Set([
                  ...selectedRef.current,
                  ...navigationTopicsRef.current,
                ]),
              ],
            }),
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
          const manual = p.selection.filter(
            (t) =>
              selectedRef.current.includes(t) ||
              !navigationTopicsRef.current.includes(t),
          );
          selectedRef.current = manual;
          setSelected(manual);
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
        if (p.boot_id !== lastBoot.current) live.current = [];
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
  const selectionLimit = Number(config.max_selected_topics) || 16;
  const select = (topics: string[]) => {
    if (topics.length > selectionLimit) {
      setError(`선택 토픽 한도 ${selectionLimit}개를 초과했습니다.`);
      return;
    }
    selectedRef.current = topics;
    setSelected(topics);
    const auto = tab === "navigation" && !replayId ? navigationTopics : [];
    const merged = [...new Set([...topics, ...auto])].slice(0, selectionLimit);
    navigationTopicsRef.current = auto.filter((t) => merged.includes(t));
    if (!replayId && socket.current?.readyState === WebSocket.OPEN)
      socket.current.send(JSON.stringify({ op: "select", topics: merged }));
  };
  const navigationSelection =
    tab === "navigation" && !replayId ? navigationTopics : [];
  const navigationSelectionKey = JSON.stringify(navigationSelection);
  useEffect(() => {
    const topics: string[] = JSON.parse(navigationSelectionKey);
    const merged = [...new Set([...selectedRef.current, ...topics])].slice(
      0,
      selectionLimit,
    );
    const automatic = topics.filter((t) => merged.includes(t));
    if (automatic.length < topics.length)
      setNotice("선택 토픽 한도 때문에 일부 주행 레이어 구독을 생략했습니다.");
    if (
      JSON.stringify(navigationTopicsRef.current) === JSON.stringify(automatic)
    )
      return;
    navigationTopicsRef.current = automatic;
    if (socket.current?.readyState === WebSocket.OPEN)
      socket.current.send(JSON.stringify({ op: "select", topics: merged }));
  }, [navigationSelectionKey, selectionLimit]);
  const openNavigationMoment = async (moment: number) => {
    setPlaying(false);
    if (replayId) {
      seekPending.current = true;
      setAt(moment);
      return;
    }
    try {
      const response = await fetch("/api/recordings");
      const data = await response.json();
      const source = data.sessions.find(
        (s: Data) => s.id === (data.active || recording.id),
      );
      if (!source || moment < source.start || moment > source.end)
        throw Error(
          "해당 시점이 아직 기록되지 않았거나 기록이 중단되었습니다.",
        );
      setSessions(data.sessions);
      setReplayId(source.id);
      setAt(moment);
      seekPending.current = true;
    } catch (e) {
      setError(String(e));
    }
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
              <NavigationDashboard
                rows={rows}
                config={config}
                graph={graph}
                timeAt={replayId ? at : now}
                replay={!!replayId}
                sessionId={replayId || recording.id || "current"}
                tab={navigationTab}
                setTab={setNavigationTab}
                map={
                  <NavigationMap
                    key={navigationTab}
                    rows={rows}
                    config={config}
                    timeAt={replayId ? at : now}
                    mode={navigationTab}
                  />
                }
                observe={setNavigationTopics}
                openTopic={(topic) => {
                  setFocusTopic(topic);
                  go("ros");
                }}
                openCamera={() => go("camera")}
                openLogs={() => go("logs")}
                seek={openNavigationMoment}
                openRun={async (run) => {
                  if (!run.session_id) {
                    await openNavigationMoment(run.start);
                    return;
                  }
                  const data = await (await fetch("/api/recordings")).json();
                  setSessions(data.sessions);
                  setPlaying(false);
                  setReplayId(run.session_id);
                  setAt(run.start);
                  seekPending.current = true;
                }}
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
