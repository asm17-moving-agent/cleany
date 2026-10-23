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
  type Node,
  type Edge,
} from "@xyflow/react";
import "@xyflow/react/dist/style.css";
import {
  RecordingControls,
  RecordingReplayNotice,
} from "@/components/recording-controls";
import {
  type TimelineSession,
  RecordingTimeline,
  recordingSegments,
  sessionAt,
} from "@/components/recording-timeline";
import { NavigationDashboard } from "@/components/navigation-dashboard";
import { ManipulationDashboard } from "@/components/manipulation-dashboard";
import { ExecuteOneDetail } from "@/components/execute-one-detail";
import { RosGraph } from "@/components/ros-graph";
import { FitGraphToViewport } from "@/components/fit-graph-to-viewport";
import { CameraView } from "@/components/camera-view";
import { NavigationMap } from "@/components/navigation-map";
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
  Hand,
  Logs as LogsIcon,
  History,
  Bot,
  ArrowUpRight,
  Radio,
  Unplug,
  PanelLeftClose,
  PanelLeftOpen,
  ChevronDown,
} from "lucide-react";
import { ResourceDashboard } from "@/components/resource-dashboard";
import { Panel } from "@/components/panel";
import { type Data, type Row, time } from "@/lib/monitor";
import { applyRuntimeEvents } from "@/lib/runtime-observation";

type Packet = {
  rows: Row[];
  boot_id?: string;
  sequence?: number;
  initial?: boolean;
  sources_changed?: boolean;
  gap?: boolean;
  recording?: Data;
  error?: string;
  selection?: string[];
};
const critical = new Set(["event", "log", "result"]);
const navIcons = [
  GitBranch,
  Network,
  Navigation2,
  Hand,
  Camera,
  LogsIcon,
  History,
  Activity,
];
const tabs = [
  ["execution", "실행", "01"],
  ["ros", "ROS", "02"],
  ["navigation", "주행", "03"],
  ["manipulation", "매니퓰레이션", "04"],
  ["camera", "카메라", "05"],
  ["logs", "로그", "06"],
  ["recordings", "기록", "07"],
  ["resources", "자원", "08"],
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

const btKinds: Record<string, string> = {
  Sequence: "→ 순서 실행",
  UntilVerified: "↻ 완료까지 반복",
  OperationLeaf: "작업",
  CheckLeaf: "조건 검사",
};
const btHelp: Record<string, string> = {
  CleanDesk:
    "처음 관측한 뒤, 완료가 확인될 때까지 물체별 작업 주기를 실행합니다.",
  ObserveBefore: "청소를 시작하기 전 책상 상태를 한 번 관측합니다.",
  UntilVerified:
    "작업 주기가 SUCCESS여도 청소 완료가 확인되지 않으면 자식을 초기화하고 PlanNext부터 다시 실행합니다.",
  ActionCheckpoint:
    "제안 → 검증 → 한 물체 실행 → 재관측 → 완료 검사를 왼쪽부터 순서대로 실행합니다.",
  PlanNext: "최신 관측과 실행 기록을 바탕으로 다음 행동 하나를 제안받습니다.",
  ValidateProposal: "관측 ID, 물체, 허용 행동과 목적지가 유효한지 검사합니다.",
  ExecuteOne:
    "물체 하나의 조작을 요청하고 결과를 처리합니다. done·skip·human_review 제안은 Action 없이 통과합니다. SUCCESS는 결과 처리 통과를 뜻하며, 실제 수거 성공은 조작 결과에서 확인합니다.",
  Reobserve: "실행 이후 새 관측을 받아 책상 상태를 다시 확인합니다.",
  VerifyCompletion:
    "새 관측을 검사하고 done 제안이면 남은 물체를 확인합니다. SUCCESS여도 청소 완료가 확정되지 않으면 UntilVerified가 다음 주기를 실행합니다.",
};

function Tree({
  runtime,
  onSelect,
  bt = false,
  selectedNode = "",
}: {
  runtime: Data;
  onSelect: (id: string) => void;
  bt?: boolean;
  selectedNode?: string;
}) {
  const visibleState = useVisibleFsmState(runtime.state);
  const spec = JSON.stringify(
    bt
      ? {
          selectedNode,
          bt_nodes: (runtime.bt_nodes || []).map((n: Data) => ({
            id: n.id,
            parent: n.parent,
            kind: n.kind,
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
          전이 재생 중, 실제 {runtime.state}
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
    positions.set(id, { x, y: depth * 145 });
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
    "NAVIGATE_TO_TARGET-WORKING": "도착, 정지",
    "WORKING-POST_MISSION": "청소 완료",
    "POST_MISSION-RETURN_HOME": "복귀 정책",
    "POST_MISSION-IDLE": "복귀 생략",
    "RETURN_HOME-IDLE": "홈 도착, 정지",
    "CANCELLING-RETURN_HOME": "정지 확인, 복귀",
    "CANCELLING-IDLE": "정지 확인, 종료",
    "CANCELLING-ERROR": "안전 미확인 / 치명 오류",
    "IDLE-ERROR": "안전 오류",
    "ERROR-IDLE": "안전 확인, 리셋",
  };
  const topology = JSON.stringify(
    bt ? btNodes.map((n) => [n.id, n.parent]) : [states, runtime.fsm_edges],
  );
  const nodes: Node[] = bt
    ? btNodes.map((n) => {
        const order =
          btNodes.find((parent) => parent.id === n.parent)?.kind === "Sequence"
            ? btNodes
                .filter((child) => child.parent === n.parent)
                .findIndex((child) => child.id === n.id) + 1
            : null;
        return {
          id: n.id,
          position: positions.get(n.id)!,
          domAttributes:
            n.id === "ExecuteOne"
              ? {
                  "aria-expanded": runtime.selectedNode === n.id,
                  "aria-controls": "execute-one-detail",
                }
              : undefined,
          ariaLabel: `${n.id} · ${btKinds[n.kind] || n.kind || "BT 노드"} · ${n.status}${n.visited ? " · 최근 tick 방문" : ""}`,
          data: {
            label: (
              <>
                <span className="bt-node-kind">
                  {order && (
                    <b
                      className="bt-node-order"
                      title={`${n.parent}의 실행 순서`}
                    >
                      {order}
                    </b>
                  )}
                  {btKinds[n.kind] || n.kind || "BT 노드"}
                </span>
                <strong
                  className={
                    n.id === "ExecuteOne" ? "bt-execute-label" : undefined
                  }
                >
                  {n.id}
                  {n.id === "ExecuteOne" && (
                    <ChevronDown
                      size={14}
                      className={runtime.selectedNode === n.id ? "is-open" : ""}
                      aria-hidden="true"
                    />
                  )}
                </strong>
                <small
                  className="bt-node-status"
                  style={{ color: statusColor(n.status) }}
                >
                  {n.visited && (
                    <span className="bt-visited" title="최근 tick 방문" />
                  )}
                  {n.status}
                </small>
              </>
            ),
          },
          style: {
            boxShadow:
              runtime.selectedNode === n.id
                ? "0 0 0 3px var(--info-bg)"
                : undefined,
            borderWidth: runtime.selectedNode === n.id ? 2 : 1,
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
          style: { stroke: "var(--graph-edge)" },
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
          height: bt ? 96 : 76,
          style: { ...n.style, width: 200, height: bt ? 96 : 76 },
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
  // Rates, timestamps, QoS refreshes and endpoint discovery order are not topology.
  const graphSpec = JSON.stringify(
    (topic
      ? topics.filter((t) => t.name === topic)
      : [...topics].sort((a, b) => a.name.localeCompare(b.name)).slice(0, 12)
    )
      .map((t) => ({
        name: t.name,
        publishers: [
          ...new Set<string>((t.publishers || []).map((ep: Data) => ep.node)),
        ].sort(),
        subscribers: [
          ...new Set<string>((t.subscribers || []).map((ep: Data) => ep.node)),
        ].sort(),
      }))
      .sort((a, b) => a.name.localeCompare(b.name)),
  );
  return (
    <>
      <Tabs value={view} onValueChange={setView} className="subtabs">
        <TabsList>
          {[
            ["topics", "토픽"],
            ["graph", "연결"],
            ["nodes", "노드"],
            ["actions", "액션"],
            ["services", "서비스"],
          ].map(([id, label]) => (
            <TabsTrigger key={id} value={id}>
              {label}
            </TabsTrigger>
          ))}
        </TabsList>
      </Tabs>
      {view === "topics" ? (
        <div className="ros-grid">
          <Panel title={`토픽 (${topics.length})`}>
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
          <RosGraph spec={graphSpec} />
        </Panel>
      ) : (
        <Panel title={view}>
          <Json value={graph[view] || []} />
        </Panel>
      )}
      {graph.missing_fixed_topics?.length > 0 && (
        <details className="missing">
          <summary>미발견 토픽, {graph.missing_fixed_topics.length}</summary>
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
  const [navigationTab, setNavigationTab] = useState(() => {
    const stored = sessionStorage.getItem("cleany.navigation.tab");
    return stored &&
      ["overview", "safety", "control", "sensors", "compare"].includes(stored)
      ? stored
      : "overview";
  });
  useEffect(() => {
    sessionStorage.setItem("cleany.navigation.tab", navigationTab);
  }, [navigationTab]);
  const [collapsed, setCollapsed] = useState(false);
  const [tab, setTab] = useState(() => {
    const requested = location.hash.slice(1);
    return tabs.some(([id]) => id === requested) ? requested : "execution";
  });
  const [rows, setRows] = useState<Row[]>([]);
  const live = useRef<Row[]>([]);
  const [connected, setConnected] = useState(false);
  const [now, setNow] = useState(Date.now() / 1000);
  const [timelineNow, setTimelineNow] = useState(Date.now() / 1000);
  const [notice, setNotice] = useState("");
  const [selected, setSelected] = useState<string[]>([]);
  const selectedRef = useRef<string[]>([]);
  const navigationTopicsRef = useRef<string[]>([]);
  const [navigationTopics, setNavigationTopics] = useState<string[]>([]);
  const socket = useRef<WebSocket | null>(null);
  const lastBoot = useRef("");
  const [recording, setRecording] = useState<Data>({});
  const [sessions, setSessions] = useState<(Data & TimelineSession)[]>([]);
  const [timelineControlsHost, setTimelineControlsHost] =
    useState<HTMLElement | null>(null);
  const [timelineFocus, setTimelineFocus] = useState<number | null>(null);
  const [replayId, setReplayId] = useState("");
  const replayRef = useRef("");
  const [at, setAt] = useState(0);
  const seekPending = useRef(false);
  const [seekRevision, setSeekRevision] = useState(0);
  const [playing, setPlaying] = useState(false);
  const [playbackSpeed, setPlaybackSpeed] = useState(1);
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
        if (p.sources_changed) setNotice("관측 토픽 변경, 이전 관측값 초기화");
        if (p.initial && lastBoot.current && !p.sources_changed) {
          setNotice(
            p.boot_id === lastBoot.current
              ? "재연결, 누락 가능"
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
        const slamList = latest.get("slam:submaps")?.data;
        for (const [key, row] of latest) {
          if (
            row.kind === "slam_texture" &&
            (!slamList?.valid ||
              row.data.epoch !== slamList.epoch ||
              !slamList.submaps?.some(
                (s: Data) => s.id === row.key && s.version >= row.data.version,
              ) ||
              latest
                .get("slam:status")
                ?.data.omitted_textures?.includes(row.key))
          )
            latest.delete(key);
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
    const timelineClock = setInterval(
      () => setTimelineNow(Date.now() / 1000),
      1000,
    );
    return () => {
      stopped = true;
      clearTimeout(retry);
      clearInterval(clock);
      clearInterval(timelineClock);
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
    refreshSessions();
    const timer = setInterval(refreshSessions, 1000);
    return () => clearInterval(timer);
  }, []);
  useEffect(() => {
    replayRef.current = replayId;
    if (!replayId) {
      setRows(applyRuntimeEvents(live.current));
      setPlaying(false);
      return;
    }
    if (replayId === "__gap__") {
      setRows([]);
      return;
    }
    const controller = new AbortController();
    const timer = setTimeout(
      () =>
        fetch(`/api/recordings/${replayId}/replay?at=${at}`, {
          signal: controller.signal,
        })
          .then((r) => {
            if (!r.ok) throw Error("기록 읽기 실패");
            return r.json();
          })
          .then((p) => {
            if (controller.signal.aborted) return;
            setRows(applyRuntimeEvents(p.rows));
            if (seekPending.current) {
              seekPending.current = false;
              setSeekRevision((value) => value + 1);
            }
            if (p.metadata.gap) setNotice(p.metadata.gap);
          })
          .catch((e) => {
            if (e.name !== "AbortError") setError(String(e));
          }),
      60,
    );
    return () => {
      clearTimeout(timer);
      controller.abort();
    };
  }, [replayId, at]);
  const session = sessions.find((s) => s.id === replayId);
  const seekAbsolute = (moment: number, available = sessions) => {
    moment = Math.min(Date.now() / 1000, moment);
    const id = sessionAt(available, moment) || "__gap__";
    replayRef.current = id;
    if (id !== replayId) setRows([]);
    seekPending.current = true;
    setReplayId(id);
    setAt(moment);
  };
  const returnLive = () => {
    replayRef.current = "";
    setReplayId("");
    setPlaying(false);
    setNotice("");
  };
  useEffect(() => {
    if (!playing) return;
    const last = Math.min(
      Date.now() / 1000,
      Math.max(...recordingSegments(sessions).map((s) => s.end)),
    );
    if (!Number.isFinite(last)) {
      setPlaying(false);
      return;
    }
    const timer = setInterval(() => {
      const next = Math.min(last, at + 0.2 * playbackSpeed);
      seekAbsolute(next);
      if (next >= last) setPlaying(false);
    }, 200);
    return () => clearInterval(timer);
  }, [playing, at, playbackSpeed, sessions]);
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
  const completedBT = !runtime.active_request && !!runtime.last_result;
  const executeDetail = graphTab === "bt" && node === "ExecuteOne";
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
    setTimelineFocus(moment);
    setPlaying(false);
    if (replayId) {
      seekAbsolute(moment);
      return;
    }
    try {
      const response = await fetch("/api/recordings");
      const data = await response.json();
      if (!sessionAt(data.sessions, moment))
        throw Error(
          "해당 시점이 아직 기록되지 않았거나 기록이 중단되었습니다.",
        );
      setSessions(data.sessions);
      seekAbsolute(moment, data.sessions);
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
        <header className="monitor-header">
          <div
            className="header-timeline-controls"
            ref={setTimelineControlsHost}
          />
          <div className="header-status">
            <ThemeToggle />
            <span title="웹 관제보드 연결 상태입니다. Runtime 상태의 최신 여부는 실행 그래프에서 확인합니다.">
              <Badge tone={replayId ? "info" : connected ? "good" : "bad"}>
                {replayId ? "REPLAY" : connected ? "LIVE" : "연결 끊김"}
              </Badge>
            </span>
            <span>Domain {config.domain ?? "—"}</span>
            {clockRow && (
              <span title="ROS clock">
                sim {clockRow.data.time.toFixed(1)}s{" "}
                {clockRow.data.unchanged_for > 1 ||
                (!replayId && now - clockRow.received_at > 2)
                  ? ", 정지/수신 지연"
                  : ""}
              </span>
            )}
            <code>{config.runtime_namespace || "/"}</code>
            <span className="header-clock" aria-label="현재 시각">
              {new Date(now * 1000).toLocaleTimeString("ko-KR", {
                hour: "2-digit",
                minute: "2-digit",
                second: "2-digit",
                hour12: false,
              })}
            </span>
          </div>
          <RecordingTimeline
            controlsHost={timelineControlsHost}
            focusAt={timelineFocus}
            sessions={sessions}
            now={timelineNow}
            at={at}
            historical={!!replayId}
            playing={playing}
            speed={playbackSpeed}
            onSpeed={setPlaybackSpeed}
            onPlaying={setPlaying}
            onSeek={seekAbsolute}
            onLive={returnLive}
          />
        </header>
        <div className="content">
          <div className="page-title">
            <div>
              <h1>{tabs.find((t) => t[0] === tab)?.[1]}</h1>
            </div>
            <div className="page-actions">
              <Button
                variant="outline"
                onClick={() => {
                  setNavigationTab("overview");
                  go("navigation");
                }}
              >
                <Navigation2 size={16} /> 개요
              </Button>
            </div>
          </div>
          {!connected && !replayId && (
            <div className="connection-banner" role="status">
              <Unplug size={19} />
              <div>
                <strong>서버 연결 끊김, 재연결 중</strong>
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
            <div className="notice warn">기록 중단, {recording.reason}</div>
          )}

          {replayId && replayId !== "__gap__" && (
            <RecordingReplayNotice
              rows={rows}
              sessionId={replayId}
              timeAt={at}
            />
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
                      <span>
                        {runtime.state === "WORKING" && fresh
                          ? "청소 단계"
                          : "마지막 청소 단계"}
                      </span>
                      <strong>{runtime.bt_stage}</strong>
                    </div>
                  )}
                </div>
                {(!fresh || runtime.drive_block_reason || runtime.reason) && (
                  <p className="execution-reason">
                    {!fresh
                      ? `상태 미확인 · 마지막 Runtime 수신 ${age?.toFixed(1) ?? "—"}초 전. 아래 FSM/BT는 마지막 관측이며, SUCCESS는 당시 실행 결과입니다.`
                      : runtime.drive_block_reason || runtime.reason}
                  </p>
                )}
                {graphTab === "bt" &&
                  !executeDetail &&
                  runtime.manipulation_feedback?.execution_id && (
                    <section
                      className="execution-summary"
                      aria-label="조작 단계 관측"
                    >
                      <div>
                        <span>조작 관측</span>
                        <Badge tone={!fresh ? "warn" : "info"}>
                          {!fresh
                            ? "마지막 조작 관측 · 상태 미확인"
                            : runtime.active_request &&
                                runtime.bt_stage === "ExecuteOne"
                              ? "진행 중"
                              : "마지막 조작 관측"}
                        </Badge>
                      </div>
                      <div>
                        <span>조작 단계</span>
                        <strong>
                          {runtime.manipulation_feedback.stage || "—"}
                        </strong>
                      </div>
                      <div>
                        <span>세부 동작</span>
                        <strong>
                          {runtime.manipulation_feedback.substage || "—"}
                        </strong>
                      </div>
                    </section>
                  )}
                <div
                  className={
                    "execution-workspace " +
                    (node && !executeDetail ? "with-detail" : "")
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
                          <TabsTrigger value="bt">BT</TabsTrigger>
                        </TabsList>
                      </Tabs>
                      <div className="graph-toolbar-actions">
                        {graphTab === "bt" &&
                          runtime.bt_nodes?.some(
                            (n: Data) => n.id === "ExecuteOne",
                          ) && (
                            <Button
                              variant={executeDetail ? "secondary" : "outline"}
                              size="sm"
                              aria-expanded={executeDetail}
                              aria-controls="execute-one-detail"
                              onClick={() =>
                                setNode(executeDetail ? "" : "ExecuteOne")
                              }
                            >
                              <Hand size={14} />
                              조작 단계 {executeDetail ? "접기" : "보기"}
                            </Button>
                          )}
                        <Badge tone={!fresh ? "warn" : ""}>
                          {!fresh
                            ? graphTab === "bt"
                              ? "마지막 BT 관측 · 상태 미확인"
                              : "마지막 FSM 관측 · 상태 미확인"
                            : replayId
                              ? "기록 시점"
                              : graphTab === "bt" && completedBT
                                ? "마지막 청소 결과"
                                : "LIVE"}
                        </Badge>
                      </div>
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
                      selectedNode={node}
                    />
                    {graphTab === "bt" && runtime.bt_nodes?.length > 0 && (
                      <div className="bt-guide" aria-label="청소 BT 읽는 법">
                        <p>
                          <strong>왼쪽부터 순서 실행</strong>
                          <span>
                            UntilVerified가 완료 확인까지 작업 주기를
                            반복합니다.
                          </span>
                        </p>
                        <p className="bt-legend">
                          <span>
                            <i className="running" />
                            RUNNING 진행
                          </span>
                          <span>
                            <i className="success" />
                            SUCCESS 통과
                          </span>
                          <span>
                            <i className="failure" />
                            FAILURE 실패
                          </span>
                          <span>
                            <i className="invalid" />
                            INVALID 대기·초기화
                          </span>
                          <span>
                            <i className="visited" />
                            최근 tick 방문
                          </span>
                        </p>
                        <p>
                          노드를 누르면 역할과 통과 조건을 확인할 수 있습니다.
                          ExecuteOne의 조작 세부 단계는 매니퓰레이션 화면에서
                          확인합니다.
                        </p>
                      </div>
                    )}
                  </section>
                  {executeDetail && (
                    <ExecuteOneDetail
                      rows={rows}
                      timeAt={replayId ? at : now}
                      replay={!!replayId}
                      connected={connected}
                      close={() => {
                        setNode("");
                        const target = document.querySelector<HTMLElement>(
                          '.flow.bt .react-flow__node[data-id="ExecuteOne"]',
                        );
                        target?.focus();
                      }}
                      openDashboard={() => go("manipulation")}
                    />
                  )}
                  {node && !executeDetail && (
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
                      {graphTab === "bt" && (
                        <div className="bt-node-explanation">
                          <Badge>
                            {btKinds[
                              runtime.bt_nodes?.find((n: Data) => n.id === node)
                                ?.kind
                            ] || "BT 노드"}
                          </Badge>
                          <p>
                            {btHelp[node] || "Runtime이 제공한 BT 노드입니다."}
                          </p>
                          {node === "ExecuteOne" && (
                            <Button
                              variant="outline"
                              size="sm"
                              onClick={() => go("manipulation")}
                            >
                              조작 상세 보기 <ArrowUpRight size={14} />
                            </Button>
                          )}
                        </div>
                      )}
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
                                  , 정지{" "}
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
                              {time(r.received_at)}, {r.data.kind || r.kind}
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
          {tab === "resources" && (
            <ResourceDashboard rows={rows} replay={!!replayId} />
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
                    rows={rows}
                    config={config}
                    timeAt={replayId ? at : now}
                  />
                }
                observe={setNavigationTopics}
                openTopic={(topic) => {
                  setFocusTopic(topic);
                  go("ros");
                }}
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
                  replayRef.current = run.session_id;
                  const moment = Math.min(Date.now() / 1000, run.start);
                  setAt(moment);
                  setTimelineFocus(moment);
                  seekPending.current = true;
                }}
              />
            </>
          )}
          {tab === "manipulation" && (
            <ManipulationDashboard
              rows={rows}
              timeAt={replayId ? at : now}
              replay={!!replayId}
              connected={connected}
              seek={openNavigationMoment}
              openTopic={(topic) => {
                setFocusTopic(topic);
                go("ros");
              }}
            />
          )}
          {tab === "logs" && (
            <Logs rows={rows} filter={filter} setFilter={setFilter} />
          )}
          {tab === "recordings" && (
            <>
              <RecordingControls replay={!!replayId} />
              <details>
                <summary>기록 관리, 내보내기</summary>
                <Panel
                  title="기록 관리, 내보내기"
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
                            {(s.metadata.gap || s.metadata.raw_gap) && (
                              <Badge tone="warn">유실 있음</Badge>
                            )}
                            {!s.bag_recording &&
                              ["incomplete", "recording"].includes(
                                s.metadata.bag_state,
                              ) && <Badge tone="warn">rosbag2 미완료</Badge>}
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
                                const moment =
                                  s.segments?.[0]?.start ??
                                  s.runtime_start ??
                                  s.start;
                                setTimelineFocus(moment);
                                seekAbsolute(moment);
                                setPlaying(false);
                              }}
                            >
                              시각으로 이동
                            </Button>{" "}
                            <a href={`/api/recordings/${s.id}/export`}>
                              SQLite 내보내기 ↓
                            </a>
                            {s.bag_available ? (
                              <>
                                {" "}
                                ,{" "}
                                <a href={`/api/recordings/${s.id}/bag`}>
                                  rosbag2 내보내기 ↓
                                </a>
                              </>
                            ) : s.bag_recording ? (
                              <small>
                                {" "}
                                , rosbag2는 녹화 정지 후 내보낼 수 있습니다.
                              </small>
                            ) : null}
                          </td>
                        </tr>
                      ))}
                    </tbody>
                  </table>
                  {!sessions.length && <Empty>기록 없음</Empty>}
                </Panel>
              </details>
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
