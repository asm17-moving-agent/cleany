import { useEffect, useMemo, useState, type ReactNode } from "react";
import { Button } from "@/components/ui/button";
import { Badge } from "@/components/ui/badge";
import { Tabs, TabsList, TabsTrigger } from "@/components/ui/tabs";
import { Panel } from "@/components/panel";
import { type Data, type Row, time as stamp } from "@/lib/monitor";
import "./navigation.css";
import { SlamView } from "./slam-view";
import { Nav2BtTree } from "./nav2-bt-tree";

type Props = {
  rows: Row[];
  config: Data;
  graph: Data;
  timeAt: number;
  replay: boolean;
  tab: string;
  setTab: (tab: string) => void;
  map: ReactNode;
  observe: (topics: string[]) => void;
  openTopic: (topic: string) => void;
  openCamera: () => void;
  openLogs: () => void;
  seek: (at: number) => void;
  openRun: (run: Data) => void;
  sessionId: string;
};
const names: Record<string, string> = {
  overview: "주행 개요",
  safety: "장애물·안전",
  control: "계획·제어",
  sensors: "센서·위치",
  slam: "SLAM 분석",
  compare: "주행 비교",
};
const motions: Record<string, string> = {
  MOVING: "이동 중",
  CREEPING: "저속 이동",
  STATIONARY: "정지",
  UNOBSERVED: "실측 미확인",
};
const n = (value: unknown, digits = 3) =>
  typeof value === "number" && Number.isFinite(value)
    ? value.toFixed(digits)
    : "—";
const angle = (value: unknown) =>
  typeof value === "number" && Number.isFinite(value)
    ? `${((value * 180) / Math.PI).toFixed(1)}°`
    : "—";
const arr = (v: any): any[] => (Array.isArray(v) ? v : []);
function Tag({
  children,
  tone = "neutral",
}: {
  children: ReactNode;
  tone?: string;
}) {
  return (
    <Badge variant="secondary" className={`badge ${tone}`}>
      {children}
    </Badge>
  );
}
function Facts({ items }: { items: [string, ReactNode][] }) {
  return (
    <dl className="facts">
      {items.map(([k, v]) => (
        <div key={k}>
          <dt>{k}</dt>
          <dd>{v}</dd>
        </div>
      ))}
    </dl>
  );
}
function Empty({ children }: { children: ReactNode }) {
  return <div className="nav-empty">{children}</div>;
}
function Raw({ data }: { data: unknown }) {
  return (
    <details>
      <summary>관측 원본</summary>
      <pre>{JSON.stringify(data, null, 2)}</pre>
    </details>
  );
}
function SpeedPlot({ summary }: { summary: Data }) {
  const history = arr(summary.history);
  const count = arr(summary.commands).length;
  const colors = ["#69a9ff", "#edba67", "#be94ee", "#66c5bf", "#e1839f"];
  const labels = [...arr(summary.commands).map((c) => c.topic), "실측"];
  const values = history
    .flatMap((p) => p.slice(1, count + 2))
    .filter((v: any) => typeof v === "number" && Number.isFinite(v));
  const sorted = [...values].sort((a, b) => a - b);
  const max = Math.max(
    0.05,
    (sorted[Math.floor((sorted.length - 1) * 0.95)] || 0) * 1.2,
  );
  const peaks = history.flatMap((p) =>
    labels.flatMap((label, j) =>
      typeof p[j + 1] === "number" && p[j + 1] > max
        ? [{ at: p[0], value: p[j + 1], label }]
        : [],
    ),
  );
  const start = history[0]?.[0] ?? 0,
    end = history.at(-1)?.[0] ?? 1;
  return (
    <>
      <svg
        className="nav-speed-plot"
        viewBox="0 0 700 170"
        role="img"
        aria-label="단계별 속도와 실측 속도 그래프"
      >
        {[0, 0.5, 1].map((t) => (
          <g key={t}>
            <line
              x1="45"
              x2="685"
              y1={145 - t * 120}
              y2={145 - t * 120}
              stroke="var(--border)"
            />
            <text x="5" y={149 - t * 120}>
              {n(max * t, 2)}
            </text>
          </g>
        ))}
        {labels.map((_, j) => {
          let d = "",
            broken = true;
          for (const p of history) {
            const v = p[j + 1];
            if (typeof v !== "number" || !Number.isFinite(v)) {
              broken = true;
              continue;
            }
            if (v > max) {
              broken = true;
              continue;
            }
            const x = 45 + ((p[0] - start) / Math.max(0.2, end - start)) * 640,
              y = 145 - (v / max) * 120;
            d += `${broken ? "M" : "L"}${x},${y} `;
            broken = false;
          }
          return (
            <path
              key={j}
              d={d}
              fill="none"
              stroke={colors[j % colors.length]}
              strokeWidth={j === count ? 3 : 1.6}
            />
          );
        })}
        {peaks.map((peak, i) => (
          <circle
            key={i}
            cx={45 + ((peak.at - start) / Math.max(0.2, end - start)) * 640}
            cy="24"
            r="3"
            fill="var(--warn-text)"
          >
            <title>
              {peak.label}: {n(peak.value)} m/s · {stamp(peak.at)}
            </title>
          </circle>
        ))}
        <text x="45" y="166">
          {history.length ? stamp(start) : "데이터 대기"}
        </text>
        <text x="600" y="166">
          {history.length ? stamp(end) : ""}
        </text>
      </svg>
      <div className="nav-legend">
        {labels.map((l, i) => (
          <span key={l}>
            <i style={{ background: colors[i % colors.length] }} />
            {l}
          </span>
        ))}
        <span>m/s · 관측 누락은 선을 끊어 표시</span>
        {!!peaks.length && (
          <span>
            그래프 상한 초과 {peaks.length}개 · 최대{" "}
            {n(Math.max(...peaks.map((p) => p.value)))} m/s · 원본 유지
          </span>
        )}
      </div>
    </>
  );
}
function Diagnostic({
  title,
  row,
  topic,
  fresh,
  openTopic,
  children,
}: {
  title: string;
  row?: Row;
  topic: string;
  fresh: boolean;
  openTopic: (t: string) => void;
  children?: ReactNode;
}) {
  return (
    <Panel
      title={title}
      aside={
        <Tag tone={fresh ? "good" : "warn"}>
          {!row
            ? "미수신"
            : row.data.valid === false
              ? "해석 불가"
              : fresh
                ? "진단 수신"
                : "오래된 진단"}
        </Tag>
      }
    >
      <button className="nav-topic" onClick={() => openTopic(topic)}>
        {topic}
      </button>
      {row ? (
        <>
          {children}
          <Raw data={row.data} />
        </>
      ) : (
        <Empty>발행·활성 여부 확인 필요</Empty>
      )}
    </Panel>
  );
}
function Comparison({
  runs,
  config,
  onReplay,
}: {
  runs: Data[];
  config: Data;
  onReplay?: (r: Data) => void;
}) {
  const [left, setLeft] = useState("");
  const [right, setRight] = useState("");
  const key = (r: Data) => `${r.session_id || "current"}:${r.operation_id}`;
  const a = runs.find((r) => key(r) === left) || runs[0];
  const b =
    runs.find((r) => key(r) === right) ||
    runs.find(
      (r) => r !== a && r.target === a?.target && r.phase === a?.phase,
    ) ||
    runs[1];
  const settings = (r: Data) => r?.configuration || config;
  const comparable =
    a &&
    b &&
    a.target === b.target &&
    a.phase === b.phase &&
    a.measured_source === b.measured_source &&
    [
      "stationary_speed_mps",
      "creep_speed_mps",
      "navigation_freshness_s",
      "map_frame",
    ].every((k) => settings(a)[k] === settings(b)[k]);
  const metrics: [string, string, string][] = [
    ["wall_s", "소요 시간", "s"],
    ["sim_s", "시뮬 시간", "s"],
    ["rtf", "실측 평균 RTF", "×"],
    ["distance_m", "추정 위치 궤적 길이", "m"],
    ["stationary_s", "정지 관측 시간", "s"],
    ["creep_s", "저속 관측 시간", "s"],
    ["stationary_episodes", "정지 진입 횟수", "회"],
    ["unobserved_s", "분류 불가 시간", "s"],
    ["replan_starts", "최초 계획 이후 ComputePath 진입", "회"],
    ["goal_error_m", "마지막 관측 목표 위치 오차", "m"],
    ["goal_yaw_error_rad", "마지막 관측 목표 방향 오차", "rad"],
    ["gt_goal_error_m", "시뮬 실제 위치 목표 오차", "m"],
  ];
  const label = (r: Data) =>
    `${r.configuration?.run_label || r.session_id?.slice(0, 8) || "현재 기록"} · ${r.target} · ${r.phase === "RETURN_HOME" ? "복귀" : "도착"} · ${stamp(r.start)}`;
  return (
    <>
      <Panel
        title="실행 전후 비교"
        aside={
          <Tag tone={comparable ? "good" : "warn"}>
            {!a || !b
              ? "실행 두 개 필요"
              : comparable
                ? "관측 기준 일치"
                : "대상·관측 기준 다름"}
          </Tag>
        }
      >
        <div className="nav-compare-select">
          {[
            ["개선 전", left, setLeft, a],
            ["개선 후", right, setRight, b],
          ].map(([title, value, setter, fallback]: any) => (
            <label key={title}>
              {title}
              <select
                aria-label={title}
                value={value || (fallback ? key(fallback) : "")}
                onChange={(e) => setter(e.target.value)}
              >
                <option value="" disabled>
                  실행 선택
                </option>
                {runs.map((r) => (
                  <option key={key(r)} value={key(r)}>
                    {label(r)}
                  </option>
                ))}
              </select>
            </label>
          ))}
        </div>
        {!runs.length ? (
          <Empty>
            기록된 주행 구간이 없습니다. Runtime 주행 작업 ID와 실측 속도를
            수신하면 집계를 시작합니다.
          </Empty>
        ) : (
          <div className="nav-table-scroll">
            <table>
              <thead>
                <tr>
                  <th>항목</th>
                  <th>개선 전</th>
                  <th>개선 후</th>
                  <th>차이 (후−전)</th>
                </tr>
              </thead>
              <tbody>
                {metrics.map(([k, label, unit]) => (
                  <tr key={k}>
                    <td>{label}</td>
                    <td>
                      {n(a?.[k], 2)} {unit}
                    </td>
                    <td>
                      {n(b?.[k], 2)} {unit}
                    </td>
                    <td>
                      {comparable &&
                      typeof a?.[k] === "number" &&
                      typeof b?.[k] === "number"
                        ? `${b[k] - a[k] > 0 ? "+" : ""}${n(b[k] - a[k], 2)} ${unit}`
                        : "—"}
                    </td>
                  </tr>
                ))}
                <tr>
                  <td>실측 출처</td>
                  <td>{a?.measured_source || "미확인"}</td>
                  <td>{b?.measured_source || "미확인"}</td>
                  <td>ground_truth는 시뮬 전용</td>
                </tr>
                <tr>
                  <td>작업 결과</td>
                  <td>{a?.result || "—"}</td>
                  <td>{b?.result || "—"}</td>
                  <td>주행 품질 판정과 별도</td>
                </tr>
              </tbody>
            </table>
          </div>
        )}
        <small>
          정지 &lt; {n(config.stationary_speed_mps)} m/s, 저속 &lt;{" "}
          {n(config.creep_speed_mps)} m/s, |회전| &lt; 0.02 rad/s. 주행 작업
          구간만 집계하며 도착 후 대기는 제외합니다. 관측 공백과 상태 전환
          사이의 시간은 분류 불가로 남깁니다.
        </small>
      </Panel>
      <div className="nav-two">
        {[
          ["개선 전 설정", a],
          ["개선 후 설정", b],
        ].map(([title, r]: any) => (
          <Panel key={title} title={title}>
            <Facts
              items={[
                ["코드 버전", settings(r).source_revision || "미기록"],
                [
                  "요청 RTF",
                  settings(r).requested_rtf > 0
                    ? `${n(settings(r).requested_rtf, 2)}×`
                    : "미기록",
                ],
                ["기록 유실", r?.recording_gap || "유실 보고 없음"],
                ["작업 ID", r?.operation_id || "—"],
              ]}
            />
            <Raw data={settings(r)} />
            {r && onReplay && (
              <Button variant="outline" onClick={() => onReplay(r)}>
                구간 기록 재생
              </Button>
            )}
          </Panel>
        ))}
      </div>
    </>
  );
}
export function NavigationDashboard(p: Props) {
  const { rows, config, graph, timeAt, replay, tab, setTab } = p;
  const observation = (key: string) =>
    rows.find((r) => r.kind === "navigation" && r.key === key);
  const treeRow = p.rows.find((row) => row.kind === "nav2_bt_tree");
  const summaryRow = observation("summary");
  const summary = summaryRow?.data || {};
  const runtimeRow = rows.find((r) => r.kind === "runtime");
  const runtime = runtimeRow?.data || {};
  const clock = rows.find((r) => r.kind === "clock")?.data.time;
  const freshness = Number(config.navigation_freshness_s) || 0.75;
  const fresh = (r?: Row) =>
    !!r &&
    r.data.valid !== false &&
    timeAt - r.received_at >= (replay ? -0.001 : -0.75) &&
    timeAt - r.received_at <= freshness &&
    (r.ros_time == null ||
      (clock != null
        ? clock - r.ros_time <= freshness && clock - r.ros_time >= -0.1
        : typeof r.data.stamp_age_s !== "number" ||
          (r.data.stamp_age_s + Math.max(0, timeAt - r.received_at) <=
            freshness &&
            r.data.stamp_age_s >= -0.1)));
  const summaryFresh =
    !!summaryRow && timeAt - summaryRow.received_at < freshness;
  const runtimeFresh = !!runtimeRow && timeAt - runtimeRow.received_at <= 3;
  const available = JSON.stringify(
    (graph.topics || []).map((t: Data) => t.name).sort(),
  );
  useEffect(() => {
    if (replay) {
      p.observe([]);
      return;
    }
    const keys =
      tab === "overview"
        ? ["footprint"]
        : tab === "safety"
          ? [
              "scan",
              "depth_points",
              "risk_points",
              "footprint",
              "stop_polygon",
              "slow_polygon",
              "obstacle_memory",
              "lidar_memory",
              "depth_memory",
            ]
          : tab === "control"
            ? ["controller_evaluation", "local_path", "footprint"]
            : tab === "sensors"
              ? ["scan", "depth_points"]
              : tab === "slam"
                ? ["scan"]
                : [];
    const known = new Set<string>(JSON.parse(available));
    p.observe(
      keys.map((k) => config[k + "_topic"]).filter((t) => t && known.has(t)),
    );
  }, [tab, replay, available, JSON.stringify(config)]);
  const [savedRuns, setSavedRuns] = useState<Data[]>([]);
  const [comparisonRevision, setComparisonRevision] = useState(0);
  const [compareError, setCompareError] = useState("");
  useEffect(() => {
    if (tab !== "compare" || replay) return;
    const controller = new AbortController();
    fetch("/api/navigation/runs", { signal: controller.signal })
      .then((r) => {
        if (!r.ok) throw Error("비교 기록 읽기 실패");
        return r.json();
      })
      .then((d) => {
        setSavedRuns(d.runs || []);
        setCompareError("");
      })
      .catch((e) => {
        if (e.name !== "AbortError") setCompareError(String(e));
      });
    return () => controller.abort();
  }, [tab, replay, comparisonRevision]);
  const pan = observation("pan"),
    guard = observation("reactive_guard"),
    voxel = observation("voxel_guard"),
    collision = observation("collision"),
    coverage = observation("coverage");
  const poseRow = rows.find((r) => r.kind === "pose");
  const pose = poseRow?.data;
  const commands = arr(summary.commands);
  const aligned =
    commands.length > 1 &&
    commands.every((c) => c.fresh && typeof c.received_at === "number") &&
    Math.max(...commands.map((c) => c.received_at)) -
      Math.min(...commands.map((c) => c.received_at)) <=
      0.2;
  const reductions = aligned
    ? commands.slice(1).flatMap((c, i) => {
        const before = Math.hypot(
          ...(commands[i].vector.slice(0, 2) as [number, number]),
        );
        const after = Math.hypot(...(c.vector.slice(0, 2) as [number, number]));
        return before - after > 0.001
          ? [`${c.topic}: ${n(before)} → ${n(after)} m/s`]
          : [];
      })
    : [];
  const events = useMemo(() => {
    const notes = arr(summary.timeline).map((entry, i) => ({
      seq: -Math.round(entry.received_at * 1e6) - i,
      kind: "event",
      key: "navigation-history",
      received_at: entry.received_at,
      ros_time: null,
      data: { kind: entry.kind, data: entry.data },
    }));
    const raw = rows.filter(
      (r) =>
        r.kind === "event" &&
        (String(r.data.kind).startsWith("navigation_") ||
          r.data.kind === "nav2_bt" ||
          r.data.data?.module === "navigation" ||
          r.data.kind === "clock_rewind"),
    );
    const extra = notes.length
      ? raw.filter(
          (r) =>
            r.data.kind === "clock_rewind" ||
            (r.data.kind === "nav2_bt" &&
              r.data.data?.status !== "FAILURE" &&
              !["ComputePath", "FollowPath", "Wait"].some((name) =>
                String(r.data.data?.node).includes(name),
              )),
        )
      : raw;
    return [...notes, ...extra].sort((a, b) => b.received_at - a.received_at);
  }, [rows, summary.timeline]);
  const [eventFilter, setEventFilter] = useState("all");
  useEffect(() => setEventFilter("all"), [tab]);
  const scopedEvents = events.filter((r) =>
    tab === "safety"
      ? ["navigation_decision", "navigation_motion", "clock_rewind"].includes(
          r.data.kind,
        )
      : tab === "control"
        ? r.data.kind === "nav2_bt" ||
          r.data.kind === "clock_rewind" ||
          r.data.data?.module === "navigation"
        : tab === "overview"
          ? r.data.kind !== "nav2_bt" && r.data.kind !== "navigation_decision"
          : true,
  );
  const currentRuns = arr(summary.runs).map((r) => ({
    ...r,
    session_id: p.sessionId,
    configuration: config,
  }));
  const mergedRuns = new Map<string, Data>();
  for (const run of [...savedRuns, ...currentRuns])
    mergedRuns.set(`${run.session_id}:${run.operation_id}`, run);
  const runs = replay
    ? currentRuns
    : [...mergedRuns.values()].sort((a, b) => a.start - b.start);
  return (
    <div className="navigation-dashboard">
      <Tabs value={tab} onValueChange={setTab}>
        <TabsList className="nav-tabs" aria-label="자율주행 상세">
          {Object.entries(names).map(([id, label]) => (
            <TabsTrigger key={id} value={id}>
              {label}
            </TabsTrigger>
          ))}
        </TabsList>
      </Tabs>
      <div className="nav-purpose">
        <h2>
          {
            (
              {
                overview: "전체 주행 진행",
                safety: "정지·감속 원인",
                control: "경로 선택과 제어 출력",
                sensors: "관측 신뢰도와 위치 추정",
                compare: "주행 결과 비교",
                slam: "SLAM 지도와 위치 보정",
              } as Record<string, string>
            )[tab]
          }
        </h2>
        <p>
          {
            (
              {
                overview: "전체 경로에서 목표와 실제 이동 진행을 확인합니다.",
                safety: "로봇 주변 장애물과 감시 모듈의 판단을 확인합니다.",
                control:
                  "Nav2의 실행 상태, 선택한 궤적과 속도 출력을 비교합니다.",
                sensors: "센서 수신 지연과 위치 추정 오차를 확인합니다.",
                compare: "같은 대상과 관측 기준의 실행 결과를 비교합니다.",
                slam: "서브맵, 스캔과 궤적을 겹쳐 보고 상단 타임라인으로 보정 전후를 재생합니다.",
              } as Record<string, string>
            )[tab]
          }
        </p>
      </div>
      {tab === "slam" && (
        <SlamView
          rows={rows}
          config={config}
          timeAt={timeAt}
          replay={replay}
          sessionId={p.sessionId}
          seek={p.seek}
        />
      )}
      {tab === "overview" && (
        <div className="nav-summary-strip">
          <div>
            <small>주행 상태</small>
            <strong>
              {summaryFresh
                ? motions[summary.motion] || "실측 미확인"
                : "관측 지연 / 미수신"}
            </strong>
          </div>
          <div>
            <small>목표 / 단계</small>
            <strong>
              {runtimeFresh
                ? `${runtime.navigation?.target || "—"} / ${runtime.state || "—"}`
                : "Runtime 미확인"}
            </strong>
          </div>
          <div>
            <small>실측 속도</small>
            <strong>
              {summaryFresh ? n(summary.speed_mps) : "—"} <em>m/s</em>
            </strong>
          </div>
          <div>
            <small>실측 RTF / 요청</small>
            <strong>
              {summaryFresh ? n(summary.rtf, 2) : "—"}×{" "}
              <em>
                /{" "}
                {config.requested_rtf > 0
                  ? `${n(config.requested_rtf, 2)}×`
                  : "미기록"}
              </em>
            </strong>
          </div>
          <Tag tone={replay ? "warn" : "neutral"}>
            {replay ? "기록 재생" : "실시간 관측"}
          </Tag>
        </div>
      )}
      {tab === "overview" && (
        <>
          <div className="nav-overview">
            {p.map}
            <div className="nav-stack">
              <Panel title="현재 주행 판단">
                <Facts
                  items={[
                    [
                      "관측된 판단",
                      summaryFresh
                        ? summary.reason || "원인 미확정"
                        : "관측 지연",
                    ],
                    [
                      "진단 출처",
                      summaryFresh ? summary.reason_source || "미확정" : "—",
                    ],
                    [
                      "실측 출처",
                      summaryFresh ? summary.measured_source || "미수신" : "—",
                    ],
                    [
                      "회전 속도",
                      summaryFresh ? `${n(summary.measured?.[2])} rad/s` : "—",
                    ],
                  ]}
                />
                <div className="inline nav-detail-links">
                  <Button variant="outline" onClick={() => setTab("safety")}>
                    정지·감속 원인 보기
                  </Button>
                  <Button variant="outline" onClick={() => setTab("control")}>
                    경로·속도 명령 보기
                  </Button>
                </div>
              </Panel>
              <Panel title="현재 구간 진행">
                <Facts
                  items={[
                    [
                      "경과 시간",
                      summaryFresh && summary.active
                        ? `${n(summary.active.wall_s, 1)} s`
                        : "주행 구간 없음",
                    ],
                    [
                      "관측 목표 오차",
                      summaryFresh && summary.active
                        ? `${n(summary.active.goal_error_m, 2)} m`
                        : "—",
                    ],
                    [
                      "실측 출처",
                      summaryFresh ? summary.measured_source || "미수신" : "—",
                    ],
                    [
                      "직전 구간 결과",
                      [...runs].reverse().find((r) => r.result)?.result ||
                        "기록 없음",
                    ],
                  ]}
                />
              </Panel>
            </div>
          </div>
        </>
      )}
      {tab === "safety" && (
        <>
          <Panel title="정지·감속 판단 요약">
            <div className="nav-safety-decisions">
              <div>
                <small>Reactive guard</small>
                <strong>
                  {fresh(guard)
                    ? guard?.data.decision || "판단 미제공"
                    : "미수신 / 지연"}
                </strong>
              </div>
              <div>
                <small>Collision Monitor</small>
                <strong>
                  {fresh(collision)
                    ? collision?.data.polygon_name ||
                      String(collision?.data.action_type ?? "동작 미제공")
                    : "미수신 / 지연"}
                </strong>
              </div>
              <div>
                <small>속도 감소 관측</small>
                <strong>
                  {summaryFresh && aligned
                    ? reductions.length
                      ? reductions.join(" / ")
                      : "단계 간 감소 없음"
                    : "시각 정렬 불충분"}
                </strong>
              </div>
            </div>
            <small>
              단계별 명령은 수신 시각이 가까운 값의 비교입니다. 원인은 아래 감시
              모듈 진단으로 확인합니다.
            </small>
          </Panel>
          <div className="nav-overview">
            {p.map}
            <div className="nav-stack">
              <Diagnostic
                title="Reactive guard"
                row={guard}
                topic={config.reactive_guard_topic || "/reactive_guard/state"}
                fresh={fresh(guard)}
                openTopic={p.openTopic}
              >
                <Facts
                  items={[
                    ["판단", guard?.data.decision || "—"],
                    [
                      "처리 시간",
                      `${n(guard?.data.processing_s * 1000, 2)} ms`,
                    ],
                    ["입력 최대 나이", `${n(guard?.data.source_age_s)} s`],
                    ["관측 범위", guard?.data.observation_scope || "미제공"],
                  ]}
                />
                <small>
                  위험점·최소 거리·예상 제동 영역은 현재 이 진단에 제공되지
                  않습니다.
                </small>
              </Diagnostic>
              <Diagnostic
                title="Collision Monitor"
                row={collision}
                topic={
                  config.collision_state_topic || "/collision_monitor_state"
                }
                fresh={fresh(collision)}
                openTopic={p.openTopic}
              >
                <Facts
                  items={[
                    ["동작", collision?.data.action_type ?? "미제공"],
                    ["영역", collision?.data.polygon_name || "미제공"],
                  ]}
                />
              </Diagnostic>
            </div>
          </div>
          <div className="nav-two">
            <Diagnostic
              title="Voxel guard · 별도 속도 감시"
              row={voxel}
              topic={config.voxel_guard_topic || "/body_guard_3d/state"}
              fresh={fresh(voxel)}
              openTopic={p.openTopic}
            >
              <Facts
                items={[
                  ["판단", voxel?.data.decision || "—"],
                  [
                    "정지 / 감속 voxel",
                    `${voxel?.data.stop_voxels ?? "—"} / ${voxel?.data.slow_voxels ?? "—"}`,
                  ],
                  ["처리 시간", `${n(voxel?.data.processing_ms, 2)} ms`],
                ]}
              />
            </Diagnostic>
            <Diagnostic
              title="Depth coverage · 관측 진단"
              row={coverage}
              topic={config.coverage_topic || "/head_pan/coverage"}
              fresh={fresh(coverage)}
              openTopic={p.openTopic}
            >
              <Facts
                items={[
                  [
                    "커버리지",
                    coverage?.data.reason ||
                      coverage?.data.decision ||
                      "원본 확인",
                  ],
                  [
                    "주행 권한",
                    coverage?.data.motion_authority === false
                      ? "없음"
                      : "미제공",
                  ],
                ]}
              />
              <small>미관측 공간을 안전한 빈 공간으로 표시하지 않습니다.</small>
            </Diagnostic>
          </div>
          <Panel title="레이어 수신과 출처">
            <div className="nav-table-scroll">
              <table>
                <thead>
                  <tr>
                    <th>레이어</th>
                    <th>용도</th>
                    <th>관측 상태</th>
                    <th>토픽</th>
                  </tr>
                </thead>
                <tbody>
                  {[
                    [
                      "local_costmap",
                      "local_costmap_topic",
                      "근거리 계획 비용",
                    ],
                    [
                      "global_costmap",
                      "global_costmap_topic",
                      "전체 경로 비용",
                    ],
                    [
                      "obstacle_memory",
                      "obstacle_memory_topic",
                      "누적 장애물 2D 투영",
                    ],
                    ["lidar_memory", "lidar_memory_topic", "LiDAR 메모리"],
                    ["depth_memory", "depth_memory_topic", "Depth 메모리"],
                    ["scan", "scan_topic", "현재 LiDAR 표면"],
                    [
                      "depth_points",
                      "depth_points_topic",
                      "현재 depth 장애물 표면",
                    ],
                    ["risk_points", "risk_points_topic", "Voxel guard 위험점"],
                  ].map(([k, c, role]) => {
                    const r = rows.find(
                      (r) =>
                        (r.kind === "layer" || r.kind === "navigation") &&
                        r.key === k,
                    );
                    return (
                      <tr key={k}>
                        <td>{k}</td>
                        <td>{role}</td>
                        <td>
                          {!r
                            ? "미수신"
                            : r.data.valid === false
                              ? r.data.reason
                              : timeAt - r.received_at > 2
                                ? "마지막 관측 · 갱신 지연"
                                : "수신"}
                          {r?.data.sampled && " · 점군 표본"}
                        </td>
                        <td>
                          <button
                            className="nav-topic"
                            onClick={() => p.openTopic(config[c])}
                          >
                            {config[c] || "미설정"}
                          </button>
                        </td>
                      </tr>
                    );
                  })}
                </tbody>
              </table>
            </div>
            <small>
              VoxelLayer는 costmap 생성, voxel memory는 기록, voxel guard는 속도
              감시입니다. 토픽 미수신만으로 비활성이나 통과를 단정하지 않습니다.
            </small>
          </Panel>
        </>
      )}
      {tab === "control" && (
        <>
          <Panel title="속도 명령 → 실제 이동">
            <div className="nav-command-chain">
              {commands.map((c, i) => (
                <div key={c.topic}>
                  <small>
                    {i === 0
                      ? "주행 요청"
                      : i === commands.length - 1
                        ? "최종 명령"
                        : "감시 후 명령"}
                  </small>
                  <button
                    className="nav-topic"
                    onClick={() => p.openTopic(c.topic)}
                  >
                    {c.topic}
                  </button>
                  <strong>
                    {summaryFresh && c.fresh
                      ? n(
                          Math.hypot(
                            ...(c.vector.slice(0, 2) as [number, number]),
                          ),
                        )
                      : "—"}{" "}
                    <em>m/s</em>
                  </strong>
                  <small>
                    vx {n(c.vector?.[0])} · vy {n(c.vector?.[1])} · ω{" "}
                    {n(c.vector?.[2])}
                  </small>
                  <Tag tone={summaryFresh && c.fresh ? "good" : "warn"}>
                    {!c.vector
                      ? "미수신"
                      : summaryFresh && c.fresh
                        ? "관측 중"
                        : "오래된 명령"}
                  </Tag>
                </div>
              ))}
            </div>
            <SpeedPlot summary={summary} />
          </Panel>
          <div className="nav-overview nav-control-flow">
            {p.map}
            <Panel
              title="Nav2 BT · 실행 트리"
              aside={
                <Tag
                  tone={
                    treeRow && timeAt - treeRow.received_at < 2
                      ? "good"
                      : "warn"
                  }
                >
                  {treeRow
                    ? timeAt - treeRow.received_at < 2
                      ? "구조·상태 수신"
                      : "관측 지연"
                    : observation("nav2_bt")
                      ? "이벤트 기반"
                      : "미수신"}
                </Tag>
              }
            >
              {treeRow ? (
                <Nav2BtTree row={treeRow} timeAt={timeAt} replay={p.replay} />
              ) : Object.keys(summary.bt_status || {}).length ? (
                <div className="nav-bt-nodes">
                  {Object.entries(summary.bt_status).map(([name, status]) => (
                    <div key={name}>
                      <strong>{name}</strong>
                      <Tag
                        tone={
                          status === "RUNNING"
                            ? "good"
                            : status === "FAILURE"
                              ? "warn"
                              : "neutral"
                        }
                      >
                        {String(status)}
                      </Tag>
                    </div>
                  ))}
                </div>
              ) : (
                <Empty>
                  Nav2 BT 이벤트를 기다리고 있습니다. 청소 BT는 실행 흐름 탭에서
                  확인합니다.
                </Empty>
              )}
              <button
                className="nav-topic"
                onClick={() =>
                  p.openTopic(config.nav2_bt_topic || "/behavior_tree_log")
                }
              >
                {config.nav2_bt_topic || "/behavior_tree_log"}
              </button>
              <small>
                실행 XML의 구조와 이벤트에서 관측한 상태를 표시합니다. 구조
                정보가 미수신이면 관측된 노드 목록만 표시합니다.
              </small>
              <Facts
                items={[
                  [
                    "현재 작업 재계획 관측",
                    summary.active?.replan_starts ?? "—",
                  ],
                  ["작업 ID", runtime.navigation?.operation_id || "—"],
                ]}
              />
            </Panel>
          </div>
          <Panel title="컨트롤러 후보 궤적과 평가">
            <Controller
              row={observation("controller_evaluation")}
              openTopic={p.openTopic}
              topic={config.controller_evaluation_topic || "/evaluation"}
            />
          </Panel>
          <Panel title="Nav2 서버 상태">
            <div className="nav-bt-nodes">
              {Object.entries(runtime.navigation?.lifecycle || {}).map(
                ([name, state]) => (
                  <div key={name}>
                    <strong>{name}</strong>
                    <Tag tone={runtimeFresh ? "neutral" : "warn"}>
                      {runtimeFresh ? String(state) : "오래된 상태"}
                    </Tag>
                  </div>
                ),
              )}
            </div>
            {!Object.keys(runtime.navigation?.lifecycle || {}).length && (
              <Empty>Runtime lifecycle 진단 미수신</Empty>
            )}
          </Panel>
        </>
      )}
      {tab === "sensors" && (
        <>
          <Panel title="센서·TF 수신 상태">
            <div className="nav-table-scroll">
              <table>
                <thead>
                  <tr>
                    <th>관측 대상</th>
                    <th>상태</th>
                    <th>수신 Hz</th>
                    <th>수신 후 경과</th>
                    <th>ROS 데이터 나이</th>
                    <th>관측 기준 여유</th>
                  </tr>
                </thead>
                <tbody>
                  {[
                    "scan",
                    "depth_points",
                    "odometry",
                    "ground_truth",
                    "amcl_pose",
                    "pan_state",
                    "reactive_guard",
                    "voxel_guard",
                    "coverage",
                  ].map((k) => {
                    const topic = config[k + "_topic"];
                    const meta = arr(graph.topics).find(
                      (t) => t.name === topic,
                    );
                    const key = k === "pan_state" ? "pan" : k;
                    const row = observation(key);
                    const age = row
                      ? Math.max(0, timeAt - row.received_at)
                      : null;
                    const rosAge =
                      row && row.ros_time != null && clock != null
                        ? clock - row.ros_time
                        : typeof row?.data.stamp_age_s === "number"
                          ? row.data.stamp_age_s + (age || 0)
                          : null;
                    return (
                      <tr key={k}>
                        <td>
                          <button
                            className="nav-topic"
                            onClick={() => p.openTopic(topic)}
                          >
                            {topic || k}
                          </button>
                        </td>
                        <td>
                          {row?.data.valid === false
                            ? row.data.reason
                            : row
                              ? fresh(row)
                                ? "관측 중"
                                : "갱신 지연"
                              : meta?.state || "미발견"}
                        </td>
                        <td>{n(meta?.received_hz, 1)}</td>
                        <td>{n(age)} s</td>
                        <td>{n(rosAge)} s</td>
                        <td>
                          {age == null
                            ? "—"
                            : `${n(freshness - Math.max(age, rosAge ?? 0))} s`}
                        </td>
                      </tr>
                    );
                  })}
                  <tr>
                    <td>map → base TF</td>
                    <td>
                      {poseRow &&
                      timeAt - poseRow.received_at <= 2 &&
                      pose?.valid
                        ? "관측 중"
                        : pose?.reason || "미확인"}
                    </td>
                    <td>—</td>
                    <td>
                      {n(poseRow ? timeAt - poseRow.received_at : null)} s
                    </td>
                    <td>{n(pose?.age)} s</td>
                    <td>—</td>
                  </tr>
                </tbody>
              </table>
            </div>
            <small>
              여유는 대시보드 관측 기준 {n(freshness)} s에 대한 값입니다. 실제
              guard의 안전 유효시간·판정은 해당 guard 진단을 따릅니다. 수신 Hz는
              발행 Hz와 다를 수 있습니다.
            </small>
          </Panel>
          <div className="nav-two">
            <Panel title="위치 추정과 실제 위치">
              <Location rows={rows} fresh={fresh} />
            </Panel>
            <Diagnostic
              title="Pan 상태"
              row={pan}
              topic={config.pan_state_topic || "/head_pan/state"}
              fresh={fresh(pan)}
              openTopic={p.openTopic}
            >
              <Facts
                items={[
                  [
                    "현재 / 목표",
                    `${angle(pan?.data.pan)} / ${angle(pan?.data.target)}`,
                  ],
                  ["상태", pan?.data.reason || "—"],
                  [
                    "차체 방향",
                    pose?.valid && poseRow && timeAt - poseRow.received_at <= 2
                      ? angle(pose.pose?.[2])
                      : "—",
                  ],
                  [
                    "차체 기준 이동 방향",
                    summaryFresh && summary.speed_mps > 0.005
                      ? angle(
                          Math.atan2(summary.measured[1], summary.measured[0]),
                        )
                      : "정지 / 미확인",
                  ],
                  [
                    "역할",
                    pan?.data.motion_authority === false
                      ? "관측 전용"
                      : pan?.data.motion_authority === true
                        ? "주행 차단 권한 있음"
                        : "권한 미제공",
                  ],
                ]}
              />
              <Button variant="outline" onClick={p.openCamera}>
                카메라 영상 보기
              </Button>
            </Diagnostic>
          </div>
        </>
      )}
      {tab === "compare" && (
        <>
          {compareError && <p role="alert">{compareError}</p>}
          {!replay && (
            <Button
              variant="outline"
              onClick={() => setComparisonRevision((v) => v + 1)}
            >
              기록 목록 새로고침
            </Button>
          )}
          <Comparison runs={runs} config={config} onReplay={p.openRun} />
          {summary.active && (
            <Panel title="현재 주행 구간 · 집계 중">
              <Facts
                items={[
                  ["목표", summary.active.target],
                  ["경과 시간", `${n(summary.active.wall_s, 2)} s`],
                  [
                    "정지 / 저속",
                    `${n(summary.active.stationary_s, 2)} / ${n(summary.active.creep_s, 2)} s`,
                  ],
                  ["분류 불가 시간", `${n(summary.active.unobserved_s, 2)} s`],
                ]}
              />
            </Panel>
          )}
        </>
      )}
      {tab !== "slam" && (
        <Panel
          title={
            tab === "safety"
              ? "감시 판단과 정지 이력"
              : tab === "control"
                ? "BT 전이와 주행 작업 이력"
                : "주행 타임라인"
          }
          aside={
            <div className="inline">
              <select
                aria-label="주행 이벤트 필터"
                value={eventFilter}
                onChange={(e) => setEventFilter(e.target.value)}
              >
                <option value="all">이 화면의 이벤트</option>
                {tab !== "control" && <option value="motion">이동·정지</option>}
                {tab !== "overview" && tab !== "control" && (
                  <option value="decision">감시 판단</option>
                )}
                {tab !== "overview" && tab !== "safety" && (
                  <option value="bt">Nav2 BT</option>
                )}
              </select>
              <Button variant="outline" onClick={p.openLogs}>
                전체 로그
              </Button>
            </div>
          }
        >
          <div className="nav-timeline">
            {scopedEvents
              .filter(
                (r) =>
                  eventFilter === "all" ||
                  (eventFilter === "motion" &&
                    r.data.kind === "navigation_motion") ||
                  (eventFilter === "decision" &&
                    r.data.kind === "navigation_decision") ||
                  (eventFilter === "bt" && r.data.kind === "nav2_bt"),
              )
              .slice(0, 80)
              .map((r) => (
                <button
                  key={r.seq}
                  className="nav-event"
                  onClick={() => p.seek(r.received_at)}
                  title="이 시각의 기록으로 이동"
                >
                  <time>{stamp(r.received_at)}</time>
                  <Tag>{r.data.kind}</Tag>
                  <span>
                    {r.data.data?.node
                      ? `${r.data.data.node}: ${r.data.data.previous} → ${r.data.data.status}`
                      : r.data.data?.motion
                        ? `${motions[r.data.data.motion]} · ${r.data.data.reason}`
                        : r.data.data?.decision
                          ? `${r.data.data.source}: ${r.data.data.decision}`
                          : JSON.stringify(r.data.data || r.data)}
                  </span>
                </button>
              ))}
          </div>
          {!scopedEvents.length && (
            <Empty>이 화면에 해당하는 주행 이벤트 없음</Empty>
          )}
          <small>
            이벤트를 선택하면 해당 수신 시각의 기록으로 이동합니다. 큰 센서
            데이터는 해당 화면에서 관측했던 구간만 재생할 수 있습니다.
          </small>
        </Panel>
      )}
    </div>
  );
}
function Controller({
  row,
  topic,
  openTopic,
}: {
  row?: Row;
  topic: string;
  openTopic: (t: string) => void;
}) {
  const ts = arr(row?.data.trajectories);
  return (
    <>
      <button className="nav-topic" onClick={() => openTopic(topic)}>
        {topic}
      </button>
      {!ts.length ? (
        <Empty>
          후보 평가 데이터 미수신. 실제 컨트롤러가 평가 토픽을 발행하는지
          확인하세요.
        </Empty>
      ) : (
        <>
          <small>
            총 {row?.data.count}개 중 {ts.length}개 표본 · 선택 후보 #
            {row?.data.best_index} 포함 · 음수 점수는 컨트롤러가 제공한 무효
            평가
          </small>
          <div className="nav-table-scroll">
            <table>
              <thead>
                <tr>
                  <th>후보</th>
                  <th>총점</th>
                  <th>vx / vy / ω</th>
                  <th>평가 항목 (원점수 × 가중치)</th>
                </tr>
              </thead>
              <tbody>
                {ts.map((t) => (
                  <tr
                    key={t.index}
                    className={t.selected ? "nav-selected" : ""}
                  >
                    <td>
                      {t.selected ? "선택" : "후보"} #{t.index}
                    </td>
                    <td>{n(t.total)}</td>
                    <td>
                      {arr(t.velocity)
                        .map((v) => n(v))
                        .join(" / ")}
                    </td>
                    <td>
                      {arr(t.scores)
                        .map(
                          (s) => `${s.name}: ${n(s.raw, 2)} × ${n(s.scale, 2)}`,
                        )
                        .join(" · ")}
                    </td>
                  </tr>
                ))}
              </tbody>
            </table>
          </div>
        </>
      )}
      <Raw data={row?.data} />
    </>
  );
}
function Location({
  rows,
  fresh,
}: {
  rows: Row[];
  fresh: (r?: Row) => boolean;
}) {
  const amcl = rows.find(
    (r) => r.kind === "navigation" && r.key === "amcl_pose",
  );
  const odom = rows.find(
    (r) => r.kind === "navigation" && r.key === "odometry",
  );
  const gt = rows.find(
    (r) => r.kind === "navigation" && r.key === "ground_truth",
  );
  const ap = fresh(amcl) ? amcl?.data.map_pose : null,
    gp = fresh(gt) ? gt?.data.map_pose : null;
  const error = ap && gp ? Math.hypot(ap[0] - gp[0], ap[1] - gp[1]) : null;
  return (
    <Facts
      items={[
        [
          "AMCL map 위치",
          ap ? `${n(ap[0])}, ${n(ap[1])}, ${angle(ap[2])}` : "미확인",
        ],
        [
          "AMCL 위치 표준편차",
          fresh(amcl)
            ? `x ${n(Math.sqrt(Math.max(0, amcl?.data.covariance?.[0])))} / y ${n(Math.sqrt(Math.max(0, amcl?.data.covariance?.[7])))} m`
            : "—",
        ],
        [
          "Odometry 위치",
          fresh(odom)
            ? `${arr(odom?.data.pose)
                .map((v) => n(v))
                .join(", ")} (${odom?.data.source_frame})`
            : "미확인",
        ],
        [
          "Ground truth map 위치",
          gp ? `${n(gp[0])}, ${n(gp[1])}` : "시뮬 전용 · 좌표 변환/수신 미확인",
        ],
        ["추정 ↔ 시뮬 실제 위치 오차", `${n(error)} m`],
      ]}
    />
  );
}
